"""Respond to ESP32 STC ADC requests with synthetic encrypted samples.

The ESP32 firmware sends one encrypted GET_ADC_ONCE request every 40 ms.
This program acts as a PC-side STC simulator on the same UART and returns
samples that draw a selectable Lissajous pattern on the LCD.
"""

from __future__ import annotations

import argparse
import math
import struct
import sys
import time
from dataclasses import dataclass

try:
    import serial
except ImportError as exc:  # pragma: no cover - exercised only on setup errors
    raise SystemExit("pyserial is required. Install it with: python -m pip install pyserial") from exc


SYNC = b"\xA5\x5A"
VERSION = 0x01
TYPE_REQUEST = 0x10
TYPE_RESPONSE = 0x90
CMD_GET_ADC_ONCE = 0x01
STATUS_OK = 0x00
REQUEST_PAYLOAD_LEN = 8
RESPONSE_PAYLOAD_LEN = 16
MAX_FRAME_LEN = 2 + 4 + RESPONSE_PAYLOAD_LEN + 2
MASK32 = 0xFFFFFFFF
DELTA = 0x9E3779B9
KEY = (0x13579BDF, 0x2468ACE0, 0x0F1E2D3C, 0x4B5A6978)


def xtea_encrypt_block(block: bytes) -> bytes:
    v0, v1 = struct.unpack("<2I", block)
    total = 0
    for _ in range(32):
        v0 = (v0 + ((((v1 << 4) ^ (v1 >> 5)) + v1) ^ (total + KEY[total & 3]))) & MASK32
        total = (total + DELTA) & MASK32
        v1 = (v1 + ((((v0 << 4) ^ (v0 >> 5)) + v0) ^ (total + KEY[(total >> 11) & 3]))) & MASK32
    return struct.pack("<2I", v0, v1)


def xtea_decrypt_block(block: bytes) -> bytes:
    v0, v1 = struct.unpack("<2I", block)
    total = 0xC6EF3720
    for _ in range(32):
        v1 = (v1 - ((((v0 << 4) ^ (v0 >> 5)) + v0) ^ (total + KEY[(total >> 11) & 3]))) & MASK32
        total = (total - DELTA) & MASK32
        v0 = (v0 - ((((v1 << 4) ^ (v1 >> 5)) + v1) ^ (total + KEY[total & 3]))) & MASK32
    return struct.pack("<2I", v0, v1)


def crypt_payload(payload: bytes, encrypt: bool) -> bytes:
    crypt = xtea_encrypt_block if encrypt else xtea_decrypt_block
    return b"".join(crypt(payload[index:index + 8]) for index in range(0, len(payload), 8))


def crc16_ccitt_false(data: bytes) -> int:
    crc = 0xFFFF
    for value in data:
        crc ^= value << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def make_frame(frame_type: int, sequence: int, payload: bytes) -> bytes:
    if len(payload) == 0 or len(payload) % 8 or len(payload) > RESPONSE_PAYLOAD_LEN:
        raise ValueError("payload length must be one or two XTEA blocks")
    header = bytes((VERSION, frame_type, sequence & 0xFF, len(payload)))
    encrypted = crypt_payload(payload, True)
    crc = crc16_ccitt_false(header + encrypted)
    return SYNC + header + encrypted + struct.pack(">H", crc)


def parse_request(frame: bytes) -> int | None:
    if len(frame) != 16 or frame[:2] != SYNC:
        return None
    version, frame_type, sequence, payload_len = frame[2:6]
    if (version, frame_type, payload_len) != (VERSION, TYPE_REQUEST, REQUEST_PAYLOAD_LEN):
        return None
    if crc16_ccitt_false(frame[2:14]) != struct.unpack(">H", frame[14:16])[0]:
        return None
    payload = crypt_payload(frame[6:14], False)
    return sequence if payload[0] == CMD_GET_ADC_ONCE else None


@dataclass
class Pattern:
    name: str
    amplitude: float
    frequency_hz: float
    center: tuple[float, float, float, float]
    started_at: float
    sample_count: int = 0

    def values(self, now: float) -> tuple[int, int, int, int]:
        phase = 2.0 * math.pi * self.frequency_hz * max(0.0, now - self.started_at)
        if self.name == "circle":
            x, y = math.cos(phase), math.sin(phase)
        elif self.name == "ellipse":
            x, y = math.cos(phase), 0.55 * math.sin(phase)
        elif self.name == "line":
            x, y = math.sin(phase), 0.0
        else:
            x, y = 1.0, 0.0

        # The difference channels are centered around their pair centers.
        sin_center = (self.center[0] + self.center[1]) / 2.0
        cos_center = (self.center[2] + self.center[3]) / 2.0
        sin_half = x * self.amplitude / 2.0
        cos_half = y * self.amplitude / 2.0
        values = (
            sin_center + sin_half,
            sin_center - sin_half,
            cos_center + cos_half,
            cos_center - cos_half,
        )
        return tuple(max(0, min(4095, int(round(value)))) for value in values)


def parse_centers(value: str) -> tuple[float, float, float, float]:
    parts = [float(part.strip()) for part in value.split(",") if part.strip()]
    if len(parts) == 1:
        parts *= 4
    if len(parts) != 4 or any(part < 0 or part > 4095 for part in parts):
        raise argparse.ArgumentTypeError("center must be one value or four comma-separated values in 0..4095")
    return tuple(parts)  # type: ignore[return-value]


def build_response(sequence: int, sample_id: int, values: tuple[int, int, int, int]) -> bytes:
    payload = struct.pack(
        ">BBHHHHH",
        STATUS_OK,
        CMD_GET_ADC_ONCE,
        sample_id & 0xFFFF,
        values[0],
        values[1],
        values[2],
        values[3],
    ) + b"\x00\x00\x00\x00"
    return make_frame(TYPE_RESPONSE, sequence, payload)


def find_frame(buffer: bytearray) -> bytes | None:
    while True:
        sync_index = buffer.find(SYNC)
        if sync_index < 0:
            if buffer and buffer[-1] == SYNC[0]:
                del buffer[:-1]
            else:
                buffer.clear()
            return None
        if sync_index:
            del buffer[:sync_index]
        if len(buffer) < 6:
            return None
        payload_length = buffer[5]
        frame_length = 2 + 4 + payload_length + 2
        if payload_length == 0 or payload_length > RESPONSE_PAYLOAD_LEN or payload_length % 8:
            del buffer[0]
            continue
        if len(buffer) < frame_length:
            return None
        return bytes(buffer[:frame_length])


def make_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Simulate STC ADC responses for the ESP32 Lissajous LCD")
    parser.add_argument("--port", required=True, help="ESP32 CH343 COM port, for example COM17")
    parser.add_argument("--baudrate", type=int, default=460800)
    parser.add_argument("--rate", type=float, default=0.5, help="waveform frequency in Hz")
    parser.add_argument("--pattern", choices=("circle", "ellipse", "line", "static"), default="circle")
    parser.add_argument("--amplitude", type=float, default=1800.0, help="peak difference between each +/- pair")
    parser.add_argument("--center", type=parse_centers, default=(2048.0,) * 4,
                        help="one center value or sin+,sin-,cos+,cos- values")
    parser.add_argument("--log-hex", action="store_true")
    return parser


def run(args: argparse.Namespace) -> None:
    if args.rate < 0 or args.amplitude < 0 or args.amplitude > 4095:
        raise SystemExit("--rate must be >= 0 and --amplitude must be in 0..4095")
    pattern = Pattern(args.pattern, args.amplitude, args.rate, args.center, time.monotonic())
    try:
        port = serial.Serial(args.port, args.baudrate, timeout=0.1, write_timeout=1.0)
    except serial.SerialException as exc:
        raise SystemExit(f"cannot open {args.port}: {exc}") from exc

    buffer = bytearray()
    print(f"simulating {args.pattern} ADC on {args.port} at {args.baudrate} 8N1", flush=True)
    print("press Ctrl+C to stop", flush=True)
    try:
        while True:
            data = port.read(256)
            if data:
                buffer.extend(data)
            frame = find_frame(buffer)
            if frame is None:
                continue
            del buffer[:len(frame)]
            sequence = parse_request(frame)
            if sequence is None:
                continue
            pattern.sample_count += 1
            values = pattern.values(time.monotonic())
            response = build_response(sequence, pattern.sample_count, values)
            port.write(response)
            if pattern.sample_count == 1 or pattern.sample_count % 25 == 0:
                print(
                    f"replies={pattern.sample_count} seq={sequence} "
                    f"ADC11/10/9/8={values} dSin={values[0] - values[1]} dCos={values[2] - values[3]}",
                    flush=True,
                )
            if args.log_hex:
                print(f"seq={sequence:3d} ADC11/10/9/8={values} TX={response.hex(' ')}", flush=True)
    except KeyboardInterrupt:
        print("stopped", flush=True)
    finally:
        port.close()


def self_test() -> None:
    plain = bytes(range(16))
    assert crypt_payload(crypt_payload(plain, True), False) == plain
    request = make_frame(TYPE_REQUEST, 7, bytes((CMD_GET_ADC_ONCE, 0, 0, 0, 0, 0, 0, 0)))
    assert parse_request(request) == 7
    response = build_response(7, 0x1234, (0x0102, 0x0304, 0x0506, 0x0708))
    assert response[:6] == bytes.fromhex("a5 5a 01 90 07 10")
    assert crc16_ccitt_false(response[2:-2]) == struct.unpack(">H", response[-2:])[0]
    print("protocol self-test passed")


if __name__ == "__main__":
    if "--self-test" in sys.argv:
        self_test()
    else:
        run(make_parser().parse_args())

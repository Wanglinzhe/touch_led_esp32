# ESP32-S3-Touch-LCD-7 STC ADC Debug Console

This is an ESP-IDF host application for the Waveshare ESP32-S3-Touch-LCD-7. It sends an encrypted `GET_ADC_ONCE` request to the STC32 board every 40 ms and displays the ordered raw ADC11, ADC10, ADC9, and ADC8 values returned by the STC firmware. The LCD draws a live Lissajous trace from those four grating 1 Vpp channels.

## Board and UART assumptions

The project targets `esp32s3` and uses UART0 on GPIO43/GPIO44 at 460800 8N1. On the Waveshare board these signals are available on the UART2 header when the UART selection switch is set to UART2. The official board documentation describes UART2 as the direct UART0 header and UART1 as the CH343 USB-UART path.

Use a 3.3 V TTL UART connection:

| ESP32-S3-Touch-LCD-7 UART2 | STC32 |
| --- | --- |
| TX, GPIO43 | RX, P3.0 |
| RX, GPIO44 | TX, P3.1 |
| GND | GND |

The STC P3.0/P3.1 pins must not be connected directly to the board's RS485 A/B lines. RS485 requires a suitable transceiver and a separate half-duplex configuration.

The project routes ESP-IDF logs to the ESP32-S3 USB Serial/JTAG console so log text does not share the binary UART0 stream sent to the STC. Keep the board's UART2 selection for the STC wires.

For firmware download through the board's CH343 UART Type-C connector, follow the board's selector and download procedure, then return the selector to UART2 before running the STC connection. The USB/CAN selector must be in the USB position when the selected USB console is used.

## Protocol

The wire format is fixed and binary. Each frame starts with `A5 5A`, followed by version, type, sequence, payload length, encrypted payload, and a CRC16-CCITT-FALSE. The request payload is one 8-byte XTEA block. The response payload is two XTEA blocks.

XTEA uses 32 rounds, little-endian 32-bit words, and the shared key words below.

```text
13579BDF 2468ACE0 0F1E2D3C 4B5A6978
```

The response payload contains the status, command, sample ID, and four ADC values in this order.

```text
status, command, sample_id, ADC11, ADC10, ADC9, ADC8
```

The four values in one response are taken from the same STC sample snapshot. The sample ID lets the host associate a response with the corresponding capture.

## Touch LCD view

The display uses the official Waveshare RGB panel and GT911 touch wiring from the vendor example. The RGB pixel clock is 16 MHz with a 10-line bounce buffer, matching the vendor example. At 12 MHz this panel does not lock and cycles full-screen green, blue, red, black, and white. RGB565 frame buffers live in octal PSRAM, so the project keeps the vendor cache settings: 64 KB data cache and a 64-byte cache line. DMA is not restarted on every VSYNC. The plot is a square 410x410 pixel area with one shared pixel scale on both axes. It maps `x = ADC11 - ADC10` and `y = ADC9 - ADC8` to the centered plot area. A differential radius of 1500 mV, at the STC 3300 mV / 12-bit reference, reaches the plot edge. A nominal 1 Vpp grating circle therefore uses most of the canvas without clipping, and the y axis is inverted for display coordinates. Accepted responses are queued and drawn in order. A timeout, a non-OK status, a pause, or a queue overflow starts a new stroke instead of drawing a chord across the missing samples.

The right side shows the latest raw values in protocol order, `sample_id`, response status, and response, timeout, CRC, and format-error counters. `Xpp` and `Ypp` are the measured peak-to-peak differential ranges accumulated since `Clear`. `X/Y` is `Xpp / Ypp`, and `ecc` is `sqrt(1 - (minor_axis / major_axis)^2)`. Read these values after at least one complete revolution. A circle should approach `X/Y = 1.000` and `ecc = 0.000`. `Clear` resets the plotted trace and the accumulated metrics. `Pause` stops issuing new ADC requests while allowing the current UART transaction to finish. `Resume` starts polling again.

## Build and flash

Open an ESP-IDF 5.x PowerShell or terminal environment before running these commands.

```text
idf.py set-target esp32s3
idf.py build
idf.py -p COMx flash monitor
```

The managed dependencies contain LVGL 8.4.0 and the GT911 touch driver. The ESP LVGL adapter is kept as a local component under `components/esp_lvgl_adapter` so optional image and font packages are not pulled into this minimal application. The board port source under `components/waveshare_rgb_lcd_port.c` is copied from the checked-in official example and retains the 800x480 RGB GPIO mapping, I2C touch reset, and backlight control. The application uses triple-buffered partial rendering. LVGL copies only dirty plot and label rectangles into a back buffer, then the adapter switches the complete RGB frame at VSYNC to prevent mixed old and new pixels.

During runtime, keep the ESP32 USB Serial/JTAG console open for logs. The binary STC link remains on the board UART2 header at GPIO43/GPIO44. On a board revision using the CH343 UART1 path, select the corresponding board UART switch before connecting the host cable and keep the same 460800 8N1 framing. The USB Serial/JTAG monitor itself remains at its configured console speed.

## PC relay test

When the PC is the temporary bridge, connect the ESP32 CH343 port and a separate 3.3 V TTL adapter to the STC application UART. Run the relay from the repository root with the two COM ports:

```text
D:\Espressif\python_env\idf5.5_py3.11_env\Scripts\python.exe Host\pc_uart_relay.py --esp-port COM17 --stc-port COMx --baudrate 460800 --log-hex
```

The relay forwards binary frames in both directions without changing the encrypted payload. The STC side must reach P3.0/P3.1 and share ground. AiCube-ISP USB-Link COM ports are reserved for programming unless the board exposes the application UART on those pins.

The defaults file selects the ESP32-S3 target, 8 MB flash, and 8 MB octal PSRAM to match the downloaded Waveshare `05_UART_Test` defaults and the N8R8 schematic. Some later board revisions list 16 MB flash. If the module marking or board documentation confirms that revision, change the flash-size Kconfig entry before flashing. No OTA partition or persistent application storage is required by this console application.

## Reference materials

The repository includes the vendor example checkout in the sibling `ESP32-S3-Touch-LCD-7-OfficialExtract` directory. The official schematic is stored in `Doc/ESP32-S3-Touch-LCD-7-Sch.pdf`.

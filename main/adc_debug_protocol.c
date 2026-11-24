#include "adc_debug_protocol.h"

#include <string.h>

static const uint32_t s_xtea_key[4] =
{
    0x13579BDFU,
    0x2468ACE0U,
    0x0F1E2D3CU,
    0x4B5A6978U
};

enum
{
    PARSER_WAIT_SYNC0 = 0,
    PARSER_WAIT_SYNC1,
    PARSER_HEADER,
    PARSER_PAYLOAD,
    PARSER_CRC_HIGH,
    PARSER_CRC_LOW
};

static uint32_t load_le32(const uint8_t *data)
{
    return ((uint32_t)data[0]) |
           ((uint32_t)data[1] << 8) |
           ((uint32_t)data[2] << 16) |
           ((uint32_t)data[3] << 24);
}

static void store_le32(uint8_t *data, uint32_t value)
{
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8);
    data[2] = (uint8_t)(value >> 16);
    data[3] = (uint8_t)(value >> 24);
}

static void xtea_encrypt_block(uint8_t *block)
{
    uint32_t v0 = load_le32(block);
    uint32_t v1 = load_le32(block + 4);
    uint32_t sum = 0;
    uint32_t round;

    for (round = 0; round < 32U; round++)
    {
        v0 += ((((v1 << 4) ^ (v1 >> 5)) + v1) ^
               (sum + s_xtea_key[sum & 3U]));
        sum += 0x9E3779B9U;
        v1 += ((((v0 << 4) ^ (v0 >> 5)) + v0) ^
               (sum + s_xtea_key[(sum >> 11) & 3U]));
    }

    store_le32(block, v0);
    store_le32(block + 4, v1);
}

static void xtea_decrypt_block(uint8_t *block)
{
    uint32_t v0 = load_le32(block);
    uint32_t v1 = load_le32(block + 4);
    uint32_t sum = 0xC6EF3720U;
    uint32_t round;

    for (round = 0; round < 32U; round++)
    {
        v1 -= ((((v0 << 4) ^ (v0 >> 5)) + v0) ^
               (sum + s_xtea_key[(sum >> 11) & 3U]));
        sum -= 0x9E3779B9U;
        v0 -= ((((v1 << 4) ^ (v1 >> 5)) + v1) ^
               (sum + s_xtea_key[sum & 3U]));
    }

    store_le32(block, v0);
    store_le32(block + 4, v1);
}

static void xtea_crypt_payload(uint8_t *payload, uint8_t length, bool encrypt)
{
    uint8_t offset;

    for (offset = 0; offset < length; offset += 8U)
    {
        if (encrypt)
        {
            xtea_encrypt_block(payload + offset);
        }
        else
        {
            xtea_decrypt_block(payload + offset);
        }
    }
}

static uint16_t crc16_ccitt_false(const uint8_t *data, size_t length)
{
    uint16_t crc = 0xFFFFU;
    size_t index;

    for (index = 0; index < length; index++)
    {
        uint8_t bit;

        crc ^= (uint16_t)data[index] << 8;
        for (bit = 0; bit < 8U; bit++)
        {
            if (crc & 0x8000U)
            {
                crc = (uint16_t)((crc << 1) ^ 0x1021U);
            }
            else
            {
                crc <<= 1;
            }
        }
    }

    return crc;
}

static bool encode_payload_frame(uint8_t type,
                                 uint8_t sequence,
                                 const uint8_t *plain_payload,
                                 uint8_t payload_length,
                                 uint8_t *frame,
                                 size_t capacity,
                                 size_t *frame_length)
{
    uint16_t crc;
    size_t total_length;

    if ((payload_length == 0U) ||
        ((payload_length & 7U) != 0U) ||
        (payload_length > ADC_DEBUG_MAX_PAYLOAD_LEN))
    {
        return false;
    }

    total_length = 2U + 4U + payload_length + 2U;
    if ((frame == NULL) || (frame_length == NULL) || (capacity < total_length))
    {
        return false;
    }

    frame[0] = ADC_DEBUG_SYNC0;
    frame[1] = ADC_DEBUG_SYNC1;
    frame[2] = ADC_DEBUG_VERSION;
    frame[3] = type;
    frame[4] = sequence;
    frame[5] = payload_length;
    memcpy(frame + 6, plain_payload, payload_length);
    xtea_crypt_payload(frame + 6, payload_length, true);

    crc = crc16_ccitt_false(frame + 2, 4U + payload_length);
    frame[6U + payload_length] = (uint8_t)(crc >> 8);
    frame[7U + payload_length] = (uint8_t)crc;
    *frame_length = total_length;
    return true;
}

void adc_debug_parser_init(adc_debug_parser_t *parser)
{
    if (parser != NULL)
    {
        memset(parser, 0, sizeof(*parser));
        parser->state = PARSER_WAIT_SYNC0;
    }
}

bool adc_debug_encode_get_adc(uint8_t sequence,
                              uint8_t *frame,
                              size_t capacity,
                              size_t *frame_length)
{
    uint8_t payload[ADC_DEBUG_REQUEST_PAYLOAD_LEN] = {0};

    payload[0] = ADC_DEBUG_CMD_GET_ADC_ONCE;
    return encode_payload_frame(ADC_DEBUG_TYPE_REQUEST,
                                sequence,
                                payload,
                                ADC_DEBUG_REQUEST_PAYLOAD_LEN,
                                frame,
                                capacity,
                                frame_length);
}

static void parser_reset(adc_debug_parser_t *parser)
{
    parser->state = PARSER_WAIT_SYNC0;
    parser->header_index = 0;
    parser->payload_length = 0;
    parser->payload_index = 0;
    parser->crc_high = 0;
}

static uint16_t parser_received_crc(const adc_debug_parser_t *parser,
                                    uint8_t crc_low)
{
    return (uint16_t)(((uint16_t)parser->crc_high << 8) | crc_low);
}

bool adc_debug_parser_feed(adc_debug_parser_t *parser,
                           uint8_t byte,
                           adc_debug_response_t *response)
{
    uint16_t expected_crc;
    uint16_t received_crc;

    if ((parser == NULL) || (response == NULL))
    {
        return false;
    }

    switch (parser->state)
    {
    case PARSER_WAIT_SYNC0:
        if (byte == ADC_DEBUG_SYNC0)
        {
            parser->state = PARSER_WAIT_SYNC1;
        }
        break;

    case PARSER_WAIT_SYNC1:
        if (byte == ADC_DEBUG_SYNC1)
        {
            parser->state = PARSER_HEADER;
            parser->header_index = 0;
        }
        else if (byte == ADC_DEBUG_SYNC0)
        {
            parser->state = PARSER_WAIT_SYNC1;
        }
        else
        {
            parser->state = PARSER_WAIT_SYNC0;
        }
        break;

    case PARSER_HEADER:
        parser->header[parser->header_index++] = byte;
        if (parser->header_index >= 4U)
        {
            if ((parser->header[0] != ADC_DEBUG_VERSION) ||
                (parser->header[1] != ADC_DEBUG_TYPE_RESPONSE) ||
                (parser->header[3] != ADC_DEBUG_RESPONSE_PAYLOAD_LEN))
            {
                parser->format_error_count++;
                parser_reset(parser);
                break;
            }

            parser->payload_length = parser->header[3];
            parser->payload_index = 0;
            parser->state = PARSER_PAYLOAD;
        }
        break;

    case PARSER_PAYLOAD:
        parser->payload[parser->payload_index++] = byte;
        if (parser->payload_index >= parser->payload_length)
        {
            parser->state = PARSER_CRC_HIGH;
        }
        break;

    case PARSER_CRC_HIGH:
        parser->crc_high = byte;
        parser->state = PARSER_CRC_LOW;
        break;

    case PARSER_CRC_LOW:
        /* Compute over the exact contiguous header+ciphertext bytes. */
        {
            uint8_t crc_data[4U + ADC_DEBUG_RESPONSE_PAYLOAD_LEN];
            memcpy(crc_data, parser->header, 4U);
            memcpy(crc_data + 4, parser->payload, parser->payload_length);
            expected_crc = crc16_ccitt_false(crc_data,
                                             4U + parser->payload_length);
        }
        received_crc = parser_received_crc(parser, byte);
        if (expected_crc != received_crc)
        {
            parser->crc_error_count++;
            parser_reset(parser);
            break;
        }

        xtea_crypt_payload(parser->payload, parser->payload_length, false);
        response->sequence = parser->header[2];
        response->status = parser->payload[0];
        response->command = parser->payload[1];
        response->sample_id = (uint16_t)(((uint16_t)parser->payload[2] << 8) |
                                         parser->payload[3]);
        response->adc11 = (uint16_t)(((uint16_t)parser->payload[4] << 8) |
                                     parser->payload[5]);
        response->adc10 = (uint16_t)(((uint16_t)parser->payload[6] << 8) |
                                     parser->payload[7]);
        response->adc9 = (uint16_t)(((uint16_t)parser->payload[8] << 8) |
                                    parser->payload[9]);
        response->adc8 = (uint16_t)(((uint16_t)parser->payload[10] << 8) |
                                    parser->payload[11]);
        parser->frame_count++;
        parser_reset(parser);
        return true;

    default:
        parser_reset(parser);
        break;
    }

    return false;
}

bool adc_debug_protocol_self_test(void)
{
    uint8_t plain[ADC_DEBUG_RESPONSE_PAYLOAD_LEN];
    uint8_t work[ADC_DEBUG_RESPONSE_PAYLOAD_LEN];
    uint8_t frame[ADC_DEBUG_MAX_FRAME_LEN];
    uint8_t request_frame[ADC_DEBUG_MAX_FRAME_LEN];
    size_t frame_length = 0;
    size_t request_length = 0;
    adc_debug_parser_t parser;
    adc_debug_response_t response;
    size_t index;

    for (index = 0; index < sizeof(plain); index++)
    {
        plain[index] = (uint8_t)index;
    }
    memcpy(work, plain, sizeof(work));
    xtea_crypt_payload(work, sizeof(work), true);
    xtea_crypt_payload(work, sizeof(work), false);
    if (memcmp(work, plain, sizeof(work)) != 0)
    {
        return false;
    }

    if (!adc_debug_encode_get_adc(7U,
                                  request_frame,
                                  sizeof(request_frame),
                                  &request_length) ||
        (request_length != 16U) ||
        (request_frame[0] != ADC_DEBUG_SYNC0) ||
        (request_frame[1] != ADC_DEBUG_SYNC1))
    {
        return false;
    }

    plain[0] = ADC_DEBUG_STATUS_OK;
    plain[1] = ADC_DEBUG_CMD_GET_ADC_ONCE;
    plain[2] = 0x12;
    plain[3] = 0x34;
    plain[4] = 0x01;
    plain[5] = 0x02;
    plain[6] = 0x03;
    plain[7] = 0x04;
    plain[8] = 0x05;
    plain[9] = 0x06;
    plain[10] = 0x07;
    plain[11] = 0x08;
    plain[12] = 0;
    plain[13] = 0;
    plain[14] = 0;
    plain[15] = 0;

    if (!encode_payload_frame(ADC_DEBUG_TYPE_RESPONSE,
                              7U,
                              plain,
                              ADC_DEBUG_RESPONSE_PAYLOAD_LEN,
                              frame,
                              sizeof(frame),
                              &frame_length))
    {
        return false;
    }

    adc_debug_parser_init(&parser);
    memset(&response, 0, sizeof(response));
    for (index = 0; index < frame_length; index++)
    {
        if (adc_debug_parser_feed(&parser, frame[index], &response))
        {
            break;
        }
    }

    return (index < frame_length) &&
           (response.sequence == 7U) &&
           (response.sample_id == 0x1234U) &&
           (response.adc11 == 0x0102U) &&
           (response.adc10 == 0x0304U) &&
           (response.adc9 == 0x0506U) &&
           (response.adc8 == 0x0708U);
}

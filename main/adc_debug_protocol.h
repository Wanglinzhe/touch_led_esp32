#ifndef ADC_DEBUG_PROTOCOL_H
#define ADC_DEBUG_PROTOCOL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define ADC_DEBUG_SYNC0                 0xA5U
#define ADC_DEBUG_SYNC1                 0x5AU
#define ADC_DEBUG_VERSION              0x01U
#define ADC_DEBUG_TYPE_REQUEST         0x10U
#define ADC_DEBUG_TYPE_RESPONSE        0x90U

#define ADC_DEBUG_CMD_GET_ADC_ONCE     0x01U

#define ADC_DEBUG_STATUS_OK             0x00U
#define ADC_DEBUG_STATUS_UNKNOWN_CMD    0x01U
#define ADC_DEBUG_STATUS_NO_SAMPLE      0x02U

#define ADC_DEBUG_REQUEST_PAYLOAD_LEN   8U
#define ADC_DEBUG_RESPONSE_PAYLOAD_LEN 16U
#define ADC_DEBUG_MAX_PAYLOAD_LEN       ADC_DEBUG_RESPONSE_PAYLOAD_LEN
#define ADC_DEBUG_MAX_FRAME_LEN         (2U + 4U + ADC_DEBUG_MAX_PAYLOAD_LEN + 2U)

typedef struct
{
    uint8_t sequence;
    uint8_t status;
    uint8_t command;
    uint16_t sample_id;
    uint16_t adc11;
    uint16_t adc10;
    uint16_t adc9;
    uint16_t adc8;
} adc_debug_response_t;

typedef struct
{
    uint8_t state;
    uint8_t header[4];
    uint8_t header_index;
    uint8_t payload[ADC_DEBUG_RESPONSE_PAYLOAD_LEN];
    uint8_t payload_length;
    uint8_t payload_index;
    uint8_t crc_high;
    uint32_t frame_count;
    uint32_t crc_error_count;
    uint32_t format_error_count;
} adc_debug_parser_t;

void adc_debug_parser_init(adc_debug_parser_t *parser);
bool adc_debug_encode_get_adc(uint8_t sequence,
                              uint8_t *frame,
                              size_t capacity,
                              size_t *frame_length);
bool adc_debug_parser_feed(adc_debug_parser_t *parser,
                           uint8_t byte,
                           adc_debug_response_t *response);
bool adc_debug_protocol_self_test(void);

#endif

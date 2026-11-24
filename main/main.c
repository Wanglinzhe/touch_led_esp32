#include "adc_debug_protocol.h"

#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "lissajous_ui.h"
#include "waveshare_rgb_lcd_port.h"

#define STC_UART_PORT             UART_NUM_0
#define STC_UART_TX_GPIO          43
#define STC_UART_RX_GPIO          44
#define STC_UART_BAUD_RATE        460800
#define STC_UART_RX_BUFFER_SIZE   2048
#define STC_UART_TX_BUFFER_SIZE   512

#define ADC_REQUEST_PERIOD_MS     40U
#define ADC_RESPONSE_TIMEOUT_MS   500U
#define ADC_UART_READ_TIMEOUT_MS  20U
#define ADC_TASK_STACK_SIZE       4096U
#define ADC_TASK_PRIORITY         10U

static const char *TAG = "stc_adc_host";

static void stc_uart_init(void)
{
    const uart_config_t config =
    {
        .baud_rate = STC_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT
    };

    ESP_ERROR_CHECK(uart_driver_install(STC_UART_PORT,
                                         STC_UART_RX_BUFFER_SIZE,
                                         STC_UART_TX_BUFFER_SIZE,
                                         0,
                                         NULL,
                                         0));
    ESP_ERROR_CHECK(uart_param_config(STC_UART_PORT, &config));
    ESP_ERROR_CHECK(uart_set_pin(STC_UART_PORT,
                                 STC_UART_TX_GPIO,
                                 STC_UART_RX_GPIO,
                                 UART_PIN_NO_CHANGE,
                                 UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_flush_input(STC_UART_PORT));
}

static bool send_adc_request(uint8_t sequence)
{
    uint8_t frame[ADC_DEBUG_MAX_FRAME_LEN];
    size_t frame_length = 0;
    int written;

    if (!adc_debug_encode_get_adc(sequence,
                                  frame,
                                  sizeof(frame),
                                  &frame_length))
    {
        ESP_LOGE(TAG, "failed to encode request");
        return false;
    }

    written = uart_write_bytes(STC_UART_PORT,
                               (const char *)frame,
                               frame_length);
    if (written != (int)frame_length)
    {
        ESP_LOGE(TAG, "UART write failed, expected=%u written=%d",
                 (unsigned)frame_length,
                 written);
        return false;
    }
    return true;
}

static lissajous_ui_counters_t make_ui_counters(const adc_debug_parser_t *parser,
                                                uint32_t response_count,
                                                uint32_t timeout_count,
                                                bool request_pending)
{
    lissajous_ui_counters_t counters = {
        .response_count = response_count,
        .timeout_count = timeout_count,
        .crc_error_count = parser->crc_error_count,
        .format_error_count = parser->format_error_count,
        .request_pending = request_pending,
    };
    return counters;
}

static void adc_protocol_task(void *arg)
{
    adc_debug_parser_t parser;
    adc_debug_response_t response;
    uint8_t read_buffer[128];
    uint8_t next_sequence = 0;
    uint8_t pending_sequence = 0;
    bool request_pending = false;
    uint32_t timeout_count = 0;
    uint32_t response_count = 0;
    uint32_t reported_crc_errors = 0;
    TickType_t next_request_tick = 0;
    TickType_t response_deadline_tick = 0;

    (void)arg;
    adc_debug_parser_init(&parser);
    next_request_tick = xTaskGetTickCount();

    while (true)
    {
        TickType_t now = xTaskGetTickCount();
        int length;
        int index;
        bool response_submitted = false;

        if (!request_pending &&
            !lissajous_ui_is_paused() &&
            (now >= next_request_tick))
        {
            pending_sequence = next_sequence++;
            if (send_adc_request(pending_sequence))
            {
                request_pending = true;
                response_deadline_tick = now +
                    pdMS_TO_TICKS(ADC_RESPONSE_TIMEOUT_MS);
            }
            next_request_tick = now + pdMS_TO_TICKS(ADC_REQUEST_PERIOD_MS);
        }

        length = uart_read_bytes(STC_UART_PORT,
                                 read_buffer,
                                 sizeof(read_buffer),
                                 pdMS_TO_TICKS(ADC_UART_READ_TIMEOUT_MS));
        for (index = 0; index < length; index++)
        {
            if (!adc_debug_parser_feed(&parser, read_buffer[index], &response))
            {
                continue;
            }

            if (!request_pending || (response.sequence != pending_sequence))
            {
                ESP_LOGW(TAG,
                         "discarding out-of-sequence response seq=%u pending=%u",
                         (unsigned)response.sequence,
                         (unsigned)pending_sequence);
                continue;
            }

            request_pending = false;
            response_count++;
            {
                const lissajous_ui_counters_t counters =
                    make_ui_counters(&parser, response_count, timeout_count, false);
                lissajous_ui_submit_response(&response, &counters);
                response_submitted = true;
            }
            if (response.status == ADC_DEBUG_STATUS_OK)
            {
                ESP_LOGI(TAG,
                         "ADC sample=%" PRIu16 " seq=%u ADC11=%" PRIu16
                         " ADC10=%" PRIu16 " ADC9=%" PRIu16 " ADC8=%" PRIu16,
                         response.sample_id,
                         (unsigned)response.sequence,
                         response.adc11,
                         response.adc10,
                         response.adc9,
                         response.adc8);
            }
            else
            {
                ESP_LOGW(TAG,
                         "STC command status=%u command=0x%02X sample=%" PRIu16,
                         (unsigned)response.status,
                         (unsigned)response.command,
                         response.sample_id);
            }
        }

        now = xTaskGetTickCount();
        if (request_pending && (now >= response_deadline_tick))
        {
            request_pending = false;
            timeout_count++;
            ESP_LOGW(TAG,
                     "ADC response timeout seq=%u timeouts=%" PRIu32,
                     (unsigned)pending_sequence,
                     timeout_count);
            {
                const lissajous_ui_counters_t counters =
                    make_ui_counters(&parser, response_count, timeout_count, false);
                lissajous_ui_submit_gap(&counters);
                response_submitted = true;
            }
        }

        if (parser.crc_error_count >= (reported_crc_errors + 10U))
        {
            reported_crc_errors = parser.crc_error_count;
            ESP_LOGW(TAG,
                     "protocol CRC errors=%" PRIu32 " format errors=%" PRIu32
                     " frames=%" PRIu32 " responses=%" PRIu32,
                     parser.crc_error_count,
                     parser.format_error_count,
                     parser.frame_count,
                     response_count);
        }

        if (!response_submitted)
        {
            const lissajous_ui_counters_t counters =
                make_ui_counters(&parser, response_count, timeout_count, request_pending);
            lissajous_ui_submit_counters(&counters);
        }

        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

void app_main(void)
{
    if (!adc_debug_protocol_self_test())
    {
        ESP_LOGE(TAG, "protocol self-test failed");
        return;
    }

    esp_lcd_panel_handle_t panel_handle = NULL;
    esp_lcd_touch_handle_t touch_handle = NULL;
    ESP_ERROR_CHECK(waveshare_esp32_s3_rgb_lcd_init(
        ESP_LV_ADAPTER_TEAR_AVOID_MODE_TRIPLE_PARTIAL,
        ESP_LV_ADAPTER_ROTATE_0,
        &panel_handle,
        &touch_handle));
    ESP_ERROR_CHECK(waveshare_rgb_lcd_backlight_on());
    ESP_ERROR_CHECK(lissajous_ui_init(panel_handle, touch_handle));

#if CONFIG_STC_DIAGNOSTIC_UART
    ESP_LOGW(TAG,
             "diagnostic UART mode: binary STC task disabled, "
             "RGB/LVGL logs are routed to UART0");
#else
    stc_uart_init();
    ESP_LOGI(TAG,
             "STC ADC host ready UART%u TX=GPIO%d RX=GPIO%d baud=%d",
             (unsigned)STC_UART_PORT,
             STC_UART_TX_GPIO,
             STC_UART_RX_GPIO,
             STC_UART_BAUD_RATE);
    ESP_LOGI(TAG,
             "STC wiring: ESP TX GPIO%d -> STC RX P3.0, "
             "ESP RX GPIO%d <- STC TX P3.1, common GND",
             STC_UART_TX_GPIO,
             STC_UART_RX_GPIO);

    xTaskCreate(adc_protocol_task,
                "stc_adc_protocol",
                ADC_TASK_STACK_SIZE,
                NULL,
                ADC_TASK_PRIORITY,
                NULL);
#endif
}

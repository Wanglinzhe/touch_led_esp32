#ifndef LISSAJOUS_UI_H
#define LISSAJOUS_UI_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_touch.h"

#include "adc_debug_protocol.h"

typedef struct
{
    uint32_t response_count;
    uint32_t timeout_count;
    uint32_t crc_error_count;
    uint32_t format_error_count;
    bool request_pending;
} lissajous_ui_counters_t;

esp_err_t lissajous_ui_init(esp_lcd_panel_handle_t panel_handle,
                            esp_lcd_touch_handle_t touch_handle);
void lissajous_ui_submit_response(const adc_debug_response_t *response,
                                  const lissajous_ui_counters_t *counters);
void lissajous_ui_submit_counters(const lissajous_ui_counters_t *counters);
void lissajous_ui_submit_gap(const lissajous_ui_counters_t *counters);
bool lissajous_ui_is_paused(void);

#endif

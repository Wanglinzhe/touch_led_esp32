#include "lissajous_ui.h"

#include <assert.h>
#include <inttypes.h>
#include <stdio.h>

#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_lv_adapter.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "lvgl.h"

#include "waveshare_rgb_lcd_port.h"

#define UI_WIDTH                    800
#define UI_HEIGHT                   480
#define PLOT_X                      12
#define PLOT_Y                      48
#define PLOT_WIDTH                  410
#define PLOT_HEIGHT                 410
#define PLOT_CENTER_X               (PLOT_WIDTH / 2)
#define PLOT_CENTER_Y               (PLOT_HEIGHT / 2)
/*
 * STC ADC reference is 3300 mV over 4095 counts. A grating 1 Vpp sin/cos pair
 * has about a 1000 mV Lissajous radius. The plot edge is 1500 mV on both axes
 * so that circle stays round, large, and unclipped.
 */
#define ADC_FULL_SCALE_COUNTS       4095
#define ADC_REF_MV                  3300
#define LISSAJOUS_EDGE_RADIUS_MV    1500
#define ADC_DIFFERENCE_FULL_SCALE   (((ADC_FULL_SCALE_COUNTS * LISSAJOUS_EDGE_RADIUS_MV) + \
                                      (ADC_REF_MV / 2)) / ADC_REF_MV)
#define UI_REFRESH_PERIOD_MS        20U
#define UI_EVENT_QUEUE_LENGTH       48U
#define UI_COUNTER_QUEUE_RESERVE    8U

static const char *TAG = "lissajous_ui";

typedef struct
{
    bool has_response;
    bool break_trace;
    adc_debug_response_t response;
    lissajous_ui_counters_t counters;
} ui_event_t;

static QueueHandle_t s_event_queue;
static volatile bool s_paused = false;
static bool s_trace_gap_pending = false;

static lv_obj_t *s_canvas;
static lv_obj_t *s_raw_label;
static lv_obj_t *s_sample_label;
static lv_obj_t *s_status_label;
static lv_obj_t *s_counter_label;
static lv_obj_t *s_metrics_label;
static lv_obj_t *s_pause_button_label;
static lv_display_t *s_display;
static uint8_t *s_canvas_buffer;
static bool s_has_previous_point;
static lv_point_t s_previous_point;

typedef struct
{
    bool valid;
    int32_t min_x;
    int32_t max_x;
    int32_t min_y;
    int32_t max_y;
} shape_metrics_t;

static shape_metrics_t s_shape_metrics;

static void reset_shape_metrics(void)
{
    s_shape_metrics.valid = false;
    s_shape_metrics.min_x = 0;
    s_shape_metrics.max_x = 0;
    s_shape_metrics.min_y = 0;
    s_shape_metrics.max_y = 0;
}

static void update_shape_metrics(const adc_debug_response_t *response)
{
    const int32_t x_difference =
        (int32_t)response->adc11 - (int32_t)response->adc10;
    const int32_t y_difference =
        (int32_t)response->adc9 - (int32_t)response->adc8;

    if (!s_shape_metrics.valid) {
        s_shape_metrics.min_x = x_difference;
        s_shape_metrics.max_x = x_difference;
        s_shape_metrics.min_y = y_difference;
        s_shape_metrics.max_y = y_difference;
        s_shape_metrics.valid = true;
        return;
    }

    if (x_difference < s_shape_metrics.min_x) {
        s_shape_metrics.min_x = x_difference;
    }
    if (x_difference > s_shape_metrics.max_x) {
        s_shape_metrics.max_x = x_difference;
    }
    if (y_difference < s_shape_metrics.min_y) {
        s_shape_metrics.min_y = y_difference;
    }
    if (y_difference > s_shape_metrics.max_y) {
        s_shape_metrics.max_y = y_difference;
    }
}

static uint32_t integer_sqrt_u64(uint64_t value)
{
    uint64_t bit = 1ULL << 62;
    uint64_t result = 0;

    while (bit > value) {
        bit >>= 2;
    }
    while (bit != 0U) {
        if (value >= result + bit) {
            value -= result + bit;
            result = (result >> 1) + bit;
        } else {
            result >>= 1;
        }
        bit >>= 2;
    }
    return (uint32_t)result;
}

static void update_metrics_label(void)
{
    uint32_t x_peak_to_peak;
    uint32_t y_peak_to_peak;
    uint32_t major_axis;
    uint32_t minor_axis;
    uint32_t ratio_scaled;
    uint32_t eccentricity_scaled;
    uint64_t major_squared;
    uint64_t minor_squared;
    uint64_t eccentricity_numerator;

    if (s_metrics_label == NULL) {
        return;
    }
    if (!s_shape_metrics.valid) {
        lv_label_set_text(s_metrics_label,
                          "Xpp     ----\nYpp     ----\nX/Y      --\necc      --");
        return;
    }

    x_peak_to_peak = (uint32_t)(s_shape_metrics.max_x - s_shape_metrics.min_x);
    y_peak_to_peak = (uint32_t)(s_shape_metrics.max_y - s_shape_metrics.min_y);
    if ((x_peak_to_peak == 0U) || (y_peak_to_peak == 0U)) {
        lv_label_set_text_fmt(s_metrics_label,
                              "Xpp %6" PRIu32 "\nYpp %6" PRIu32
                              "\nX/Y      --\necc      --",
                              x_peak_to_peak, y_peak_to_peak);
        return;
    }

    ratio_scaled = (uint32_t)(((uint64_t)x_peak_to_peak * 1000U +
                               (y_peak_to_peak / 2U)) / y_peak_to_peak);
    major_axis = (x_peak_to_peak > y_peak_to_peak) ?
                 x_peak_to_peak : y_peak_to_peak;
    minor_axis = (x_peak_to_peak > y_peak_to_peak) ?
                 y_peak_to_peak : x_peak_to_peak;
    major_squared = (uint64_t)major_axis * major_axis;
    minor_squared = (uint64_t)minor_axis * minor_axis;
    eccentricity_numerator = (major_squared - minor_squared) * 1000000U;
    eccentricity_scaled = integer_sqrt_u64(eccentricity_numerator /
                                           major_squared);

    lv_label_set_text_fmt(s_metrics_label,
                          "Xpp %6" PRIu32 "\nYpp %6" PRIu32
                          "\nX/Y %3" PRIu32 ".%03" PRIu32
                          "\necc %3" PRIu32 ".%03" PRIu32,
                          x_peak_to_peak, y_peak_to_peak,
                          ratio_scaled / 1000U, ratio_scaled % 1000U,
                          eccentricity_scaled / 1000U,
                          eccentricity_scaled % 1000U);
}

static void draw_line(lv_color_t color, uint8_t width,
                      const lv_point_t *points, uint32_t count)
{
    lv_draw_line_dsc_t line_dsc;
    lv_draw_line_dsc_init(&line_dsc);
    line_dsc.color = color;
    line_dsc.width = width;
    line_dsc.round_start = 1;
    line_dsc.round_end = 1;
    lv_canvas_draw_line(s_canvas, points, count, &line_dsc);
}

static void draw_plot_background(void)
{
    lv_canvas_fill_bg(s_canvas, lv_color_hex(0x08111F), LV_OPA_COVER);

    lv_point_t horizontal[] = {{0, PLOT_CENTER_Y}, {PLOT_WIDTH - 1, PLOT_CENTER_Y}};
    lv_point_t vertical[] = {{PLOT_CENTER_X, 0}, {PLOT_CENTER_X, PLOT_HEIGHT - 1}};
    draw_line(lv_color_hex(0x53657A), 1, horizontal, 2);
    draw_line(lv_color_hex(0x53657A), 1, vertical, 2);

    lv_point_t border_top[] = {{0, 0}, {PLOT_WIDTH - 1, 0}};
    lv_point_t border_bottom[] = {{0, PLOT_HEIGHT - 1}, {PLOT_WIDTH - 1, PLOT_HEIGHT - 1}};
    lv_point_t border_left[] = {{0, 0}, {0, PLOT_HEIGHT - 1}};
    lv_point_t border_right[] = {{PLOT_WIDTH - 1, 0}, {PLOT_WIDTH - 1, PLOT_HEIGHT - 1}};
    draw_line(lv_color_hex(0x20334B), 2, border_top, 2);
    draw_line(lv_color_hex(0x20334B), 2, border_bottom, 2);
    draw_line(lv_color_hex(0x20334B), 2, border_left, 2);
    draw_line(lv_color_hex(0x20334B), 2, border_right, 2);
    s_has_previous_point = false;
}

static void clear_plot(void)
{
    draw_plot_background();
    reset_shape_metrics();
    update_metrics_label();
}

static lv_point_t map_sample(const adc_debug_response_t *response)
{
    const int32_t x_difference = (int32_t)response->adc11 - (int32_t)response->adc10;
    const int32_t y_difference = (int32_t)response->adc9 - (int32_t)response->adc8;
    const int32_t radius =
        ((PLOT_WIDTH < PLOT_HEIGHT ? PLOT_WIDTH : PLOT_HEIGHT) / 2) - 12;
    lv_point_t point;

    point.x = (lv_coord_t)(PLOT_CENTER_X +
        (x_difference * radius) / ADC_DIFFERENCE_FULL_SCALE);
    point.y = (lv_coord_t)(PLOT_CENTER_Y -
        (y_difference * radius) / ADC_DIFFERENCE_FULL_SCALE);

    if (point.x < 1) {
        point.x = 1;
    } else if (point.x >= PLOT_WIDTH - 1) {
        point.x = PLOT_WIDTH - 2;
    }
    if (point.y < 1) {
        point.y = 1;
    } else if (point.y >= PLOT_HEIGHT - 1) {
        point.y = PLOT_HEIGHT - 2;
    }
    return point;
}

static void draw_sample(const adc_debug_response_t *response)
{
    const lv_point_t point = map_sample(response);
    if (s_has_previous_point) {
        lv_point_t segment[] = {s_previous_point, point};
        draw_line(lv_color_hex(0x28D7FE), 2, segment, 2);
    }
    s_previous_point = point;
    s_has_previous_point = true;
}

static void update_labels(const ui_event_t *event)
{
    const adc_debug_response_t *response = &event->response;
    const lissajous_ui_counters_t *counters = &event->counters;
    const char *state = counters->request_pending ? "REQUESTING" : "ONLINE";

    lv_label_set_text_fmt(s_raw_label,
                          "ADC11  %4" PRIu16 "\nADC10  %4" PRIu16
                          "\nADC9   %4" PRIu16 "\nADC8   %4" PRIu16,
                          response->adc11, response->adc10,
                          response->adc9, response->adc8);
    lv_label_set_text_fmt(s_sample_label, "sample_id  %5" PRIu16, response->sample_id);
    lv_label_set_text_fmt(s_status_label, "%s   status=%u",
                          state, (unsigned)response->status);
    lv_label_set_text_fmt(s_counter_label,
                          "responses %" PRIu32 "\ntimeouts  %" PRIu32
                          "\nCRC       %" PRIu32 "\nformat    %" PRIu32,
                          counters->response_count,
                          counters->timeout_count,
                          counters->crc_error_count,
                          counters->format_error_count);
}

static void update_counter_labels(const lissajous_ui_counters_t *counters)
{
    lv_label_set_text_fmt(s_status_label, "%s",
                          counters->request_pending ? "REQUESTING" : "ONLINE");
    lv_label_set_text_fmt(s_counter_label,
                          "responses %" PRIu32 "\ntimeouts  %" PRIu32
                          "\nCRC       %" PRIu32 "\nformat    %" PRIu32,
                          counters->response_count,
                          counters->timeout_count,
                          counters->crc_error_count,
                          counters->format_error_count);
}

static void button_event_cb(lv_event_t *event)
{
    const uintptr_t action = (uintptr_t)lv_event_get_user_data(event);
    if (lv_event_get_code(event) != LV_EVENT_CLICKED) {
        return;
    }

    if (action == 1U) {
        clear_plot();
    } else if (action == 2U) {
        s_paused = !s_paused;
        s_has_previous_point = false;
        lv_label_set_text(s_pause_button_label, s_paused ? "Resume" : "Pause");
        lv_label_set_text(s_status_label, s_paused ? "PAUSED" : "ONLINE");
    }
}

static lv_obj_t *create_button(lv_obj_t *parent, const char *text,
                               lv_coord_t x, uintptr_t action)
{
    lv_obj_t *button = lv_btn_create(parent);
    lv_obj_set_size(button, 88, 42);
    lv_obj_set_pos(button, x, 424);
    lv_obj_set_style_bg_color(button, lv_color_hex(0x1B6B8A), LV_PART_MAIN);
    lv_obj_add_event_cb(button, button_event_cb, LV_EVENT_CLICKED, (void *)action);

    lv_obj_t *label = lv_label_create(button);
    lv_label_set_text(label, text);
    lv_obj_center(label);
    if (action == 2U) {
        s_pause_button_label = label;
    }
    return button;
}

static void create_ui(void)
{
    lv_obj_t *screen = lv_scr_act();
    lv_obj_set_style_bg_color(screen, lv_color_hex(0x02060C), LV_PART_MAIN);

    lv_obj_t *title = lv_label_create(screen);
    lv_label_set_text(title, "STC ADC  Lissajous 1:1");
    lv_obj_set_style_text_color(title, lv_color_hex(0xE9F5FF), LV_PART_MAIN);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_20, LV_PART_MAIN);
    lv_obj_set_pos(title, 16, 10);

    s_canvas = lv_canvas_create(screen);
    s_canvas_buffer = heap_caps_calloc(1,
                                       PLOT_WIDTH * PLOT_HEIGHT * sizeof(lv_color_t),
                                       MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (s_canvas_buffer == NULL) {
        s_canvas_buffer = heap_caps_calloc(1,
                                           PLOT_WIDTH * PLOT_HEIGHT * sizeof(lv_color_t),
                                           MALLOC_CAP_8BIT);
    }
    assert(s_canvas_buffer != NULL);
    lv_canvas_set_buffer(s_canvas, s_canvas_buffer, PLOT_WIDTH, PLOT_HEIGHT,
                         LV_IMG_CF_TRUE_COLOR);
    lv_obj_set_pos(s_canvas, PLOT_X, PLOT_Y);
    clear_plot();

    lv_obj_t *raw_title = lv_label_create(screen);
    lv_label_set_text(raw_title, "RAW ADC");
    lv_obj_set_style_text_color(raw_title, lv_color_hex(0x8DE8FF), LV_PART_MAIN);
    lv_obj_set_pos(raw_title, 450, 58);

    s_raw_label = lv_label_create(screen);
    lv_label_set_text(s_raw_label, "ADC11  ----\nADC10  ----\nADC9   ----\nADC8   ----");
    lv_obj_set_style_text_color(s_raw_label, lv_color_hex(0xE9F5FF), LV_PART_MAIN);
    lv_obj_set_pos(s_raw_label, 450, 84);

    s_sample_label = lv_label_create(screen);
    lv_label_set_text(s_sample_label, "sample_id  -----");
    lv_obj_set_style_text_color(s_sample_label, lv_color_hex(0xB6C8D8), LV_PART_MAIN);
    lv_obj_set_pos(s_sample_label, 450, 180);

    s_status_label = lv_label_create(screen);
    lv_label_set_text(s_status_label, "WAITING");
    lv_obj_set_style_text_color(s_status_label, lv_color_hex(0xFFC857), LV_PART_MAIN);
    lv_obj_set_pos(s_status_label, 450, 214);

    s_counter_label = lv_label_create(screen);
    lv_label_set_text(s_counter_label, "responses 0\ntimeouts  0\nCRC       0\nformat    0");
    lv_obj_set_style_text_color(s_counter_label, lv_color_hex(0xB6C8D8), LV_PART_MAIN);
    lv_obj_set_pos(s_counter_label, 450, 248);

    s_metrics_label = lv_label_create(screen);
    lv_label_set_text(s_metrics_label,
                      "Xpp     ----\nYpp     ----\nX/Y      --\necc      --");
    lv_obj_set_style_text_color(s_metrics_label, lv_color_hex(0xB6C8D8), LV_PART_MAIN);
    lv_obj_set_pos(s_metrics_label, 450, 316);

    create_button(screen, "Clear", 450, 1U);
    create_button(screen, "Pause", 550, 2U);
}

static void apply_ui_event(const ui_event_t *event)
{
    if (event->break_trace) {
        s_has_previous_point = false;
    }

    if (event->has_response) {
        if (event->response.status == ADC_DEBUG_STATUS_OK) {
            draw_sample(&event->response);
            update_shape_metrics(&event->response);
            update_metrics_label();
        } else {
            s_has_previous_point = false;
        }
        update_labels(event);
        return;
    }

    update_counter_labels(&event->counters);
}

static void ui_timer_cb(lv_timer_t *timer)
{
    ui_event_t event;

    (void)timer;
    while (xQueueReceive(s_event_queue, &event, 0) == pdTRUE) {
        apply_ui_event(&event);
    }
}

esp_err_t lissajous_ui_init(esp_lcd_panel_handle_t panel_handle,
                            esp_lcd_touch_handle_t touch_handle)
{
    if (panel_handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    s_event_queue = xQueueCreate(UI_EVENT_QUEUE_LENGTH, sizeof(ui_event_t));
    if (s_event_queue == NULL) {
        return ESP_ERR_NO_MEM;
    }

    esp_lv_adapter_config_t adapter_config = ESP_LV_ADAPTER_DEFAULT_CONFIG();
    adapter_config.task_stack_size = 12 * 1024;
    adapter_config.stack_in_psram = true;
    ESP_RETURN_ON_ERROR(esp_lv_adapter_init(&adapter_config), TAG, "adapter init failed");

    esp_lv_adapter_display_config_t display_config =
        ESP_LV_ADAPTER_DISPLAY_RGB_DEFAULT_CONFIG(panel_handle, NULL,
                                                  UI_WIDTH, UI_HEIGHT,
                                                  ESP_LV_ADAPTER_ROTATE_0);
    /* Copy dirty rectangles into a PSRAM back buffer, then switch a complete
     * RGB frame at VSYNC so a scan never mixes old and new pixels. */
    display_config.tear_avoid_mode = ESP_LV_ADAPTER_TEAR_AVOID_MODE_TRIPLE_PARTIAL;
    display_config.profile.use_psram = true;
    s_display = esp_lv_adapter_register_display(&display_config);
    if (s_display == NULL) {
        return ESP_FAIL;
    }

    if (touch_handle != NULL) {
        esp_lv_adapter_touch_config_t touch_config =
            ESP_LV_ADAPTER_TOUCH_DEFAULT_CONFIG(s_display, touch_handle);
        if (esp_lv_adapter_register_touch(&touch_config) == NULL) {
            return ESP_FAIL;
        }
    }

    ESP_RETURN_ON_ERROR(esp_lv_adapter_start(), TAG, "adapter start failed");
    ESP_RETURN_ON_ERROR(esp_lv_adapter_lock(-1), TAG, "adapter lock failed");
    create_ui();
    lv_timer_create(ui_timer_cb, UI_REFRESH_PERIOD_MS, NULL);
    esp_lv_adapter_unlock();
    return ESP_OK;
}

static void enqueue_event(ui_event_t event)
{
    if (s_trace_gap_pending) {
        event.break_trace = true;
        s_trace_gap_pending = false;
    }

    if (xQueueSend(s_event_queue, &event, 0) == pdTRUE) {
        return;
    }

    /*
     * Drop the oldest queued event and keep this one. The lost sample is a
     * real gap, so the next segment must not be drawn back to the old point.
     */
    ui_event_t discarded;
    (void)xQueueReceive(s_event_queue, &discarded, 0);
    event.break_trace = true;
    if (xQueueSend(s_event_queue, &event, 0) != pdTRUE) {
        s_trace_gap_pending = true;
        ESP_LOGW(TAG, "UI sample queue full");
    }
}

static void submit_event(bool has_response,
                         bool break_trace,
                         const adc_debug_response_t *response,
                         const lissajous_ui_counters_t *counters)
{
    if (s_event_queue == NULL || counters == NULL) {
        return;
    }

    ui_event_t event = {
        .has_response = has_response,
        .break_trace = break_trace,
        .counters = *counters,
    };
    if (response != NULL) {
        event.response = *response;
    }
    enqueue_event(event);
}

static bool counters_equal(const lissajous_ui_counters_t *left,
                           const lissajous_ui_counters_t *right)
{
    return left->response_count == right->response_count &&
           left->timeout_count == right->timeout_count &&
           left->crc_error_count == right->crc_error_count &&
           left->format_error_count == right->format_error_count &&
           left->request_pending == right->request_pending;
}

static lissajous_ui_counters_t s_last_counters;
static bool s_have_counters;

void lissajous_ui_submit_response(const adc_debug_response_t *response,
                                  const lissajous_ui_counters_t *counters)
{
    if (counters != NULL) {
        s_last_counters = *counters;
        s_have_counters = true;
    }
    submit_event(true, false, response, counters);
}

void lissajous_ui_submit_counters(const lissajous_ui_counters_t *counters)
{
    if (counters == NULL || s_event_queue == NULL ||
        (s_have_counters && counters_equal(counters, &s_last_counters))) {
        return;
    }
    if (uxQueueSpacesAvailable(s_event_queue) < UI_COUNTER_QUEUE_RESERVE) {
        return;
    }
    s_last_counters = *counters;
    s_have_counters = true;
    submit_event(false, false, NULL, counters);
}

void lissajous_ui_submit_gap(const lissajous_ui_counters_t *counters)
{
    if (counters == NULL) {
        return;
    }
    s_last_counters = *counters;
    s_have_counters = true;
    submit_event(false, true, NULL, counters);
}

bool lissajous_ui_is_paused(void)
{
    return s_paused;
}

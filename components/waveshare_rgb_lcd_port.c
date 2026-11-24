/*
 * SPDX-FileCopyrightText: 2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include "waveshare_rgb_lcd_port.h"

#include "esp_cache.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_touch_gt911.h"
#include "esp_log.h"
#include "esp_rom_sys.h"

static const char *TAG = "lcd_port";
static bool s_i2c_ready = false;

#define CH422G_MODE_REG             0x24
#define CH422G_OUTPUT_REG           0x38
#define CH422G_MODE_OUTPUT          0x01
#define CH422G_BIT_TP_RST           (1U << 1)
#define CH422G_BIT_DISP             (1U << 2)
#define CH422G_BIT_LCD_RST          (1U << 3)
#define CH422G_BIT_SD_CS            (1U << 4)

static uint8_t s_exio_output = CH422G_BIT_TP_RST | CH422G_BIT_DISP |
                               CH422G_BIT_LCD_RST | CH422G_BIT_SD_CS;

#if CONFIG_EXAMPLE_LCD_TOUCH_CONTROLLER_GT911
static esp_err_t i2c_master_init_if_needed(void)
{
    if (s_i2c_ready) {
        return ESP_OK;
    }

    const i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    esp_err_t ret = i2c_param_config(I2C_MASTER_NUM, &i2c_conf);
    if (ret != ESP_OK) {
        return ret;
    }

    ret = i2c_driver_install(I2C_MASTER_NUM, i2c_conf.mode, 0, 0, 0);
    if (ret == ESP_OK || ret == ESP_ERR_INVALID_STATE) {
        s_i2c_ready = true;
        return ESP_OK;
    }

    return ret;
}

static esp_err_t touch_reset_gpio_init(void)
{
    const gpio_config_t io_conf = {
        .pin_bit_mask = GPIO_INPUT_PIN_SEL,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    return gpio_config(&io_conf);
}

static esp_err_t ch422g_init_for_output(void)
{
    uint8_t write_buf = CH422G_MODE_OUTPUT;
    return i2c_master_write_to_device(I2C_MASTER_NUM, CH422G_MODE_REG, &write_buf, 1,
                                      I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

static esp_err_t ch422g_write_output(uint8_t value)
{
    s_exio_output = value;
    return i2c_master_write_to_device(I2C_MASTER_NUM, CH422G_OUTPUT_REG, &s_exio_output, 1,
                                      I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

static esp_err_t waveshare_lcd_release_reset(void)
{
    esp_err_t ret = i2c_master_init_if_needed();
    if (ret != ESP_OK) {
        return ret;
    }
    ret = ch422g_init_for_output();
    if (ret != ESP_OK) {
        return ret;
    }

    /* EXIO3 is LCD_RST. The glass stays in its black/white/red/green/blue
     * self-test until this pin is pulsed after power-up. */
    uint8_t held_in_reset = (uint8_t)((CH422G_BIT_TP_RST | CH422G_BIT_DISP | CH422G_BIT_SD_CS) &
                                      ~CH422G_BIT_LCD_RST);
    ret = ch422g_write_output(held_in_reset);
    if (ret != ESP_OK) {
        return ret;
    }
    esp_rom_delay_us(10 * 1000);
    ret = ch422g_write_output((uint8_t)(held_in_reset | CH422G_BIT_LCD_RST));
    if (ret != ESP_OK) {
        return ret;
    }
    esp_rom_delay_us(100 * 1000);
    ESP_LOGI(TAG, "LCD reset released");
    return ESP_OK;
}

static esp_err_t waveshare_esp32_s3_touch_reset(void)
{
    esp_err_t ret = touch_reset_gpio_init();
    if (ret != ESP_OK) {
        return ret;
    }

    ret = ch422g_write_output((uint8_t)(s_exio_output & ~CH422G_BIT_TP_RST));
    if (ret != ESP_OK) {
        return ret;
    }
    esp_rom_delay_us(10 * 1000);
    gpio_set_level(EXAMPLE_TOUCH_RESET_GPIO, 0);
    esp_rom_delay_us(100 * 1000);
    ret = ch422g_write_output((uint8_t)(s_exio_output | CH422G_BIT_TP_RST));
    if (ret != ESP_OK) {
        return ret;
    }
    esp_rom_delay_us(200 * 1000);
    return ESP_OK;
}
#endif

static void paint_signal_lock_pattern(esp_lcd_panel_handle_t panel, uint8_t fb_count)
{
    void *framebuffers[3] = {0};
    const size_t pixel_count = (size_t)EXAMPLE_LCD_H_RES * EXAMPLE_LCD_V_RES;

    if ((fb_count == 0) || (fb_count > 3) ||
        (esp_lcd_rgb_panel_get_frame_buffer(panel, fb_count, &framebuffers[0],
                                            &framebuffers[1], &framebuffers[2]) != ESP_OK)) {
        return;
    }

    for (int index = 0; index < 3; index++) {
        uint16_t *pixels = framebuffers[index];
        size_t pixel;

        if (pixels == NULL) {
            continue;
        }
        for (pixel = 0; pixel < pixel_count; pixel++) {
            pixels[pixel] = (pixel < (pixel_count / 2U)) ? 0xF800 : 0x001F;
        }
        esp_cache_msync(pixels, pixel_count * sizeof(uint16_t),
                        ESP_CACHE_MSYNC_FLAG_DIR_C2M | ESP_CACHE_MSYNC_FLAG_UNALIGNED);
    }
}

esp_err_t waveshare_rgb_lcd_backlight_on(void)
{
    esp_err_t ret = i2c_master_init_if_needed();
    if (ret != ESP_OK) {
        return ret;
    }
    ret = ch422g_init_for_output();
    if (ret != ESP_OK) {
        return ret;
    }
    return ch422g_write_output((uint8_t)(s_exio_output | CH422G_BIT_DISP | CH422G_BIT_LCD_RST));
}

esp_err_t waveshare_esp32_s3_rgb_lcd_init(esp_lv_adapter_tear_avoid_mode_t tear_mode,
                                          esp_lv_adapter_rotation_t rotation,
                                          esp_lcd_panel_handle_t *panel_handle,
                                          esp_lcd_touch_handle_t *touch_handle)
{
    if ((panel_handle == NULL) || (touch_handle == NULL)) {
        return ESP_ERR_INVALID_ARG;
    }

    *panel_handle = NULL;
    *touch_handle = NULL;

    const uint8_t num_fbs = esp_lv_adapter_get_required_frame_buffer_count(tear_mode, rotation);

    esp_err_t reset_ret = waveshare_lcd_release_reset();
    if (reset_ret != ESP_OK) {
        ESP_LOGW(TAG, "CH422G LCD reset failed: %s", esp_err_to_name(reset_ret));
    }

    ESP_LOGI(TAG, "Install RGB LCD panel driver");
    esp_lcd_rgb_panel_config_t panel_config = {
        .clk_src = LCD_CLK_SRC_DEFAULT,
        .timings =  {
            .pclk_hz = EXAMPLE_LCD_PIXEL_CLOCK_HZ,
            .h_res = EXAMPLE_LCD_H_RES,
            .v_res = EXAMPLE_LCD_V_RES,
            .hsync_pulse_width = 4,
            .hsync_back_porch = 8,
            .hsync_front_porch = 8,
            .vsync_pulse_width = 4,
            .vsync_back_porch = 8,
            .vsync_front_porch = 8,
            .flags = {
                .pclk_active_neg = 1,
            },
        },
        .data_width = EXAMPLE_RGB_DATA_WIDTH,
        .bits_per_pixel = EXAMPLE_RGB_BIT_PER_PIXEL,
        .num_fbs = num_fbs,
        .bounce_buffer_size_px = EXAMPLE_RGB_BOUNCE_BUFFER_SIZE,
        .sram_trans_align = 4,
        .psram_trans_align = 64,
        .hsync_gpio_num = EXAMPLE_LCD_IO_RGB_HSYNC,
        .vsync_gpio_num = EXAMPLE_LCD_IO_RGB_VSYNC,
        .de_gpio_num = EXAMPLE_LCD_IO_RGB_DE,
        .pclk_gpio_num = EXAMPLE_LCD_IO_RGB_PCLK,
        .disp_gpio_num = EXAMPLE_LCD_IO_RGB_DISP,
        .data_gpio_nums = {
            EXAMPLE_LCD_IO_RGB_DATA0,
            EXAMPLE_LCD_IO_RGB_DATA1,
            EXAMPLE_LCD_IO_RGB_DATA2,
            EXAMPLE_LCD_IO_RGB_DATA3,
            EXAMPLE_LCD_IO_RGB_DATA4,
            EXAMPLE_LCD_IO_RGB_DATA5,
            EXAMPLE_LCD_IO_RGB_DATA6,
            EXAMPLE_LCD_IO_RGB_DATA7,
            EXAMPLE_LCD_IO_RGB_DATA8,
            EXAMPLE_LCD_IO_RGB_DATA9,
            EXAMPLE_LCD_IO_RGB_DATA10,
            EXAMPLE_LCD_IO_RGB_DATA11,
            EXAMPLE_LCD_IO_RGB_DATA12,
            EXAMPLE_LCD_IO_RGB_DATA13,
            EXAMPLE_LCD_IO_RGB_DATA14,
            EXAMPLE_LCD_IO_RGB_DATA15,
        },
        .flags = {
            .fb_in_psram = 1,
        },
    };

    esp_err_t ret = esp_lcd_new_rgb_panel(&panel_config, panel_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "create RGB panel failed: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "Initialize RGB LCD panel");
    ret = esp_lcd_panel_init(*panel_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "RGB panel init failed: %s", esp_err_to_name(ret));
        return ret;
    }
    paint_signal_lock_pattern(*panel_handle, num_fbs);

#if CONFIG_EXAMPLE_LCD_TOUCH_CONTROLLER_GT911
    ret = i2c_master_init_if_needed();
    if (ret == ESP_OK) {
        ret = waveshare_esp32_s3_touch_reset();
    }
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "touch reset skipped: %s", esp_err_to_name(ret));
        return ESP_OK;
    }

    esp_lcd_panel_io_handle_t tp_io_handle = NULL;
    esp_lcd_panel_io_i2c_config_t tp_io_config = ESP_LCD_TOUCH_IO_I2C_GT911_CONFIG();
    tp_io_config.scl_speed_hz = 0;
    ESP_LOGI(TAG, "Initialize I2C panel IO");
    ret = esp_lcd_new_panel_io_i2c((esp_lcd_i2c_bus_handle_t)I2C_MASTER_NUM, &tp_io_config, &tp_io_handle);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "touch IO skipped: %s", esp_err_to_name(ret));
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Initialize touch controller GT911");
    const esp_lcd_touch_config_t tp_cfg = {
        .x_max = EXAMPLE_LCD_H_RES,
        .y_max = EXAMPLE_LCD_V_RES,
        .rst_gpio_num = EXAMPLE_PIN_NUM_TOUCH_RST,
        .int_gpio_num = EXAMPLE_PIN_NUM_TOUCH_INT,
        .levels = {
            .reset = 0,
            .interrupt = 0,
        },
        .flags = {
            .swap_xy = 0,
            .mirror_x = 0,
            .mirror_y = 0,
        },
    };
    ret = esp_lcd_touch_new_i2c_gt911(tp_io_handle, &tp_cfg, touch_handle);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "GT911 skipped: %s", esp_err_to_name(ret));
        *touch_handle = NULL;
    }
#endif

    return ESP_OK;
}

idf_component_register(
    SRCS
        "main.c"
        "adc_debug_protocol.c"
        "lissajous_ui.c"
        "../components/waveshare_rgb_lcd_port.c"
    INCLUDE_DIRS
        "."
        "../components"
    REQUIRES driver esp_lvgl_adapter esp_mm
)

idf_component_register(SRCS "esp_lcd_panel_ssd1680.c"
                       INCLUDE_DIRS "include"
                       REQUIRES "esp_lcd"
                       PRIV_REQUIRES driver logger_common)

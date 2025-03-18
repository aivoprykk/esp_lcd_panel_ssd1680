set(SRCS esp_lcd_panel_ssd168x.c)
idf_component_register(SRCS ${SRCS}
                       INCLUDE_DIRS "include"
                       REQUIRES esp_lcd
                       PRIV_REQUIRES driver esp_timer)

set(REQ esp_lcd)
set(SRCS esp_lcd_panel_ssd168x.c)
idf_component_register(SRCS ${SRCS}
                       INCLUDE_DIRS "include"
                       REQUIRES ${REQ}
                       PRIV_REQUIRES driver)

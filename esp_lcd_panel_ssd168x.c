#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#if CONFIG_LCD_ENABLE_DEBUG_LOG
// The local log level must be defined before including esp_log.h
// Set the maximum log level for this source file
#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG
#endif
#include "driver/gpio.h"
// #include "esp_attr.h"
// #include "esp_check.h"
#include "esp_lcd_panel_interface.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ssd168x.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_ssd168x_private.h"
#include "esp_log.h"
#include "esp_memory_utils.h"

#define SSD168X_LUT_SIZE 159

#if defined(CONFIG_SSD168X_PANEL_SSD1680)
#define SSD168X_SOURCE_SIZE 176
#define SSD168X_GATE_SIZE 296
#endif
#if defined(CONFIG_SSD168X_PANEL_SSD1681)
#define SSD168X_SOURCE_SIZE 200
#define SSD168X_GATE_SIZE 200
#endif
#if !defined(SSD168X_SOURCE_SIZE)
#define SSD168X_SOURCE_SIZE 0
#endif
#if !defined(SSD168X_GATE_SIZE)
#define SSD168X_GATE_SIZE 0
#endif

#define SSD168X_RAM_SIZE (SSD168X_SOURCE_SIZE * SSD168X_GATE_SIZE / 8)  // 6512

static const char *TAG = "lcd_panel.epaper";
static const char *drv_msg[] = {"SSD168X_CMD_", "SET_", "err", "panel handler is NULL"};
static esp_err_t set_ram_params(epaper_panel_t *epaper_panel, int x, int y, int xe, int ye, uint8_t em, bool swap_xy) {
    ram_params_t *p = &(epaper_panel->_ram_params);
#if (defined(CONFIG_LCD_ENABLE_DEBUG_LOG))
#if defined(CONFIG_LCD_ENABLE_DEBUG_LOG)
    DEBUG_LOG(TAG,"[%s]",__func__);
#endif
    ESP_LOGD(TAG, "[%s] 0 x=%d, xe=%d | y=%d, ye=%d | em=%02x, swapxy:%d", __func__, x, xe, y, ye, em, swap_xy);
#endif
    // always panel initial direction source=x gate=y
    if (swap_xy) {
        SWAP_INT(x, y)
        SWAP_INT(xe, ye)
    }
    // if(epaper_panel->_mirror_x && epaper_panel->gap_x) {
    //     x += epaper_panel->gap_x;
    //     xe += epaper_panel->gap_x;
    // }
    // y += epaper_panel->gap_y;
    // ye += epaper_panel->gap_y;
    int16_t w = xe - x;
    int16_t h = ye - y;
    // int16_t wb = (w + 7) >> 3u;                                               // calculate source size in bytes (w+7)/8
    // x -= (x & 0xff);                                                     // align to x % 256
    // w = p->wb << 3u;                                                     // byte boundary p->wb * 8
    p->x = x < 0 ? 0 : x;                                                // limit to 0
    p->y = y < 0 ? 0 : y;                                                // limit to 0
    p->w = x + xe < epaper_panel->width ? w : epaper_panel->width - x;   // limit to panel width
    p->h = y + h < epaper_panel->height ? h : epaper_panel->height - y;  // limit to panel height
    p->xe = p->x + p->w;
    p->ye = p->y + p->h;
    // p->dx = p->x - x;
    // p->dxe = p->xe - xe;
    // p->dy = p->y - y;
    // p->dye = p->ye - ye;
    // p->w -= p->dx;
    // p->h -= p->dy;
    // p->w += p->w % 8;
    // if (p->w % 8 > 0) p->w += 8 - p->w % 8;
    // p->x -= p->x % 8;
#if (defined(CONFIG_LCD_ENABLE_DEBUG_LOG))
    ESP_LOGD(TAG, "[%s] 1 x=%d, xe=%d, w=%d | y=%d, ye=%d, h=%d", __func__, p->x, p->xe, p->w, p->y, p->ye, p->h);
    ESP_RETURN_ON_FALSE((p->w > 0) && (p->h > 0), ESP_ERR_INVALID_ARG, TAG, "Invalid x,y,xe,ye");
#endif
    p->buffer_size = (((p->w + 7) >> 3u) * p->h);
    p->xs_d8 = ((p->x >> 3u) & 0xff);                 // startx/8 means (x >> 3u)
    p->xe_d8 = (((p->x + p->w - 1) >> 3u) & 0xff);    // endx/8 means (xe >> 3u)
    p->ys_d256 = ((p->y >> 8u) & 0xff);               // starty/256 means (y >> 8)
    p->ys_m256 = (p->y & 0xff);                       // starty%256 means (y & 0xFF)
    p->ye_d256 = (((p->y + p->h - 1) >> 8u) & 0xff);  // endy/256 means (ye >> 8u)
    p->ye_m256 = ((p->y + p->h - 1) & 0xff);          // endy%256 means (ye & 0xFF)
    p->ram_mode = min(em, 0x07);
#if (defined(CONFIG_LCD_ENABLE_DEBUG_LOG))
    ESP_LOGD(TAG, "[%s] 2 x=%hu, xe=%hu, w=%d | y=%hu, ye=%hu, h=%d", __func__, p->x, p->xe, p->w, p->y, p->ye, p->h);
    ESP_LOGD(TAG, "[%s] 2 xs_d8=%hhu, xe_d8=%hhu, ys_m256=%hhu, ys_d256=%hhu, ye_m256=%hhu, ye_d256=%hhu | buffer_size=%u, ram_mode=%02x", __func__,
             p->xs_d8, p->xe_d8, p->ys_m256, p->ys_d256, p->ye_m256, p->ye_d256, p->buffer_size, p->ram_mode);
#endif
    return ESP_OK;
}

static void epaper_driver_gpio_isr_handler(void *arg) {
    epaper_panel_t *epaper_panel = arg;
    // --- Disable ISR handling
    gpio_intr_disable(epaper_panel->busy_gpio_num);

    // --- Call user callback func
    if (epaper_panel->epaper_refresh_done_isr_callback.callback_ptr) {
        (epaper_panel->epaper_refresh_done_isr_callback.callback_ptr)(&(epaper_panel->base), NULL, epaper_panel->epaper_refresh_done_isr_callback.args);
    }
}

esp_err_t epaper_panel_register_event_callbacks_ssd168x(esp_lcd_panel_t *panel, epaper_panel_callbacks_t *cbs, void *user_ctx) {
#if defined(CONFIG_LCD_ENABLE_DEBUG_LOG)
    DEBUG_LOG(TAG,"[%s]",__func__);
#endif
    if(panel == NULL) {
        ESP_LOGE(TAG, "%s", drv_msg[3]);
        return ESP_ERR_INVALID_ARG;
    }
    if(cbs == NULL) {
        ESP_LOGE(TAG, "cbs is NULL");
        return ESP_ERR_INVALID_ARG;
    }
    epaper_panel_t *epaper_panel = __containerof(panel, epaper_panel_t, base);
    (epaper_panel->epaper_refresh_done_isr_callback).callback_ptr = cbs->on_epaper_refresh_done;
    (epaper_panel->epaper_refresh_done_isr_callback).args = user_ctx;
    return ESP_OK;
}

esp_err_t epaper_panel_set_custom_lut_ssd168x(esp_lcd_panel_t *panel, const uint8_t *lut, size_t size) {
#if defined(CONFIG_LCD_ENABLE_DEBUG_LOG)
    DEBUG_LOG(TAG,"[%s]",__func__);
#endif
    if(panel == NULL) {
        ESP_LOGE(TAG, "%s", drv_msg[3]);
        return ESP_ERR_INVALID_ARG;
    }
    if(lut == NULL) {
        ESP_LOGE(TAG, "lut is NULL");
        return ESP_ERR_INVALID_ARG;
    }
    if(size != SSD168X_LUT_SIZE) {
        ESP_LOGE(TAG, "Invalid lut size");
        return ESP_ERR_INVALID_ARG;
    }
    epaper_panel_t *epaper_panel = __containerof(panel, epaper_panel_t, base);
    epaper_set_lut(epaper_panel->io, lut);
    return ESP_OK;
}

static esp_err_t epaper_set_display_sequence(esp_lcd_panel_io_handle_t io, uint8_t mode) {
#if defined(CONFIG_LCD_ENABLE_DEBUG_LOG)
    DEBUG_LOG(TAG,"[%s] mode: %02x",__func__, mode);
    DEBUG_MEAS_START();
#endif
    if(!mode) mode = SSD168X_PARAM_DISP_UPDATE_MODE_2;
    if(esp_lcd_panel_io_tx_param(io, SSD168X_CMD_SET_DISP_UPDATE_CTRL, (const uint8_t[]){mode}, 1)!=ESP_OK) {
        ESP_LOGE(TAG, "%s%sDISP_UPDATE_CTRL %s", drv_msg[0], drv_msg[1], drv_msg[2]);
        return ESP_FAIL;
    }
#if defined(CONFIG_LCD_ENABLE_DEBUG_LOG)
    DEBUG_MEAS_END(TAG, "[%s] took %llu us", __func__);
#endif
    return ESP_OK;
}

static esp_err_t epaper_set_lut(esp_lcd_panel_io_handle_t io, const uint8_t *lut) {
#if defined(CONFIG_LCD_ENABLE_DEBUG_LOG)
    DEBUG_LOG(TAG,"[%s]", __func__);
    DEBUG_MEAS_START();
#endif
    if(esp_lcd_panel_io_tx_param(io, SSD168X_CMD_SET_LUT_REG, lut, 153) != ESP_OK) {
        ESP_LOGE(TAG, "%s%sLUT_REG  %s", drv_msg[0], drv_msg[1], drv_msg[2]);
        return ESP_FAIL;
    }
    if(epaper_set_display_sequence(io, lut[153]) != ESP_OK) {
        return ESP_FAIL;
    }
    if(esp_lcd_panel_io_tx_param(io, SSD168X_CMD_SET_GATE_DRIVING_VOLTAGE, (const uint8_t[]){lut[154]}, 1) != ESP_OK) {
        ESP_LOGE(TAG, "%s%sGATE_DRIVING_VOLTAGE  %s", drv_msg[0], drv_msg[1], drv_msg[2]);
        return ESP_FAIL;
    }
    if(esp_lcd_panel_io_tx_param(io, SSD168X_CMD_SET_SRC_DRIVING_VOLTAGE, (const uint8_t[]){lut[155], lut[156], lut[157]}, 3) != ESP_OK) {
        ESP_LOGE(TAG, "%s%sSRC_DRIVING_VOLTAGE  %s", drv_msg[0], drv_msg[1], drv_msg[2]);
        return ESP_FAIL;
    }
    if(esp_lcd_panel_io_tx_param(io, SSD168X_CMD_SET_VCOM_REG, (const uint8_t[]){lut[158]}, 1) != ESP_OK) {
        ESP_LOGE(TAG, "%s%sVCOM_REG  %s", drv_msg[0], drv_msg[1], drv_msg[2]);
        return ESP_FAIL;
    }
    
#if defined(CONFIG_LCD_ENABLE_DEBUG_LOG)
    DEBUG_MEAS_END(TAG, "[%s] took %llu us", __func__);
#endif
    return ESP_OK;
}

static esp_err_t epaper_set_ram_x_addr_counter(esp_lcd_panel_io_handle_t io, uint8_t start_x) {
#if defined(CONFIG_LCD_ENABLE_DEBUG_LOG)
    DEBUG_LOG(TAG, "[%s] start_x=%02x:%hhu", __func__, start_x, start_x);
    DEBUG_MEAS_START();
#endif
    if(esp_lcd_panel_io_tx_param(io, SSD168X_CMD_SET_INIT_X_ADDR_COUNTER, (const uint8_t[]){start_x}, 1)!=ESP_OK) {
        ESP_LOGE(TAG, "%s%sINIT_X_ADDR_COUNTER %s", drv_msg[0], drv_msg[1], drv_msg[2]);;
        return ESP_FAIL;
    }
#if defined(CONFIG_LCD_ENABLE_DEBUG_LOG)
    DEBUG_MEAS_END(TAG, "[%s] took %llu us", __func__);
#endif
    return ESP_OK;
}

static esp_err_t epaper_set_ram_y_addr_counter(esp_lcd_panel_io_handle_t io, uint8_t start_y, uint8_t start_y1) {
#if defined(CONFIG_LCD_ENABLE_DEBUG_LOG)
    DEBUG_LOG(TAG, "[%s] start_y=%02x:%hhu, start_y1:%02x:%hhu", __func__, start_y, start_y, start_y1, start_y1);
    DEBUG_MEAS_START();
#endif
    if(esp_lcd_panel_io_tx_param(io, SSD168X_CMD_SET_INIT_Y_ADDR_COUNTER, (const uint8_t[]){start_y, start_y1}, 2)!=ESP_OK) {
        ESP_LOGE(TAG, "%s%sINIT_Y_ADDR_COUNTER %s", drv_msg[0], drv_msg[1], drv_msg[2]);
        return ESP_FAIL;
    }
#if defined(CONFIG_LCD_ENABLE_DEBUG_LOG)
    DEBUG_MEAS_END(TAG, "[%s] took %llu us", __func__);
#endif
    return ESP_OK;
}

static esp_err_t epaper_set_cursor(esp_lcd_panel_io_handle_t io, uint8_t cur_x, uint8_t cur_y, uint8_t cur_y1) {
#if defined(CONFIG_LCD_ENABLE_DEBUG_LOG)
    DEBUG_LOG(TAG, "[%s]", __func__);
#endif
    if(epaper_set_ram_x_addr_counter(io, cur_x)!=ESP_OK) {
        return ESP_FAIL;
    }
    if(epaper_set_ram_y_addr_counter(io, cur_y, cur_y1)!=ESP_OK) {
        return ESP_FAIL;
    }
    return ESP_OK;
}

static esp_err_t epaper_panel_set_sleep_ctrl(esp_lcd_panel_io_handle_t io, uint8_t sleep_mode) {
#if defined(CONFIG_LCD_ENABLE_DEBUG_LOG)
    DEBUG_LOG(TAG, "[%s]", __func__);
#endif
    if(esp_lcd_panel_io_tx_param(io, SSD168X_CMD_SLEEP_CTRL, (const uint8_t[]){sleep_mode}, 1)!=ESP_OK) {
        ESP_LOGE(TAG, "%sSLEEP_CTRL %s", drv_msg[0], drv_msg[2]);
        return ESP_FAIL;
    }
    return ESP_OK;
}

static esp_err_t epaper_panel_set_cursor(esp_lcd_panel_t *panel) {
#if defined(CONFIG_LCD_ENABLE_DEBUG_LOG)
    DEBUG_LOG(TAG,"[%s]",__func__);
#endif
    epaper_panel_t *epaper_panel = __containerof(panel, epaper_panel_t, base);
    ram_params_t *p = &(epaper_panel->_ram_params);
    esp_err_t ret = ESP_OK;
    uint8_t add_x = 0, add_xx = 0;
#ifdef CONFIG_SSD168X_PANEL_SSD1680
#if defined CONFIG_SSD168X_SCREEN_GDEY0213B74
    add_x = 0;
#elif defined CONFIG_SSD168X_SCREEN_DEPG0213BN
    add_xx = 1;
#endif
#endif
    switch (epaper_panel->_ram_params.ram_mode) {
        case 0x00:                                                                                // 000 x decrease, y decrease
        case 0x04:                                                                                // 100 x derease, y decrease : xy changed
            ret = epaper_set_cursor(epaper_panel->io, p->xe_d8 + add_xx - add_x, p->ye_m256, p->ye_d256);  // set ram
            break;
        case 0x01:                                                                                 // 001 x increase, y decrease
        case 0x05:                                                                                 // 101 x increase, y decrease : xy changed
            ret = epaper_set_cursor(epaper_panel->io, p->xs_d8 + add_xx, p->ye_m256, p->ye_d256);  // set ram
            break;
        case 0x02:                                                                                // 010 x decrease, y increase
        case 0x06:                                                                                // 110 x decrease, y increase : xy changed
            ret = epaper_set_cursor(epaper_panel->io, p->xe_d8 + add_xx - add_x, p->ys_m256, p->ys_d256);  // set ram
            break;
        case 0x03:                                                                                 // 011 x increase, y increase : normal mode
        case 0x07:                                                                                 // 111 x increase, y increase : xy changed
            ret = epaper_set_cursor(epaper_panel->io, p->xs_d8 + add_xx, p->ys_m256, p->ys_d256);  // set ram
            break;
    }
    return ret;
}

static esp_err_t epaper_set_ram_area_x(esp_lcd_panel_io_handle_t io, uint8_t start_x, uint8_t end_x) {
#if defined(CONFIG_LCD_ENABLE_DEBUG_LOG)
    DEBUG_LOG(TAG, "[%s] start_x=%02x:%hhu, end_x=%02x:%hhu", __func__, start_x, start_x, end_x, end_x);
    DEBUG_MEAS_START();
#endif
    if(esp_lcd_panel_io_tx_param(io, SSD168X_CMD_SET_RAMX_START_END_POS, (const uint8_t[]){start_x, end_x}, 2)!=ESP_OK) {
        ESP_LOGE(TAG, "%s%sRAMX_START_END_POS %s", drv_msg[0], drv_msg[1], drv_msg[2]);
        return ESP_FAIL;
    }
#if defined(CONFIG_LCD_ENABLE_DEBUG_LOG)
    DEBUG_MEAS_END(TAG, "[%s] took %llu us", __func__);
#endif
    return ESP_OK;
}

static esp_err_t epaper_set_ram_area_y(esp_lcd_panel_io_handle_t io, uint8_t start_y, uint8_t start_y1, uint8_t end_y, uint8_t end_y1) {
#if defined(CONFIG_LCD_ENABLE_DEBUG_LOG)
    DEBUG_LOG(TAG, "[%s] start_y=%02x:%hhu, start_y1:%02x:%hhu end_y=%02x:%hhu, end_y1:%02x:%hhu", __func__, start_y, start_y, start_y1, start_y1, end_y, end_y, end_y1, end_y1);
    DEBUG_MEAS_START();
#endif
    if(esp_lcd_panel_io_tx_param(io, SSD168X_CMD_SET_RAMY_START_END_POS, (const uint8_t[]){start_y, start_y1, end_y, end_y1}, 4)!=ESP_OK) {
        ESP_LOGE(TAG, "%s%sRAMY_START_END_POS %s", drv_msg[0], drv_msg[1], drv_msg[2]);
        return ESP_FAIL;
    }
#if defined(CONFIG_LCD_ENABLE_DEBUG_LOG)
    DEBUG_MEAS_END(TAG, "[%s] took %llu us", __func__);
#endif
    return ESP_OK;
}

static esp_err_t epaper_set_ram_area(esp_lcd_panel_io_handle_t io, uint8_t start_x, uint8_t end_x, uint8_t start_y, uint8_t start_y1, uint8_t end_y, uint8_t end_y1) {
#if defined(CONFIG_LCD_ENABLE_DEBUG_LOG)
    DEBUG_LOG(TAG, "[%s]", __func__);
#endif
    // --- Set RAMX/SOUCE Start/End Position
    if(epaper_set_ram_area_x(io, start_x, end_x)!=ESP_OK) {
        return ESP_FAIL;
    }
    // --- Set RAMY/GATE Start/End Position
    if(epaper_set_ram_area_y(io, start_y, start_y1, end_y, end_y1)!=ESP_OK) {
        return ESP_FAIL;
    }
    return ESP_OK;
}

static esp_err_t epaper_panel_set_ram_area(esp_lcd_panel_t *panel) {
#if defined(CONFIG_LCD_ENABLE_DEBUG_LOG)
    DEBUG_LOG(TAG,"[%s]",__func__);
#endif
    epaper_panel_t *epaper_panel = __containerof(panel, epaper_panel_t, base);
    ram_params_t *p = &(epaper_panel->_ram_params);
    esp_err_t ret = ESP_OK;
    uint8_t add_x = 0, add_xx = 0;
#ifdef CONFIG_SSD168X_PANEL_SSD1680
#if defined CONFIG_SSD168X_SCREEN_GDEY0213B74
    add_x = 0;
#elif defined CONFIG_SSD168X_SCREEN_DEPG0213BN
    add_xx = 1;
#endif
#endif
    switch (epaper_panel->_ram_params.ram_mode) {
        case 0x00:                                                                                                                        // 000 x decrease, y decrease
        case 0x04:                                                                                                                        // 100 x derease, y decrease : xy changed
            ret = epaper_set_ram_area(epaper_panel->io, p->xe_d8 + add_xx - add_x, p->xs_d8 + add_xx - add_x, p->ye_m256, p->ye_d256, p->ys_m256, p->ys_d256);  // X-source area,Y-gate area
            break;
        case 0x01:                                                                                                                          // 001 x increase, y decrease
        case 0x05:                                                                                                                          // 101 x increase, y decrease : xy changed
            ret = epaper_set_ram_area(epaper_panel->io, p->xs_d8 + add_xx, p->xe_d8 + add_xx, p->ye_m256, p->ye_d256, p->ys_m256, p->ys_d256);  // X-source area,Y-gate area
            break;
        case 0x02:                                                                                                                        // 010 x decrease, y increase
        case 0x06:                                                                                                                        // 110 x decrease, y increase : xy changed
            ret = epaper_set_ram_area(epaper_panel->io, p->xe_d8 + add_xx - add_x, p->xs_d8 + add_xx - add_x, p->ys_m256, p->ys_d256, p->ye_m256, p->ye_d256);  // X-source area,Y-gate area
            break;
        case 0x03:                                                                                                                          // 011 x increase, y increase : normal mode
        case 0x07:                                                                                                                          // 111 x increase, y increase : xy changed
            ret = epaper_set_ram_area(epaper_panel->io, p->xs_d8 + add_xx, p->xe_d8 + add_xx, p->ys_m256, p->ys_d256, p->ye_m256, p->ye_d256);  // X-source area,Y-gate area
            break;
    }
    return ret;
}

static esp_err_t panel_epaper_wait_busy(esp_lcd_panel_t *panel) {
#if defined(CONFIG_LCD_ENABLE_DEBUG_LOG)
    DEBUG_LOG(TAG,"[%s]",__func__);
    DEBUG_MEAS_START();
#endif
    epaper_panel_t *epaper_panel = __containerof(panel, epaper_panel_t, base);
    while (gpio_get_level(epaper_panel->busy_gpio_num)) {
        vTaskDelay(pdMS_TO_TICKS(15));
    }
#if defined(CONFIG_LCD_ENABLE_DEBUG_LOG)
    DEBUG_MEAS_END(TAG, "[%s] took %llu us", __func__);
#endif
    return ESP_OK;
}

static esp_err_t epaper_panel_set_black_vram(esp_lcd_panel_io_handle_t io, const uint8_t *bw_bitmap, size_t size) {
#if defined(CONFIG_LCD_ENABLE_DEBUG_LOG)
    DEBUG_LOG(TAG,"[%s]",__func__);
    DEBUG_MEAS_START();
#endif
    if(esp_lcd_panel_io_tx_color(io, SSD168X_CMD_WRITE_BLACK_VRAM, bw_bitmap, size)!=ESP_OK) {
        ESP_LOGE(TAG, "%sWRITE_BLACK_VRAM %s", drv_msg[0], drv_msg[2]);
        return ESP_FAIL;
    }
#if defined(CONFIG_LCD_ENABLE_DEBUG_LOG)
    DEBUG_MEAS_END(TAG, "[%s] took %llu us", __func__);
#endif
    return ESP_OK;
}

static esp_err_t epaper_panel_set_red_vram(esp_lcd_panel_io_handle_t io, const uint8_t *red_bitmap, size_t size) {
#if defined(CONFIG_LCD_ENABLE_DEBUG_LOG)
    DEBUG_LOG(TAG,"[%s]",__func__);
    DEBUG_MEAS_START();
#endif
    if(esp_lcd_panel_io_tx_color(io, SSD168X_CMD_WRITE_RED_VRAM, red_bitmap, size)!=ESP_OK) {
        ESP_LOGE(TAG, "%sWRITE_RED_VRAM %s", drv_msg[0], drv_msg[2]);
        return ESP_FAIL;
    }
#if defined(CONFIG_LCD_ENABLE_DEBUG_LOG)
    DEBUG_MEAS_END(TAG, "[%s] took %llu us", __func__);
#endif
    return ESP_OK;
}

static esp_err_t epaper_panel_set_vram(esp_lcd_panel_io_handle_t io, const uint8_t *bw_bitmap, const uint8_t *red_bitmap, size_t size) {
#if defined(CONFIG_LCD_ENABLE_DEBUG_LOG)
    DEBUG_LOG(TAG, "[%s] red: %d, black: %d, size: %u", __func__, bw_bitmap ? 1 : 0, red_bitmap ? 1 : 0, size);
#endif
    // Note: the screen region to be used to draw bitmap had been defined
    // The region of BLACK VRAM and RED VRAM are set by the same series of command, the two bitmaps will be drawn at
    // the same region, so the two bitmaps can share a same size.
    if (bw_bitmap && (size > 0)) {
        if(epaper_panel_set_black_vram(io, bw_bitmap, size)!=ESP_OK) {
            return ESP_FAIL;
        }
    }
    if (red_bitmap && (size > 0)) {
        if(epaper_panel_set_red_vram(io, red_bitmap, size)!=ESP_OK) {
            return ESP_FAIL;
        }
    }
    return ESP_OK;
}

static esp_err_t epaper_set_display_update_control(esp_lcd_panel_t *panel) {
#if defined(CONFIG_LCD_ENABLE_DEBUG_LOG)
    DEBUG_LOG(TAG,"[%s]",__func__);
    DEBUG_MEAS_START();
#endif
    epaper_panel_t *epaper_panel = __containerof(panel, epaper_panel_t, base);
    uint8_t duc_flag = 0x00;
    if (!(epaper_panel->_invert_color)) {
        duc_flag |= SSD168X_PARAM_COLOR_BW_INVERSE_BIT;
        duc_flag &= (~SSD168X_PARAM_COLOR_RW_INVERSE_BIT);
    } else {
        duc_flag &= (~SSD168X_PARAM_COLOR_BW_INVERSE_BIT);
        duc_flag |= SSD168X_PARAM_COLOR_RW_INVERSE_BIT;
    }
    if(esp_lcd_panel_io_tx_param(epaper_panel->io, SSD168X_CMD_DISP_UPDATE_CTRL, (const uint8_t[]){duc_flag}, 1) != ESP_OK) { // color invert flag
        ESP_LOGE(TAG, "%s%sDISP_UPDATE_CTRL %s", drv_msg[0], drv_msg[1], drv_msg[2]);
        return ESP_FAIL;
    }
#if defined(CONFIG_LCD_ENABLE_DEBUG_LOG)
    DEBUG_MEAS_END(TAG, "[%s] took %llu us", __func__);
#endif
    return ESP_OK;
}

static esp_err_t epaper_set_active_display_update_sequence(esp_lcd_panel_io_handle_t io) {
#if defined(CONFIG_LCD_ENABLE_DEBUG_LOG)
    DEBUG_LOG(TAG,"[%s]",__func__);
    DEBUG_MEAS_START();
#endif
    if(esp_lcd_panel_io_tx_param(io, SSD168X_CMD_ACTIVE_DISP_UPDATE_SEQ, NULL, 0)!=ESP_OK) {
        ESP_LOGE(TAG, "%sACTIVE_DISP_UPDATE_SEQ %s", drv_msg[0], drv_msg[2]);
        return ESP_FAIL;
    }
#if defined(CONFIG_LCD_ENABLE_DEBUG_LOG)
    DEBUG_MEAS_END(TAG, "[%s] took %llu us", __func__);
#endif
    return ESP_OK;
}

static uint8_t epaper_get_ram_mode(esp_lcd_panel_t *panel) {
#if defined(CONFIG_LCD_ENABLE_DEBUG_LOG)
    DEBUG_LOG(TAG, "[%s]", __func__);
#endif
    epaper_panel_t *epaper_panel = __containerof(panel, epaper_panel_t, base);
    uint8_t mode = 0x03;
    if ((!(epaper_panel->_mirror_x)) && (epaper_panel->_mirror_y)) {
        mode = epaper_panel->_swap_xy ? SSD168X_PARAM_DATA_ENTRY_MODE_1 : SSD168X_PARAM_DATA_ENTRY_MODE_1;
    } else if (((epaper_panel->_mirror_x)) && (!(epaper_panel->_mirror_y))) {
        // mirror x, means x is flipped
        mode = epaper_panel->_swap_xy ? SSD168X_PARAM_DATA_ENTRY_MODE_2 : SSD168X_PARAM_DATA_ENTRY_MODE_2;
    } else if ((epaper_panel->_mirror_x) && (epaper_panel->_mirror_y)) {
        mode = epaper_panel->_swap_xy ? SSD168X_PARAM_DATA_ENTRY_MODE_0 : SSD168X_PARAM_DATA_ENTRY_MODE_0;
    // } else {
    //     mode = epaper_panel->_swap_xy ? SSD168X_PARAM_DATA_ENTRY_MODE_3 : SSD168X_PARAM_DATA_ENTRY_MODE_3;
    }
    return mode;
}

static esp_err_t epaper_set_data_entry_mode(esp_lcd_panel_t *panel) {
#if defined(CONFIG_LCD_ENABLE_DEBUG_LOG)
    DEBUG_LOG(TAG, "[%s]", __func__);
    DEBUG_MEAS_START();
#endif
    epaper_panel_t *epaper_panel = __containerof(panel, epaper_panel_t, base);
    uint8_t mode = epaper_panel->_ram_params.ram_mode;
#if defined(CONFIG_LCD_ENABLE_DEBUG_LOG)
    ESP_LOGD(TAG, "set ram data entry mode, mode: %02x, modestr: %s", mode,
             mode == SSD168X_PARAM_DATA_ENTRY_MODE_0 ? "000" : mode == SSD168X_PARAM_DATA_ENTRY_MODE_1 ? "001"
                                                           : mode == SSD168X_PARAM_DATA_ENTRY_MODE_2   ? "010"
                                                           : mode == SSD168X_PARAM_DATA_ENTRY_MODE_3   ? "011"
                                                           : mode == SSD168X_PARAM_DATA_ENTRY_MODE_4   ? "100"
                                                           : mode == SSD168X_PARAM_DATA_ENTRY_MODE_5   ? "101"
                                                           : mode == SSD168X_PARAM_DATA_ENTRY_MODE_6   ? "110"
                                                           : mode == SSD168X_PARAM_DATA_ENTRY_MODE_7   ? "111"
                                                                                                       : "unknown");
#endif
    if(esp_lcd_panel_io_tx_param(epaper_panel->io, SSD168X_CMD_DATA_ENTRY_MODE, (const uint8_t[]){mode}, 1)!=ESP_OK) {
        ESP_LOGE(TAG, "%s%sDATA_ENTRY_MODE %s", drv_msg[0], drv_msg[1], drv_msg[2]);
        return ESP_FAIL;
    }
#if defined(CONFIG_LCD_ENABLE_DEBUG_LOG)
    DEBUG_MEAS_END(TAG, "[%s] took %llu us", __func__);
#endif
    return ESP_OK;
}

static esp_err_t epaper_set_data_entry_sequence(esp_lcd_panel_t *panel, bool setcursor) {
#if defined(CONFIG_LCD_ENABLE_DEBUG_LOG)
    DEBUG_LOG(TAG, "[%s]", __func__);
    DEBUG_MEAS_START();
#endif
    // epaper_panel_t *epaper_panel = __containerof(panel, epaper_panel_t, base);
    // set data entry mode
    if(epaper_set_data_entry_mode(panel)!=ESP_OK) {
        return ESP_FAIL;
    }
    // area by cmd 0x44 0x45
    if(epaper_panel_set_ram_area(panel)!=ESP_OK) {
        return ESP_FAIL;
    }
    // cursor by cmd 0x4e 0x4f
    if (setcursor) {
        if(epaper_panel_set_cursor(panel)!=ESP_OK) {
            return ESP_FAIL;
        }
    }
#if defined(CONFIG_LCD_ENABLE_DEBUG_LOG)
    DEBUG_MEAS_END(TAG, "[%s] took %llu us", __func__);
#endif
    return ESP_OK;
}

static esp_err_t epaper_set_driver_output(esp_lcd_panel_t *panel) {
#if defined(CONFIG_LCD_ENABLE_DEBUG_LOG)
    DEBUG_LOG(TAG, "[%s]", __func__);
    DEBUG_MEAS_START();
#endif
    epaper_panel_t *epaper_panel = __containerof(panel, epaper_panel_t, base);
    if(esp_lcd_panel_io_tx_param(epaper_panel->io, SSD168X_CMD_OUTPUT_CTRL, SSD168X_PARAM_OUTPUT_CTRL, 3)!=ESP_OK) {
        ESP_LOGE(TAG, "%sOUTPUT_CTRL %s", drv_msg[0], drv_msg[2]);
        return ESP_FAIL;
    }
#if defined(CONFIG_LCD_ENABLE_DEBUG_LOG)
    DEBUG_MEAS_END(TAG, "[%s] took %llu us", __func__);
#endif
    return ESP_OK;
}

esp_err_t epaper_panel_update_full_screen_ssd168x(esp_lcd_panel_t *panel) {
    epaper_panel_t *epaper_panel = __containerof(panel, epaper_panel_t, base);
    if(epaper_set_active_display_update_sequence(epaper_panel->io)!=ESP_OK) {
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t epaper_panel_refresh_screen_ssd168x(esp_lcd_panel_t *panel, uint8_t update_mode) {
#if defined(CONFIG_LCD_ENABLE_DEBUG_LOG)
    DEBUG_LOG(TAG,"[%s]",__func__);
#endif
    if(!panel) {
        ESP_LOGE(TAG, "%s", drv_msg[3]);
        return ESP_ERR_INVALID_ARG;
    }
    epaper_panel_t *epaper_panel = __containerof(panel, epaper_panel_t, base);
    // --- Set color invert
    if(epaper_set_display_update_control(panel)!=ESP_OK) {
        return ESP_FAIL;
    }
    // --- Enable refresh done handler isr
    gpio_intr_enable(epaper_panel->busy_gpio_num);
    // --- Send refresh command
    if(epaper_set_display_sequence(epaper_panel->io, update_mode)!=ESP_OK) {
        return ESP_FAIL;
    }
    if(epaper_set_active_display_update_sequence(epaper_panel->io)!=ESP_OK) {
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t
esp_lcd_new_panel_ssd168x(const esp_lcd_panel_io_handle_t io, const esp_lcd_panel_dev_config_t *const panel_dev_config,
                          esp_lcd_panel_handle_t *const ret_panel) {
#if defined(CONFIG_LCD_ENABLE_DEBUG_LOG)
    DEBUG_LOG(TAG,"[%s]",__func__);
    DEBUG_MEAS_START();
#endif
#if CONFIG_LCD_ENABLE_DEBUG_LOG
    esp_log_level_set(TAG, ESP_LOG_DEBUG);
#endif
    if(!(io && panel_dev_config && ret_panel)) {
        ESP_LOGE(TAG, "Invalid args");
        return ESP_ERR_INVALID_ARG;
    }
    esp_lcd_ssd168x_config_t *epaper_ssd168x_conf = panel_dev_config->vendor_config;
    esp_err_t ret = ESP_OK;
    // --- Allocate epaper_panel memory on HEAP
    epaper_panel_t *epaper_panel = NULL;
    epaper_panel = calloc(1, sizeof(epaper_panel_t));
    if(!epaper_panel) {
        ESP_LOGE(TAG, "no mem for epaper panel");
        goto err;
    }

    // --- Construct panel & implement interface
    // defaults
    epaper_panel->is_on = false;
    epaper_panel->next_init_lut = NULL;
    epaper_panel->next_init_mode = INIT_MODE_FAST_2;
    epaper_panel->next_sleep_mode = SLEEP_MODE_DEEP_1;
    epaper_panel->_invert_color = false;
    epaper_panel->_swap_xy = false;
    epaper_panel->_mirror_x = false;
    epaper_panel->_mirror_y = false;
    epaper_panel->_framebuffer = NULL;
    epaper_panel->gap_x = 0;
    epaper_panel->gap_y = 0;
    epaper_panel->bitmap_color = SSD168X_EPAPER_BITMAP_BLACK;
    epaper_panel->full_refresh = true;
    // configurations
    epaper_panel->io = io;
    epaper_panel->reset_gpio_num = panel_dev_config->reset_gpio_num;
    epaper_panel->busy_gpio_num = epaper_ssd168x_conf->busy_gpio_num;
    epaper_panel->reset_level = panel_dev_config->flags.reset_active_high;
    epaper_panel->_non_copy_mode = epaper_ssd168x_conf->non_copy_mode;
    // functions
    epaper_panel->base.del = epaper_panel_del;
    epaper_panel->base.reset = epaper_panel_reset;
    epaper_panel->base.init = epaper_panel_init;
    epaper_panel->base.draw_bitmap = epaper_panel_draw_bitmap;
    epaper_panel->base.invert_color = epaper_panel_invert_color;
    epaper_panel->base.set_gap = epaper_panel_set_gap;
    epaper_panel->base.mirror = epaper_panel_mirror;
    epaper_panel->base.swap_xy = epaper_panel_swap_xy;
    epaper_panel->base.disp_on_off = epaper_panel_disp_on_off;
    epaper_panel->height = epaper_ssd168x_conf->height;
    epaper_panel->width = epaper_ssd168x_conf->width;
    *ret_panel = &(epaper_panel->base);
    // --- Init framebuffer
    if (!(epaper_panel->_non_copy_mode)) {
        epaper_panel->_framebuffer_size = epaper_ssd168x_conf->buffer_size;
        epaper_panel->_framebuffer = heap_caps_malloc(epaper_ssd168x_conf->buffer_size, MALLOC_CAP_DMA);
        if(!epaper_panel->_framebuffer) {
            ESP_LOGE(TAG, "epaper_panel_init allocating buffer memory err");
            goto err;
        }
    }
    // size_t img_size = epaper_panel->height * epaper_panel->width / 8;
    // epaper_panel->clearbuffer = epaper_ssd168x_conf->clear_img; // heap_caps_malloc(img_size, MALLOC_CAP_DMA);
    // ESP_RETURN_ON_FALSE(epaper_panel->_clearbuffer, ESP_ERR_NO_MEM, TAG, "epaper_panel_init allocating buffer memory err");
    // memset(epaper_panel->_clearbuffer, 0xFF, img_size);
    // --- Init GPIO
    // init RST GPIO
    if (epaper_panel->reset_gpio_num >= 0) {
        gpio_config_t io_conf = {
            .mode = GPIO_MODE_OUTPUT,
            .pin_bit_mask = 1ULL << panel_dev_config->reset_gpio_num,
        };
        if(gpio_config(&io_conf)) {
            ESP_LOGE(TAG, "configure GPIO for RST line err");
            goto err;
        }
    }
    // init BUSY GPIO
    if (epaper_panel->busy_gpio_num >= 0) {
        gpio_config_t io_conf = {
            .mode = GPIO_MODE_INPUT,
            .pull_down_en = 0x01,
            .pin_bit_mask = 1ULL << epaper_panel->busy_gpio_num,
        };
        io_conf.intr_type = GPIO_INTR_NEGEDGE;
        ESP_LOGI(TAG, "Add handler for GPIO %d", epaper_panel->busy_gpio_num);
        if(gpio_config(&io_conf)) {
            ESP_LOGE(TAG, "configure GPIO for BUSY line err");
            goto err;
        }
        if(gpio_isr_handler_add(epaper_panel->busy_gpio_num, epaper_driver_gpio_isr_handler, epaper_panel)) {
            ESP_LOGE(TAG, "add GPIO ISR handler err");
            goto err;
        }
        // Enable GPIO intr only before refreshing, to avoid other commands caused intr trigger
        gpio_intr_disable(epaper_panel->busy_gpio_num);
    }
    ESP_LOGD(TAG, "new epaper panel @%p", epaper_panel);
    return ret;
err:
    if (epaper_panel) {
        if (panel_dev_config->reset_gpio_num >= 0) {
            gpio_reset_pin(panel_dev_config->reset_gpio_num);
        }
        if (epaper_ssd168x_conf->busy_gpio_num >= 0) {
            gpio_reset_pin(epaper_ssd168x_conf->busy_gpio_num);
        }
        free(epaper_panel);
    }
    DEBUG_MEAS_END(TAG, "[%s] took %llu us", __func__);
    return ret;
}

static esp_err_t epaper_panel_del(esp_lcd_panel_t *panel) {
#if defined(CONFIG_LCD_ENABLE_DEBUG_LOG)
    DEBUG_LOG(TAG, "[%s]", __func__);
#endif
    epaper_panel_t *epaper_panel = __containerof(panel, epaper_panel_t, base);
    // --- Reset used GPIO pins
    if ((epaper_panel->reset_gpio_num) >= 0) {
        gpio_reset_pin(epaper_panel->reset_gpio_num);
    }
    gpio_reset_pin(epaper_panel->busy_gpio_num);
    // --- Free allocated RAM
    if ((epaper_panel->_framebuffer) && (!(epaper_panel->_non_copy_mode))) {
        // Should not free if buffer is not allocated by driver
        free(epaper_panel->_framebuffer);
    }
    free(epaper_panel);
    return ESP_OK;
}

static esp_err_t epaper_panel_gpio_reset(esp_lcd_panel_t *panel) {
#if defined(CONFIG_LCD_ENABLE_DEBUG_LOG)
    DEBUG_LOG(TAG, "[%s]", __func__);
#endif
    epaper_panel_t *epaper_panel = __containerof(panel, epaper_panel_t, base);
    if(gpio_set_level(epaper_panel->reset_gpio_num, epaper_panel->reset_level)){
        ESP_LOGE(TAG, "gpio_set_level error");
        return ESP_FAIL;
    }
    vTaskDelay(pdMS_TO_TICKS(10));
    if(gpio_set_level(epaper_panel->reset_gpio_num, !epaper_panel->reset_level)) {
        ESP_LOGE(TAG, "gpio_set_level error");
        return ESP_FAIL;
    }
    vTaskDelay(pdMS_TO_TICKS(10));
    return ESP_OK;
}

static esp_err_t epaper_panel_software_reset(esp_lcd_panel_t *panel) {
#if defined(CONFIG_LCD_ENABLE_DEBUG_LOG)
    DEBUG_LOG(TAG, "[%s]", __func__);
#endif
    epaper_panel_t *epaper_panel = __containerof(panel, epaper_panel_t, base);
    esp_lcd_panel_io_handle_t io = epaper_panel->io;
    // perform software reset
    if(esp_lcd_panel_io_tx_param(io, SSD168X_CMD_SWRST, NULL, 0)!=ESP_OK) {
        ESP_LOGE(TAG, "%sSWRST %s", drv_msg[0], drv_msg[2]);
        return ESP_FAIL;
    }
    epaper_panel->is_on = true;
    return ESP_OK;
}

static esp_err_t epaper_panel_reset(esp_lcd_panel_t *panel) {
#if defined(CONFIG_LCD_ENABLE_DEBUG_LOG)
    DEBUG_LOG(TAG, "[%s]", __func__);
    DEBUG_MEAS_START();
#endif
    epaper_panel_t *epaper_panel = __containerof(panel, epaper_panel_t, base);
    // esp_lcd_panel_io_handle_t io = epaper_panel->io;

    // perform hardware reset
    if (epaper_panel->reset_gpio_num >= 0) {
        if(epaper_panel_gpio_reset(panel)!= ESP_OK) {
            return ESP_FAIL;
        }
    } else {
        if(epaper_panel_software_reset(panel)!= ESP_OK) {
            return ESP_FAIL;
        }
    }
    panel_epaper_wait_busy(panel);
    DEBUG_MEAS_END(TAG, "[%s] took %llu us", __func__);
    return ESP_OK;
}

esp_err_t epaper_panel_set_bitmap_color_ssd168x(esp_lcd_panel_t *panel, esp_lcd_ssd168x_bitmap_color_t color) {
#if defined(CONFIG_LCD_ENABLE_DEBUG_LOG)
    DEBUG_LOG(TAG, "[%s]", __func__);
#endif
    if(!panel) {
        ESP_LOGE(TAG, "%s", drv_msg[3]);
        return ESP_ERR_INVALID_ARG;
    }
    epaper_panel_t *epaper_panel = __containerof(panel, epaper_panel_t, base);
    epaper_panel->bitmap_color = color;
    return ESP_OK;
}

esp_err_t epaper_panel_set_next_init_mode_ssd168x(esp_lcd_panel_t *panel, epaper_panel_init_mode_t next_init_mode) {
#if defined(CONFIG_LCD_ENABLE_DEBUG_LOG)
    DEBUG_LOG(TAG, "[%s]", __func__);
#endif
    epaper_panel_t *epaper_panel = __containerof(panel, epaper_panel_t, base);
    epaper_panel->next_init_mode = next_init_mode;
    return ESP_OK;
}

esp_err_t epaper_panel_set_next_sleep_mode_ssd168x(esp_lcd_panel_t *panel, epaper_panel_sleep_mode_t next_sleep_mode) {
#if defined(CONFIG_LCD_ENABLE_DEBUG_LOG)
    DEBUG_LOG(TAG, "[%s]", __func__);
#endif
    epaper_panel_t *epaper_panel = __containerof(panel, epaper_panel_t, base);
    epaper_panel->next_sleep_mode = next_sleep_mode;
    return ESP_OK;
}

static esp_err_t epaper_panel_init_stage_1(esp_lcd_panel_t *panel) {
#if defined(CONFIG_LCD_ENABLE_DEBUG_LOG)
    DEBUG_LOG(TAG, "[%s]", __func__);
#endif
    // epaper_panel_t *epaper_panel = __containerof(panel, epaper_panel_t, base);
    // esp_lcd_panel_io_handle_t io = epaper_panel->io;
    // --- SWRST
    if(epaper_panel_software_reset(panel)!= ESP_OK) {
        return ESP_FAIL;
    }
    panel_epaper_wait_busy(panel);
    // --- Driver Output Control
    if(epaper_set_driver_output(panel)!= ESP_OK) {
        return ESP_FAIL;
    }
    return ESP_OK;
}

static esp_err_t epaper_panel_init_stage_2(esp_lcd_panel_t *panel) {
#if defined(CONFIG_LCD_ENABLE_DEBUG_LOG)
    DEBUG_LOG(TAG, "[%s]", __func__);
    DEBUG_MEAS_START();
#endif
    epaper_panel_t *epaper_panel = __containerof(panel, epaper_panel_t, base);
    set_ram_params(epaper_panel, 0, 0, epaper_panel->width, epaper_panel->height, epaper_get_ram_mode(panel), false); // default
    // --- Set RAM data entry mode
    if(epaper_set_data_entry_sequence(panel, true)!= ESP_OK) {
        return ESP_FAIL;
    }
    DEBUG_MEAS_END(TAG, "[%s] took %llu us", __func__);
    return ESP_OK;
}

static esp_err_t epaper_panel_init_stage_3(esp_lcd_panel_t *panel, const uint8_t *lut) {
#if defined(CONFIG_LCD_ENABLE_DEBUG_LOG)
    DEBUG_LOG(TAG, "[%s]", __func__);
    DEBUG_MEAS_START();
#endif
    epaper_panel_t *epaper_panel = __containerof(panel, epaper_panel_t, base);
    esp_lcd_panel_io_handle_t io = epaper_panel->io;
    // --- Border Waveform Control
    if(esp_lcd_panel_io_tx_param(epaper_panel->io, SSD168X_CMD_SET_BORDER_WAVEFORM, (const uint8_t[]){SSD168X_PARAM_BORDER_WAVEFORM_0}, 1)!= ESP_OK) {
        ESP_LOGE(TAG, "%s%sBORDER_WAVEFORM %s", drv_msg[0], drv_msg[1], drv_msg[2]);
        return ESP_FAIL;
    }

    // --- Temperature Sensor Control
    if(esp_lcd_panel_io_tx_param(epaper_panel->io, SSD168X_CMD_SET_TEMP_SENSOR, (const uint8_t[]){SSD168X_PARAM_TEMP_SENSOR}, 1)!= ESP_OK) {
        ESP_LOGE(TAG, "%s%sTEMP_SENSOR %s", drv_msg[0], drv_msg[1], drv_msg[2]);
        return ESP_FAIL;
    }

    if (lut) {
        if(epaper_set_lut(io, lut)) {
            return ESP_FAIL;
        }
        if(esp_lcd_panel_io_tx_param(epaper_panel->io, 0x37, (const uint8_t[]){0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00}, 10)!= ESP_OK) {
            return ESP_FAIL;
        }
    } else {
        // --- Load built-in waveform LUT
        if(epaper_set_display_sequence(epaper_panel->io, epaper_panel->next_init_mode)!= ESP_OK) {
            return ESP_FAIL;
        }
    }
    // --- Active Display Update Sequence
    if(epaper_set_active_display_update_sequence(epaper_panel->io)!= ESP_OK) {
        return ESP_FAIL;
    }
    panel_epaper_wait_busy(panel);
    epaper_panel->next_init_mode = INIT_MODE_PARTIAL;
    DEBUG_MEAS_END(TAG, "[%s] took %llu us", __func__);
    return ESP_OK;
}

static esp_err_t epaper_panel_init_stage_4(esp_lcd_panel_t *panel, uint8_t color) {
#if defined(CONFIG_LCD_ENABLE_DEBUG_LOG)
    DEBUG_LOG(TAG, "[%s]", __func__);
    DEBUG_MEAS_START();
#endif
    // --- Set LUT
    epaper_panel_t *epaper_panel = __containerof(panel, epaper_panel_t, base);
    // esp_lcd_panel_io_handle_t io = epaper_panel->io;
    // ESP_RETURN_ON_ERROR(epaper_set_lut(io, SSD168X_LUT_DEFAULT), TAG, "epaper_set_lut error");
    // ESP_RETURN_ON_ERROR(set_cursor(epaper_panel), TAG, "epaper_set_cursor() error");
    // clear screen
    if (!(epaper_panel->_non_copy_mode)) {
        memset(epaper_panel->_framebuffer, color, epaper_panel->_framebuffer_size);
        if(epaper_panel_set_vram(epaper_panel->io, (epaper_panel->_framebuffer), (epaper_panel->_framebuffer), epaper_panel->_framebuffer_size)) {
            return ESP_FAIL;
        }
    }
    DEBUG_MEAS_END(TAG, "[%s] took %llu us", __func__);
    return ESP_OK;
}

static esp_err_t epaper_panel_init_stage_5(esp_lcd_panel_t *panel) {
#if defined(CONFIG_LCD_ENABLE_DEBUG_LOG)
    DEBUG_LOG(TAG, "[%s]", __func__);
    DEBUG_MEAS_START();
#endif
    epaper_panel_t *epaper_panel = __containerof(panel, epaper_panel_t, base);
    // esp_lcd_panel_io_handle_t io = epaper_panel->io;
    // --- Display end option
    // ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(epaper_panel->io, SSD168X_CMD_SET_END_OPTION, (const uint8_t[]) {
    //     SSD168X_PARAM_END_OPTION_KEEP
    // }, 1), TAG, "SSD1681_CMD_SET_END_OPTION err");
    // --- Active Display Update Sequence
    if(epaper_set_active_display_update_sequence(epaper_panel->io)!= ESP_OK) {
        return ESP_FAIL;
    }
    panel_epaper_wait_busy(panel);
    DEBUG_MEAS_END(TAG, "[%s] took %llu us", __func__);
    return ESP_OK;
}

static esp_err_t epaper_panel_init(esp_lcd_panel_t *panel) {
#if defined(CONFIG_LCD_ENABLE_DEBUG_LOG)
    DEBUG_LOG(TAG, "[%s]", __func__);
#endif
    epaper_panel_t *epaper_panel = __containerof(panel, epaper_panel_t, base);
    // esp_lcd_panel_io_handle_t io = epaper_panel->io;
    if(epaper_panel_init_stage_1(panel)) {
        return ESP_FAIL;
    }
    if(epaper_panel_init_stage_2(panel)) {
        return ESP_FAIL;
    }
    if(epaper_panel_init_stage_3(panel, epaper_panel->next_init_lut)) {
        return ESP_FAIL;
    }
    if(epaper_panel_init_stage_4(panel, SSD168X_PARAM_DISP_UPDATE_MODE_0)) {
        return ESP_FAIL;
    }
    // if(epaper_panel_init_stage_5(panel)) {
    //     return ESP_FAIL;
    // }
    epaper_panel->next_init_lut = NULL;
    return ESP_OK;
}

esp_err_t epaper_panel_init_screen_ssd168x(esp_lcd_panel_t *panel, epaper_panel_init_mode_t next_init_mode, const uint8_t *lut) {
#if defined(CONFIG_LCD_ENABLE_DEBUG_LOG)
    DEBUG_LOG(TAG, "[%s] mode: %02x", __func__, next_init_mode);
#endif
    epaper_panel_t *epaper_panel = __containerof(panel, epaper_panel_t, base);
    epaper_panel->next_init_mode = next_init_mode;
    epaper_panel->next_init_lut = lut;
    return epaper_panel_init(panel);
}

esp_err_t epaper_panel_clear_screen_ssd168x(esp_lcd_panel_t *panel, uint8_t *color_data, uint8_t color) {
    epaper_panel_t *epaper_panel = __containerof(panel, epaper_panel_t, base);
#if defined(CONFIG_LCD_ENABLE_DEBUG_LOG)
    DEBUG_LOG(TAG, "[%s] color: %02x panelw: %hd panelh: %hd", __func__, color, epaper_panel->width, epaper_panel->height);
    DEBUG_MEAS_START();
#endif
    ram_params_t *p = &(epaper_panel->_ram_params);
    set_ram_params(epaper_panel, 0, 0, epaper_panel->width, epaper_panel->height, epaper_get_ram_mode(panel), false);
    // --- Set cursor & data entry sequence
    if(epaper_set_data_entry_sequence(panel, true)) {
        return ESP_FAIL;
    }
    memset(color_data, color, p->buffer_size);
    if(epaper_panel_set_vram(epaper_panel->io, (uint8_t *)(color_data), (color_data), p->buffer_size)) {
        return ESP_FAIL;
    }
    // ESP_ERROR_CHECK(epaper_panel_refresh_screen_ssd168x(panel, 0));
    DEBUG_MEAS_END(TAG, "[%s] took %llu us", __func__);
    return ESP_OK;
}


static esp_err_t epaper_panel_draw_bitmap(esp_lcd_panel_t *panel, int x_start, int y_start, int x_end, int y_end, const void *color_data) {
#if defined(CONFIG_LCD_ENABLE_DEBUG_LOG)
    DEBUG_LOG(TAG, "[%s] start bounds: {x:%d, y:%d} -> {x:%d, y:%d}",__func__ , x_start, y_start, x_end, y_end);
#endif
    epaper_panel_t *epaper_panel = __containerof(panel, epaper_panel_t, base);
    if (gpio_get_level(epaper_panel->busy_gpio_num)) {
        return ESP_ERR_NOT_FINISHED;
    }
    // --- Assert & check configuration
    // if (epaper_panel->_non_copy_mode) {
    //     ESP_RETURN_ON_FALSE(!(epaper_panel->_swap_xy), ESP_ERR_INVALID_ARG, TAG, "swap-xy is unavailable when enabling non-copy mode");
    //     ESP_RETURN_ON_FALSE(!(epaper_panel->_mirror_y), ESP_ERR_INVALID_ARG, TAG, "mirror_y is unavailable when enabling non-copy mode");
    // }
    if(!color_data) {
        ESP_LOGE(TAG, "bitmap is NULL");
        return ESP_ERR_INVALID_ARG;
    }
    if((!(x_start < x_end) && (y_start < y_end))) {
        ESP_LOGE(TAG, "start position must be smaller than end position");
        return ESP_ERR_INVALID_ARG;
    }
    // --- Calculate coordinates & sizes
    ram_params_t *p = &(epaper_panel->_ram_params);
    set_ram_params(epaper_panel, x_start, y_start, x_end, y_end, epaper_get_ram_mode(panel), epaper_panel->_swap_xy);
    // p->ram_mode = epaper_get_ram_mode(panel);
#if (defined(CONFIG_LCD_ENABLE_DEBUG_LOG))
    ESP_LOGD(TAG, "[%s] converted bounds: {x:%d, y:%d} -> {x:%d, y:%d}", __func__, x_start, y_start, x_end, y_end);
#endif
    // --- Data copy & preprocess
    // prepare buffer
    if (epaper_panel->_non_copy_mode) {
        // Use user-passed framebuffer
        epaper_panel->_framebuffer = (uint8_t *)color_data;
        if (!esp_ptr_dma_capable(epaper_panel->_framebuffer)) {
            ESP_LOGW(TAG, "Bitmap not DMA capable, use DMA capable memory to avoid additional data copy.");
        }
    } else {
        // Copy & convert image according to configuration
        process_bitmap(panel, color_data);
    }
    // --- Set cursor & data entry sequence
    if(epaper_set_data_entry_sequence(panel, true)) {
        return ESP_FAIL;
    }
    // --- Send bitmap to e-Paper VRAM
    uint8_t *data = (uint8_t *)(!epaper_panel->_non_copy_mode ? epaper_panel->_framebuffer : color_data);
    // if (epaper_panel->bitmap_color == SSD168X_EPAPER_BITMAP_BLACK) {
    //    if(epaper_panel_set_vram(epaper_panel->io, (uint8_t *)(!epaper_panel->_non_copy_mode ? epaper_panel->_framebuffer : color_data), NULL, p->buffer_size) {
    //        return ESP_FAIL;
    //    }
    // } else if (epaper_panel->bitmap_color == SSD168X_EPAPER_BITMAP_RED) {
        if(epaper_panel_set_vram(epaper_panel->io, data, data, p->buffer_size)) {
            return ESP_FAIL;
        }
    // }
    // --- Refresh the display, show image in VRAM
    // tx_param will wait until DMA transaction finishes, so it is safe to call panel_epaper_refresh_screen at once.
    // The driver will not call the `epaper_panel_refresh_screen` automatically, please call it manually.
    return ESP_OK;
}

static esp_err_t epaper_panel_invert_color(esp_lcd_panel_t *panel, bool invert_color_data) {
#if defined(CONFIG_LCD_ENABLE_DEBUG_LOG)
    DEBUG_LOG(TAG, "[%s] %s", __func__, invert_color_data ? "true" : "false");
#endif
    epaper_panel_t *epaper_panel = __containerof(panel, epaper_panel_t, base);
    epaper_panel->_invert_color = invert_color_data;
    return ESP_OK;
}

static esp_err_t epaper_panel_mirror(esp_lcd_panel_t *panel, bool mirror_x, bool mirror_y) {
#if defined(CONFIG_LCD_ENABLE_DEBUG_LOG)
    DEBUG_LOG(TAG, "[%s] %s %s", __func__, mirror_x ? "mirror_x" : "", mirror_y ? "mirror_y" : "");
#endif
    epaper_panel_t *epaper_panel = __containerof(panel, epaper_panel_t, base);
    // if (mirror_y) {
    //     if (epaper_panel->_non_copy_mode) {
    //         ESP_LOGE(TAG, "mirror_y is unavailable when enabling non-copy mode");
    //         return ESP_ERR_INVALID_ARG;
    //     }
    // }
    epaper_panel->_mirror_x = mirror_x;
    epaper_panel->_mirror_y = mirror_y;

    return ESP_OK;
}

static esp_err_t epaper_panel_swap_xy(esp_lcd_panel_t *panel, bool swap_axes) {
#if defined(CONFIG_LCD_ENABLE_DEBUG_LOG)
    DEBUG_LOG(TAG, "[%s] %s", __func__, swap_axes ? "true" : "false");
#endif
    epaper_panel_t *epaper_panel = __containerof(panel, epaper_panel_t, base);
    // if (swap_axes) {
    //     if (epaper_panel->_non_copy_mode) {
    //         ESP_LOGE(TAG, "swap_xy is unavailable when enabling non-copy mode");
    //         return ESP_ERR_INVALID_ARG;
    //     }
    // }
    epaper_panel->_swap_xy = swap_axes;
    return ESP_OK;
}

static esp_err_t epaper_panel_set_gap(esp_lcd_panel_t *panel, int x_gap, int y_gap) {
#if defined(CONFIG_LCD_ENABLE_DEBUG_LOG)
    DEBUG_LOG(TAG, "[%s]", __func__);
#endif
    epaper_panel_t *epaper_panel = __containerof(panel, epaper_panel_t, base);
    epaper_panel->gap_x = x_gap;
    epaper_panel->gap_y = y_gap;
    return ESP_OK;
}

static esp_err_t epaper_panel_disp_on_off(esp_lcd_panel_t *panel, bool on_off) {
#if defined(CONFIG_LCD_ENABLE_DEBUG_LOG)
    DEBUG_LOG(TAG, "[%s] %s", __func__, on_off ? "on" : "off");
    DEBUG_MEAS_START();
#endif
    epaper_panel_t *epaper_panel = __containerof(panel, epaper_panel_t, base);
    esp_lcd_panel_io_handle_t io = epaper_panel->io;
    if (on_off) {
        // Turn on display
        if(epaper_set_display_sequence(io, SSD168X_PARAM_DISP_UPDATE_MODE_3)) {
            return ESP_FAIL;
        }
        if(epaper_set_active_display_update_sequence(io)) {
            return ESP_FAIL;
        }
        panel_epaper_wait_busy(panel);
    /// don't turn off display here, otherways partial refresh will not work
    // } else {
        // if(epaper_panel_shut_down(panel)) {
        //     return ESP_FAIL;
        // }
    }
    DEBUG_MEAS_END(TAG, "[%s] took %llu us", __func__);
    return ESP_OK;
}

esp_err_t epaper_panel_shut_down(esp_lcd_panel_t *panel) {
#if defined(CONFIG_LCD_ENABLE_DEBUG_LOG)
    DEBUG_LOG(TAG, "[%s]", __func__);
#endif
    epaper_panel_t *epaper_panel = __containerof(panel, epaper_panel_t, base);
    esp_lcd_panel_io_handle_t io = epaper_panel->io;
    uint8_t sleep_mode = SSD168X_PARAM_SLEEP_MODE_1;
    if(epaper_panel->next_sleep_mode==SLEEP_MODE_NORMAL) {
        sleep_mode = SSD168X_PARAM_SLEEP_MODE_0;
    }
    else if(epaper_panel->next_sleep_mode==SLEEP_MODE_DEEP_2) {
        sleep_mode = SSD168X_PARAM_SLEEP_MODE_2;
    }
    // Sleep mode, BUSY pin will keep HIGH after entering sleep mode
    // Perform reset and re-run init to resume the display
    if(epaper_panel_set_sleep_ctrl(io, sleep_mode)) {
        return ESP_FAIL;
    }
    // BUSY pin will stay HIGH, so do not call panel_epaper_wait_busy() here
    epaper_panel->next_sleep_mode=SLEEP_MODE_DEEP_1;
    return ESP_OK;
}

void rotate(uint8_t *img, uint8_t *fb, int width, int height, uint16_t rotation) {
#if defined(CONFIG_LCD_ENABLE_DEBUG_LOG)
    DEBUG_LOG(TAG, "[%s]", __func__);
#endif
    switch (rotation) {
        case 0:
            rotate_bitmap(img, fb, width, height, 0);
            break;
        case 90:
            rotate_bitmap(img, fb, width, height, 1);
            break;
        case 180:
            rotate_bitmap(img, fb, width, height, 2);
            break;
        case 270:
            rotate_bitmap(img, fb, width, height, 3);
            break;
        default:
            break;
    }
}

void rotate_bitmap(unsigned char *src, unsigned char *dest, int width, int height, unsigned char rotation) {
#if defined(CONFIG_LCD_ENABLE_DEBUG_LOG)
    DEBUG_LOG(TAG, "[%s]", __func__);
#endif
    if(rotation==0) {
        memcpy(dest, src, width * height);
        return;
    }
    int byte_width = (width + 7) >> 3u;        // Calculate the width in bytes
    int byte_height = (height + 7) >> 3u;      // Calculate the height in bytes
    int new_width = height; // paddedHeight;              // The new width is the old padded height
    int new_height = width; // paddedWidth;              // The new height is the old padded width
    int total_bytes = (byte_width << 3u) * (byte_height << 3u); // Total number of bytes in the src
    int old_byte_index;
    int new_byte_index;
    int old_bit_index;
    int new_bit_index;
#if (defined(CONFIG_LCD_ENABLE_DEBUG_LOG))
    ESP_LOGD(TAG, "[%s] w:%d h:%d byte_width:%d byte_height:%d new_width:%d new_height:%d total_bytes:%d", __func__, width, height, byte_width, byte_height, new_width, new_height, total_bytes);
#endif
    memset(dest, 0, total_bytes / 8);
    for (int y = 0; y < height; y++) {  // max 122
        for (int x = 0; x < width; x++) {  // max 250
            old_byte_index = y * byte_width + (x >> 3u); // 0-127 * 32 + 0-32
            old_bit_index = 7u - (x & 7u); // 0-7
            if(rotation == 1){ // cw
                new_byte_index = (((new_height - 1 - x) * byte_height) + ((y) >> 3u));
                new_bit_index = y & 7u;
            }
            else if(rotation==2) {
                new_byte_index = ((height - 1 - y) * byte_width) + ((width - 1 - x) >> 3u);
                new_bit_index = 7u - ((width - 1 - x) & 7u);
            }
            else { // if(rotation == 3){ // ccw
                new_byte_index = (((x) * byte_height) + ((new_width - 1 - (y)) >> 3u)); // ((0..249) * 16) + ((122 - 1 - (0..121)) / 8) )
                new_bit_index = 7u - ((new_width - 1 - (y)) & 7u);
            }
            if (BIT_CHECK(src[old_byte_index], old_bit_index)) {  // Check if x and y are within the original width and height
                //new_byte_index--;
                if (new_byte_index >= 0) {
                    BIT_SET(dest[new_byte_index], new_bit_index);
                }
            }
        }
    }
}

static esp_err_t process_bitmap(esp_lcd_panel_t *panel, const void *color_data) {
    epaper_panel_t *epaper_panel = __containerof(panel, epaper_panel_t, base);
    ram_params_t *p = &(epaper_panel->_ram_params);
#if (defined(CONFIG_LCD_ENABLE_DEBUG_LOG))
#if defined(CONFIG_LCD_ENABLE_DEBUG_LOG)
    DEBUG_LOG(TAG, "[%s] w:%hd h:%hd buffer_size:%u", __func__, p->w, p->h, p->buffer_size);
#endif
    ESP_LOGD(TAG, "mirror_x:%d mirror_y:%d swap_xy:%d", epaper_panel->_mirror_x, epaper_panel->_mirror_y, epaper_panel->_swap_xy);
#endif
    // --- Convert image according to configuration
    if (!(epaper_panel->_non_copy_mode)) {
        memset(epaper_panel->_framebuffer, 0xff, p->buffer_size);
    }
    if ((!(epaper_panel->_mirror_x)) && (!(epaper_panel->_mirror_y))) {
        // No mirror 03
        if (!(epaper_panel->_non_copy_mode)) {
            if (epaper_panel->_swap_xy) {
                rotate_bitmap((uint8_t *)color_data, epaper_panel->_framebuffer, p->h, p->w, 3);
            } else {
                memcpy(epaper_panel->_framebuffer, color_data, p->buffer_size);
            }
        }
    }
    if ((!(epaper_panel->_mirror_x)) && (epaper_panel->_mirror_y)) {
        // Mirror Y 01
        if (epaper_panel->_swap_xy) {
            rotate_bitmap((uint8_t *)color_data, epaper_panel->_framebuffer, p->h, p->w, 3);
        } else {
            memcpy(epaper_panel->_framebuffer, color_data, p->buffer_size);
        }
    }
    if (((epaper_panel->_mirror_x)) && (!(epaper_panel->_mirror_y))) {
        // Mirror X 02
        if (!(epaper_panel->_non_copy_mode)) {
            if (epaper_panel->_swap_xy) {  // 006
                rotate_bitmap((uint8_t *)color_data, epaper_panel->_framebuffer, p->h, p->w, 3);
            } else {
                memcpy(epaper_panel->_framebuffer, color_data, p->buffer_size);
            }
        }
    }
    if (((epaper_panel->_mirror_x)) && (epaper_panel->_mirror_y)) {
        // Mirror X & Y 00
        if (epaper_panel->_swap_xy) {
            rotate_bitmap((uint8_t *)color_data, epaper_panel->_framebuffer, p->h, p->w, 3);
        } else {
            for (uint16_t i = p->buffer_size; i > 0; i--) {
                (epaper_panel->_framebuffer)[i] = ((uint8_t *)(color_data))[i];
            }
        }
    }

    return ESP_OK;
}

static inline uint8_t byte_reverse(uint8_t data) {
    static const uint8_t _4bit_reverse_lut[] = {
        0x00, 0x08, 0x04, 0x0C, 0x02, 0x0A, 0x06, 0x0E,
        0x01, 0x09, 0x05, 0x0D, 0x03, 0x0B, 0x07, 0x0F};
    uint8_t result = 0x00;
    // Reverse low 4 bits
    result |= (uint8_t)((_4bit_reverse_lut[data & 0x0f]) << 4);
    // Reverse high 4 bits
    result |= _4bit_reverse_lut[data >> 4];
    return result;
}

uint8_t is_xy_swapped(esp_lcd_panel_t *panel) {
    epaper_panel_t *epaper_panel = __containerof(panel, epaper_panel_t, base);
    if (epaper_panel->_swap_xy)
        return 1;
    return 0;
}

uint8_t is_mirrored(esp_lcd_panel_t *panel) {
    epaper_panel_t *epaper_panel = __containerof(panel, epaper_panel_t, base);
    if (epaper_panel->_mirror_x && epaper_panel->_mirror_y)
        return 3;
    if (epaper_panel->_mirror_x)
        return 2;
    if (epaper_panel->_mirror_y)
        return 1;
    return 0;
}

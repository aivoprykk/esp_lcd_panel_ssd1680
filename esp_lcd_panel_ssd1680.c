/*
 * SPDX-FileCopyrightText: 2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
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
#include "esp_attr.h"
#include "esp_check.h"
#include "esp_lcd_panel_interface.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ssd1680.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_ssd1680_commands.h"
#include "esp_log.h"
#include "esp_memory_utils.h"
//#include "logger_common.h"

#define SSD1680_LUT_SIZE 159
#define SSD1680_SOURCE_SIZE 176
#define SSD1680_GATE_SIZE 296
#define SSD1680_RAM_SIZE (SSD1680_SOURCE_SIZE * SSD1680_GATE_SIZE / 8)  // 6512

#ifndef min
#define min(a, b) (((a) < (b)) ? (a) : (b))
#endif

#ifndef SWAP_INT
#define SWAP_INT(a, b) \
    {                  \
        int t = a;     \
        a = b;         \
        b = t;         \
    }
#endif

static const char *TAG = "lcd_panel.epaper";

typedef struct ram_params_s {
    int16_t x;
    // int16_t dx;
    // int16_t dxe;
    int16_t y;
    // int16_t dy;
    // int16_t dye;
    int16_t xe;
    //int16_t xe_orig;
    int16_t ye;
    //int16_t ye_orig;
    int16_t w;
    //int16_t wb;
    int16_t h;
    uint8_t xs_d8;
    uint8_t xe_d8;
    uint8_t ys_m256;
    uint8_t ys_d256;
    uint8_t ye_m256;
    uint8_t ye_d256;
    size_t buffer_size;
    uint8_t ram_mode;
} ram_params_t;

typedef struct {
    esp_lcd_epaper_panel_cb_t callback_ptr;
    void *args;
} epaper_panel_callback_t;

typedef struct {
    esp_lcd_panel_t base;
    esp_lcd_panel_io_handle_t io;
    // --- Normal configurations
    // Configurations from panel_dev_config
    int reset_gpio_num;
    bool reset_level;
    // Configurations from epaper_ssd1680_conf
    int busy_gpio_num;
    bool full_refresh;
    // Configurations from interface functions
    int gap_x;
    int gap_y;
    // Configurations from e-Paper specific public functions
    epaper_panel_callback_t epaper_refresh_done_isr_callback;
    esp_lcd_ssd1680_bitmap_color_t bitmap_color;
    // --- Associated configurations
    // SHOULD NOT modify directly
    // in order to avoid going into undefined state
    bool _non_copy_mode;
    bool _mirror_y;
    bool _swap_xy;
    // --- Other private fields
    bool _mirror_x;
    bool _invert_color;
    bool _is_rotation_done;
    ram_params_t _ram_params;
    int16_t width;
    int16_t height;
    //const uint8_t *clearbuffer;
    uint8_t *_framebuffer;
    size_t _framebuffer_size;
    epaper_panel_init_mode_t next_init_mode;
    epaper_panel_sleep_mode_t next_sleep_mode;
    const uint8_t * next_init_lut;
} epaper_panel_t;

// --- Utility functions
static inline uint8_t byte_reverse(uint8_t data);
static esp_err_t process_bitmap(esp_lcd_panel_t *panel, const void *color_data);
static esp_err_t panel_epaper_wait_busy(esp_lcd_panel_t *panel);
// --- Callback functions & ISRs
static void epaper_driver_gpio_isr_handler(void *arg);
// --- IO wrapper functions, simply send command/param/buffer
static esp_err_t epaper_set_lut(esp_lcd_panel_io_handle_t io, const uint8_t *lut);
static esp_err_t epaper_set_cursor(esp_lcd_panel_io_handle_t io, uint8_t cur_x, uint8_t cur_y, uint8_t cur_y1);
static esp_err_t epaper_set_area(esp_lcd_panel_io_handle_t io, uint8_t start_x, uint8_t end_x, uint8_t start_y, uint8_t start_y1, uint8_t end_y, uint8_t end_y1);
static esp_err_t panel_epaper_set_vram(esp_lcd_panel_io_handle_t io, const uint8_t *bw_bitmap, const uint8_t *red_bitmap, size_t size);
// --- SSD1680 specific functions, exported to user in public header file
// extern esp_err_t esp_lcd_new_panel_ssd1680(const esp_lcd_panel_io_handle_t io, const esp_lcd_panel_dev_config_t *panel_dev_config,
//                                            esp_lcd_panel_handle_t *ret_panel);
// extern esp_err_t epaper_panel_register_event_callbacks(esp_lcd_panel_t *panel, epaper_panel_callbacks_t* cbs, void* user_ctx);
// extern esp_err_t epaper_panel_refresh_screen(esp_lcd_panel_t *panel);
// extern esp_err_t epaper_panel_set_bitmap_color(esp_lcd_panel_t* panel, esp_lcd_ssd1680_bitmap_color_t color);
// extern esp_err_t epaper_panel_set_custom_lut(esp_lcd_panel_t *panel, uint8_t *lut, size_t size);
// --- Used to implement esp_lcd_panel_interface
static esp_err_t epaper_panel_del(esp_lcd_panel_t *panel);
static esp_err_t epaper_panel_reset(esp_lcd_panel_t *panel);
static esp_err_t epaper_panel_init(esp_lcd_panel_t *panel);
static esp_err_t epaper_panel_draw_bitmap(esp_lcd_panel_t *panel, int x_start, int y_start, int x_end, int y_end, const void *color_data);
static esp_err_t epaper_panel_invert_color(esp_lcd_panel_t *panel, bool invert_color_data);
static esp_err_t epaper_panel_mirror(esp_lcd_panel_t *panel, bool mirror_x, bool mirror_y);
static esp_err_t epaper_panel_swap_xy(esp_lcd_panel_t *panel, bool swap_axes);
static esp_err_t epaper_panel_set_gap(esp_lcd_panel_t *panel, int x_gap, int y_gap);
static esp_err_t epaper_panel_disp_on_off(esp_lcd_panel_t *panel, bool on_off);

void rotateBitmap(unsigned char *src, unsigned char *dest, int width, int height, unsigned char rotation);
void rotate(uint8_t *bitmap, uint8_t *framebuffer, int width, int height, uint16_t rotation);

static esp_err_t set_ram_params(epaper_panel_t *epaper_panel, int x, int y, int xe, int ye, uint8_t em, bool swap_xy) {
    ram_params_t *p = &(epaper_panel->_ram_params);
    ESP_LOGD(TAG, "set_ram_params0:x=%d, xe=%d | y=%d, ye=%d | em=%02x, swapxy:%d", x, xe, y, ye, em, swap_xy);
    // always panel initial direction source=x gate=y
    if (swap_xy) {
        SWAP_INT(x, y)
        SWAP_INT(xe, ye)
    }
    x += epaper_panel->gap_x;
    xe += epaper_panel->gap_x;
    y += epaper_panel->gap_y;
    ye += epaper_panel->gap_y;
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
    ESP_LOGD(TAG, "set_ram_params1:x=%d, xe=%d, w=%d | y=%d, ye=%d, h=%d", p->x, p->xe, p->w, p->y, p->ye, p->h);
    ESP_RETURN_ON_FALSE((p->w > 0) && (p->h > 0), ESP_ERR_INVALID_ARG, TAG, "Invalid x,y,xe,ye");
    p->buffer_size = (((p->w + 7) >> 3u) * p->h);
    p->xs_d8 = ((p->x >> 3u) & 0xff);                 // startx/8 means (x >> 3u)
    p->xe_d8 = (((p->x + p->w - 1) >> 3u) & 0xff);    // endx/8 means (xe >> 3u)
    p->ys_d256 = ((p->y >> 8u) & 0xff);               // starty/256 means (y >> 8)
    p->ys_m256 = (p->y & 0xff);                       // starty%256 means (y & 0xFF)
    p->ye_d256 = (((p->y + p->h - 1) >> 8u) & 0xff);  // endy/256 means (ye >> 8u)
    p->ye_m256 = ((p->y + p->h - 1) & 0xff);          // endy%256 means (ye & 0xFF)
    p->ram_mode = min(em, 0x07);
    ESP_LOGD(TAG, "set_ram_params2:x=%hu, xe=%hu, w=%d | y=%hu, ye=%hu, h=%d", p->x, p->xe, p->w, p->y, p->ye, p->h);
    ESP_LOGD(TAG, "set_ram_params2: xs_d8=%hhu, xe_d8=%hhu, ys_m256=%hhu, ys_d256=%hhu, ye_m256=%hhu, ye_d256=%hhu | buffer_size=%u, ram_mode=%02x",
             p->xs_d8, p->xe_d8, p->ys_m256, p->ys_d256, p->ye_m256, p->ye_d256, p->buffer_size, p->ram_mode);
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

esp_err_t epaper_panel_register_event_callbacks_ssd1680(esp_lcd_panel_t *panel, epaper_panel_callbacks_t *cbs, void *user_ctx) {
    ESP_RETURN_ON_FALSE(panel, ESP_ERR_INVALID_ARG, TAG, "panel handler is NULL");
    ESP_RETURN_ON_FALSE(cbs, ESP_ERR_INVALID_ARG, TAG, "cbs is NULL");
    epaper_panel_t *epaper_panel = __containerof(panel, epaper_panel_t, base);
    (epaper_panel->epaper_refresh_done_isr_callback).callback_ptr = cbs->on_epaper_refresh_done;
    (epaper_panel->epaper_refresh_done_isr_callback).args = user_ctx;
    return ESP_OK;
}

esp_err_t epaper_panel_set_custom_lut_ssd1680(esp_lcd_panel_t *panel, const uint8_t *lut, size_t size) {
    ESP_RETURN_ON_FALSE(panel, ESP_ERR_INVALID_ARG, TAG, "panel handler is NULL");
    ESP_RETURN_ON_FALSE(lut, ESP_ERR_INVALID_ARG, TAG, "lut is NULL");
    ESP_RETURN_ON_FALSE(size == SSD1680_LUT_SIZE, ESP_ERR_INVALID_ARG, TAG, "Invalid lut size");
    epaper_panel_t *epaper_panel = __containerof(panel, epaper_panel_t, base);
    epaper_set_lut(epaper_panel->io, lut);
    return ESP_OK;
}

static esp_err_t epaper_set_lut(esp_lcd_panel_io_handle_t io, const uint8_t *lut) {
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, SSD1680_CMD_SET_LUT_REG, lut, 153), TAG, "SSD1680_CMD_OUTPUT_CTRL err");

    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, SSD1680_CMD_SET_DISP_UPDATE_CTRL, (const uint8_t[]){lut[153]}, 1), TAG, "SSD1680_CMD_SET_END_OPTION err");

    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, SSD1680_CMD_SET_GATE_DRIVING_VOLTAGE, (const uint8_t[]){lut[154]}, 1), TAG, "SSD1680_CMD_SET_END_OPTION err");

    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, SSD1680_CMD_SET_SRC_DRIVING_VOLTAGE, (const uint8_t[]){lut[155], lut[156], lut[157]}, 3), TAG, "SSD1680_CMD_SET_SRC_DRIVING_VOLTAGE err");

    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, SSD1680_CMD_SET_VCOM_REG, (const uint8_t[]){lut[158]}, 1), TAG, "SSD1680_CMD_SET_VCOM_REG err");

    return ESP_OK;
}

static esp_err_t epaper_set_cursor(esp_lcd_panel_io_handle_t io, uint8_t cur_x, uint8_t cur_y, uint8_t cur_y1) {
    ESP_LOGD(TAG, "set cursor: cur_x=%02x:%hhu, cur_y=%02x:%hhu, cur_y1=%02x:%hhu", cur_x, cur_x, cur_y, cur_y, cur_y1, cur_y1);
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, SSD1680_CMD_SET_INIT_X_ADDR_COUNTER, (const uint8_t[]){(cur_x)}, 1), TAG, "SSD1680_CMD_SET_INIT_X_ADDR_COUNTER err");

    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, SSD1680_CMD_SET_INIT_Y_ADDR_COUNTER, (const uint8_t[]){cur_y, cur_y1}, 2), TAG, "SSD1680_CMD_SET_INIT_Y_ADDR_COUNTER err");

    return ESP_OK;
}

#define DEPG0213B74 1
// #define GDEQ0213B74 1

static esp_err_t set_cursor(epaper_panel_t *epaper_panel) {
    ram_params_t *p = &(epaper_panel->_ram_params);
    esp_err_t ret = ESP_OK;
    uint8_t add_x = 0, add_xx = 0;
#if defined GDEQ0213B74
    add_x = p->w == epaper_panel->width ? 1 : 0;
#elif defined DEPG0213B74
    add_xx = 1;
#endif
    switch (epaper_panel->_ram_params.ram_mode) {
        case 0x00:                                                                                // 000 x decrease, y decrease
        case 0x04:                                                                                // 100 x derease, y decrease : xy changed
            ret = epaper_set_cursor(epaper_panel->io, p->xe_d8 - add_x, p->ye_m256, p->ye_d256);  // set ram
            break;
        case 0x01:                                                                                 // 001 x increase, y decrease
        case 0x05:                                                                                 // 101 x increase, y decrease : xy changed
            ret = epaper_set_cursor(epaper_panel->io, p->xs_d8 + add_xx, p->ye_m256, p->ye_d256);  // set ram
            break;
        case 0x02:                                                                                // 010 x decrease, y increase
        case 0x06:                                                                                // 110 x decrease, y increase : xy changed
            ret = epaper_set_cursor(epaper_panel->io, p->xe_d8 - add_x, p->ys_m256, p->ys_d256);  // set ram
            break;
        case 0x03:                                                                                 // 011 x increase, y increase : normal mode
        case 0x07:                                                                                 // 111 x increase, y increase : xy changed
            ret = epaper_set_cursor(epaper_panel->io, p->xs_d8 + add_xx, p->ys_m256, p->ys_d256);  // set ram
            break;
    }
    return ret;
}
static esp_err_t epaper_set_area(esp_lcd_panel_io_handle_t io, uint8_t start_x, uint8_t end_x, uint8_t start_y, uint8_t start_y1, uint8_t end_y, uint8_t end_y1) {
    ESP_LOGD(TAG, "set area: start_x=%02x:%hhu, end_x=%02x:%hhu, start_y=%02x:%hhu, start_y1:%02x:%hhu end_y=%02x:%hhu, end_y1:%02x:%hhu", start_x, start_x, end_x, end_x, start_y, start_y, start_y1, start_y1, end_y, end_y, end_y1, end_y1);
    // --- Set RAMX/SOUCE Start/End Position
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, SSD1680_CMD_SET_RAMX_START_END_POS, (const uint8_t[]){start_x, end_x}, 2), TAG, "SSD1680_CMD_SET_RAMX_START_END_POS err");

    // --- Set RAMY/GATE Start/End Position
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, SSD1680_CMD_SET_RAMY_START_END_POS, (const uint8_t[]){start_y, start_y1, end_y, end_y1}, 4), TAG, "SSD1680_CMD_SET_RAMX_START_END_POS err");

    return ESP_OK;
}
static esp_err_t set_area(epaper_panel_t *epaper_panel) {
    ram_params_t *p = &(epaper_panel->_ram_params);
    esp_err_t ret = ESP_OK;
    uint8_t add_x = 0, add_xx = 0;
#if defined GDEQ0213B74
    add_x = p->w == epaper_panel->width ? 1 : 0;
#elif defined DEPG0213B74
    add_xx = 1;  // width is 128, so when w=128, add 1
#endif
    switch (epaper_panel->_ram_params.ram_mode) {
        case 0x00:                                                                                                                        // 000 x decrease, y decrease
        case 0x04:                                                                                                                        // 100 x derease, y decrease : xy changed
            ret = epaper_set_area(epaper_panel->io, p->xe_d8 - add_x, p->xs_d8 - add_x, p->ye_m256, p->ye_d256, p->ys_m256, p->ys_d256);  // X-source area,Y-gate area
            break;
        case 0x01:                                                                                                                          // 001 x increase, y decrease
        case 0x05:                                                                                                                          // 101 x increase, y decrease : xy changed
            ret = epaper_set_area(epaper_panel->io, p->xs_d8 + add_xx, p->xe_d8 + add_xx, p->ye_m256, p->ye_d256, p->ys_m256, p->ys_d256);  // X-source area,Y-gate area
            break;
        case 0x02:                                                                                                                        // 010 x decrease, y increase
        case 0x06:                                                                                                                        // 110 x decrease, y increase : xy changed
            ret = epaper_set_area(epaper_panel->io, p->xe_d8 - add_x, p->xs_d8 - add_x, p->ys_m256, p->ys_d256, p->ye_m256, p->ye_d256);  // X-source area,Y-gate area
            break;
        case 0x03:                                                                                                                          // 011 x increase, y increase : normal mode
        case 0x07:                                                                                                                          // 111 x increase, y increase : xy changed
            ret = epaper_set_area(epaper_panel->io, p->xs_d8 + add_xx, p->xe_d8 + add_xx, p->ys_m256, p->ys_d256, p->ye_m256, p->ye_d256);  // X-source area,Y-gate area
            break;
    }
    return ret;
}

static esp_err_t panel_epaper_wait_busy(esp_lcd_panel_t *panel) {
    epaper_panel_t *epaper_panel = __containerof(panel, epaper_panel_t, base);
    while (gpio_get_level(epaper_panel->busy_gpio_num)) {
        vTaskDelay(pdMS_TO_TICKS(15));
    }
    return ESP_OK;
}

static esp_err_t panel_epaper_set_vram(esp_lcd_panel_io_handle_t io, const uint8_t *bw_bitmap, const uint8_t *red_bitmap, size_t size) {
    ESP_LOGD(TAG, "panel_epaper_set_vram write red and black, size: %u", size);
    // Note: the screen region to be used to draw bitmap had been defined
    // The region of BLACK VRAM and RED VRAM are set by the same series of command, the two bitmaps will be drawn at
    // the same region, so the two bitmaps can share a same size.
    // --- Validate and Transfer data
    if (bw_bitmap && (size > 0)) {
        // tx WHITE/BLACK bitmap
        ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_color(io, SSD1680_CMD_WRITE_BLACK_VRAM, bw_bitmap, size), TAG,
                            "data bw_bitmap err");
    }
    if (red_bitmap && (size > 0)) {
        // tx RED bitmap
        ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_color(io, SSD1680_CMD_WRITE_RED_VRAM, red_bitmap, size), TAG,
                            "data red_bitmap err");
    }
    return ESP_OK;
}

esp_err_t epaper_panel_refresh_screen_ssd1680(esp_lcd_panel_t *panel, uint8_t update_mode) {
    ESP_RETURN_ON_FALSE(panel, ESP_ERR_INVALID_ARG, TAG, "panel handler is NULL");
    // if(update_mode==2) return ESP_OK;
    epaper_panel_t *epaper_panel = __containerof(panel, epaper_panel_t, base);
    // --- Set color invert
    uint8_t duc_flag = 0x00;
    if (!(epaper_panel->_invert_color)) {
        duc_flag |= SSD1680_PARAM_COLOR_BW_INVERSE_BIT;
        duc_flag &= (~SSD1680_PARAM_COLOR_RW_INVERSE_BIT);
    } else {
        duc_flag &= (~SSD1680_PARAM_COLOR_BW_INVERSE_BIT);
        duc_flag |= SSD1680_PARAM_COLOR_RW_INVERSE_BIT;
    }
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(epaper_panel->io, SSD1680_CMD_DISP_UPDATE_CTRL, (const uint8_t[]){
                    duc_flag  // Color invert flag
                },
            1),
        TAG, "SSD1680_CMD_DISP_UPDATE_CTRL err");
    // --- Enable refresh done handler isr
    gpio_intr_enable(epaper_panel->busy_gpio_num);
    // --- Send refresh command
    uint8_t init_mode = SSD1680_PARAM_DISP_UPDATE_MODE_2; // default partial refresh
        if(epaper_panel->next_init_mode == INIT_MODE_FULL_1)
            init_mode = SSD1680_PARAM_DISP_UPDATE_MODE_1;
        else if(epaper_panel->next_init_mode == INIT_MODE_FULL_2)
            init_mode = SSD1680_PARAM_DISP_UPDATE_MODE_0;
        else if(epaper_panel->next_init_mode == INIT_MODE_FAST_1)
            init_mode = SSD1680_PARAM_DISP_UPDATE_MODE_3;
        else if(epaper_panel->next_init_mode == INIT_MODE_FAST_2)
            init_mode = SSD1680_PARAM_DISP_UPDATE_MODE_2;
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(epaper_panel->io, SSD1680_CMD_SET_DISP_UPDATE_CTRL, (const uint8_t[]){init_mode}, 1), TAG, "SSD1680_CMD_SET_DISP_UPDATE_CTRL err");

    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(epaper_panel->io, SSD1680_CMD_ACTIVE_DISP_UPDATE_SEQ, NULL, 0), TAG,
                        "SSD1680_CMD_ACTIVE_DISP_UPDATE_SEQ err");

    return ESP_OK;
}

esp_err_t
esp_lcd_new_panel_ssd1680(const esp_lcd_panel_io_handle_t io, const esp_lcd_panel_dev_config_t *const panel_dev_config,
                          esp_lcd_panel_handle_t *const ret_panel) {
#if CONFIG_LCD_ENABLE_DEBUG_LOG
    esp_log_level_set(TAG, ESP_LOG_DEBUG);
#endif
    ESP_RETURN_ON_FALSE(io && panel_dev_config && ret_panel, ESP_ERR_INVALID_ARG, TAG, "1 or more args is NULL");
    esp_lcd_ssd1680_config_t *epaper_ssd1680_conf = panel_dev_config->vendor_config;
    esp_err_t ret = ESP_OK;
    // --- Allocate epaper_panel memory on HEAP
    epaper_panel_t *epaper_panel = NULL;
    epaper_panel = calloc(1, sizeof(epaper_panel_t));
    ESP_GOTO_ON_FALSE(epaper_panel, ESP_ERR_NO_MEM, err, TAG, "no mem for epaper panel");

    // --- Construct panel & implement interface
    // defaults
    epaper_panel->next_init_lut = NULL;
    epaper_panel->next_init_mode = INIT_MODE_FULL_1;
    epaper_panel->next_sleep_mode = SLEEP_MODE_DEEP_1;
    epaper_panel->_invert_color = false;
    epaper_panel->_swap_xy = false;
    epaper_panel->_mirror_x = false;
    epaper_panel->_mirror_y = false;
    epaper_panel->_framebuffer = NULL;
    epaper_panel->gap_x = 0;
    epaper_panel->gap_y = 0;
    epaper_panel->bitmap_color = SSD1680_EPAPER_BITMAP_BLACK;
    epaper_panel->full_refresh = true;
    // configurations
    epaper_panel->io = io;
    epaper_panel->reset_gpio_num = panel_dev_config->reset_gpio_num;
    epaper_panel->busy_gpio_num = epaper_ssd1680_conf->busy_gpio_num;
    epaper_panel->reset_level = panel_dev_config->flags.reset_active_high;
    epaper_panel->_non_copy_mode = epaper_ssd1680_conf->non_copy_mode;
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
    epaper_panel->height = epaper_ssd1680_conf->height;
    epaper_panel->width = epaper_ssd1680_conf->width;
    *ret_panel = &(epaper_panel->base);
    // --- Init framebuffer
    if (!(epaper_panel->_non_copy_mode)) {
        epaper_panel->_framebuffer_size = epaper_ssd1680_conf->buffer_size;
        epaper_panel->_framebuffer = heap_caps_malloc(epaper_ssd1680_conf->buffer_size, MALLOC_CAP_DMA);
        ESP_RETURN_ON_FALSE(epaper_panel->_framebuffer, ESP_ERR_NO_MEM, TAG, "epaper_panel_init allocating buffer memory err");
    }
    size_t img_size = epaper_panel->height * epaper_panel->width / 8;
    // epaper_panel->clearbuffer = epaper_ssd1680_conf->clear_img; // heap_caps_malloc(img_size, MALLOC_CAP_DMA);
    // ESP_RETURN_ON_FALSE(epaper_panel->_clearbuffer, ESP_ERR_NO_MEM, TAG, "epaper_panel_init allocating buffer memory err");
    // memset(epaper_panel->_clearbuffer, 0xFF, img_size);
    // --- Init GPIO
    // init RST GPIO
    if (epaper_panel->reset_gpio_num >= 0) {
        gpio_config_t io_conf = {
            .mode = GPIO_MODE_OUTPUT,
            .pin_bit_mask = 1ULL << panel_dev_config->reset_gpio_num,
        };
        ESP_GOTO_ON_ERROR(gpio_config(&io_conf), err, TAG, "configure GPIO for RST line err");
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
        ESP_GOTO_ON_ERROR(gpio_config(&io_conf), err, TAG, "configure GPIO for BUSY line err");
        ESP_GOTO_ON_ERROR(gpio_isr_handler_add(epaper_panel->busy_gpio_num, epaper_driver_gpio_isr_handler, epaper_panel),
                          err, TAG, "configure GPIO for BUSY line err");
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
        if (epaper_ssd1680_conf->busy_gpio_num >= 0) {
            gpio_reset_pin(epaper_ssd1680_conf->busy_gpio_num);
        }
        free(epaper_panel);
    }
    return ret;
}

static esp_err_t epaper_panel_del(esp_lcd_panel_t *panel) {
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
    ESP_LOGD(TAG, "del ssd1680 epaper panel @%p", epaper_panel);
    free(epaper_panel);
    return ESP_OK;
}

static esp_err_t epaper_panel_reset(esp_lcd_panel_t *panel) {
#if CONFIG_LCD_ENABLE_DEBUG_LOG
    ESP_LOGI(TAG, "epaper_panel_reset");
#endif
    epaper_panel_t *epaper_panel = __containerof(panel, epaper_panel_t, base);
    esp_lcd_panel_io_handle_t io = epaper_panel->io;

    // perform hardware reset
    if (epaper_panel->reset_gpio_num >= 0) {
        ESP_RETURN_ON_ERROR(gpio_set_level(epaper_panel->reset_gpio_num, epaper_panel->reset_level), TAG, "gpio_set_level error");
        vTaskDelay(pdMS_TO_TICKS(10));
        ESP_RETURN_ON_ERROR(gpio_set_level(epaper_panel->reset_gpio_num, !epaper_panel->reset_level), TAG, "gpio_set_level error");
        vTaskDelay(pdMS_TO_TICKS(10));
    } else {
        ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, SSD1680_CMD_SWRST, NULL, 0), TAG, "param SSD1680_CMD_SWRST err");
    }
    panel_epaper_wait_busy(panel);
    return ESP_OK;
}

esp_err_t epaper_panel_set_bitmap_color_ssd1680(esp_lcd_panel_t *panel, esp_lcd_ssd1680_bitmap_color_t color) {
    ESP_RETURN_ON_FALSE(panel, ESP_ERR_INVALID_ARG, TAG, "panel handler is NULL");
    epaper_panel_t *epaper_panel = __containerof(panel, epaper_panel_t, base);
    epaper_panel->bitmap_color = color;
    return ESP_OK;
}

static esp_err_t epaper_set_data_entry_mode(epaper_panel_t *epaper_panel, bool setcursor) {
    uint8_t mode = epaper_panel->_ram_params.ram_mode;
    ESP_LOGD(TAG, "set ram data entry mode, mode: %02x, modestr: %s", mode,
             mode == SSD1680_PARAM_DATA_ENTRY_MODE_0 ? "000" : mode == SSD1680_PARAM_DATA_ENTRY_MODE_1 ? "001"
                                                           : mode == SSD1680_PARAM_DATA_ENTRY_MODE_2   ? "010"
                                                           : mode == SSD1680_PARAM_DATA_ENTRY_MODE_3   ? "011"
                                                           : mode == SSD1680_PARAM_DATA_ENTRY_MODE_4   ? "100"
                                                           : mode == SSD1680_PARAM_DATA_ENTRY_MODE_5   ? "101"
                                                           : mode == SSD1680_PARAM_DATA_ENTRY_MODE_6   ? "110"
                                                           : mode == SSD1680_PARAM_DATA_ENTRY_MODE_7   ? "111"
                                                                                                       : "unknown");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(epaper_panel->io, SSD1680_CMD_DATA_ENTRY_MODE, (const uint8_t[]){mode}, 1), TAG, "SSD1680_CMD_DATA_ENTRY_MODE err");
    // area by cmd 0x44 0x45
    ESP_RETURN_ON_ERROR(set_area(epaper_panel), TAG, "epaper_set_area() error");
    // cursor by cmd 0x4e 0x4f
    if (setcursor) {
        ESP_RETURN_ON_ERROR(set_cursor(epaper_panel), TAG, "epaper_set_cursor() error");
    }
    return ESP_OK;
}

static esp_err_t epaper_panel_init_stage_1(esp_lcd_panel_t *panel) {
    epaper_panel_t *epaper_panel = __containerof(panel, epaper_panel_t, base);
    esp_lcd_panel_io_handle_t io = epaper_panel->io;
    // --- SWRST
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, SSD1680_CMD_SWRST, NULL, 0), TAG, "param SSD1680_CMD_SWRST err");
    panel_epaper_wait_busy(panel);
    // --- Driver Output Control
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(epaper_panel->io, SSD1680_CMD_OUTPUT_CTRL, SSD1680_PARAM_OUTPUT_CTRL, 3), TAG, "SSD1680_CMD_OUTPUT_CTRL err");
    return ESP_OK;
}

static esp_err_t epaper_panel_init_stage_2(esp_lcd_panel_t *panel) {
    epaper_panel_t *epaper_panel = __containerof(panel, epaper_panel_t, base);
    set_ram_params(epaper_panel, 0, 0, epaper_panel->width, epaper_panel->height, 0x03, false);
    // --- Set RAM data entry mode
    ESP_RETURN_ON_ERROR(epaper_set_data_entry_mode(epaper_panel, true), TAG, "pepaper_set_data_entry_mode error");
    return ESP_OK;
}

static esp_err_t epaper_panel_init_stage_3(esp_lcd_panel_t *panel, const uint8_t *lut) {
    epaper_panel_t *epaper_panel = __containerof(panel, epaper_panel_t, base);
    esp_lcd_panel_io_handle_t io = epaper_panel->io;
    // --- Border Waveform Control
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(epaper_panel->io, SSD1680_CMD_SET_BORDER_WAVEFORM, (const uint8_t[]){SSD1680_PARAM_BORDER_WAVEFORM}, 1), TAG, "SSD1680_CMD_SET_BORDER_WAVEFORM err");

    // --- Temperature Sensor Control
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(epaper_panel->io, SSD1680_CMD_SET_TEMP_SENSOR, (const uint8_t[]){SSD1680_PARAM_TEMP_SENSOR}, 1), TAG, "SSD1680_CMD_SET_TEMP_SENSOR err");

    if (lut) {
        ESP_RETURN_ON_ERROR(epaper_set_lut(io, lut), TAG, "epaper_set_lut error");
    } else {
        // --- Load built-in waveform LUT
        uint8_t init_mode = SSD1680_PARAM_DISP_UPDATE_MODE_2; // default partial refresh
        if(epaper_panel->next_init_mode == INIT_MODE_FULL_1)
            init_mode = SSD1680_PARAM_DISP_UPDATE_MODE_1;
        else if(epaper_panel->next_init_mode == INIT_MODE_FULL_2)
            init_mode = SSD1680_PARAM_DISP_UPDATE_MODE_0;
        else if(epaper_panel->next_init_mode == INIT_MODE_FAST_1)
            init_mode = SSD1680_PARAM_DISP_UPDATE_MODE_3;
        else if(epaper_panel->next_init_mode == INIT_MODE_FAST_2)
            init_mode = SSD1680_PARAM_DISP_UPDATE_MODE_2;
        ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(epaper_panel->io, SSD1680_CMD_SET_DISP_UPDATE_CTRL, (const uint8_t[]){init_mode}, 1), TAG, "SSD1680_CMD_SET_DISP_UPDATE_CTRL err");
    }
    // --- Active Display Update Sequence
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, SSD1680_CMD_ACTIVE_DISP_UPDATE_SEQ, NULL, 0), TAG, "param SSD1680_CMD_SET_DISP_UPDATE_CTRL err");
    panel_epaper_wait_busy(panel);
    epaper_panel->next_init_mode = INIT_MODE_PARTIAL;
    return ESP_OK;
}

esp_err_t epaper_panel_set_next_init_mode_ssd1680(esp_lcd_panel_t *panel, epaper_panel_init_mode_t next_init_mode) {
    epaper_panel_t *epaper_panel = __containerof(panel, epaper_panel_t, base);
    epaper_panel->next_init_mode = next_init_mode;
    return ESP_OK;
}

esp_err_t epaper_panel_set_next_sleep_mode_ssd1680(esp_lcd_panel_t *panel, epaper_panel_sleep_mode_t next_sleep_mode) {
    epaper_panel_t *epaper_panel = __containerof(panel, epaper_panel_t, base);
    epaper_panel->next_sleep_mode = next_sleep_mode;
    return ESP_OK;
}

static esp_err_t epaper_panel_init_stage_4(esp_lcd_panel_t *panel, uint8_t color) {
    // --- Set LUT
    epaper_panel_t *epaper_panel = __containerof(panel, epaper_panel_t, base);
    // esp_lcd_panel_io_handle_t io = epaper_panel->io;
    // ESP_RETURN_ON_ERROR(epaper_set_lut(io, SSD1680_LUT_DEFAULT), TAG, "epaper_set_lut error");
    // ESP_RETURN_ON_ERROR(set_cursor(epaper_panel), TAG, "epaper_set_cursor() error");
    // clear screen
    if (!(epaper_panel->_non_copy_mode)) {
        memset(epaper_panel->_framebuffer, color, epaper_panel->_framebuffer_size);
        ESP_RETURN_ON_ERROR(panel_epaper_set_vram(epaper_panel->io, (uint8_t *)(epaper_panel->_framebuffer), (epaper_panel->_framebuffer), epaper_panel->_framebuffer_size),
                            TAG, "panel_epaper_set_vram error");
    }
    return ESP_OK;
}

static esp_err_t epaper_panel_init_stage_5(esp_lcd_panel_t *panel) {
    epaper_panel_t *epaper_panel = __containerof(panel, epaper_panel_t, base);
    esp_lcd_panel_io_handle_t io = epaper_panel->io;
    // // --- Display end option
    // ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(epaper_panel->io, SSD1680_CMD_SET_END_OPTION, (const uint8_t[]) {
    //     SSD1680_PARAM_END_OPTION_KEEP
    // }, 1), TAG, "SSD1681_CMD_SET_END_OPTION err");
    // --- Active Display Update Sequence
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, SSD1680_CMD_ACTIVE_DISP_UPDATE_SEQ, NULL, 0), TAG, "param SSD1681_CMD_SET_DISP_UPDATE_CTRL err");
    panel_epaper_wait_busy(panel);
    return ESP_OK;
}

static esp_err_t epaper_panel_init(esp_lcd_panel_t *panel) {
#if CONFIG_LCD_ENABLE_DEBUG_LOG
    ESP_LOGI(TAG, "epaper_panel_init");
#endif
    epaper_panel_t *epaper_panel = __containerof(panel, epaper_panel_t, base);
    // esp_lcd_panel_io_handle_t io = epaper_panel->io;
    ESP_RETURN_ON_ERROR(epaper_panel_init_stage_1(panel), TAG, "param epaper_panel_init_stage_1 err");
    ESP_RETURN_ON_ERROR(epaper_panel_init_stage_2(panel), TAG, "param epaper_panel_init_stage_2 err");
    ESP_RETURN_ON_ERROR(epaper_panel_init_stage_3(panel, epaper_panel->next_init_lut), TAG, "param epaper_panel_init_stage_3 err");
    ESP_RETURN_ON_ERROR(epaper_panel_init_stage_4(panel, 0xff), TAG, "param epaper_panel_init_stage_4 err");
    // ESP_RETURN_ON_ERROR(epaper_panel_init_stage_5(panel), TAG, "param epaper_panel_init_stage_5 err");
    epaper_panel->next_init_lut = NULL;
    return ESP_OK;
}

esp_err_t epaper_panel_init_screen_ssd1680(esp_lcd_panel_t *panel, epaper_panel_init_mode_t next_init_mode, const uint8_t *lut) {
    epaper_panel_t *epaper_panel = __containerof(panel, epaper_panel_t, base);
    epaper_panel->next_init_mode = next_init_mode;
    epaper_panel->next_init_lut = lut;
    return epaper_panel_init(panel);
}

esp_err_t epaper_panel_clear_screen_ssd1680(esp_lcd_panel_t *panel, uint8_t *color_data, uint8_t color) {
    epaper_panel_t *epaper_panel = __containerof(panel, epaper_panel_t, base);
    ESP_LOGI(TAG, "epaper_panel_clear_screen color: %02x panelw: %hd panelh: %hd", color, epaper_panel->width, epaper_panel->height);
    ram_params_t *p = &(epaper_panel->_ram_params);
    set_ram_params(epaper_panel, 0, 0, epaper_panel->width, epaper_panel->height, 0x03, false);
    // --- Set cursor & data entry sequence
    ESP_RETURN_ON_ERROR(epaper_set_data_entry_mode(epaper_panel, true), TAG, "pepaper_set_data_entry_mode error");
    memset(color_data, color, p->buffer_size);
    ESP_RETURN_ON_ERROR(panel_epaper_set_vram(epaper_panel->io, (uint8_t *)(color_data), (color_data), p->buffer_size),
                        TAG, "panel_epaper_set_vram error");
    // ESP_ERROR_CHECK(epaper_panel_refresh_screen_ssd1680(panel, 0));
    return ESP_OK;
}

static esp_err_t epaper_panel_draw_bitmap(esp_lcd_panel_t *panel, int x_start, int y_start, int x_end, int y_end, const void *color_data) {
    ESP_LOGD("epaper_panel_draw_bitmap", "start bounds: {x:%d, y:%d} -> {x:%d, y:%d}", x_start, y_start, x_end, y_end);
    epaper_panel_t *epaper_panel = __containerof(panel, epaper_panel_t, base);
    if (gpio_get_level(epaper_panel->busy_gpio_num)) {
        return ESP_ERR_NOT_FINISHED;
    }
    // --- Assert & check configuration
    // if (epaper_panel->_non_copy_mode) {
    //     ESP_RETURN_ON_FALSE(!(epaper_panel->_swap_xy), ESP_ERR_INVALID_ARG, TAG, "swap-xy is unavailable when enabling non-copy mode");
    //     ESP_RETURN_ON_FALSE(!(epaper_panel->_mirror_y), ESP_ERR_INVALID_ARG, TAG, "mirror_y is unavailable when enabling non-copy mode");
    // }
    ESP_RETURN_ON_FALSE(color_data, ESP_ERR_INVALID_ARG, TAG, "bitmap is null");
    ESP_RETURN_ON_FALSE((x_start < x_end) && (y_start < y_end), ESP_ERR_INVALID_ARG, TAG, "start position must be smaller than end position");
    // --- Calculate coordinates & sizes
    ram_params_t *p = &(epaper_panel->_ram_params);
    set_ram_params(epaper_panel, x_start, y_start, x_end, y_end, 0x03, epaper_panel->_swap_xy);
    if ((!(epaper_panel->_mirror_x)) && (epaper_panel->_mirror_y)) {
        p->ram_mode = epaper_panel->_swap_xy ? SSD1680_PARAM_DATA_ENTRY_MODE_1 : SSD1680_PARAM_DATA_ENTRY_MODE_1;
    } else if (((epaper_panel->_mirror_x)) && (!(epaper_panel->_mirror_y))) {
        // mirror x, means x is flipped
        p->ram_mode = epaper_panel->_swap_xy ? SSD1680_PARAM_DATA_ENTRY_MODE_2 : SSD1680_PARAM_DATA_ENTRY_MODE_2;
    } else if ((epaper_panel->_mirror_x) && (epaper_panel->_mirror_y)) {
        p->ram_mode = epaper_panel->_swap_xy ? SSD1680_PARAM_DATA_ENTRY_MODE_0 : SSD1680_PARAM_DATA_ENTRY_MODE_0;
    } else {
        p->ram_mode = epaper_panel->_swap_xy ? SSD1680_PARAM_DATA_ENTRY_MODE_3 : SSD1680_PARAM_DATA_ENTRY_MODE_3;
    }
    ESP_LOGD("epaper_panel_draw_bitmap", "converted bounds: {x:%d, y:%d} -> {x:%d, y:%d}", x_start, y_start, x_end, y_end);

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
    ESP_RETURN_ON_ERROR(epaper_set_data_entry_mode(epaper_panel, true), TAG, "pepaper_set_data_entry_mode error");
    // --- Send bitmap to e-Paper VRAM
    if (epaper_panel->bitmap_color == SSD1680_EPAPER_BITMAP_BLACK) {
        ESP_RETURN_ON_ERROR(panel_epaper_set_vram(epaper_panel->io, (uint8_t *)(!epaper_panel->_non_copy_mode ? epaper_panel->_framebuffer : color_data), NULL, p->buffer_size),
                            TAG, "panel_epaper_set_vram error");
    } else if (epaper_panel->bitmap_color == SSD1680_EPAPER_BITMAP_RED) {
        ESP_RETURN_ON_ERROR(panel_epaper_set_vram(epaper_panel->io, NULL, (uint8_t *)(!epaper_panel->_non_copy_mode ? epaper_panel->_framebuffer : color_data), p->buffer_size),
                            TAG, "panel_epaper_set_vram error");
    }
    // --- Refresh the display, show image in VRAM
    // tx_param will wait until DMA transaction finishes, so it is safe to call panel_epaper_refresh_screen at once.
    // The driver will not call the `epaper_panel_refresh_screen` automatically, please call it manually.
    return ESP_OK;
}

static esp_err_t epaper_panel_invert_color(esp_lcd_panel_t *panel, bool invert_color_data) {
    epaper_panel_t *epaper_panel = __containerof(panel, epaper_panel_t, base);
    epaper_panel->_invert_color = invert_color_data;
    return ESP_OK;
}

static esp_err_t epaper_panel_mirror(esp_lcd_panel_t *panel, bool mirror_x, bool mirror_y) {
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
    epaper_panel_t *epaper_panel = __containerof(panel, epaper_panel_t, base);
    epaper_panel->gap_x = x_gap;
    epaper_panel->gap_y = y_gap;
    return ESP_OK;
}

static esp_err_t epaper_panel_disp_on_off(esp_lcd_panel_t *panel, bool on_off) {
    epaper_panel_t *epaper_panel = __containerof(panel, epaper_panel_t, base);
    esp_lcd_panel_io_handle_t io = epaper_panel->io;
    if (on_off) {
        // Turn on display
        ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(epaper_panel->io, SSD1680_CMD_SET_DISP_UPDATE_CTRL, (const uint8_t[]){0xc7}, 1), TAG, "SSD1680_CMD_SET_DISP_UPDATE_CTRL err");
        ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, SSD1680_CMD_ACTIVE_DISP_UPDATE_SEQ, NULL, 0), TAG,
                            "SSD1680_CMD_ACTIVE_DISP_UPDATE_SEQ err");
        panel_epaper_wait_busy(panel);
    } else {
        uint8_t sleep_mode = SSD1680_PARAM_SLEEP_MODE_1;
        if(epaper_panel->next_sleep_mode==SLEEP_MODE_NORMAL) {
            sleep_mode = SSD1680_PARAM_SLEEP_MODE_0;
        }
        else if(epaper_panel->next_sleep_mode==SLEEP_MODE_DEEP_2) {
            sleep_mode = SSD1680_PARAM_SLEEP_MODE_2;
        }
        // Sleep mode, BUSY pin will keep HIGH after entering sleep mode
        // Perform reset and re-run init to resume the display
        ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(epaper_panel->io, SSD1680_CMD_SLEEP_CTRL, (const uint8_t[]){sleep_mode}, 1), TAG, "SSD1680_CMD_SLEEP_CTRL err");
        // BUSY pin will stay HIGH, so do not call panel_epaper_wait_busy() here
        epaper_panel->next_sleep_mode=SLEEP_MODE_DEEP_1;
    }

    return ESP_OK;
}

#define BIT_SET(a, b) ((a) |= (1u << (b)))
#define BIT_CLEAR(a, b) ((a) &= ~(1u << (b)))
#define BIT_FLIP(a, b) ((a) ^= (1u << (b)))
#define BIT_CHECK(a, b) ((a) & (1u << (b)))

void rotate(uint8_t *img, uint8_t *fb, int width, int height, uint16_t rotation) {
    switch (rotation) {
        case 0:
            rotateBitmap(img, fb, width, height, 0);
            break;
        case 90:
            rotateBitmap(img, fb, width, height, 1);
            break;
        case 180:
            rotateBitmap(img, fb, width, height, 2);
            break;
        case 270:
            rotateBitmap(img, fb, width, height, 3);
            break;
        default:
            break;
    }
}

void rotateBitmap(unsigned char *src, unsigned char *dest, int width, int height, unsigned char rotation) {
    if(rotation==0) {
        memcpy(dest, src, width * height);
        return;
    }
    int byteWidth = (width + 7) >> 3u;        // Calculate the width in bytes
    int byteHeight = (height + 7) >> 3u;      // Calculate the height in bytes
    int newWidth = height; // paddedHeight;              // The new width is the old padded height
    int newHeight = width; // paddedWidth;              // The new height is the old padded width
    int totalBytes = (byteWidth << 3u) * (byteHeight << 3u); // Total number of bytes in the src
    int oldByteIndex;
    int newByteIndex;
    int oldBitIndex;
    int newBitIndex;
    ESP_LOGD(TAG, "rotateBitmap w:%d h:%d byteWidth:%d byteHeight:%d newWidth:%d newHeight:%d totalBytes:%d", width, height, byteWidth, byteHeight, newWidth, newHeight, totalBytes);
    memset(dest, 0, totalBytes / 8);
    for (int y = 0; y < height; y++) {  // max 122
        for (int x = 0; x < width; x++) {  // max 250
            oldByteIndex = y * byteWidth + (x >> 3u); // 0-127 * 32 + 0-32
            oldBitIndex = 7u - (x & 7u); // 0-7
            if(rotation == 1){ // cw
                newByteIndex = (((newHeight - 1 - x) * byteHeight) + ((y) >> 3u));
                newBitIndex = y & 7u;
            }
            else if(rotation==2) {
                newByteIndex = ((height - 1 - y) * byteWidth) + ((width - 1 - x) >> 3u);
                newBitIndex = 7u - ((width - 1 - x) & 7u);
            }
            else { // if(rotation == 3){ // ccw
                newByteIndex = (((x) * byteHeight) + ((newWidth - 1 - (y)) >> 3u)); // ((0..249) * 16) + ((122 - 1 - (0..121)) / 8) )
                newBitIndex = 7u - ((newWidth - 1 - (y)) & 7u);
            }
            if (BIT_CHECK(src[oldByteIndex], oldBitIndex)) {  // Check if x and y are within the original width and height
                //newByteIndex--;
                if (newByteIndex >= 0) {
                    BIT_SET(dest[newByteIndex], newBitIndex);
                }
            }
        }
    }
}

static esp_err_t process_bitmap(esp_lcd_panel_t *panel, const void *color_data) {
    epaper_panel_t *epaper_panel = __containerof(panel, epaper_panel_t, base);
    ram_params_t *p = &(epaper_panel->_ram_params);
    ESP_LOGD(TAG, "process_bitmap w:%hd h:%hd buffer_size:%u", p->w, p->h, p->buffer_size);
    ESP_LOGD(TAG, "mirror_x:%d mirror_y:%d swap_xy:%d", epaper_panel->_mirror_x, epaper_panel->_mirror_y, epaper_panel->_swap_xy);
    // --- Convert image according to configuration
    if (!(epaper_panel->_non_copy_mode)) {
        memset(epaper_panel->_framebuffer, 0xff, p->buffer_size);
    }
    if ((!(epaper_panel->_mirror_x)) && (!(epaper_panel->_mirror_y))) {
        // No mirror 03
        if (!(epaper_panel->_non_copy_mode)) {
            if (epaper_panel->_swap_xy) {
                rotateBitmap((uint8_t *)color_data, epaper_panel->_framebuffer, p->h, p->w, 3);
            } else {
                memcpy(epaper_panel->_framebuffer, color_data, p->buffer_size);
            }
        }
    }
    if ((!(epaper_panel->_mirror_x)) && (epaper_panel->_mirror_y)) {
        // Mirror Y 01
        if (epaper_panel->_swap_xy) {
            rotateBitmap((uint8_t *)color_data, epaper_panel->_framebuffer, p->h, p->w, 3);
        } else {
            memcpy(epaper_panel->_framebuffer, color_data, p->buffer_size);
        }
    }
    if (((epaper_panel->_mirror_x)) && (!(epaper_panel->_mirror_y))) {
        // Mirror X 02
        if (!(epaper_panel->_non_copy_mode)) {
            if (epaper_panel->_swap_xy) {  // 006
                rotateBitmap((uint8_t *)color_data, epaper_panel->_framebuffer, p->h, p->w, 3);
            } else {
                memcpy(epaper_panel->_framebuffer, color_data, p->buffer_size);
            }
        }
    }
    if (((epaper_panel->_mirror_x)) && (epaper_panel->_mirror_y)) {
        // Mirror X & Y 00
        if (epaper_panel->_swap_xy) {
            rotateBitmap((uint8_t *)color_data, epaper_panel->_framebuffer, p->h, p->w, 3);
        } else {
            for (uint16_t i = p->buffer_size; i > 0; i--) {
                (epaper_panel->_framebuffer)[i] = ((uint8_t *)(color_data))[i];
            }
        }
    }

    return ESP_OK;
}

static inline uint8_t byte_reverse(uint8_t data) {
    static uint8_t _4bit_reverse_lut[] = {
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

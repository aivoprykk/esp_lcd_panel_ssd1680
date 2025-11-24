#include "private.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_log.h"

static const char *TAG = "demo_epaper_lvgl";

# define SPIx_HOST SPI3_HOST

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////// Please update the following configuration according to your LCD spec //////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define LCD_PIXEL_CLOCK_HZ               1000000
#define LCD_PANEL_PIN_NUM_SCLK           18
#define LCD_PANEL_PIN_NUM_MOSI           23
#define LCD_PANEL_PIN_NUM_MISO           (-1)   // Unused
#define LCD_PANEL_PIN_NUM_DC         17
#define LCD_PANEL_PIN_NUM_RST        16
#define LCD_PANEL_PIN_NUM_CS         5
#define LCD_PANEL_PIN_NUM_BUSY       4

// Bit number used to represent command and parameter
#define LCD_CMD_BITS           8
#define LCD_PARAM_BITS         8

#define TIMEOUT_IMMEDIATE 0
#define TIMEOUT_MAX portMAX_DELAY
static const TickType_t timeout_100ms = pdMS_TO_TICKS(100);

display_driver_t drv = {
    .lv_mem_buf = {0},
    .is_initialized_lvgl = false,
    .lv_disp = NULL,
    .sem = 0
};
#ifdef CONFIG_SSD168X_PANEL_SSD1681
#include "ssd1681_waveshare_1in54_lut.h"
#else
#include "ssd1680_waveshare_2in13_lut.h"
#endif

static esp_lcd_panel_handle_t panel_handle = NULL;
static esp_lcd_panel_io_handle_t io_handle = NULL;
#ifdef CONFIG_SSD168X_PANEL_SSD1681
static uint8_t fast_refresh_lut[] = SSD1681_WAVESHARE_1IN54_V2_LUT_FAST_REFRESH_KEEP;
#else
static uint8_t fast_refresh_lut[] = SSD1680_WAVESHARE_2IN13_V2_LUT_FAST_REFRESH_O;
#endif
uint32_t flush_count = 0;
static bool init_requested = true;
uint32_t last_flush_ms = 0;
static epaper_panel_init_mode_t init_mode = INIT_MODE_FULL_2;
unsigned long IRAM_ATTR get_millis() { return (unsigned long) (esp_timer_get_time() / 1000); }
#define DELAY_MS(x) vTaskDelay((x + (portTICK_PERIOD_MS - 1)) / portTICK_PERIOD_MS)
inline void delay_ms(uint32_t ms) { DELAY_MS(ms); }

static int8_t rotation = DISP_ROT_270;

void print_lv_mem_mon() {
    lv_mem_monitor_t mon;
    lv_mem_monitor(&mon);
#if LVGL_VERSION_MAJOR < 9
    printf("used: %6lu (%3hhu %%), frag: %3hhu %%, biggest free: %6d\n", mon.total_size - mon.free_size,
#else
    printf("used: %6u (%3u %%), frag: %3u %%, biggest free: %6d\n", mon.total_size - mon.free_size,
#endif
            mon.used_pct,
            mon.frag_pct,
            (int)mon.free_biggest_size);
}

static bool lock(int timeout_ms) {
    // Convert timeout in milliseconds to FreeRTOS ticks
    // If `timeout_ms` is set to -1, the program will block until the condition is met
    if(!drv.sem)
        return true;
        
    TickType_t timeout_ticks;
    // Optimize common timeout values
    if (timeout_ms == -1) {
        timeout_ticks = TIMEOUT_MAX;
    } else if (timeout_ms == 0) {
        timeout_ticks = TIMEOUT_IMMEDIATE;
    } else if (timeout_ms == 100) {
        timeout_ticks = timeout_100ms;
    } else {
        timeout_ticks = pdMS_TO_TICKS(timeout_ms);
    }
    
    return xSemaphoreTake(drv.sem, timeout_ticks) == pdTRUE;
}

static void unlock(void) {
    if(drv.sem)
        xSemaphoreGive(drv.sem);
}

IRAM_ATTR bool _flush_ready_callback(const esp_lcd_panel_handle_t handle, const void *edata, void *user_data) {
    if(user_data) {
// #ifdef CONFIG_DISPLAY_USE_LVGL
        FLUSH_READY_CB(user_data);
// #endif
    }
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if(drv.sem)
        xSemaphoreGiveFromISR(drv.sem, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken == pdTRUE) {
        return true;
    }
    return false;
}

// static void clearScreen(esp_lcd_panel_handle_t panel_handle, uint8_t * color_data)
// {
//     xSemaphoreTake(drv.sem, portMAX_DELAY);
//     if(epaper_panel_clear_screen_ssd168x(panel_handle, color_data, 0xff)) {
//         ESP_LOGE(TAG, "Failed to clear screen");
//     }
// }

// #define BIT_SET(a, b)       ((a) |= (1u << (b)))
// #define BIT_CLEAR(a, b)     ((a) &= ~(1u << (b)))

// /* omitted irrelevant code */
// void my_set_px_cb(lv_disp_drv_t * disp_drv, uint8_t * buf, lv_coord_t buf_w, lv_coord_t x, lv_coord_t y, lv_color_t color, lv_opa_t opa)
// {
//     // printf("x: %d, y: %d, color: %d\n", x, y, color.full);
//     uint16_t byte_index = (x >> 3u) + (y * LCD_ROW_LEN);
//     uint8_t bit_index = x & 0x07u;

//     if (color.full) {
//         BIT_SET(buf[byte_index], 7 - bit_index);
//     } else {
//         BIT_CLEAR(buf[byte_index], 7 - bit_index);
//     }
// }

int display_drv_set_rotation(int8_t rot) {
    ESP_LOGW(TAG, "[%s] %d", __func__, rot);
    if(rot > DISP_ROT_270)
        return ESP_ERR_INVALID_ARG;
    if(rot == rotation)
        return ESP_OK;
    DISPLAY_SET_ROTATION(drv.lv_disp, rot);
    if(lv_scr_act()) {
        lv_obj_invalidate(lv_scr_act());
    }
    ESP_LOGD(TAG, "[%s] New orientation is %d:, rotated flag is :%d, hor_res is: %d, ver_res is: %d", __func__,
        (int)rot, DISPLAY_GET_ROTATION(), DISPLAY_GET_HOR_RES(), DISPLAY_GET_VER_RES()
    );
    if(lock(portMAX_DELAY)) {
        rotation = rot;
        unlock();
    }
    return ESP_OK;
}

int8_t display_drv_get_rotation() {
    return rotation;
}

static esp_err_t _turn_off(esp_lcd_panel_handle_t panel_handle) {
    ESP_LOGI(TAG, "[%s]", __func__);
    if(epaper_panel_shut_down(panel_handle)) {
        return ESP_FAIL;
    }
    return ESP_OK;
}

static esp_err_t _turn_on(esp_lcd_panel_handle_t panel_handle) {
    ESP_LOGI(TAG, "[%s]", __func__);
    const char *x = "e-Paper display...";
    // UNUSED_PARAMETER(x);
    if(init_requested) {
        ESP_LOGI(TAG, "[%s] %s %s with init mode 0x%02x", "Reset/Init", __func__, x, init_mode);
        if(flush_count > 0) {
            if(_turn_off(panel_handle)) {
                return ESP_FAIL;
            }
        }
        if(esp_lcd_panel_reset(panel_handle)) {
            return ESP_FAIL;
        }
        delay_ms(50);
        if(epaper_panel_init_screen_ssd168x(panel_handle, init_mode, 0)) {
            return ESP_FAIL;
        }
        init_mode = INIT_MODE_FULL_2;
        delay_ms(50);
    }
    if(esp_lcd_panel_disp_on_off(panel_handle, true)) {
        return ESP_FAIL;
    }
    if(!init_requested) {
        ESP_LOGI(TAG, "[%s] %s %s", "Refresh", __func__, x);
        if(epaper_panel_set_custom_lut_ssd168x(panel_handle, fast_refresh_lut, 159)) {
            return ESP_FAIL;
        }
    }
    else {
        init_requested = false;
    }
    return ESP_OK;
}

static esp_err_t _refresh_and_turn_off(esp_lcd_panel_handle_t panel_handle, int rotated, m_area_t *area, uint8_t *color_map) {
    ESP_LOGI(TAG, "[%s]", __func__);
    if(rotated == DISP_ROT_NONE || rotated == DISP_ROT_180) {
        if(esp_lcd_panel_swap_xy(panel_handle, false)) {
            return ESP_FAIL;
        }
    }
    else {
        if(esp_lcd_panel_swap_xy(panel_handle, true)) {
            return ESP_FAIL;
        }
    }
// #ifdef CONFIG_DISPLAY_USE_LVGL
            // if(esp_lcd_panel_invert_color(panel_handle, true)) {
            //     return ESP_FAIL;
            // }
// #endif

    if(rotated == DISP_ROT_NONE) {
        if(esp_lcd_panel_mirror(panel_handle, false, false)) {
            return ESP_FAIL;
        } // x inc y inc
    }
    else if(rotated == DISP_ROT_90) {
        if(esp_lcd_panel_mirror(panel_handle, true, false)) {
            return ESP_FAIL;
        } // x dec y inc
    }
    else if(rotated == DISP_ROT_180) {
        if(esp_lcd_panel_mirror(panel_handle, true, true)) {
            return ESP_FAIL;
        } // x dec y dec
    }
    else { // DISP_ROT_270 (270 degrees)
        if(esp_lcd_panel_mirror(panel_handle, false, true)) {
            return ESP_FAIL;
        } // x inc y dec
    }

    if(epaper_panel_set_bitmap_color_ssd168x(panel_handle, SSD168X_EPAPER_BITMAP_BLACK)) {
        return ESP_FAIL;
    }
    if(esp_lcd_panel_draw_bitmap(panel_handle,  area->x1,  area->y1, area->x2 + 1, area->y2 + 1, color_map)) {
        return ESP_FAIL;
    }
    if(epaper_panel_refresh_screen_ssd168x(panel_handle, 0xcf)) {
        return ESP_FAIL;
    }
    // if(epaper_panel_set_bitmap_color_ssd168x(panel_handle, SSD168X_EPAPER_BITMAP_RED)) {
    //     return ESP_FAIL;
    // }
    // if(esp_lcd_panel_draw_bitmap(panel_handle,  area->x1,  area->y1, area->x2 + 1, area->y2 + 1, color_map)) {
    //     return ESP_FAIL;
    // }
    if(epaper_panel_update_full_screen_ssd168x(panel_handle)) {
        return ESP_FAIL;
    }
    // if(esp_lcd_panel_disp_on_off(panel_handle, false)) {
    //         return ESP_FAIL;
    // }
    return ESP_OK;
}

// Apply 6-bit shift to the entire buffer after rotation
static void apply_bit_shift(uint8_t *buffer, int width, int height, int shift_bits) {
    if (shift_bits == 0) return;
    int abs_shift = abs(shift_bits);
    if (abs_shift <= 0 || abs_shift >= 8) return;
    int bytes_per_row = (width + 7) / 8;
    
    for (int row = 0; row < height; row++) {
        uint8_t carry = 0;
        for (int col = 0; col < bytes_per_row; col++) {
            int idx = row * bytes_per_row + col;
            uint8_t current = buffer[idx];
            uint8_t new_byte;
            
            if (shift_bits > 0) {
                // RIGHT shift
                new_byte = (current >> abs_shift) | (carry << (8 - abs_shift));
                carry = current & ((1 << abs_shift) - 1); // Save bottom bits
            } else {
                // LEFT shift  
                new_byte = (current << abs_shift) | carry;
                carry = current >> (8 - abs_shift); // Save top bits
            }
            
            buffer[idx] = new_byte;
        }
    }
}

// Stage 1: Pixel conversion with rotation
static uint8_t lvgl_pixel_convert_cb(const unsigned char *src_data, esp_lcd_ssd168x_area_t src_area, int rotation, int data_format, void *user_data) {
    // Calculate simple linear pixel index - no rotation adjustments needed here
    // The rotate_bitmap function handles all coordinate transformations
    int pixel_idx = src_area.y1 * src_area.x2 + src_area.x1;
    // int rotation = *(int*)user_data; // Get rotation value from user_data
    
    uint8_t pixel_value = 0;
    
    // Convert pixel based on LVGL format
    if (data_format == 1) {
        // LVGL v8: color array
#if (LVGL_VERSION_MAJOR < 9)
        lv_color_t *colors = (lv_color_t *)src_data;
        pixel_value = (lv_color_brightness(colors[pixel_idx]) < 128) ? 1 : 0;
#else
        pixel_value = 0; // Fallback for LVGL v9 with wrong format
#endif
    } else if (data_format == 2) {
        // LVGL v9: 1-bit packed data
        pixel_value = (src_data[pixel_idx / 8] >> (7 - (pixel_idx % 8))) & 1;
    }
    
    return pixel_value;
}

// Streamlined flush callback using callback-based rotate_bitmap
#if (LVGL_VERSION_MAJOR < 9)
#define MYINT int
#define MYINT_D "d"
static void _lvgl_flush_cb(lv_disp_drv_t *dspl, const lv_area_t *area, lv_color_t *color_map)
#else  
#define MYINT int32_t
#define MYINT_D PRId32
static void _lvgl_flush_cb(lv_display_t *dspl, const lv_area_t *area, uint8_t *color_map)
#endif
{
    ESP_LOGI(TAG, "[%s] x1:%"MYINT_D" y1:%"MYINT_D", x2:%"MYINT_D" y2:%"MYINT_D"", __func__, 
         area->x1, area->y1, area->x2, area->y2);
    
    esp_lcd_panel_handle_t panel_handle = GET_USER_DATA(dspl);
    
    MYINT offsetx1 = area->x1;
    MYINT offsetx2 = area->x2;
    MYINT offsety1 = area->y1;
    MYINT offsety2 = area->y2;
    
    // Calculate area dimensions
    MYINT len_x = abs(offsetx1 - offsetx2) + 1;
    MYINT len_y = abs(offsety1 - offsety2) + 1;
    int rotated = display_drv_get_rotation();
    
    // Adjust dimensions for SSD1680 padding requirements
// #if defined(CONFIG_SSD168X_PANEL_SSD1680)
//     if(rotated == DISP_ROT_270 || rotated == DISP_ROT_90) {
//         len_y = ROUND_UP_TO_8(len_y);
//     } else {
//         len_x = ROUND_UP_TO_8(len_x);
//     }
// #endif
    
    // Get buffer for e-paper display
    uint8_t *converted_buffer_black = drv.lv_mem_buf[LV_DRAW_BUF_SZ];
    
    // Determine data format and prepare source data
    int data_format = 0;
    unsigned char *src_data = (unsigned char *)color_map;
    
#if (LVGL_VERSION_MAJOR < 9)
    // LVGL v8: color array
    data_format = 1;
#else
    // LVGL v9: handle 1-bit packed data
    data_format = 2;
    if (LV_COLOR_DEPTH == 1) {
        src_data = color_map + 8; // Skip header for 1-bit data
    }
#endif
    
    // Convert LVGL rotation to driver rotation constants
    // Note: Driver rotation values are counter-clockwise: 1=270°, 3=90°
    unsigned char driver_rotation = 0;
    switch(rotated) {
        case DISP_ROT_90:   driver_rotation = 3; break;  // 90° CW = driver 3
        case DISP_ROT_180:  driver_rotation = 2; break;  // 180° = driver 2
        case DISP_ROT_270:  driver_rotation = 1; break;  // 270° CW = driver 1  
        default:            driver_rotation = 0; break;  // 0° = driver 0
    }
    
    // Use rotate_bitmap with LVGL conversion callback
    rotate_bitmap(src_data, converted_buffer_black, 
                 len_x, len_y, driver_rotation, 
                 lvgl_pixel_convert_cb, data_format, 0);
    
// #if defined(CONFIG_SSD168X_PANEL_SSD1680)
    // Apply SSD1680-specific 6-bit vertical shift compensation using helper function
    // apply_ssd1680_hardware_shift(converted_buffer_black, drv.lv_mem_buf_size[LV_DRAW_BUF_SZ], rotation);
    // apply_bit_shift(converted_buffer_black, len_x, len_y, 2); // Shift right by 6 bits
// #endif
    
    // Adjust area coordinates for rotated buffer dimensions
    MYINT final_offsetx1 = offsetx1;
    MYINT final_offsetx2 = offsetx2; 
    MYINT final_offsety1 = offsety1;
    MYINT final_offsety2 = offsety2;
    
    if(driver_rotation == 1 || driver_rotation == 3) { // 270° or 90° 
        // Dimensions are swapped, so adjust area to match rotated buffer
        final_offsetx1 = offsety1;
        final_offsetx2 = offsety2;
        final_offsety1 = offsetx1; 
        final_offsety2 = offsetx2;
    }

    ESP_LOGI(TAG, "[%s] processed %"MYINT_D"x%"MYINT_D" buffer, rotation=%d", __func__, len_x, len_y, rotated);
    
    // Handle display refresh - always use DISP_ROT_NONE to prevent double rotation
    // (software rotation already applied by rotate_bitmap above)
    bool needs_init = init_requested;
    if(needs_init) {
        _turn_on(panel_handle);
        _refresh_and_turn_off(panel_handle, DISP_ROT_NONE, 
                             &((m_area_t){final_offsetx1, final_offsety1, final_offsetx2, final_offsety2}), 
                             converted_buffer_black);
        _turn_on(panel_handle);
    }
    
    // Main refresh with software-rotated buffer (no hardware rotation)
    _refresh_and_turn_off(panel_handle, DISP_ROT_NONE, 
                         &((m_area_t){final_offsetx1, final_offsety1, final_offsetx2, final_offsety2}), 
                         converted_buffer_black);
    
    // Update statistics and notify completion
    flush_count++;
    last_flush_ms = get_millis();
    // esp_event_post(UI_EVENT, UI_EVENT_FLUSH_DONE, 0, 0, portMAX_DELAY);
}

static void _increase_lvgl_tick(void *arg)
{
    /* Tell LVGL how many milliseconds has elapsed */
    lv_tick_inc(LVGL_TICK_PERIOD_MS);
}

#if (LVGL_VERSION_MAJOR < 9)
static void _lvgl_wait_cb(struct _lv_disp_drv_t *disp_drv)
#else
static void _lvgl_wait_cb(lv_display_t *disp)
#endif
{
    if(drv.sem)
        xSemaphoreTake(drv.sem, portMAX_DELAY);
}

static void _init_cb(void *dsp) {

#if (LVGL_VERSION_MAJOR < 9)
    lv_disp_drv_t *disp_drv = (lv_disp_drv_t *)dsp;
    // Set the callback functions
    disp_drv->hor_res = LCD_H_VISIBLE;
    disp_drv->ver_res = LCD_V_VISIBLE;
    disp_drv->rotated = rotation;
    // NOTE: The ssd168x e-paper is monochrome and 1 byte represents 8 pixels
    // so full_refresh is MANDATORY because we cannot set position to bitmap at pixel level
    disp_drv->full_refresh = 0;
    disp_drv->direct_mode = 1;
    disp_drv->sw_rotate = 0;
    disp_drv->user_data = panel_handle;
    // alloc bitmap buffer to draw
    disp_drv->flush_cb = _lvgl_flush_cb;
    disp_drv->wait_cb = _lvgl_wait_cb;
    // disp_drv->set_px_cb = set_px_cb;
    // disp_drv->drv_update_cb = epaper_lvgl_port_update_callback;
#else
    lv_display_t *disp = (lv_display_t *)dsp;
    lv_display_drv_set_rotation(disp, rotation);
    // NOTE: The ssd168x e-paper is monochrome and 1 byte represents 8 pixels
    // so full_refresh is MANDATORY because we cannot set position to bitmap at pixel level
    // lv_disp_set_full_refresh(disp, true);
    // lv_disp_set_direct_mode(disp, 1);
    // lv_disp_set_sw_rotate(disp, false);
    lv_display_set_user_data(disp, panel_handle);
    lv_display_set_flush_cb(disp, _lvgl_flush_cb);
    lv_display_set_flush_wait_cb(disp, _lvgl_wait_cb);
    // lv_display_set_color_format(disp, LV_COLOR_FORMAT_I8);
#endif
}

esp_err_t init_draw_buffers(size_t lvbuf, uint8_t lvbuf_num, size_t convbuf, uint8_t convbuf_num) {
    size_t bufsz;
    esp_err_t ret = ESP_OK;
    for (uint8_t i=0, j=lvbuf_num + convbuf_num; i < j; i++) {
        bufsz = i < lvbuf_num ? lvbuf : convbuf;
        ESP_LOGW(TAG, "Allocate %dKb memory for buf %d" , (bufsz>>10), i);
		drv.lv_mem_buf[i] = heap_caps_malloc(bufsz, MALLOC_CAP_DMA);
		if(drv.lv_mem_buf[i] == NULL) {
            drv.lv_mem_buf_size[i] = 0;
            ESP_LOGE(TAG, "[%s] Failed to allocate memory for buffer %d", __func__, i);
            ret = ESP_ERR_NO_MEM;
        }
        else {
            drv.lv_mem_buf_size[i] = bufsz;
        }
	}
    return ret;
}

#define TO_K_UL(x) ((x) * 1000UL)
#define MS_TO_US(x) TO_K_UL(x)

void init_lv_screen(void (*cb)(void *)) {
    ESP_LOGI(TAG, "Initialize LVGL library");
    lv_init();
    size_t bufsz = LBUFSZ;
    init_draw_buffers(LBUFSZ, LV_DRAW_BUF_SZ, LCD_PIXELS_MEM_ALIGNED, CONV_BUF_SZ);
    uint8_t *buf[2] = {drv.lv_mem_buf[0], (LV_DRAW_BUF_SZ > 1 ? drv.lv_mem_buf[1] : NULL)};
    ESP_LOGI(TAG, "Register display driver / create display to LVGL");
#if (LVGL_VERSION_MAJOR < 9)
    lv_disp_draw_buf_init(&drv.disp_buf, buf[0], buf[1], bufsz);
    lv_disp_drv_init(&drv.disp_drv);
    drv.disp_drv.draw_buf = &drv.disp_buf;
    cb(&drv.disp_drv);
    drv.lv_disp = lv_disp_drv_register(&drv.disp_drv);
#else
    drv.lv_disp = lv_display_create(LCD_H_RES, LCD_V_RES);
    cb(drv.lv_disp);
    lv_display_set_buffers(drv.lv_disp, buf[0], buf[1], bufsz, LV_DISPLAY_RENDER_MODE_FULL);
#endif
    drv.is_initialized_lvgl = true;

    // init lvgl tick
    ESP_LOGI(TAG, "Install LVGL tick timer");
    const esp_timer_create_args_t lvgl_tick_timer_args = {
        .callback = &_increase_lvgl_tick,
        .name = "lvgl_tick"
    };
    esp_timer_handle_t lvgl_tick_timer = NULL;
    if(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer)) {
        return;
    }
    if(esp_timer_start_periodic(lvgl_tick_timer, MS_TO_US(LVGL_TICK_PERIOD_MS))) {
        return;
    }
    ESP_LOGI(TAG, "LVGL refr timer stop");
    lv_disp_t * disp = lv_disp_get_default();
    lv_timer_del(disp->refr_timer);
    disp->refr_timer = NULL;
}

static void _d_init() {
    ESP_LOGI(TAG, "[%s]", __func__);
#if (LVGL_VERSION_MAJOR < 9)
    if(drv.disp_drv.user_data == NULL)
#else
    if(lv_display_get_user_data(drv.lv_disp) == NULL)
#endif
    {
        init_lv_screen(_init_cb);
        // --- Register the e-Paper refresh done callback
        epaper_panel_callbacks_t cbs = {
            .on_epaper_refresh_done = _flush_ready_callback
        };
#if (LVGL_VERSION_MAJOR < 9)
        epaper_panel_register_event_callbacks_ssd168x(panel_handle, &cbs, &drv.disp_drv);
#else
        epaper_panel_register_event_callbacks_ssd168x(panel_handle, &cbs, drv.lv_disp);
#endif
    }
}

void app_main(void)
{

    drv.sem = xSemaphoreCreateBinary();
    xSemaphoreGive(drv.sem);

    ESP_LOGI(TAG, "Initialize SPI bus");
    spi_bus_config_t buscfg = {
        .sclk_io_num = LCD_PANEL_PIN_NUM_SCLK,
        .mosi_io_num = LCD_PANEL_PIN_NUM_MOSI,
        .miso_io_num = -1,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = LCD_RESOLUTION,
    };
    if(spi_bus_initialize(SPIx_HOST, &buscfg, SPI_DMA_CH_AUTO)) {
        ESP_LOGE(TAG, "Failed to initialize bus");
    }

    ESP_LOGI(TAG, "Install panel IO");
    esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = LCD_PANEL_PIN_NUM_DC,
        .cs_gpio_num = LCD_PANEL_PIN_NUM_CS,
        .pclk_hz = LCD_PIXEL_CLOCK_HZ,
        .lcd_cmd_bits = LCD_CMD_BITS,
        .lcd_param_bits = LCD_PARAM_BITS,
        .spi_mode = 0,
        .trans_queue_depth = 10,
        .on_color_trans_done = NULL,
    };
    // --- Attach the LCD to the SPI bus
    if(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t) SPIx_HOST, &io_config, &io_handle)) {
        ESP_LOGE(TAG, "Failed to create panel io");
    }

    // --- Create esp_lcd panel
    esp_lcd_ssd168x_config_t lcd_ssd168x_config = {
        .busy_gpio_num = LCD_PANEL_PIN_NUM_BUSY,
        .non_copy_mode = true,
        .height = LCD_V_RES,
        .width = LCD_H_RES,
        .buffer_size = LCD_PIXELS_MEM_ALIGNED,
        // .clear_img = clear_img,
    };
    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = LCD_PANEL_PIN_NUM_RST,
        .flags.reset_active_high = false,
        .vendor_config = &lcd_ssd168x_config
    };
    gpio_install_isr_service(0);
    if(esp_lcd_new_panel_ssd168x(io_handle, &panel_config, &panel_handle)) {
        ESP_LOGE(TAG, "Failed to create panel");
    }

#if (LCD_H_GAP > 0) || (LCD_V_GAP > 0)
    esp_lcd_panel_set_gap(panel_handle, LCD_H_GAP, LCD_V_GAP);
#endif

    _d_init();

    ui_demo();
    
    while (1) {
        // printf("LVGL tick handler running...\n");
        // raise the task priority of LVGL and/or reduce the handler period can improve the performance
        // The task running lv_timer_handler should have lower priority than that running `lv_tick_inc`
        lv_timer_handler();
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

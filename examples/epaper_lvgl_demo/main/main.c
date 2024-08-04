/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <esp_lcd_panel_ssd1680.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "lvgl.h"

#include "logger_common.h"
#include "driver_vendor.h"

#include "ssd1680_waveshare_2in13_lut.h"

static const char *TAG = "example";

// Using SPI2 in the example
#define LCD_HOST  SPI2_HOST

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////// Please update the following configuration according to your LCD spec //////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define LCD_PIXEL_CLOCK_HZ     1000000 // (20 * 1000 * 1000)
#define LCD_PANEL_PIN_NUM_SCLK           18
#define LCD_PANEL_PIN_NUM_MOSI           23
#define LCD_PANEL_PIN_NUM_MISO           (-1)   // Unused
#define LCD_PANEL_PIN_NUM_EPD_DC         17
#define LCD_PANEL_PIN_NUM_EPD_RST        16
#define LCD_PANEL_PIN_NUM_EPD_CS         5
#define LCD_PANEL_PIN_NUM_EPD_BUSY       4

// The pixel number in horizontal and vertical
// screen
// #define LCD_H_GAP 0
// #define LCD_V_GAP 0
// #define LCD_H_RES              128 // screen native x resolution
// #define LCD_H_VISIBLE          122 // screen visible x resolution
// #define LCD_V_RES              250 // screen native y resolution
// #define LCD_RESOLUTION  (LCD_H_RES * LCD_V_RES)
// #define LCD_ROW_LEN     (LCD_H_RES / 8)           // gates for x resolution
// #define LCD_PIXELS      (LCD_V_RES * LCD_ROW_LEN) // total pixels
// #define BYTE_PADDING(w) (((w + 7u) >> 3u) << 3u) // Align to nearest 8 bits
// #define LCD_PIXELS_MEM_ALIGNED (LCD_H_RES * BYTE_PADDING(LCD_V_RES) / 8)
// driver
#define SSD1680_MEM_SOURCE_SIZE   176   // driver native source size
#define SSD1680_MEM_GATE_SIZE     296   // driver native gate size
#define SSD1680_MEM_SOURCE_LEN    (SSD1680_MEM_SOURCE_SIZE / 8)
#define SSD1680_MEM_SIZE (SSD1680_MEM_GATE_SIZE * SSD1680_MEM_SOURCE_LEN)
// Bit number used to represent command and parameter
#define LCD_CMD_BITS           8
#define LCD_PARAM_BITS         8

#define LVGL_TICK_PERIOD_MS    1

static SemaphoreHandle_t panel_refreshing_sem = NULL;

extern void example_lvgl_demo_ui(lv_disp_t *disp);
extern void ui_demo(void);
extern lv_obj_t *ui_InfoScreen;
static lv_disp_draw_buf_t disp_buf; // contains internal graphic buffer(s) called draw buffer(s)
static lv_disp_drv_t disp_drv;      // contains callback functions

bool _lvgl_lock(int timeout_ms) {
    // Convert timeout in milliseconds to FreeRTOS ticks
    // If `timeout_ms` is set to -1, the program will block until the condition is met
    const TickType_t timeout_ticks = (timeout_ms == -1) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    return xSemaphoreTake(panel_refreshing_sem, timeout_ticks) == pdTRUE;
}

void _lvgl_unlock(void) {
    xSemaphoreGive(panel_refreshing_sem);
}

IRAM_ATTR bool epaper_flush_ready_callback(const esp_lcd_panel_handle_t handle, const void *edata, void *user_data)
{
    lv_disp_drv_t *disp_driver = (lv_disp_drv_t *) user_data;
    lv_disp_flush_ready(disp_driver);
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(panel_refreshing_sem, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken == pdTRUE) {
        return true;
    }
    return false;
}

static const uint8_t fast_refresh_lut[] = SSD1680_WAVESHARE_2IN13_V2_LUT_FAST_REFRESH_KEEP_1;
static uint32_t update_count = 0;
static uint8_t *converted_buffer_black;
// static uint8_t *converted_buffer_red;
uint8_t *empty_bitmap;

static void clearScreen(esp_lcd_panel_handle_t panel_handle, uint8_t * color_data)
{
    MEAS_START();
    xSemaphoreTake(panel_refreshing_sem, portMAX_DELAY);
    //ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));
    // if(update_count++>0)
    //     ESP_ERROR_CHECK(epaper_panel_set_custom_lut_ssd1680(panel_handle, fast_refresh_lut, 159));
    ESP_ERROR_CHECK(epaper_panel_clear_screen_ssd1680(panel_handle, color_data, 0xff));
    // ESP_ERROR_CHECK(epaper_panel_refresh_screen_ssd1680(panel_handle, 0));
    // ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, false));
    MEAS_END(TAG, "[%s] took %llu us",__func__);
}

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

static void example_lvgl_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map)
{
    MEAS_START();
    esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t) drv->user_data;
    if(update_count==0 || update_count==1000){
        ESP_LOGI(TAG, "Resetting e-Paper display...");
        ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
        delay_ms(100);
        //ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
        ESP_ERROR_CHECK(epaper_panel_init_screen_ssd1680(panel_handle, INIT_MODE_FULL_1, 0));
        delay_ms(100);
    }
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));
    if(update_count!=0 && update_count!=1000) {
        ESP_LOGI(TAG, "Refreshing e-Paper display...");
        ESP_ERROR_CHECK(epaper_panel_set_custom_lut_ssd1680(panel_handle, fast_refresh_lut, PARTIAL_UPDATE_LUT_SIZE));
    }

    int offsetx1 = area->x1;
    int offsetx2 = area->x2;
    int offsety1 = area->y1;
    int offsety2 = area->y2;
    // Used to vertical traverse lvgl framebuffer
    int len_x = abs(offsetx1 - offsetx2) + 1;
    int len_y = abs(offsety1 - offsety2) + 1;
    // --- Convert buffer from color to monochrome bitmap
    int len_bits = len_x * len_y;

    ESP_LOGI(TAG, "Flushing area: x1=%d, y1=%d, x2=%d, y2=%d, len_x=%d, len_y=%d, len_bits=%d", offsetx1, offsety1, offsetx2, offsety2, len_x, len_y, len_bits);
    memset(converted_buffer_black, 0x00, ((len_bits + 7) / 8));

    for (int i = 0; i < len_bits; i++) {
        // NOTE: Set bits of converted_buffer[] FROM LOW ADDR TO HIGH ADDR, FROM HSB TO LSB
        // NOTE: 1 means BLACK/RED, 0 means WHITE
        if(drv->rotated == 1) {
        // Vertical traverse lvgl framebuffer (by column), needs to uncomment len_x and len_y
        // NOTE: If your screen rotation requires setting the pixels vertically, you could use the code below
          converted_buffer_black[i / 8] |= (((lv_color_brightness(color_map[((i*len_x)%len_bits) + i/len_y])) > 250) << (7 - (i % 8)));
        }
        else{
        // Horizontal traverse lvgl framebuffer (by row)
          converted_buffer_black[i / 8] |= (((lv_color_brightness(color_map[i])) <= 250) << (7 - (i % 8)));
        }
    }
     
    // --- Draw bitmap
    ESP_LOGI(TAG, "Drawing e-Paper display, update_count=%lu ...", update_count);
    if(drv->rotated == 1){
        ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(panel_handle, true));
        ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_handle, false, true));
        ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_handle, true));
    }
    else
        ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(panel_handle, false));

    ESP_ERROR_CHECK(epaper_panel_set_bitmap_color_ssd1680(panel_handle, SSD1680_EPAPER_BITMAP_BLACK));
    ESP_ERROR_CHECK(esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, converted_buffer_black));
    ESP_ERROR_CHECK(epaper_panel_set_bitmap_color_ssd1680(panel_handle, SSD1680_EPAPER_BITMAP_RED));
    ESP_ERROR_CHECK(esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, converted_buffer_black));
    ESP_ERROR_CHECK(epaper_panel_refresh_screen_ssd1680(panel_handle, 0));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, false));
    if(update_count++>15) 
        update_count=0;
    
    lv_disp_flush_ready(drv);
    MEAS_END(TAG, "[%s] took %llu us",__func__);
}

static void example_lvgl_wait_cb(struct _lv_disp_drv_t *disp_drv)
{
    xSemaphoreTake(panel_refreshing_sem, portMAX_DELAY);
}

/* Rotate display and touch, when rotated screen in LVGL. Called when driver parameters are updated. */
// static void example_lvgl_port_update_callback(lv_disp_drv_t *drv)
// {
//     esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t) drv->user_data;

//     switch (drv->rotated) {
//     case LV_DISP_ROT_NONE:
//         // Rotate LCD display
//         //esp_lcd_panel_swap_xy(panel_handle, false);
//         //esp_lcd_panel_mirror(panel_handle, false, false);
//         break;
//     case LV_DISP_ROT_90:
//         // Rotate LCD display
//         //esp_lcd_panel_swap_xy(panel_handle, true);
//         //esp_lcd_panel_mirror(panel_handle, false, true);
//         break;
//     case LV_DISP_ROT_180:
//         // Rotate LCD display
//         // esp_lcd_panel_swap_xy(panel_handle, false);
//         //esp_lcd_panel_mirror(panel_handle, true, true);
//         break;
//     case LV_DISP_ROT_270:
//         // Rotate LCD display
//         // esp_lcd_panel_swap_xy(panel_handle, true);
//         //esp_lcd_panel_mirror(panel_handle, true, false);
//         break;
//     }
// }


static void example_increase_lvgl_tick(void *arg)
{
    /* Tell LVGL how many milliseconds has elapsed */
    lv_tick_inc(LVGL_TICK_PERIOD_MS);
}

void app_main(void)
{

    panel_refreshing_sem = xSemaphoreCreateBinary();
    xSemaphoreGive(panel_refreshing_sem);

    ESP_LOGI(TAG, "Initialize SPI bus");
    spi_bus_config_t buscfg = {
        .sclk_io_num = LCD_PANEL_PIN_NUM_SCLK,
        .mosi_io_num = LCD_PANEL_PIN_NUM_MOSI,
        .miso_io_num = LCD_PANEL_PIN_NUM_MISO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = LCD_RESOLUTION,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO));

    ESP_LOGI(TAG, "Install panel IO");
    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = LCD_PANEL_PIN_NUM_EPD_DC,
        .cs_gpio_num = LCD_PANEL_PIN_NUM_EPD_CS,
        .pclk_hz = LCD_PIXEL_CLOCK_HZ,
        .lcd_cmd_bits = LCD_CMD_BITS,
        .lcd_param_bits = LCD_PARAM_BITS,
        .spi_mode = 0,
        .trans_queue_depth = 10,
        .on_color_trans_done = NULL,
    };
    // --- Attach the LCD to the SPI bus
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t) LCD_HOST, &io_config, &io_handle));
    esp_lcd_panel_handle_t panel_handle = NULL;

    // --- Create esp_lcd panel
    esp_lcd_ssd1680_config_t epaper_ssd1680_config = {
        .busy_gpio_num = LCD_PANEL_PIN_NUM_EPD_BUSY,
        .non_copy_mode = true,
        .height = LCD_V_RES,
        .width = LCD_H_RES,
        .buffer_size = LCD_PIXELS_MEM_ALIGNED
    };
    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = LCD_PANEL_PIN_NUM_EPD_RST,
        .flags.reset_active_high = false,
        .vendor_config = &epaper_ssd1680_config
        
    };
    gpio_install_isr_service(0);
    ESP_ERROR_CHECK(esp_lcd_new_panel_ssd1680(io_handle, &panel_config, &panel_handle));
    
    // --- Reset the display
    ESP_LOGI(TAG, "Resetting e-Paper display...");
    //ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    //vTaskDelay(100 / portTICK_PERIOD_MS);
    
    // --- Initialize panel
    ESP_LOGI(TAG, "Initializing e-Paper display...");
    // --- Register the e-Paper refresh done callback
    epaper_panel_callbacks_t cbs = {
        .on_epaper_refresh_done = epaper_flush_ready_callback
    };
    epaper_panel_register_event_callbacks_ssd1680(panel_handle, &cbs, &disp_drv);
    //ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    //vTaskDelay(100 / portTICK_PERIOD_MS);

    // --- Turn on display
    ESP_LOGI(TAG, "Turning e-Paper display on...");
    // ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));
    // vTaskDelay(100 / portTICK_PERIOD_MS);
    // update_count++;
    // --- Configurate the screen
    // NOTE: the configurations below are all FALSE by default
    // esp_lcd_panel_invert_color(panel_handle, true);
    // NOTE: Calling esp_lcd_panel_disp_on_off(panel_handle, true) will reset the LUT to the panel built-in one,
    // custom LUT will not take effect any more after calling esp_lcd_panel_disp_on_off(panel_handle, true)
    // ESP_ERROR_CHECK(epaper_panel_set_custom_lut_ssd1680(panel_handle, fast_refresh_lut, 159));

    // ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));
    //ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_handle, true, true));
    
    //clearScreen(panel_handle);
    
    // --- Initialize LVGL
    ESP_LOGI(TAG, "Initialize LVGL library");
    lv_init();
    // alloc draw buffers used by LVGL
    // it's recommended to choose the size of the draw buffer(s) to be at least 1/10 screen sized
    lv_color_t *buf1 = heap_caps_malloc(LCD_RESOLUTION * sizeof(lv_color_t), MALLOC_CAP_DMA);
    assert(buf1);
    lv_color_t *buf2 = heap_caps_malloc(LCD_RESOLUTION * sizeof(lv_color_t), MALLOC_CAP_DMA);
    assert(buf2);
    // alloc bitmap buffer to draw
    converted_buffer_black = heap_caps_malloc(LCD_RESOLUTION, MALLOC_CAP_DMA);
    //converted_buffer_red = heap_caps_malloc(LCD_RESOLUTION, MALLOC_CAP_DMA);
    //memset(converted_buffer_red, 0xff, LCD_RESOLUTION);
    // initialize LVGL draw buffers
    lv_disp_draw_buf_init(&disp_buf, buf1, buf2, LCD_RESOLUTION);
    // initialize LVGL display driver
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = LCD_H_RES;
    disp_drv.ver_res = LCD_V_RES;
    disp_drv.rotated = 1;
    //ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(panel_handle, true));
    //disp_drv.set_px_cb = my_set_px_cb;
    disp_drv.flush_cb = example_lvgl_flush_cb;
    disp_drv.wait_cb = example_lvgl_wait_cb;
    disp_drv.direct_mode = 1;

    // disp_drv.drv_update_cb = example_lvgl_port_update_callback;
    disp_drv.draw_buf = &disp_buf;
    disp_drv.user_data = panel_handle;
    // NOTE: The ssd1680 e-paper is monochrome and 1 byte represents 8 pixels
    // so full_refresh is MANDATORY because we cannot set position to bitmap at pixel level
    disp_drv.full_refresh = true;
    ESP_LOGI(TAG, "Register display driver to LVGL");
    lv_disp_t *disp = lv_disp_drv_register(&disp_drv);
    // init lvgl tick
    ESP_LOGI(TAG, "Install LVGL tick timer");
    // Tick interface for LVGL (using esp_timer to generate 2ms periodic event)
    const esp_timer_create_args_t lvgl_tick_timer_args = {
        .callback = &example_increase_lvgl_tick,
        .name = "lvgl_tick"
    };
    esp_timer_handle_t lvgl_tick_timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer, LVGL_TICK_PERIOD_MS * 1000));

    ESP_LOGI(TAG, "Display LVGL Meter Widget");
    //example_lvgl_demo_ui(disp);
    ui_demo();
    
    while (1) {
        // raise the task priority of LVGL and/or reduce the handler period can improve the performance
        // The task running lv_timer_handler should have lower priority than that running `lv_tick_inc`
        lv_timer_handler();
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

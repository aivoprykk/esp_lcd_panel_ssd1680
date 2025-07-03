#include <inttypes.h>
#include <string.h>
#include "sdkconfig.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_ssd168x_private.h"
#include "esp_lcd_panel_ssd168x.h"
#include "esp_lcd_panel_ops.h"
#include "driver/spi_common.h"
#include "driver/gpio.h"
#include "logger_common.h"

#include "ssd1680_waveshare_2in13_lut.h"
#include "img_bitmap.h"

// SPI Bus
#define EPD_PANEL_SPI_CLK           1000000
#define EPD_PANEL_SPI_CMD_BITS      8
#define EPD_PANEL_SPI_PARAM_BITS    8
#define EPD_PANEL_SPI_MODE          0
// e-Paper GPIO
#define EXAMPLE_PIN_NUM_EPD_DC      17
#define EXAMPLE_PIN_NUM_EPD_RST     16
#define EXAMPLE_PIN_NUM_EPD_CS      5
#define EXAMPLE_PIN_NUM_EPD_BUSY    4
// e-Paper SPI
#define EXAMPLE_PIN_NUM_MOSI        23
#define EXAMPLE_PIN_NUM_SCLK        18
#define LCD_HOST SPI2_HOST

// e-Paper panel
#define LCD_H_GAP 0
#define LCD_V_GAP 0
#define LCD_H_RES 128 // panel x res
#define LCD_V_RES 250 // panel y res
#define LCD_RESOLUTION         (LCD_H_RES * LCD_V_RES)
#define LCD_ROW_LEN (LCD_H_RES / 8)
#define LCD_PIXELS (LCD_V_RES * LCD_ROW_LEN)
#define LCD_PIXELS_MEM_ALIGNED (LCD_H_RES * ROUND_UP_TO_8(LCD_V_RES))

// // driver resolution
// #define SSD1680_MEM_SOURCE_SIZE   176   // driver source size (x)
// #define SSD1680_MEM_GATE_SIZE     296   // driver gate size (y)
// #define SSD1680_MEM_SOURCE_LEN    (SSD1680_MEM_SOURCE_SIZE / 8)
#define SSD1680_MEM_SIZE (SSD1680_MEM_GATE_SIZE * SSD1680_MEM_SOURCE_LEN)
#define WAIT_TIME_MS 5000

static const char *TAG = "epaper_demo_plain";
static SemaphoreHandle_t epaper_panel_semaphore = 0;
static uint8_t fast_refresh_lut[] = SSD1680_WAVESHARE_2IN13_V2_LUT_FAST_REFRESH_KEEP;

static bool give_semaphore_in_isr(const esp_lcd_panel_handle_t handle, const void *edata, void *user_data)
{
    SemaphoreHandle_t *epaper_panel_semaphore_ptr = user_data;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(*epaper_panel_semaphore_ptr, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken == pdTRUE) {
        portYIELD_FROM_ISR();
        return true;
    }
    return false;
}

static void set_rotation(esp_lcd_panel_handle_t panel_handle, uint8_t rotation)
{
    switch (rotation) {
    case 0:
        ESP_LOGI(TAG, "Mirror none");
        // set rotation 0 degrees
        //ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(panel_handle, false));
        ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_handle, false, false));
        break;
    case 1:
        ESP_LOGI(TAG, "Mirror y");
        // set rotation 90 degrees
        //ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(panel_handle, true));
        ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_handle, false, true));
        break;
    case 2:
        ESP_LOGI(TAG, "Mirror xy");
        // set rotation 180 degrees
        // ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(panel_handle, false));
        ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_handle, true, true));
        break;
    case 3:
        ESP_LOGI(TAG, "Mirror x");
        // set rotation 270 degrees
        // ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(panel_handle, true));
        ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_handle, true, false));
        break;
    }
}

void drawImg(esp_lcd_panel_handle_t panel_handle, const uint8_t *img, uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint8_t rotation, bool swap, bool invert_color, bool partial)
{
    //vTaskDelay(pdMS_TO_TICKS(WAIT_TIME_MS));
    // MEAS_START();
    ESP_LOGI(TAG, "Drawing image x=%hu, y=%hu, w=%hu, h=%hu, rotation=%hhu", x, y, w, h, rotation);
    xSemaphoreTake(epaper_panel_semaphore, portMAX_DELAY);
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));
    if(invert_color)
        ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_handle, invert_color));
    if(rotation || is_mirrored(panel_handle)){
        set_rotation(panel_handle, rotation);
    }
    if(swap || is_xy_swapped(panel_handle)){
        ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(panel_handle, swap));
    }
    if(partial)
        ESP_ERROR_CHECK(epaper_panel_set_custom_lut_ssd168x(panel_handle, fast_refresh_lut, 159));
    // ESP_ERROR_CHECK(epaper_panel_set_bitmap_color_ssd1680(panel_handle, SSD1680_EPAPER_BITMAP_RED));
    // ESP_ERROR_CHECK(esp_lcd_panel_draw_bitmap(panel_handle, x, y, w, h, img));
    ESP_ERROR_CHECK(epaper_panel_set_bitmap_color_ssd168x(panel_handle, SSD168X_EPAPER_BITMAP_BLACK));
    ESP_ERROR_CHECK(esp_lcd_panel_draw_bitmap(panel_handle, x, y, w, h, img));
    ESP_ERROR_CHECK(epaper_panel_refresh_screen_ssd168x(panel_handle, 0));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, false));
    //MEAS_END(TAG, "[%s] took %llu us",__func__);
}

void app_main(void)
{
    esp_err_t ret;
    // --- Init SPI Bus
    ESP_LOGI(TAG, "Initializing SPI Bus...");
    spi_bus_config_t buscfg = {
        .sclk_io_num = EXAMPLE_PIN_NUM_SCLK,
        .mosi_io_num = EXAMPLE_PIN_NUM_MOSI,
        .miso_io_num = -1,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = SOC_SPI_MAXIMUM_BUFFER_SIZE
    };
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO));
    // --- Init ESP_LCD IO
    ESP_LOGI(TAG, "Initializing panel IO...");
    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = EXAMPLE_PIN_NUM_EPD_DC,
        .cs_gpio_num = EXAMPLE_PIN_NUM_EPD_CS,
        .pclk_hz = EPD_PANEL_SPI_CLK,
        .lcd_cmd_bits = EPD_PANEL_SPI_CMD_BITS,
        .lcd_param_bits = EPD_PANEL_SPI_PARAM_BITS,
        .spi_mode = EPD_PANEL_SPI_MODE,
        .trans_queue_depth = 10,
        .on_color_trans_done = NULL
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t) LCD_HOST, &io_config, &io_handle));
    // --- Create esp_lcd panel
    ESP_LOGI(TAG, "Creating SSD1680 panel...");
    esp_lcd_ssd168x_config_t epaper_ssd1680_config = {
        .busy_gpio_num = EXAMPLE_PIN_NUM_EPD_BUSY,
        // NOTE: Enable this to reduce one buffer copy if you do not use swap-xy, mirror y or invert color
        // since those operations are not supported by ssd1680 and are implemented by software
        // Better use DMA-capable memory region, to avoid additional data copy
        .non_copy_mode = false,
        .height = LCD_V_RES,
        .width = LCD_H_RES,
        .buffer_size = LCD_PIXELS_MEM_ALIGNED
    };
    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = EXAMPLE_PIN_NUM_EPD_RST,
        .flags.reset_active_high = false,
        .vendor_config = &epaper_ssd1680_config
    };
    esp_lcd_panel_handle_t panel_handle = NULL;
    // NOTE: Please call gpio_install_isr_service() manually before esp_lcd_new_panel_ssd1680()
    // because gpio_isr_handler_add() is called in esp_lcd_new_panel_ssd1680()
    gpio_install_isr_service(0);
    ret = esp_lcd_new_panel_ssd168x(io_handle, &panel_config, &panel_handle);
    ESP_ERROR_CHECK(ret);
    // --- Reset the display
    ESP_LOGI(TAG, "Resetting e-Paper display...");
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    vTaskDelay(100 / portTICK_PERIOD_MS);
    // --- Initialize LCD panel
    ESP_LOGI(TAG, "Initializing e-Paper display...");
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    vTaskDelay(100 / portTICK_PERIOD_MS);
#if (LCD_H_GAP>0) || (LCD_V_GAP>0)
    //  the gap is LCD panel specific, even panels with the same driver IC, can
    //  have different gap value
    esp_lcd_panel_set_gap(panel_handle, LCD_H_GAP, LCD_V_GAP);
#endif
    // Turn on the screen
    ESP_LOGI(TAG, "Turning e-Paper display on...");
    //ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));
    // Set custom lut
    // NOTE: Setting custom LUT is not necessary. Panel built-in LUT is used calling after esp_lcd_panel_disp_on_off()
    // NOTE: Uncomment code below to see difference between full refresh & fast refresh
    // NOTE: epaper_panel_set_custom_lut() must be called AFTER calling esp_lcd_panel_disp_on_off()
    // static uint8_t fast_refresh_lut[] = SSD1680_WAVESHARE_2IN13_V2_LUT_FAST_REFRESH_KEEP;
    // ESP_ERROR_CHECK(epaper_panel_set_custom_lut(panel_handle, fast_refresh_lut, 159));

    //vTaskDelay(100 / portTICK_PERIOD_MS);

    epaper_panel_semaphore = xSemaphoreCreateBinary();
    xSemaphoreGive(epaper_panel_semaphore);

    // --- Clear the VRAM of RED and BLACK
    
    // --- Register the e-Paper refresh done callback
    // cbs does not have to be static for ssd1680 driver, for the callback ptr is copied, not pointed
    epaper_panel_callbacks_t cbs = {
        .on_epaper_refresh_done = give_semaphore_in_isr,
    };

    epaper_panel_register_event_callbacks_ssd168x(panel_handle, &cbs, &epaper_panel_semaphore);
    
    // --- Draw full-screen bitmap
    // epaper_panel_clear_screen_ssd1680(panel_handle, 0x0);
    // ESP_LOGI(TAG, "Drawing speed_raw_122 1, false, false, true");
    // drawImg(panel_handle, speed_raw_122, 0, 0, 122, 122, 1, false, false, true);
    // vTaskDelay(pdMS_TO_TICKS(2000));
    
    // epaper_panel_clear_screen_ssd1680(panel_handle);
    // ESP_LOGI(TAG, "Drawing speed_raw_122 1, false, false, true");
    // drawImg(panel_handle, speed_raw_122, 0, 0, 122, 122, 1, true, false, true);
    // vTaskDelay(pdMS_TO_TICKS(2000));
    
    // // epaper_panel_clear_screen_ssd1680(panel_handle);
    // ESP_LOGI(TAG, "Drawing speed_raw_122 2, false, false, true");
    // drawImg(panel_handle, speed_raw_122, 0, 0, 122, 122, 2, false, false, true);
    // vTaskDelay(pdMS_TO_TICKS(2000));
    
    // // epaper_panel_clear_screen_ssd1680(panel_handle);
    // ESP_LOGI(TAG, "Drawing speed_raw_122 2, false, false, true");
    // drawImg(panel_handle, speed_raw_122, 0, 0, 122, 122, 2, true, false, true);
    // vTaskDelay(pdMS_TO_TICKS(2000));

    // epaper_panel_clear_screen_ssd1680(panel_handle);
    ESP_LOGI(TAG, "Drawing speed_raw_122x250 0, false, false, true");
    drawImg(panel_handle, speed_raw_122x250, 0, 0, 122, 250, 0, false, false, true);
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    ESP_LOGI(TAG, "Drawing speed_raw_250x122 0, true, false, true");
    drawImg(panel_handle, speed_raw_250x122, 0, 0, 250, 122, 0, true, false, true);
    vTaskDelay(pdMS_TO_TICKS(2000));

    // epaper_panel_clear_screen_ssd1680(panel_handle);
    ESP_LOGI(TAG, "Drawing speed_raw_122x250 1, false, false, true");
    drawImg(panel_handle, speed_raw_122x250, 0, 0, 122, 250, 1, false, false, true);
    vTaskDelay(pdMS_TO_TICKS(2000));

    ESP_LOGI(TAG, "Drawing speed_raw_250x122 1, true, false, true");
    drawImg(panel_handle, speed_raw_250x122, 0, 0, 250, 122, 1, true, false, true);
    vTaskDelay(pdMS_TO_TICKS(2000));

    // epaper_panel_clear_screen_ssd1680(panel_handle);
    ESP_LOGI(TAG, "Drawing speed_raw_122x250 2, false, false, true");
    drawImg(panel_handle, speed_raw_122x250, 0, 0, 122, 250, 2, false, false, true);
    vTaskDelay(pdMS_TO_TICKS(2000));

    ESP_LOGI(TAG, "Drawing speed_raw_250x122 2, true, false, true");
    drawImg(panel_handle, speed_raw_250x122, 0, 0, 250, 122, 2, true, false, true);
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    // epaper_panel_clear_screen_ssd1680(panel_handle);
    ESP_LOGI(TAG, "Drawing speed_raw_122x250 3, false, false, true");
    drawImg(panel_handle, speed_raw_122x250, 0, 0, 122, 250, 3, false, false, true);
    vTaskDelay(pdMS_TO_TICKS(2000));

    ESP_LOGI(TAG, "Drawing speed_raw_250x122 3, true, false, true");
    drawImg(panel_handle, speed_raw_250x122, 0, 0, 250, 122, 3, true, false, true);
    vTaskDelay(pdMS_TO_TICKS(2000));

    // ESP_LOGI(TAG, "Drawing speed_raw_122 0, true, false, true");
    // drawImg(panel_handle, speed_raw_122, 0, 0, 122, 122, 0, false, false, true); // try to rotate 90
    // vTaskDelay(pdMS_TO_TICKS(3000));

    // ESP_LOGI(TAG, "Drawing speed_raw_122 1, true, false, true");
    // drawImg(panel_handle, speed_raw_122, 0, 0, 122, 122, 0, true, false, true); // try to rotate 90
    // vTaskDelay(pdMS_TO_TICKS(3000));
    

    
    //ESP_LOGI(TAG, "Go to sleep mode...");
    //esp_lcd_panel_disp_on_off(panel_handle, false);
    // vTaskDelay(pdMS_TO_TICKS(3000));

    // // ESP_LOGI(TAG, "e-Paper resuming...");
    // esp_lcd_panel_reset(panel_handle);
    // // vTaskDelay(100 / portTICK_PERIOD_MS);
    // esp_lcd_panel_init(panel_handle);
    // vTaskDelay(100 / portTICK_PERIOD_MS);
    // epaper_panel_clear_screen_ssd1680(panel_handle, 0x0);

    // NOTE: If you want to use a custom LUT, you'll have to set it again after resume
    // --- Draw partial bitmap
    // ESP_LOGI(TAG, "Show image partial");
    // drawImg(panel_handle, empty_bitmap, 0, 0, 122, 250, empty_bitmap, 0, false, false, true);
    // drawImg(panel_handle, BITMAP_64_128, 32, 61, 96, 189, empty_bitmap, 0, false, false, true);
    // //drawImg(panel_handle, empty_bitmap, 0, 0, 122, 250, empty_bitmap, 0, false, false, true);
    // drawImg(panel_handle, BITMAP_128_64, 0, 0, 122, 64, empty_bitmap, 3, true, false, true);
    // vTaskDelay(pdMS_TO_TICKS(3000));
    
    // drawImg(panel_handle, speed_raw_122, 0, 0, 122, 122, 0, false, false, true);
    
    ESP_LOGI(TAG, "Go to sleep mode...");
    esp_lcd_panel_disp_on_off(panel_handle, false);
}

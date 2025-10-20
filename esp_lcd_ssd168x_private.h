#ifndef F5CBFF08_82AE_4990_9783_EAB40C00C544
#define F5CBFF08_82AE_4990_9783_EAB40C00C544

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_lcd_panel_ssd168x.h"

#if defined(CONFIG_SSD168X_PANEL_SSD1680)
#define SSD168X_SOURCE_SIZE (176)
#define SSD168X_GATE_SIZE (296)
#endif
#if defined(CONFIG_SSD168X_PANEL_SSD1681)
#define SSD168X_SOURCE_SIZE (200)
#define SSD168X_GATE_SIZE (200)
#endif
#if !defined(SSD168X_SOURCE_SIZE)
#define SSD168X_SOURCE_SIZE (0)
#endif
#if !defined(SSD168X_GATE_SIZE)
#define SSD168X_GATE_SIZE (0)
#endif

#define SSD168X_RAM_SIZE ((SSD168X_SOURCE_SIZE * SSD168X_GATE_SIZE) << 3u)  // 6512

#if (defined(CONFIG_LCD_ENABLE_DEBUG_LOG))
#include "esp_timer.h"
    #define ILOG(tag, format, ...) ESP_LOGI(tag, format, ##__VA_ARGS__)
    #define DLOG(tag, format, ...) ESP_LOGD(tag, format, ##__VA_ARGS__)
    #define DMEAS_START() uint64_t _start = (esp_timer_get_time()), _end = 0
    #define DMEAS_END(a, b, ...) \
       ESP_LOGI(tag, "[%s] took %llu us", __func__, (esp_timer_get_time() - _start))
#else
    #define ILOG(tag, format, ...) ((void)0)
    #define DLOG(tag, format, ...) ((void)0)
    #define DMEAS_START() ((void)0)
    #define DMEAS_END(a, b, ...) ((void)0)
#endif

// Error handling optimization macros
#define SSD168X_CHECK_IO_TX_PARAM(io, cmd, params, size, cmd_name) \
    do { \
        if (esp_lcd_panel_io_tx_param((io), (cmd), (params), (size)) != ESP_OK) { \
            ESP_LOGE(TAG, "%s%s%s %s", drv_msg[0], drv_msg[1], (cmd_name), drv_msg[2]); \
            return ESP_FAIL; \
        } \
    } while(0)

#define SSD168X_CHECK_IO_TX_COLOR(io, cmd, data, size, cmd_name) \
    do { \
        if (esp_lcd_panel_io_tx_color((io), (cmd), (data), (size)) != ESP_OK) { \
            ESP_LOGE(TAG, "%s%s %s", drv_msg[0], (cmd_name), drv_msg[2]); \
            return ESP_FAIL; \
        } \
    } while(0)

#define SSD168X_CHECK_FUNCTION_CALL(func_call, error_msg) \
    do { \
        if ((func_call) != ESP_OK) { \
            ESP_LOGE(TAG, "%s", (error_msg)); \
            return ESP_FAIL; \
        } \
    } while(0)

#define SSD168X_RETURN_ON_FALSE(a, ret, tag, error_msg) \
    do { \
        if (!(a)) { \
            ESP_LOGE(tag, "%s", (error_msg)); \
            return (ret); \
        } \
    } while(0)

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

#define BIT_SET(a, b)   ((a) |= (1u << (uint8_t)(b)))
#define BIT_CLEAR(a, b) ((a) &= (~(1u << (uint8_t)(b))))
#define BIT_FLIP(a, b)  ((a) ^= (1u << (uint8_t)(b)))
#define BIT_CHECK(a, b) ((a) &  (1u << (uint8_t)(b)))

typedef struct m_area_s {
    int16_t x1;
    int16_t y1;
    int16_t x2;
    int16_t y2;
} area_t;

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
    // Configurations from epaper_ssd168x_conf
    int busy_gpio_num;
    bool full_refresh;
    // Configurations from interface functions
    int gap_x;
    int gap_y;
    // Configurations from e-Paper specific public functions
    epaper_panel_callback_t epaper_refresh_done_isr_callback;
    esp_lcd_ssd168x_bitmap_color_t bitmap_color;
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
    bool is_on;
} epaper_panel_t;

#define EPAPER_PANEL_DEFAULTS { \
    .base = { \
        .del = NULL, \
        .reset = NULL, \
        .init = NULL, \
        .draw_bitmap = NULL, \
        .invert_color = NULL, \
        .set_gap = NULL, \
        .mirror = NULL, \
        .swap_xy = NULL, \
        .disp_on_off = NULL, \
    }, \
    .io = 0, \
    .reset_gpio_num = -1, \
    .reset_level = false, \
    .busy_gpio_num = -1, \
    .full_refresh = true, \
    .gap_x = 0, \
    .gap_y = 0, \
    .epaper_refresh_done_isr_callback = { \
        .callback_ptr = NULL, \
        .args = NULL, \
    }, \
    .bitmap_color = SSD168X_EPAPER_BITMAP_BLACK, \
    .non_copy_mode = false, \
    .mirror_x = false, \
    .mirror_y = false, \
    .swap_xy = false, \
    .invert_color = false, \
    .is_rotation_done = false, \
    ._ram_params = { \
        .x = 0, \
        .y = 0, \
        .xe = 0, \
        .ye = 0, \
        .w = 0, \
        .h = 0, \
        .xs_d8 = 0, \
        .xe_d8 = 0, \
        .ys_m256 = 0, \
        .ys_d256 = 0, \
        .ye_m256 = 0, \
        .ye_d256 = 0, \
        .buffer_size = 0, \
    }, \
    .width = 0, \
    .height = 0, \
    .framebuffer = NULL, \
    .framebuffer_size = 0, \
    .next_init_mode = INIT_MODE_FAST_2, \
    .next_sleep_mode = SLEEP_MODE_DEEP_1, \
    .next_init_lut = NULL, \
    .is_on = true, \
}

static esp_err_t process_bitmap(esp_lcd_panel_t *panel, const void *color_data);
static void epaper_driver_gpio_isr_handler(void *arg);
static esp_err_t epaper_set_lut(esp_lcd_panel_io_handle_t io, const uint8_t *lut);
static esp_err_t epaper_panel_del(esp_lcd_panel_t *panel);
static esp_err_t epaper_panel_reset(esp_lcd_panel_t *panel);
static esp_err_t epaper_panel_init(esp_lcd_panel_t *panel);
static esp_err_t epaper_panel_draw_bitmap(esp_lcd_panel_t *panel, int x_start, int y_start, int x_end, int y_end, const void *color_data);
static esp_err_t epaper_panel_invert_color(esp_lcd_panel_t *panel, bool invert_color_data);
static esp_err_t epaper_panel_mirror(esp_lcd_panel_t *panel, bool mirror_x, bool mirror_y);
static esp_err_t epaper_panel_swap_xy(esp_lcd_panel_t *panel, bool swap_axes);
static esp_err_t epaper_panel_set_gap(esp_lcd_panel_t *panel, int x_gap, int y_gap);
static esp_err_t epaper_panel_disp_on_off(esp_lcd_panel_t *panel, bool on_off);

#if defined(cpp)
} // extern "C"
 #endif


#endif /* F5CBFF08_82AE_4990_9783_EAB40C00C544 */

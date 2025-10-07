#ifndef D1A942CD_8799_44BA_919D_944D3E28E376
#define D1A942CD_8799_44BA_919D_944D3E28E376

#ifdef __cplusplus
extern "C" {
#endif

#include "sdkconfig.h"
#include "stdbool.h"
#include "stdint.h"
#include "stdio.h"
#include "freertos/FreeRTOS.h"

#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#include <esp_lcd_panel_ssd168x.h>

#include "lvgl.h"

// e-Paper panel
#if defined(CONFIG_SSD168X_PANEL_SSD1681)
#define LCD_H_RES (200) // panel x res
#define LCD_V_RES (200) // panel y res
#define LCD_H_GAP (0)
#define LCD_V_GAP (0)
#else
#define LCD_H_RES (128) // panel x res
#define LCD_V_RES (250) // panel y res
#define LCD_H_GAP (6)
#define LCD_V_GAP (0)
#endif
#define LCD_H_VISIBLE (LCD_H_RES-LCD_H_GAP)           // vertical
#define LCD_V_VISIBLE (LCD_V_RES-LCD_V_GAP)           // horizontal

#define LCD_RESOLUTION         (LCD_H_RES * LCD_V_RES)
#define LCD_ROW_LEN (LCD_H_RES >> 3u)
#define LCD_PIXELS (LCD_V_RES * LCD_ROW_LEN)

#define ROUND_UP_TO_8(x)   (((x) + 7) & ~7U)
#define LCD_PIXELS_ALIGNED (ROUND_UP_TO_8(LCD_H_RES) * ROUND_UP_TO_8(LCD_V_RES))
#define LCD_PIXELS_MEM_ALIGNED (LCD_PIXELS_ALIGNED >> 3u)

#define LCD_BUF_SIZE (LCD_PIXELS)

#define LVGL_TICK_PERIOD_MS 10UL
#define LV_DRAW_BUF_SZ 1 // 1 lvgl draw buf for epd
#define CONV_BUF_SZ 1 // only black
#define LBUFSZ LCD_PIXELS_ALIGNED // size of the draw buffer(s) in bytes

#if (LVGL_VERSION_MAJOR < 9)
    #define LVGL_V8_MODE 1
    #define DISPLAY_GET_ROTATION() (drv.disp_drv.rotated)
    #define DISPLAY_SET_ROTATION(disp, r) do { \
        if(drv.disp_drv.rotated != (r)) { \
            drv.disp_drv.rotated = (r); \
            if(drv.lv_disp) lv_disp_drv_update(drv.lv_disp, &drv.disp_drv); \
        } \
    } while(0)
    #define DISPLAY_GET_HOR_RES() lv_disp_get_hor_res(drv.lv_disp)
    #define DISPLAY_GET_VER_RES() lv_disp_get_ver_res(drv.lv_disp)
    #define FLUSH_READY_CB(disp_drv) lv_disp_flush_ready((lv_disp_drv_t*)(disp_drv))
    #define DISPLAY_USER_DATA_T lv_disp_drv_t
    #define GET_USER_DATA(cb_param) ((esp_lcd_panel_handle_t)((lv_disp_drv_t*)(cb_param))->user_data)
#else
    #define LVGL_V8_MODE 0
    #define DISPLAY_GET_ROTATION() lv_display_get_rotation(drv.lv_disp)
    #define DISPLAY_SET_ROTATION(disp, r) do { \
        if((r) != lv_display_get_rotation(drv.lv_disp)) { \
            lv_display_set_rotation(drv.lv_disp, (r)); \
        } \
    } while(0)
    #define DISPLAY_GET_HOR_RES() lv_display_get_horizontal_resolution(drv.lv_disp)
    #define DISPLAY_GET_VER_RES() lv_display_get_vertical_resolution(drv.lv_disp)
    #define FLUSH_READY_CB(disp) lv_display_flush_ready((lv_display_t*)(disp))
    #define DISPLAY_USER_DATA_T lv_display_t
    #define GET_USER_DATA(cb_param) ((esp_lcd_panel_handle_t)lv_display_get_user_data((lv_display_t*)(cb_param)))
#endif
typedef struct display_driver_s {
#if (LVGL_VERSION_MAJOR < 9)
    lv_disp_draw_buf_t disp_buf; // contains internal graphic buffer(s) called draw buffer(s)
    lv_disp_drv_t disp_drv;      // contains callback functions
#else
    #define BYTE_PER_PIXEL (LV_COLOR_FORMAT_GET_SIZE(LV_COLOR_FORMAT_I1)) /*will be 2 for RGB565 */
#endif
    uint8_t *lv_mem_buf[LV_DRAW_BUF_SZ + CONV_BUF_SZ];
    size_t lv_mem_buf_size[LV_DRAW_BUF_SZ + CONV_BUF_SZ];
    lv_disp_t *lv_disp;
    bool is_initialized_lvgl;
    SemaphoreHandle_t sem;
} display_driver_t;

extern display_driver_t drv;
typedef enum m_rot_e {
    DISP_ROT_NONE = 0,
    DISP_ROT_90 = 1,
    DISP_ROT_180 = 2,
    DISP_ROT_270 = 3,
} m_rot_t;

typedef struct m_area_s {
    int x1;
    int y1;
    int x2;
    int y2;
} m_area_t;

bool _drv_lock(int timeout_ms);
void _drv_unlock(void);
int display_drv_set_rotation(int8_t rot);
int8_t display_drv_get_rotation();
void example_lvgl_demo_ui(lv_disp_t *disp);
void ui_demo(void);
void print_lv_mem_mon();

#ifdef __cplusplus
}
#endif

#endif /* D1A942CD_8799_44BA_919D_944D3E28E376 */

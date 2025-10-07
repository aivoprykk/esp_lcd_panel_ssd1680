#include <stdio.h>

#include "private.h"

#include "esp_log.h"
#include "img_bitmap.h"
#include "lvgl.h"

static const char *TAG = "lvgl_demo_ui";

static int count = 0;
LV_FONT_DECLARE(ui_font_OswaldRegular36p1);

static lv_obj_t * ui_common_panel_init(lv_obj_t * parent, uint8_t w, uint8_t h) {
    lv_obj_t * panel = lv_obj_create(parent);
    lv_obj_remove_style_all(panel);
    if(w > 0) lv_obj_set_width(panel, lv_pct(w));
    if(h > 0) lv_obj_set_height(panel, lv_pct(h));
    lv_obj_clear_flag(panel, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);  /// Flags
    lv_obj_set_style_bg_color(panel, lv_color_white(), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(panel, LV_OPA_COVER, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(panel, lv_color_black(), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(panel, LV_OPA_COVER, LV_PART_MAIN | LV_STATE_DEFAULT);
    return panel;
}

lv_obj_t * textScreenLoad() {
    ESP_LOGI(TAG, "load text screen");
    lv_obj_t * text = ui_common_panel_init(NULL, 100, 100);
    lv_obj_set_style_border_color(text, lv_color_black(), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(text, 1, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_t *label = lv_label_create(text);
    lv_obj_set_size(label, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
    int8_t rot = display_drv_get_rotation();
    display_drv_set_rotation(rot >= DISP_ROT_270 ? 0 : rot + 1); // to update lvgl disp_drv
    if(rot == DISP_ROT_90 || rot == DISP_ROT_270) {
        lv_label_set_text(label, "LVGL 90/270\nRotation");
    } else {
        lv_label_set_text(label, "LVGL 0/180\nRotation");
    }
    lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);
    lv_scr_load(text);
    return text;
}

lv_obj_t * splashScreenLoad() {
    ESP_LOGI(TAG, "load splash screen");
    lv_obj_t * splash = ui_common_panel_init(NULL, 100, 100);
    lv_obj_t *img = lv_img_create(splash);
    lv_obj_set_size(img, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
    int8_t rot = display_drv_get_rotation();
    display_drv_set_rotation(rot >= DISP_ROT_270 ? 0 : rot + 1); // to update lvgl disp_drv
    if(rot == DISP_ROT_90 || rot == DISP_ROT_270) {
        lv_img_set_src(img, &speed_raw_122x250);
    } else {
        lv_img_set_src(img, &speed_raw_250x122);
    }
    lv_obj_align(img, LV_ALIGN_TOP_LEFT, 0, 0);
    lv_scr_load(splash);
    return splash;
}

lv_obj_t * blankScreenLoad(bool invert) {
    ESP_LOGI(TAG, "load blank screen with color %s", invert ? "black" : "white");
    lv_obj_t * panel = ui_common_panel_init(NULL, 100, 100);
    lv_obj_set_style_bg_color(panel, invert ? lv_color_black() : lv_color_white(), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(panel, invert ? lv_color_white() : lv_color_black(), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(panel, invert ? lv_color_white() : lv_color_black(), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(panel, 1, LV_PART_MAIN | LV_STATE_DEFAULT);
    int8_t rot = display_drv_get_rotation();
    display_drv_set_rotation(rot >= DISP_ROT_270 ? 0 : rot + 1); // to update lvgl disp_drv
    lv_scr_load(panel);
    return panel;
}

static lv_obj_t * scr = 0, *prevscr = 0;

static void timer_cb(lv_timer_t *timer) {
    printf("----------------------------------------\n");
    printf("timer_cb: %d\n", count);
    printf("----------------------------------------\n");
    if(scr) {
        lv_obj_invalidate(scr);
        prevscr = scr;
    }
    if(count > 8) 
    {
        scr = blankScreenLoad((count & 1));
        if(count > 12) {
            count = -1;
        }
    }
    else 
    if(count > 4) 
    {
        scr = textScreenLoad();
    }
    else 
    {
        scr = splashScreenLoad();
    }
    if(prevscr) {
        lv_obj_del(prevscr);
        prevscr = 0;
    }
    ++count;
    print_lv_mem_mon();
    lv_timer_ready(timer);
    _lv_disp_refr_timer(NULL);
}

void ui_init(void) {
    lv_disp_t* dispp = lv_disp_get_default();
    lv_theme_t* theme = lv_theme_mono_init(dispp, false, &ui_font_OswaldRegular36p1);
    lv_disp_set_theme(dispp, theme);
}

void ui_demo(void) {
    ESP_LOGI(TAG, "create timer with 3,5sec interval");
    lv_timer_t *timer = lv_timer_create(timer_cb, 3000, NULL);
    lv_timer_ready(timer);
}


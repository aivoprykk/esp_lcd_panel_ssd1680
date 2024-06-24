/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

// This demo UI is adapted from LVGL official example: https://docs.lvgl.io/master/widgets/extra/meter.html#simple-meter

#include <stdio.h>

#include "private.h"

#include "logger_common.h"
#include "str.h"
#include "adc.h"
#include "button.h"

static lv_obj_t *meter;
static lv_obj_t *label;
static const char *TAG = "lvgl_demo_ui";
static int i = 0;
static float voltage_bat = 0.0;
#define MINIMUM_VOLTAGE 3.2

static void update_bat(uint8_t verbose) {
    voltage_bat = volt_read();
    if (verbose)
        ESP_LOGI(TAG, "[%s] Battery measured (computed:%.02f, required_min:%.02f)\n", __FUNCTION__, voltage_bat, MINIMUM_VOLTAGE);
}

// static void set_value(void *indic, int32_t v) {
//     lv_meter_set_indicator_end_value(meter, indic, v);
// }

// #include "ui.h"
// #include "ui_themes.h"
#include "lv_comp_cell.h"
#include "lv_comp_info_panel.h"
#include "lv_comp_main_screen.h"
#include "lv_comp_speed_panel.h"
#include "lv_comp_stat_panel.h"
#include "lv_comp_statusbar.h"
#include "lv_comp_two_col_panel.h"
#include "ui_common.h"

static int count = 0, angle = 0;
static float max_speed = 0, avg_speed = 0, cur_speed[5] = {0};
static uint8_t index_speed = 0;

lv_main_screen_t *ui_main_screen = NULL;

// static void set_angle(void * img, int32_t v)
// {
//     lv_img_set_src(img, &ui_img_near_me_fill0_wght400_grad0_opsz24_png);
//     lv_img_set_angle(img, v);
// }
static bool button_down = false;
static void button_cb(int num, l_button_ev_t ev) {
    ESP_LOGI(TAG, "Button %d event: %d", num, ev);
    l_button_t *btn = 0;
    if(num<1)
        btn = &btns[num];
    switch (ev) {
    case L_BUTTON_UP:
        button_down = false;
        break;
    case L_BUTTON_DOWN:
        button_down = true;
        if(num==0)
            showPushScreen(0);
        break;
    case L_BUTTON_LONG_PRESS_START:
        if(num==0)
            showPushScreen(1);
        break;
    case L_BUTTON_LONG_LONG_PRESS_START:
        if(num==0)
            showPushScreen(2);
        break;
    default:
        break;
    }
}

#include "esp_random.h"
float randomFloat() {
    uint32_t r = esp_random();               // Generate a random 32-bit number
    float f = (float)r / (float)UINT32_MAX;  // Convert to a float between 0 and 1
    return f * 127;                          // Scale to the range 0-127
}

int randomInteger() {
    uint32_t r = esp_random();  // Generate a random 32-bit number
    return r % 1271;            // Scale to the range 0-1500
}

float averageFloat(float *array, int count) {
    float sum = 0.0;
    for (int i = 0; i < count; i++) {
        sum += array[i];
    }
    return sum / count;
}

lv_obj_t * splashScreenLoad() {
    ESP_LOGI(TAG, "load splash screen");
    lv_obj_t * splash = ui_common_panel_init(NULL, 100, 100);
    lv_obj_t *img = lv_img_create(splash);
    lv_obj_set_size(img, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
    lv_img_set_src(img, &speed_raw_250x122);
    lv_obj_align(img, LV_ALIGN_TOP_LEFT, 0, 0); 
    lv_scr_load(splash);
    return splash;
}

lv_obj_t * blankScreenLoad(bool invert) {
    ESP_LOGI(TAG, "load blank screen with color %s", invert ? "black" : "white");
    lv_obj_t * panel = lv_obj_create(0);
    lv_obj_set_width(panel, lv_pct(100));
    lv_obj_set_height(panel, lv_pct(100));
    lv_obj_set_style_radius(panel, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_clear_flag(panel, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);  /// Flags
    lv_obj_set_style_bg_color(panel, invert ? lv_color_black() : lv_color_white(), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_scr_load(panel);
    return panel;
}


lv_obj_t * textScreenLoad(bool invert) {
    ESP_LOGI(TAG, "load blank screen with color %s", invert ? "black" : "white");
    lv_obj_t * panel = lv_obj_create(0);
    lv_obj_set_width(panel, lv_pct(100));
    lv_obj_set_height(panel, lv_pct(100));
    lv_obj_set_style_radius(panel, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_clear_flag(panel, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);  /// Flags
    lv_obj_set_style_bg_color(panel, invert ? lv_color_black() : lv_color_white(), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(panel, invert ? lv_color_white() : lv_color_black(), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(panel, &ui_font_OpenSansBold28p2, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_t * upbox = ui_common_panel_init(panel, 100, 49);
    lv_obj_align(upbox, LV_ALIGN_TOP_MID, 0, 0);
    lv_obj_t *label = lv_label_create(upbox);
    lv_label_set_text(label, "109.05");
    lv_obj_align(label, LV_ALIGN_BOTTOM_MID, 0, lv_pct(0));

    lv_obj_t * lowbox = ui_common_panel_init(panel, 100, 49);
    lv_obj_align(lowbox, LV_ALIGN_BOTTOM_MID, 0, 0);
    label = lv_label_create(lowbox);
    lv_label_set_text(label, "105.72");
    lv_obj_align(label, LV_ALIGN_TOP_MID, 0, lv_pct(10));

    lv_scr_load(panel);
    return panel; 
}


typedef struct sleep_scr_s {
    float data;
    const char *info;
};
static struct sleep_scr_s sleep_scr_info_fields[2][6] = {
    {
        {100.49, "AV:"},
        {100.49, "R1:"},
        {100.49, "R2:"},
        {100.49, "R3:"},
        {100.49, "R4:"},
        {100.49, "R5:"}
    },
    {
        {101.25, "2sec:"},
        {54.16, "1h:"},
        {95.99, "500m:"},
        {94.81, "NM:"},
        {54.26, "Dist:"},
        {54.99, "Alfa:"}
    }
};
static lv_obj_t * scr = 0;

static void timer_cb(lv_timer_t *timer) {
    update_bat(0);
    char tmp[24] = {0}, *p = tmp;
    lv_obj_t *panel;
    printf("----------------------------------------\n");
    printf("timer_cb: %d\n", count);
    printf("----------------------------------------\n");
    float last_speed = randomFloat();
    cur_speed[index_speed++ % 5] = last_speed;
    if (last_speed > max_speed)
        max_speed = last_speed;
    avg_speed = averageFloat(cur_speed, 5);
    lv_obj_t * lscr = scr;
    if(button_down)
        return;
    // if (count == 0) {
    //     scr = splashScreenLoad();
    // }
    // else if (count < 2) {
    //     scr = textScreenLoad(false);
    // }
    else if (count < 3) {
        scr = blankScreenLoad(false);
    }
    
    // else if(count < 3) {
    //     showBootScreen("Booting");
    // }

    // else if(count < 5) {
    //     showLowBatScreen();
    // }

    else if(count < 7) {
        showGpsScreen("GPS", "gps test", angle);
        angle += 150;
        if(angle > 3500)
        angle = 0;
    }
    else if(count < 9){
        if(ui_info_screen.screen.unload)
            ui_info_screen.screen.unload();
        showSpeedScreen();
#if defined(STATUS_PANEL_V1)
        ui_status_panel_t * statusbar = &ui_status_panel;
#else
    lv_statusbar_t * statusbar = (lv_statusbar_t *)ui_StatusPanel;
#endif
        f2_to_char(voltage_bat, p);
        lv_label_set_text(statusbar->bat_label, p);
        if(count % 3 == 0) {
            f2_to_char(last_speed, p);
                lv_label_set_text(ui_speed_screen.speed, p);
                f2_to_char(max_speed, p);
                lv_label_set_text(ui_speed_screen.cells[0][0].title, p);
                f2_to_char(avg_speed, p);
                lv_label_set_text(ui_speed_screen.cells[0][1].title, p);
        }
        else {
                lv_label_set_text(ui_speed_screen.speed, "0.00");
                f2_to_char(max_speed, p);
                lv_label_set_text(ui_speed_screen.cells[0][0].title, "0.00");
                f2_to_char(avg_speed, p);
                lv_label_set_text(ui_speed_screen.cells[0][1].title, "0.00");
        }
    }
    
    else if(count < 11) {
        showStatsScreen12();
        f2_to_char(last_speed, p);
        lv_label_set_text(ui_stats_screen.cells[0][0].title, p);
        lv_label_set_text(ui_stats_screen.cells[0][0].info, "500M");
    }

    else if(count < 13) {
        showStatsScreen22();
        f2_to_char(last_speed, p);
        lv_label_set_text(ui_stats_screen.cells[0][0].title, p);
        lv_label_set_text(ui_stats_screen.cells[0][0].info, "MILE");
    }

    else if(count < 15) {
        showStatsScreen32();
        f2_to_char(last_speed, p);
        lv_label_set_text(ui_stats_screen.cells[0][0].title, p);
        lv_label_set_text(ui_stats_screen.cells[0][0].info, "AVG");
    }

    else 
    if(count < 17) {
        if(ui_stats_screen.screen.unload)
            ui_stats_screen.screen.unload();
        showSleepScreen();
#if defined(STATUS_PANEL_V1)
        ui_status_panel_t * statusbar = &ui_status_panel;
#else
    lv_statusbar_t * statusbar = (lv_statusbar_t *)ui_StatusPanel;
#endif
        lv_label_set_text(statusbar->time_label, "12:00 2024-01-01");
        lv_label_set_text(statusbar->bat_label, "97%");
        for(int i = 0; i < 6; i++) {
            for(int j = 0; j < 2; j++) {
                f2_to_char(sleep_scr_info_fields[j][i].data, p);
                lv_label_set_text(ui_sleep_screen.cells[i][j].title, p);
                lv_label_set_text(ui_sleep_screen.cells[i][j].info, sleep_scr_info_fields[j][i].info);
            }
        }
    }
    else 
    if(count < 19) {
        showGpsTroubleScreen();
    }
    else 
    if(count < 21) {
        showWifiScreen("majasa","10.0.0.1");
    }
    // else if (count < 24) {
    //     mainScereenLoad();
    //     panel = ui_main_screen->panel_info;
    //     if (_lvgl_lock(-1)) {
    //         lv_main_screen_show_panel(ui_main_screen, LV_MAIN_SCREEN_PANEL_BOOT);
    //         // lv_obj_report_style_change(0);
    //         // lv_obj_update_layout(panel);
    //         _lvgl_unlock();
    //     }
    // } else if (count < 27) {
    //     ESP_LOGI(TAG, "l < 10");
    //     panel = ui_main_screen->panel_info;
    //     if (_lvgl_lock(-1)) {
    //         lv_main_screen_show_panel(ui_main_screen, LV_MAIN_SCREEN_PANEL_INFO);
    //         lv_statusbar_set_bat(ui_main_screen->statusbar, "90%");
    //         f2_to_char(last_speed, p);
    //         lv_info_panel_set_title(panel, p);
    //         // lv_obj_report_style_change(0);
    //         // lv_obj_update_layout(panel);
    //         _lvgl_unlock();
    //     }
    //     //     lv_disp_load_scr( ui_InfoScreen);
    //     //     lv_obj_clear_flag( ui_InfoInitPanel, LV_OBJ_FLAG_HIDDEN );
    //     //     //delay_ms(1500);
    // } else if (count < 30) {
    //     ESP_LOGI(TAG, "l < 15");
    //     panel = ui_main_screen->panel_stat;
    //     if (_lvgl_lock(-1)) {
    //         lv_main_screen_show_panel(ui_main_screen, LV_MAIN_SCREEN_PANEL_STAT);
    //         lv_stat_panel_setup_mode(panel, 0);
    //         f2_to_char(last_speed, p);
    //         lv_cell_set_data(((lv_stat_panel_t *)panel)->row_panels[0], p, "10SEC");
    //         f2_to_char(max_speed, p);
    //         lv_two_col_panel_set_left_data(((lv_stat_panel_t *)panel)->row_panels[1], p, "MAX");
    //         f2_to_char(avg_speed, p);
    //         lv_two_col_panel_set_right_data(((lv_stat_panel_t *)panel)->row_panels[1], p, "AVG");
    //         // lv_obj_report_style_change(0);
    //         // lv_obj_update_layout(panel);
    //         _lvgl_unlock();
    //     }
    //     //     lv_obj_add_flag( ui_InfoInitPanel, LV_OBJ_FLAG_HIDDEN );
    //     //         angle += 150;
    //     //     if(angle > 35000)
    //     //         angle = 0;
    //     //     set_angle(ui_MainImage, angle);
    // } else if (count < 33) {
    //     ESP_LOGI(TAG, "l < 20");
    //     panel = ui_main_screen->panel_stat;
    //     if (_lvgl_lock(-1)) {
    //         lv_main_screen_show_panel(ui_main_screen, LV_MAIN_SCREEN_PANEL_STAT);
    //         lv_stat_panel_setup_mode(panel, 1);
    //         // lv_obj_report_style_change(0);
    //         // lv_obj_update_layout(panel);
    //         _lvgl_unlock();
    //     }
    // } else if (count < 36) {
    //     ESP_LOGI(TAG, "l < 25");
    //     panel = ui_main_screen->panel_stat;
    //     if (_lvgl_lock(-1)) {
    //         lv_main_screen_show_panel(ui_main_screen, LV_MAIN_SCREEN_PANEL_STAT);
    //         lv_stat_panel_setup_mode(panel, 2);
    //         // lv_obj_report_style_change(0);
    //         // lv_obj_update_layout(panel);
    //         _lvgl_unlock();
    //     }
    // } else if (count < 39) {
    //     ESP_LOGI(TAG, "l < 30");
    //     panel = ui_main_screen->panel_stat;
    //     if (_lvgl_lock(-1)) {
    //         lv_main_screen_show_panel(ui_main_screen, LV_MAIN_SCREEN_PANEL_STAT);
    //         lv_stat_panel_setup_mode(panel, 3);
    //         // lv_obj_report_style_change(0);
    //         // lv_obj_update_layout(panel);
    //         _lvgl_unlock();
    //     }
    // } else if (count < 342) {
    //     ESP_LOGI(TAG, "l < 35");
    //     panel = ui_main_screen->panel_speed;
    //     if (_lvgl_lock(-1)) {
    //         lv_main_screen_show_panel(ui_main_screen, LV_MAIN_SCREEN_PANEL_SPEED);
    //         f2_to_char(last_speed, p);
    //         lv_cell_set_data(((lv_speed_panel_t *)panel)->main_panel, p, "km/h");
    //         f2_to_char(max_speed, p);
    //         lv_two_col_panel_set_left_data((((lv_speed_panel_t *)panel)->secondary_panel), p, "MAX");
    //         f2_to_char(avg_speed, p);
    //         lv_two_col_panel_set_right_data((((lv_speed_panel_t *)panel)->secondary_panel), p, "MAX");

    //         // lv_obj_report_style_change(0);
    //         // lv_obj_update_layout(panel);
    //         _lvgl_unlock();
    //     }
        //     lv_obj_add_flag( ui_InfoInitPanel, LV_OBJ_FLAG_HIDDEN );
        //         angle += 150;
        //     if(angle > 35000)
        //         angle = 0;
        //     set_angle(ui_MainImage, angle);
    // }
// end:
   if(lscr) {
        if(lscr==scr)
            scr = 0;
        lv_obj_del(lscr);
    }
    if (count++ >= 21)
        count = 1;
}

void ui_demo(void) {
    button_init();
    btns[0].cb = button_cb;
    init_adc();
    ui_common_init();
    ESP_LOGI(TAG, "create timer with 3,5sec interval");

    lv_timer_t *timer = lv_timer_create(timer_cb, 4000, NULL);
    lv_timer_ready(timer);
}
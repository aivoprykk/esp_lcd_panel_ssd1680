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
#include "esp_log.h"

static lv_obj_t *meter;
static lv_obj_t *label;
static const char *TAG = "lvgl_demo_ui";
static int i = 0;
static float voltage_bat = 0.0;
#define MINIMUM_VOLTAGE 3.2

static void update_bat(uint8_t verbose) {
    voltage_bat = volt_read();
#if defined(DEBUG)
    if (verbose)
        ESP_LOGI(TAG, "[%s] Battery measured (computed:%.02f, required_min:%.02f)\n", __FUNCTION__, voltage_bat, MINIMUM_VOLTAGE);
#endif
}

// static void set_value(void *indic, int32_t v) {
//     lv_meter_set_indicator_end_value(meter, indic, v);
// }

// #include "ui.h"
// #include "ui_themes.h"
#include "ui_common.h"

static int count = 0, angle = 0;
static float max_speed = 0, avg_speed = 0, cur_speed[5] = {0};
static uint8_t index_speed = 0;

// static void set_angle(void * img, int32_t v)
// {
//     lv_img_set_src(img, &ui_img_near_me_fill0_wght400_grad0_opsz24_png);
//     lv_img_set_angle(img, v);
// }
static bool button_down = false;
static void button_cb(int num, l_button_ev_t ev) {
#if defined(DEBUG)
    ESP_LOGI(TAG, "Button %d event: %d", num, ev);
#endif
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
            showPushScreen(0,"");
        break;
    case L_BUTTON_LONG_PRESS_START:
        if(num==0)
            showPushScreen(1,"");
        break;
    case L_BUTTON_LONG_LONG_PRESS_START:
        if(num==0)
            showPushScreen(2,"");
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
#if defined(DEBUG)
    ESP_LOGI(TAG, "load splash screen");
#endif
    lv_obj_t * splash = ui_common_panel_init(NULL, 100, 100);
    lv_obj_t *img = lv_img_create(splash);
    lv_obj_set_size(img, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
    lv_img_set_src(img, &speed_raw_250x122);
    lv_obj_align(img, LV_ALIGN_TOP_LEFT, 0, 0); 
    lv_scr_load(splash);
    return splash;
}

lv_obj_t * blankScreenLoad(bool invert) {
#if defined(DEBUG)
    ESP_LOGI(TAG, "load blank screen with color %s", invert ? "black" : "white");
#endif
    lv_obj_t * panel = lv_obj_create(0);
    lv_obj_set_width(panel, lv_pct(100));
    lv_obj_set_height(panel, lv_pct(100));
    lv_obj_set_style_radius(panel, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_clear_flag(panel, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);  /// Flags
    lv_obj_set_style_bg_color(panel, invert ? lv_color_black() : lv_color_white(), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_scr_load(panel);
    return panel;
}

typedef struct sleep_scr_s {
    float data;
    const char *info;
} sleep_scr_t;

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
    else if (count == 0) {
        ESP_LOGI(TAG, "load blank screen");
        scr = blankScreenLoad(false);
    }
    if(count == 1) {
        ESP_LOGI(TAG, "load sleep screen");
        showSleepScreen();
        ui_status_panel_t * statusbar = &ui_status_panel;
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
    else if (count <= 2) {
        ESP_LOGI(TAG, "load record screen");
        showRecordScreen(0);
    }
    
    else if(count == 3) {
        ESP_LOGI(TAG, "load boot screen");
         showBootScreen("Booting");
    }

    else if(count == 4) {
        ESP_LOGI(TAG, "load low battery screen");
         showLowBatScreen();
    }

    else if(count == 5) {
        ESP_LOGI(TAG, "load gps screen");
        showGpsScreen("GPS", "gps test", "gps data", 0, angle);
        angle += 150;
        if(angle > 3500)
        angle = 0;
    }
    else if(count == 6) {
        ESP_LOGI(TAG, "load wifi screen");
        loadStatsScreen(4,1);
        f2_to_char(last_speed, p);
        lv_label_set_text(ui_stats_screen.cells[0][0].title, p);
        lv_label_set_text(ui_stats_screen.cells[0][0].info, "AVG");
    }
    else if(count == 7){
        ESP_LOGI(TAG, "load speed screen");
        showSpeedScreen();
        ui_status_panel_t * statusbar = &ui_status_panel;
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
    
    else if(count == 8) {
        ESP_LOGI(TAG, "load stats screen");
        loadStatsScreen(3,1);
        f2_to_char(last_speed, p);
        lv_label_set_text(ui_stats_screen.cells[0][0].title, p);
        lv_label_set_text(ui_stats_screen.cells[0][0].info, "500M");
    }

    else if(count == 9) {
        ESP_LOGI(TAG, "load stats screen");
        showStatsScreen22();
        f2_to_char(last_speed, p);
        lv_label_set_text(ui_stats_screen.cells[0][0].title, p);
        lv_label_set_text(ui_stats_screen.cells[0][0].info, "MILE");
    }

    else if(count == 10) {
        ESP_LOGI(TAG, "load stats screen");
        showStatsScreen32();
        f2_to_char(last_speed, p);
        lv_label_set_text(ui_stats_screen.cells[0][0].title, p);
        lv_label_set_text(ui_stats_screen.cells[0][0].info, "AVG");
    }

    else if(count == 11) {
        ESP_LOGI(TAG, "load gps trouble screen");
        showGpsTroubleScreen();
    }
    else if(count == 12) {
        ESP_LOGI(TAG, "load wifi screen");
        showWifiScreen("majasa","10.0.0.1", "password");
    }
    else if (count == 13) {
        ESP_LOGI(TAG, "load splash screen");
        scr = splashScreenLoad();
    }

   if(lscr) {
    ESP_LOGI(TAG, "delete scr");
        if(lscr==scr)
            scr = 0;
        lv_obj_del(lscr);
    }

    if (count++ > 12)
        count = 0;
}

void ui_demo(void) {
    button_init();
    btns[0].cb = button_cb;
    init_adc();
    ui_common_init();
#if defined(DEBUG)
    ESP_LOGI(TAG, "create timer with 3,5sec interval");
#endif
    lv_timer_t *timer = lv_timer_create(timer_cb, 1000, NULL);
    lv_timer_ready(timer);
}
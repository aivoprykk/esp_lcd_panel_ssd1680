/*
 * SPDX-FileCopyrightText: 2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
#pragma once

#include <stdint.h>
#include "lvgl.h"

#ifdef __cplusplus
extern "C" {
#endif

extern const uint8_t speed_raw_250x122_map[];
extern const lv_img_dsc_t speed_raw_250x122;
extern const uint8_t speed_raw_122x250_map[];
extern const lv_img_dsc_t speed_raw_122x250;
extern const uint8_t speed_raw_122[];
extern const uint8_t BITMAP_128_64[];
extern const uint8_t BITMAP_64_128[];
extern const uint8_t esp_gps_logo_122[];

#ifdef __cplusplus
}
#endif

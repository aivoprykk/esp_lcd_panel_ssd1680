/*
 * SPDX-FileCopyrightText: 2021-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#include <stdint.h>

#if (defined(LCD_ENABLE_DEBUG_LOG))
#include "esp_timer.h"
#define DEBUG_LOG(a, b, ...) ESP_LOGI(a, b, __VA_ARGS__)
#define DEBUG_MEAS_START() uint64_t _start = (esp_timer_get_time()), _end = 0
#define DEBUG_MEAS_END(a, b, ...) \
    _end = (esp_timer_get_time());  \
    ESP_LOGI(a, b, __VA_ARGS__, _end - _start)
#else
#define DEBUG_LOG(a, b, ...) ((void)0)
#define DEBUG_MEAS_START() ((void)0)
#define DEBUG_MEAS_END(a, b, ...) ((void)0)
#endif

/* SSD1680 panel commands */


// --- reset
#define SSD1680_CMD_SWRST                   0x12
// --- Driver output control
#define SSD1680_CMD_OUTPUT_CTRL             0x01
#define SSD1680_PARAM_OUTPUT_CTRL           ((const uint8_t[]) {0x00, 0x01, 0x00}) // 100000000 = 01 00 = 256 gates used of 296
// --- Data Entry Sequence Setting
#define SSD1680_CMD_DATA_ENTRY_MODE         0x11
// A [1:0] = ID[1:0], A[2] = AM
// the address counter is updated in the X direction
// 000 - Y decrement, X decrement
#define SSD1680_PARAM_DATA_ENTRY_MODE_0       0x00
// 001 – Y decrement, X increment
#define SSD1680_PARAM_DATA_ENTRY_MODE_1       0x01
// 010 - Y increment, X decrement
#define SSD1680_PARAM_DATA_ENTRY_MODE_2       0x02
// 011 - Y increment, X increment
// AM = 1, the address counter is updated in the Y direction
#define SSD1680_PARAM_DATA_ENTRY_MODE_3       0x03
// 100 - Y decrement, X decrement
#define SSD1680_PARAM_DATA_ENTRY_MODE_4       0x04
// 101 – Y decrement, X increment
#define SSD1680_PARAM_DATA_ENTRY_MODE_5       0x05
// 110 - Y increment, X decrement
#define SSD1680_PARAM_DATA_ENTRY_MODE_6       0x06
// 111 - Y increment, X increment
#define SSD1680_PARAM_DATA_ENTRY_MODE_7       0x07
// --- Set RAMX Start/End Position
#define SSD1680_CMD_SET_RAMX_START_END_POS  0x44
// --- Set RAMY Start/End Position
#define SSD1680_CMD_SET_RAMY_START_END_POS  0x45
// --- Border Waveform Control
#define SSD1680_CMD_SET_BORDER_WAVEFORM     0x3c
// Select VBD as GS Transition,
// Fix Level Setting for VBD VSS,
// GS Transition control Follow LUT
// GS Transition setting for VBD LUT1
#define SSD1680_PARAM_BORDER_WAVEFORM       0x05
// --- Temperature Sensor Control
#define SSD1680_CMD_SET_TEMP_SENSOR         0x18
// Select to use internal sensor, 0x48 for external
#define SSD1680_PARAM_TEMP_SENSOR           0x80
// --- Display Update Control 2
#define SSD1680_CMD_SET_DISP_UPDATE_CTRL    0x22
// Enable clock signal
// Enable Analog
// -- >>
// Load LUT with DISPLAY mode 1
// Disable clock signal
// Disable OSC
#define SSD1680_PARAM_DISP_UPDATE_MODE_3    0xc7
// Like prev but:
// Display with DISPLAY Mode 2
#define SSD1680_PARAM_DISP_UPDATE_MODE_2    0xcf
// Enable clock signal
// Enable Analog
// Load temperature value -- >>
// Display with DISPLAY Mode 1
// Disable Analog
// Disable OSC
#define SSD1680_PARAM_DISP_UPDATE_MODE_1    0xf7
// (Default) Like prev but:
// Display with DISPLAY Mode 2
#define SSD1680_PARAM_DISP_UPDATE_MODE_0    0xff
// --- Active display update sequence
#define SSD1680_CMD_ACTIVE_DISP_UPDATE_SEQ  0x20
// ---
#define SSD1680_CMD_DISP_UPDATE_CTRL        0x21
#define SSD1680_PARAM_COLOR_BW_INVERSE_BIT  (1<<3)
#define SSD1680_PARAM_COLOR_RW_INVERSE_BIT  (1<<7)
// --- Init settings for the RAM address
#define SSD1680_CMD_SET_INIT_X_ADDR_COUNTER 0x4e
#define SSD1680_CMD_SET_INIT_Y_ADDR_COUNTER 0x4f
// --- Options for LUT
// Write LUT Register
// Write LUT register from MCU interface
// [153 bytes], which contains the content of
// VS[nX-LUTm], TP[nX], RP[n], SR[nXY],
// and FR[n]
#define SSD1680_CMD_SET_LUT_REG             0x32
// 153 bytes of data
// End Option
#define SSD1680_CMD_SET_END_OPTION          0x3f
#define SSD1680_PARAM_END_OPTION_KEEP       0x07
// Gate driving voltage
#define SSD1680_CMD_SET_GATE_DRIVING_VOLTAGE    0x03
// 20V
#define SSD1680_PARAM_GATE_DRIVING_VOLTAGE  0x17
// Source driving voltage
#define SSD1680_CMD_SET_SRC_DRIVING_VOLTAGE 0x04
#define SSD1680_PARAM_SRC_DRIVING_VOLTAGE   ((const uint8_t[]) {0x41, 0x00, 0x32})
// Write VCOM Register
#define SSD1680_CMD_SET_VCOM_REG            0x2c
// -0.8V
#define SSD1680_PARAM_VCOM_VOLTAGE          0x20
// --- Commands for VRAM
#define SSD1680_CMD_WRITE_BLACK_VRAM        0x24
#define SSD1680_CMD_WRITE_RED_VRAM          0x26

#define SSD1680_CMD_SLEEP_CTRL              0x10
// normal sleep mode
#define SSD1680_PARAM_SLEEP_MODE_0          0x00
// deep sleep 1
#define SSD1680_PARAM_SLEEP_MODE_1          0x01
// deep sleep 2
#define SSD1680_PARAM_SLEEP_MODE_2          0x03
// used for exit sleep mode
#define SSD1680_PARAM_SLEEP_MODE_F          0xff

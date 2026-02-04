#include <inttypes.h>
#include <stdlib.h>
#include <string.h>
#define CONFIG_LCD_ENABLE_DEBUG_LOG 1
#if CONFIG_LCD_ENABLE_DEBUG_LOG
// The local log level must be defined before including esp_log.h
// Set the maximum log level for this source file
#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG
#endif
#include "driver/gpio.h"
#include "driver/spi_common.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_ssd168x.h"

#include "img_bitmap.h"
#include "ssd168x_waveshare_lut.h"

// ============================================================================
// LUT Test Configuration
// ============================================================================
#define LUT_SIZE 159
#define CYCLES_PER_ROUND 5 // Number of partial refresh cycles per LUT
#define NUM_TEST_CONFIGS 5 // Number of LUT configurations to test

// VS voltage patterns (3 bytes per LUT: voltage settings)
typedef struct {
	uint8_t lut0[3]; // BB: Black → Black
	uint8_t lut1[3]; // WW: White → White
	uint8_t lut2[3]; // BW: Black → White
	uint8_t lut3[3]; // WB: White → Black
} vs_pattern_t;

// Timing group patterns (7 bytes per group)
typedef struct {
	uint8_t grp0[7]; // Group 0
	uint8_t grp1[7]; // Group 1
	uint8_t grp2[7]; // Group 2
} timing_pattern_t;

// Test configuration entry
typedef struct {
	const char *name;
	vs_pattern_t vs;
	timing_pattern_t timing;
	uint8_t phase_ctrl[6]; // FR[0-5]
	uint8_t keep_bit;	   // END_OPTION byte
} lut_test_config_t;

// Generated LUT buffer
static uint8_t generated_lut[LUT_SIZE];

// Test configurations array
static const lut_test_config_t
	test_configs[NUM_TEST_CONFIGS] =
		{
			{
				.name = "Standard Fast (baseline)",
				.vs =
					{
						.lut0 = {0x80, 0x48, 0x40}, // BB
						.lut1 = {0x40, 0x48, 0x80}, // WW
						.lut2 = {0x80, 0x48, 0x40}, // BW
						.lut3 = {0x40, 0x48, 0x80}, // WB
					},
				.timing =
					{
						.grp0 = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
						.grp1 = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
						.grp2 = {0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
					},
				.phase_ctrl = {0x22, 0x22, 0x22, 0x22, 0x22, 0x22},
				.keep_bit = 0x22,
			},
			{
				.name = "Enhanced White (WW stronger)",
				.vs =
					{
						.lut0 = {0x80, 0x48, 0x40}, // BB
						.lut1 = {0xA0, 0x60, 0x30}, // WW - stronger white
						.lut2 = {0x80, 0x48, 0x40}, // BW
						.lut3 = {0xA0, 0x60, 0x30}, // WB - consistent
					},
				.timing =
					{
						.grp0 = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
						.grp1 = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
						.grp2 = {0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
					},
				.phase_ctrl = {0x22, 0x22, 0x22, 0x22, 0x22, 0x22},
				.keep_bit = 0x07, // Keep previous output
			},
			{
				.name = "Low Power Phases",
				.vs =
					{
						.lut0 = {0x80, 0x48, 0x40},
						.lut1 = {0x40, 0x48, 0x80},
						.lut2 = {0x80, 0x48, 0x40},
						.lut3 = {0x40, 0x48, 0x80},
					},
				.timing =
					{
						.grp0 = {0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
						.grp1 = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
						.grp2 = {0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
					},
				.phase_ctrl = {0x03, 0x03, 0x03, 0x03, 0x03, 0x03},
				.keep_bit = 0x22,
			},
			{
				.name = "Aggressive Groups",
				.vs =
					{
						.lut0 = {0x80, 0x48, 0x40},
						.lut1 = {0x40, 0x48, 0x80},
						.lut2 = {0x80, 0x48, 0x40},
						.lut3 = {0x40, 0x48, 0x80},
					},
				.timing =
					{
						.grp0 = {0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
						.grp1 = {0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
						.grp2 = {0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
					},
				.phase_ctrl = {0x26, 0x26, 0x26, 0x26, 0x26, 0x26},
				.keep_bit = 0x22,
			},
			{
				.name = "Balanced Speed/Quality",
				.vs =
					{
						.lut0 = {0x80, 0x48, 0x40},
						.lut1 = {0xA0, 0x60, 0x30}, // Enhanced white
						.lut2 = {0x80, 0x48, 0x40},
						.lut3 = {0xA0, 0x60, 0x30},
					},
				.timing =
					{
						.grp0 = {0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
						.grp1 = {0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
						.grp2 = {0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
					},
				.phase_ctrl = {0x26, 0x26, 0x26, 0x26, 0x26, 0x26},
				.keep_bit = 0x07, // Keep previous
			},
};

/**
 * @brief Generate a test LUT from configuration
 */
static void generate_test_lut(uint8_t *out, const lut_test_config_t *config) {
	memset(out, 0, LUT_SIZE);
	int idx = 0;

	// VS Groups (60 bytes): 5 LUTs × 12 bytes each
	// LUT0 (BB)
	memcpy(&out[idx], config->vs.lut0, 3);
	idx += 3;
	memset(&out[idx], 0, 9);
	idx += 9;
	// LUT1 (WW)
	memcpy(&out[idx], config->vs.lut1, 3);
	idx += 3;
	memset(&out[idx], 0, 9);
	idx += 9;
	// LUT2 (BW)
	memcpy(&out[idx], config->vs.lut2, 3);
	idx += 3;
	memset(&out[idx], 0, 9);
	idx += 9;
	// LUT3 (WB)
	memcpy(&out[idx], config->vs.lut3, 3);
	idx += 3;
	memset(&out[idx], 0, 9);
	idx += 9;
	// LUT4 (VCOM/border) - all zeros
	memset(&out[idx], 0, 12);
	idx += 12;

	// Timing Groups (84 bytes): 12 groups × 7 bytes
	memcpy(&out[idx], config->timing.grp0, 7);
	idx += 7;
	memcpy(&out[idx], config->timing.grp1, 7);
	idx += 7;
	memcpy(&out[idx], config->timing.grp2, 7);
	idx += 7;
	// Groups 3-11: all zeros
	memset(&out[idx], 0, 9 * 7);
	idx += 9 * 7;

	// Phase Control (9 bytes)
	memcpy(&out[idx], config->phase_ctrl, 6);
	idx += 6;
	memset(&out[idx], 0, 3);
	idx += 3;

	// Registers (6 bytes): END_OPTION, GATE_V, SRC_V[3], VCOM
	out[idx++] = config->keep_bit; // END_OPTION
	out[idx++] = 0x17;			   // Gate driving voltage (20V)
	out[idx++] = 0x41;			   // Source voltage 1
	out[idx++] = 0x00;			   // Source voltage 2
	out[idx++] = 0x32;			   // Source voltage 3
	out[idx++] = 0x20;			   // VCOM (-0.8V) 0x36 = -1.0V
}

// Rotation definitions matching driver_ssd168x.c
#define DISP_ROT_NONE 0
#define DISP_ROT_90 1
#define DISP_ROT_180 2
#define DISP_ROT_270 3

// Remove conflicting macro definitions - use from logger_common.h
// #define ROUND_UP_TO_8(x) (((x) + 7) & ~7)

// SPI Bus
#define EPD_PANEL_SPI_CLK 1000000
#define EPD_PANEL_SPI_CMD_BITS 8
#define EPD_PANEL_SPI_PARAM_BITS 8
#define EPD_PANEL_SPI_MODE 0
// e-Paper GPIO
#define EXAMPLE_PIN_NUM_EPD_DC 17
#define EXAMPLE_PIN_NUM_EPD_RST 16
#define EXAMPLE_PIN_NUM_EPD_CS 5
#define EXAMPLE_PIN_NUM_EPD_BUSY 4
// e-Paper SPI
#define EXAMPLE_PIN_NUM_MOSI 23
#define EXAMPLE_PIN_NUM_SCLK 18
#define LCD_HOST SPI2_HOST

// e-Paper panel
#if defined(CONFIG_SSD168X_PANEL_SSD1681)
#define LCD_H_RES (200)		// panel x res
#define LCD_V_RES (200)		// panel y res
#define LCD_H_VISIBLE (200) // vertical
#define LCD_H_GAP (0)
#define LCD_V_GAP (0)
#else
#define LCD_H_RES (128)		// panel x res
#define LCD_H_VISIBLE (122) // vertical
#define LCD_V_RES (250)		// panel y res
#define LCD_H_GAP (6)
#define LCD_V_GAP (0)
#endif

#define LCD_RESOLUTION (LCD_H_RES * LCD_V_RES)
#define LCD_ROW_LEN (LCD_H_RES >> 3u)
#define LCD_PIXELS (LCD_V_RES * LCD_ROW_LEN)
#define ROUND_UP_TO_8(x) (((x) + 7) & ~7U)
#define LCD_PIXELS_ALIGNED (ROUND_UP_TO_8(LCD_H_RES) * ROUND_UP_TO_8(LCD_V_RES))
#define LCD_PIXELS_MEM_ALIGNED (LCD_PIXELS_ALIGNED >> 3u)

#define H_NORM_PX_VISIBLE(h_scr_percent)                                       \
	((int16_t)((LCD_H_VISIBLE / 100.0) * (h_scr_percent)))
#define V_NORM_PX(v_scr_percent)                                               \
	((int16_t)((LCD_V_RES / 100.0) * (v_scr_percent)))

#define WAIT_TIME_MS 5000
unsigned long IRAM_ATTR get_millis() {
	return (unsigned long)(esp_timer_get_time() / 1000);
}
#define DELAY_MS(x)                                                            \
	vTaskDelay((x + (portTICK_PERIOD_MS - 1)) / portTICK_PERIOD_MS)
inline void delay_ms(uint32_t ms) { DELAY_MS(ms); }

static const char *TAG = "epaper_demo_plain";
static SemaphoreHandle_t epaper_panel_semaphore = 0;
#ifdef CONFIG_SSD168X_PANEL_SSD1681
static uint8_t fast_refresh_lut[] = _1681_LUT_FAST_REFRESH_KEEP;
#else
static uint8_t fast_refresh_lut[] = _1680_LUT_FAST_REFRESH_0;
#endif
static uint8_t *bitmap_buffer = NULL;

/**
 * @brief Area definition for partial refresh operations
 */
typedef struct {
	int x1; /*!< Left coordinate */
	int y1; /*!< Top coordinate */
	int x2; /*!< Right coordinate */
	int y2; /*!< Bottom coordinate */
} epaper_area_t;

// Utility macro for null pointer checking
#define SSD168X_CHECK_NULL_RET(ptr, err)                                       \
	do {                                                                       \
		if (!(ptr)) {                                                          \
			return (err);                                                      \
		}                                                                      \
	} while (0)

// Error checking macros to replace ESP_ERROR_CHECK
#define CHECK_ERR_RET(x)                                                       \
	do {                                                                       \
		esp_err_t err_rc_ = (x);                                               \
		if (err_rc_ != ESP_OK) {                                               \
			ESP_LOGE(TAG, "Error %s at %s:%d", esp_err_to_name(err_rc_),       \
					 __FILE__, __LINE__);                                      \
			return err_rc_;                                                    \
		}                                                                      \
	} while (0)

#define CHECK_ERR_GOTO(x, label)                                               \
	do {                                                                       \
		esp_err_t err_rc_ = (x);                                               \
		if (err_rc_ != ESP_OK) {                                               \
			ESP_LOGE(TAG, "Error %s at %s:%d", esp_err_to_name(err_rc_),       \
					 __FILE__, __LINE__);                                      \
			ret = err_rc_;                                                     \
			goto label;                                                        \
		}                                                                      \
	} while (0)

static bool give_semaphore_in_isr(const esp_lcd_panel_handle_t handle,
								  const void *edata, void *user_data) {
	SemaphoreHandle_t *epaper_panel_semaphore_ptr = user_data;
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(*epaper_panel_semaphore_ptr,
						  &xHigherPriorityTaskWoken);
	if (xHigherPriorityTaskWoken == pdTRUE) {
		portYIELD_FROM_ISR();
		return true;
	}
	return false;
}

static bool init_requested = true;
static epaper_panel_init_mode_t init_mode = INIT_MODE_FULL_2;
uint32_t flush_count = 0;
uint32_t last_flush_ms = 0;

static esp_err_t _turn_off(esp_lcd_panel_handle_t panel_handle) {
	if (epaper_panel_shut_down(panel_handle)) {
		return ESP_FAIL;
	}
	return ESP_OK;
}

static esp_err_t _turn_on(esp_lcd_panel_handle_t panel_handle) {
	ESP_LOGW(TAG, "Turning on e-paper with init mode 0x%02x", init_mode);

	const char *x = "e-Paper display...";
	if (init_requested) {
		ESP_LOGD(TAG, "[%s] %s %s with init mode 0x%02x", "Reset/Init",
				 __func__, x, init_mode);
		// if(flush_count > 0) {
		//     if(_turn_off(panel_handle)) {
		//         return ESP_FAIL;
		//     }
		// }
		if (esp_lcd_panel_reset(panel_handle)) {
			return ESP_FAIL;
		}
		delay_ms(50);
		// Initialize with built-in LUT first (pass NULL)
		// This ensures a stable state before loading custom LUT
		if (epaper_panel_init_screen_ssd168x(panel_handle, init_mode, NULL)) {
			return ESP_FAIL;
		}
		init_mode = INIT_MODE_FULL_2;
		delay_ms(50);

		// NOW load custom LUT after init completes successfully
		ESP_LOGD(TAG, "[%s] Loading custom LUT after init", __func__);
		if (epaper_panel_set_custom_lut_ssd168x(panel_handle, fast_refresh_lut,
												159)) {
			return ESP_FAIL;
		}
		init_requested = false;
	}
	if (esp_lcd_panel_disp_on_off(panel_handle, true)) {
		return ESP_FAIL;
	}
	return ESP_OK;
}

static esp_err_t _refresh_and_turn_off(esp_lcd_panel_handle_t panel_handle,
									   int rotated, epaper_area_t *area,
									   uint8_t *color_map) {
	ESP_LOGW(TAG, "Refreshing display with rotation %d", rotated);

	if (rotated == DISP_ROT_NONE || rotated == DISP_ROT_180) {
		if (esp_lcd_panel_swap_xy(panel_handle, false)) {
			return ESP_FAIL;
		}
	} else {
		if (esp_lcd_panel_swap_xy(panel_handle, true)) {
			return ESP_FAIL;
		}
	}
	// #ifdef CONFIG_DISPLAY_USE_LVGL
	//         if(esp_lcd_panel_invert_color(panel_handle, true)) {
	//             return ESP_FAIL;
	//         }
	// #endif

	if (rotated == DISP_ROT_NONE) {
		if (esp_lcd_panel_mirror(panel_handle, false, false)) {
			return ESP_FAIL;
		} // x inc y inc
	} else if (rotated == DISP_ROT_90) {
		if (esp_lcd_panel_mirror(panel_handle, false, false)) {
			return ESP_FAIL;
		} // x *dec y inc
	} else if (rotated == DISP_ROT_180) {
		if (esp_lcd_panel_mirror(panel_handle, true, true)) {
			return ESP_FAIL;
		} // x *dec y *dec
	} else { // DISP_ROT_270 (270 degrees)
		if (esp_lcd_panel_mirror(panel_handle, true, true)) {
			return ESP_FAIL;
		} // x inc y *dec
	}

	if (epaper_panel_set_bitmap_color_ssd168x(panel_handle,
											  SSD168X_EPAPER_BITMAP_BLACK)) {
		return ESP_FAIL;
	}
	if (esp_lcd_panel_draw_bitmap(panel_handle, area->x1, area->y1,
								  area->x2 + 1, area->y2 + 1, color_map)) {
		return ESP_FAIL;
	}
	if (epaper_panel_refresh_screen_ssd168x(panel_handle, 0xcf)) {
		return ESP_FAIL;
	}
	// if(epaper_panel_set_bitmap_color_ssd168x(panel_handle,
	// SSD168X_EPAPER_BITMAP_RED)) {
	//     return ESP_FAIL;
	// }
	// if(esp_lcd_panel_draw_bitmap(panel_handle,  area->x1,  area->y1, area->x2
	// + 1, area->y2 + 1, color_map)) {
	//     return ESP_FAIL;
	// }
	if (epaper_panel_update_full_screen_ssd168x(panel_handle)) {
		return ESP_FAIL;
	}
	// if(esp_lcd_panel_disp_on_off(panel_handle, false)) {
	//         return ESP_FAIL;
	// }
	return ESP_OK;
}

static inline uint8_t get_bit(const uint8_t *buf, int bit_idx) {
	return (buf[(bit_idx / 8)] >> (7u - (bit_idx % 8))) & 1u;
}

// Optional debug comparator: build canonical buffer using driver's rotate logic
#ifdef CONFIG_USE_DEBUG_IMG
static void drawImg(lcd_display_t *lcd, bool speedmode_used, int offset_x,
					int offset_y, int offsetx1, int offsety1, int offsetx2,
					int offsety2, const uint8_t *img, size_t len_x,
					size_t len_y, int w, int h, int rotated) {
	esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t)lcd->panel;
	if (!panel_handle) {
#endif

		// New optimized drawImg function using simple bitmap approach
		esp_err_t drawImg(esp_lcd_panel_handle_t panel_handle,
						  const uint8_t *img, int32_t x, int32_t y, int32_t w,
						  int32_t h, int rotated, bool use_fast_refresh) {
			ESP_LOGW(TAG, "Drawing image: x=%d, y=%d, w=%d, h=%d, rotated=%d",
					 (int)x, (int)y, (int)w, (int)h, rotated);

			if (!img || !panel_handle) {
				ESP_LOGE(TAG, "Invalid parameters");
				return ESP_ERR_INVALID_ARG;
			}

			// Wait for previous operation to complete
			xSemaphoreTake(epaper_panel_semaphore, portMAX_DELAY);

			int offsetx1 = x;
			int offsetx2 = x + w - 1;
			int offsety1 = y;
			int offsety2 = y + h - 1;

			// Used to vertical traverse lvgl framebuffer
			int len_x = abs(offsetx1 - offsetx2) + 1;
			int len_y = abs(offsety1 - offsety2) + 1;

#if !defined(CONFIG_SSD168X_PANEL_SSD1681)
			if (rotated == DISP_ROT_270 || rotated == DISP_ROT_90)
				len_y = ROUND_UP_TO_8(len_y);
			else
				len_x = ROUND_UP_TO_8(len_x);
#endif
			// else len_x = ROUND_UP_TO_8(len_x);
			// --- Convert buffer from color to monochrome bitmap
			int len_bits = (len_x * len_y);

			int32_t buffer_size = ((len_bits + 7) / 8);

			// For rotated buffers in non-copy mode, we need to use the size
			// that rotate_bitmap_debug expects
			if (epaper_panel_is_in_non_copy_mode_ssd168x(panel_handle) &&
				rotated != DISP_ROT_NONE) {
				// Calculate buffer size using source dimensions (what
				// rotate_bitmap_debug expects)
				int byte_width = (w + 7) >> 3u;
				int byte_height = (h + 7) >> 3u;
				int total_bits = (byte_width << 3u) * (byte_height << 3u);
				buffer_size = total_bits / 8;
				ESP_LOGW(
					TAG,
					"Adjusted buffer size for rotation: %d bytes (from %d)",
					(int)buffer_size, (len_bits + 7) / 8);
			}

			ESP_LOGW(TAG,
					 "Buffer dimensions: len_x=%d, len_y=%d, buffer_size=%d",
					 len_x, len_y, (int)buffer_size);
			ESP_LOGW(TAG, "Source dimensions: w=%d, h=%d", (int)w, (int)h);

			// Allocate bitmap buffer if needed
			if (!bitmap_buffer) {
				bitmap_buffer = heap_caps_malloc(buffer_size, MALLOC_CAP_DMA);
				if (!bitmap_buffer) {
					ESP_LOGE(TAG, "Failed to allocate bitmap buffer");
					return ESP_ERR_NO_MEM;
				}
			}

			// const uint8_t * buf = img;  // Unused variable removed
			if (!epaper_panel_is_in_non_copy_mode_ssd168x(panel_handle)) {
				memcpy(bitmap_buffer, img, buffer_size);
			} else {
				// Non-copy mode: produce a pre-rotated buffer that matches
				// panel memory layout. Destination stride should be the aligned
				// width in pixels (len_x), which already was rounded earlier to
				// match panel byte alignment.
				memset(bitmap_buffer, 0x00, buffer_size);
				for (int i = 0; i < 32; ++i) {
					ESP_LOGI(TAG, "buf[%d]=0x%02x", i, img[i]);
				}
				// If not rotated, simple copy works (source already packed for
				// display)
				if (rotated == DISP_ROT_NONE) {
					memcpy(bitmap_buffer, img, buffer_size);
				} else {
					// Use rotate_bitmap with NULL callback for raw bitmap data
					memset(bitmap_buffer, 0, buffer_size);
					rotate_bitmap((uint8_t *)img, bitmap_buffer, w, h,
								  rotated == DISP_ROT_NONE ? 0 : rotated, NULL,
								  0, NULL);
				}
			}

			bool ir = init_requested;
			_turn_on(panel_handle);
			// --- Draw bitmap
			int driver_rot =
				epaper_panel_is_in_non_copy_mode_ssd168x(panel_handle)
					? DISP_ROT_NONE
					: rotated;
			ESP_LOGW(TAG, "[%s] refresh and turn off (driver_rot=%d)", __func__,
					 driver_rot);
			// If we created a temporary driver-format buffer (drv_buf), use it
			// for drawing.
			uint8_t *draw_buf = bitmap_buffer;
			// size_t drv_buf_size = 0;  // Unused variable removed

			uint8_t *drv_buf_local = NULL;

			epaper_area_t draw_area = {offsetx1, offsety1, offsetx2, offsety2};
			if (epaper_panel_is_in_non_copy_mode_ssd168x(panel_handle) &&
				rotated != DISP_ROT_NONE) {
				// For rotated buffers, use source image dimensions padded to
				// byte boundaries
				int byte_width = (w + 7) >> 3u;
				int byte_height = (h + 7) >> 3u;
				int padded_w = byte_width
							   << 3u; // Source width padded to byte boundary
				int padded_h = byte_height
							   << 3u; // Source height padded to byte boundary

				// Rotated dimensions: 90/270 swap w/h, 180 keeps same
				int draw_w, draw_h;
				if (rotated == DISP_ROT_90 || rotated == DISP_ROT_270) {
					// 90°/270°: swap dimensions (w,h) → (h,w)
					draw_w = padded_h; // new width = old height
					draw_h = padded_w; // new height = old width
				} else {			   // rotated == DISP_ROT_180
					// 180°: keep same dimensions
					draw_w = padded_w;
					draw_h = padded_h;
				}
				draw_area.x2 = draw_area.x1 + draw_w - 1;
				draw_area.y2 = draw_area.y1 + draw_h - 1;
				ESP_LOGI(TAG,
						 "adjusted draw area for pre-rotated buffer: x1=%d "
						 "y1=%d x2=%d y2=%d (w=%d h=%d)",
						 draw_area.x1, draw_area.y1, draw_area.x2, draw_area.y2,
						 draw_w, draw_h);
				ESP_LOGI(TAG,
						 "source padded dimensions: padded_w=%d padded_h=%d "
						 "(from w=%d h=%d)",
						 padded_w, padded_h, (int)w, (int)h);
			}

			_refresh_and_turn_off(panel_handle, driver_rot, &draw_area,
								  draw_buf);
			if (ir) {
				_turn_on(panel_handle);
				_refresh_and_turn_off(
					panel_handle, driver_rot,
					&((epaper_area_t){offsetx1, offsety1, offsetx2, offsety2}),
					draw_buf);
			}

			if (drv_buf_local) {
				heap_caps_free(drv_buf_local);
			}

			flush_count++;
			last_flush_ms = get_millis();
			ESP_LOGW(TAG, "Image drawn successfully");
			return ESP_OK;
		}

		static esp_lcd_panel_handle_t panel_handle = NULL;

		static esp_err_t init(void) {
			esp_err_t ret = ESP_OK;
			// --- Init SPI Bus
			ESP_LOGW(TAG, "Initializing SPI Bus...");
			spi_bus_config_t buscfg = {.sclk_io_num = EXAMPLE_PIN_NUM_SCLK,
									   .mosi_io_num = EXAMPLE_PIN_NUM_MOSI,
									   .miso_io_num = -1,
									   .quadwp_io_num = -1,
									   .quadhd_io_num = -1,
									   .max_transfer_sz =
										   SOC_SPI_MAXIMUM_BUFFER_SIZE};
			ret = spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO);
			if (ret != ESP_OK) {
				ESP_LOGE(TAG, "SPI bus initialization failed: %s",
						 esp_err_to_name(ret));
				return ret;
			}

			// --- Init ESP_LCD IO
			ESP_LOGW(TAG, "Initializing panel IO...");
			esp_lcd_panel_io_handle_t io_handle = NULL;
			esp_lcd_panel_io_spi_config_t io_config = {
				.dc_gpio_num = EXAMPLE_PIN_NUM_EPD_DC,
				.cs_gpio_num = EXAMPLE_PIN_NUM_EPD_CS,
				.pclk_hz = EPD_PANEL_SPI_CLK,
				.lcd_cmd_bits = EPD_PANEL_SPI_CMD_BITS,
				.lcd_param_bits = EPD_PANEL_SPI_PARAM_BITS,
				.spi_mode = EPD_PANEL_SPI_MODE,
				.trans_queue_depth = 10,
				.on_color_trans_done = NULL};
			ret = esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST,
										   &io_config, &io_handle);
			if (ret != ESP_OK) {
				ESP_LOGE(TAG, "Panel IO creation failed: %s",
						 esp_err_to_name(ret));
				return ret;
			}

			// --- Create esp_lcd panel
			ESP_LOGW(TAG, "Creating SSD1680 panel...");
			esp_lcd_ssd168x_config_t epaper_ssd1680_config = {
				.busy_gpio_num = EXAMPLE_PIN_NUM_EPD_BUSY,
				.non_copy_mode = true, // Enable for better performance
				.height = LCD_V_RES,
				.width = LCD_H_RES,
				.buffer_size = LCD_PIXELS_MEM_ALIGNED};
			esp_lcd_panel_dev_config_t panel_config = {
				.reset_gpio_num = EXAMPLE_PIN_NUM_EPD_RST,
				.flags.reset_active_high = false,
				.vendor_config = &epaper_ssd1680_config};

			// Install GPIO ISR service before creating panel
			gpio_install_isr_service(0);
			ret = esp_lcd_new_panel_ssd168x(io_handle, &panel_config,
											&panel_handle);
			if (ret != ESP_OK) {
				ESP_LOGE(TAG, "Panel creation failed: %s",
						 esp_err_to_name(ret));
				return ret;
			}

			// Set gap if needed
#if (LCD_H_GAP > 0) || (LCD_V_GAP > 0)
			esp_lcd_panel_set_gap(panel_handle, LCD_H_GAP, LCD_V_GAP);
#endif

			// Create semaphore for synchronization
			epaper_panel_semaphore = xSemaphoreCreateBinary();
			if (!epaper_panel_semaphore) {
				ESP_LOGE(TAG, "Failed to create semaphore");
				return ESP_FAIL;
			}
			xSemaphoreGive(epaper_panel_semaphore);

			// Register refresh done callback
			epaper_panel_callbacks_t cbs = {
				.on_epaper_refresh_done = give_semaphore_in_isr,
			};
			ret = epaper_panel_register_event_callbacks_ssd168x(
				panel_handle, &cbs, &epaper_panel_semaphore);
			if (!epaper_panel_semaphore) {
				ESP_LOGE(TAG, "Failed to register callbacks");
				return ret;
			}
			return ESP_OK;
		}

		void uninit(void) {
			ESP_LOGW(TAG, "All tests completed. Going to sleep mode...");
			esp_err_t sleep_ret =
				esp_lcd_panel_disp_on_off(panel_handle, false);
			if (sleep_ret != ESP_OK) {
				ESP_LOGE(TAG, "Failed to turn off display: %s",
						 esp_err_to_name(sleep_ret));
			}

			// Clean up
			if (bitmap_buffer) {
				heap_caps_free(bitmap_buffer);
				bitmap_buffer = NULL;
			}
		}

		void app_main(void) {
			esp_err_t ret = init();
			if (ret != ESP_OK) {
				ESP_LOGE(TAG, "Initialization failed: %s",
						 esp_err_to_name(ret));
				return;
			}

			ESP_LOGW(TAG, "Starting LUT round-based tests...");
			ESP_LOGW(TAG, "Configurations: %d, Cycles per round: %d",
					 NUM_TEST_CONFIGS, CYCLES_PER_ROUND);

			// =========================================================================
			// ROUND-BASED LUT TESTING
			// =========================================================================
			for (int round = 0; round < NUM_TEST_CONFIGS; round++) {
				const lut_test_config_t *config = &test_configs[round];

				ESP_LOGW(TAG, "");
				ESP_LOGW(
					TAG,
					"=======================================================");
				ESP_LOGW(TAG, "ROUND %d: %s", round, config->name);
				ESP_LOGW(
					TAG,
					"=======================================================");

				// --- Full init at start of each round ---
				init_requested = true;
				init_mode = INIT_MODE_FULL_2;

				// Generate LUT from configuration
				generate_test_lut(generated_lut, config);

				// Copy to mutable buffer for the driver
				memcpy(fast_refresh_lut, generated_lut, LUT_SIZE);

				ESP_LOGI(TAG, "LUT VS: BB=%02x%02x%02x WW=%02x%02x%02x",
						 config->vs.lut0[0], config->vs.lut0[1],
						 config->vs.lut0[2], config->vs.lut1[0],
						 config->vs.lut1[1], config->vs.lut1[2]);
				ESP_LOGI(TAG, "Phase ctrl: %02x, Keep: %02x",
						 config->phase_ctrl[0], config->keep_bit);

				// --- Run multiple partial refresh cycles with this LUT ---
				uint32_t round_total_ms = 0;
				for (int cycle = 0; cycle < CYCLES_PER_ROUND; cycle++) {
					uint32_t cycle_start = get_millis();

					// Alternate between images for visual feedback
					const uint8_t *img;
					int img_w, img_h, rotation;

					if (cycle % 2 == 0) {
						img = speed_raw_122x250;
						img_w = 122;
						img_h = 250;
						rotation = DISP_ROT_NONE;
					} else {
						img = speed_raw_250x122;
						img_w = 250;
						img_h = 122;
						rotation = DISP_ROT_90;
					}

					ret = drawImg(panel_handle, img, 0, 0, img_w, img_h,
								  rotation, true);
					if (ret != ESP_OK) {
						ESP_LOGE(TAG, "Round %d Cycle %d: Drawing failed: %s",
								 round, cycle, esp_err_to_name(ret));
						goto cleanup;
					}

					uint32_t cycle_ms = get_millis() - cycle_start;
					round_total_ms += cycle_ms;

					ESP_LOGI(TAG, "Round %d Cycle %d: %lu ms", round, cycle,
							 (unsigned long)cycle_ms);

					// Only reset init_requested after first cycle (init happens
					// in first drawImg)
					if (cycle == 0) {
						init_requested = false;
					}
				}

				// Round summary
				uint32_t avg_ms = round_total_ms / CYCLES_PER_ROUND;
				ESP_LOGW(TAG,
						 "Round %d Summary: Total=%lu ms, Avg=%lu ms/cycle",
						 round, (unsigned long)round_total_ms,
						 (unsigned long)avg_ms);

				// Brief pause between rounds for visual inspection
				vTaskDelay(pdMS_TO_TICKS(2000));
			}
			if (ret == ESP_OK) {
				ESP_LOGW(TAG, "Demo completed successfully!");
			} else {
				ESP_LOGE(TAG, "Demo failed with error: %s",
						 esp_err_to_name(ret));
			}
		cleanup:
			uninit();
		}

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#define lengthof(x) (sizeof(x) / sizeof((x)[0]))
#define CONFIG_USE_CUSTOM_LUT 1
#if defined(CONFIG_USE_CUSTOM_LUT)
#include "ssd168x_waveshare_lut.h"
#ifdef CONFIG_SSD168X_PANEL_SSD1681
static uint8_t lut_base_25c[] = _1681_LUT_FAST_REFRESH_KEEP;
static uint8_t lut_base_25c_full[] = _1681_LUT_FULL_REFRESH;
#else
static const uint8_t lut_base_25c[] = _1680_LUT_FAST_REFRESH_0;
static const uint8_t lut_base_25c_full[] = _1680_LUT_FULL_REFRESH_1;
#endif
size_t lut_base_25c_size = lengthof(lut_base_25c);
size_t lut_base_25c_full_size = lengthof(lut_base_25c_full);
#endif

// Temperature compensation offsets structure
typedef struct {
	int8_t temp;		// Temperature in °C
	int8_t vcom_offset; // VCOM voltage adjustment
	int8_t gate_offset; // Gate voltage adjustment
	uint8_t phase_mult; // Phase timing multiplier
	bool extra_cycle;	// Add extra refresh cycle
} temp_comp_t;

// LUT mode enumeration
typedef enum { LUT_MODE_FAST_REFRESH, LUT_MODE_FULL_REFRESH } lut_mode_t;

// Temperature compensation table
const temp_comp_t temp_comp_table[] = {
	// Temp (°C), vcom, gate, phase_mult, extra_cycle
	{-10, +12, +8, 120, true}, // Very cold
	{0, +8, +6, 110, true},	   // Cold
	{10, +4, +3, 105, false},  // Cool
	{20, +2, +1, 102, false},  // Mild
	{25, 0, 0, 100, false},	   // Room temp (reference)
	{30, -2, -1, 98, false},   // Warm
	{40, -4, -3, 95, false},   // Hot
	{50, -8, -6, 90, true},	   // Very hot
};

#define TEMP_COMP_COUNT (sizeof(temp_comp_table) / sizeof(temp_comp_t))

// Compensated LUT structure
typedef struct {
	uint8_t lut[159]; // The compensated LUT data
	bool extra_cycle; // Whether to perform extra refresh cycle
} compensated_lut_t;

// Static buffer for the current compensated LUT
static compensated_lut_t current_lut;
static bool lut_initialized = false;
static lut_mode_t current_mode = LUT_MODE_FAST_REFRESH;

// Initialize the LUT generator with base LUT
void init_lut_generator(lut_mode_t mode) {
	if (!lut_initialized || mode != current_mode) {
		const uint8_t *base_lut;
		if (mode == LUT_MODE_FULL_REFRESH) {
			base_lut = lut_base_25c_full;
		} else {
			base_lut = lut_base_25c;
		}
		memcpy(current_lut.lut, base_lut, sizeof(current_lut.lut));
		current_lut.extra_cycle = false;
		current_mode = mode;
		lut_initialized = true;
	}
}
uint8_t apply_offset(uint8_t base, int8_t offset);
temp_comp_t get_compensation_values(float temp);

// Generate temperature-compensated LUT
compensated_lut_t *generate_temp_compensated_lut(float temperature,
												 lut_mode_t mode) {
	if (!lut_initialized || mode != current_mode) {
		init_lut_generator(mode);
	}

	temp_comp_t comp = get_compensation_values(temperature);

	current_lut.extra_cycle = comp.extra_cycle;

	// Apply compensation to each phase (first 153 bytes)
	for (int i = 0; i < 153; i++) {
		uint8_t value = current_lut.lut[i];

		// Only compensate voltage/timing bytes (not control bytes)
		if (i % 7 < 3) { // First 3 bytes of each 7-byte group are voltages
			// Apply voltage compensation
			if (i % 7 == 0) { // VCOM
				value = apply_offset(value, comp.vcom_offset);
			} else { // Gate voltages
				value = apply_offset(value, comp.gate_offset);
			}

			// Apply timing compensation
			if (i % 7 >= 1 && i % 7 <= 2) {
				value = (value * comp.phase_mult) / 100;
			}
		}

		current_lut.lut[i] = value;
	}
	// Apply compensation to control registers (bytes 153-158)
	// 153: DISP_UPDATE_CTRL
	current_lut.lut[153] = apply_offset(current_lut.lut[153], comp.gate_offset);
	// 154: Gate driving voltage
	current_lut.lut[154] = apply_offset(current_lut.lut[154], comp.gate_offset);
	// 155-157: Source driving voltages
	for (int i = 155; i <= 157; i++) {
		current_lut.lut[i] =
			apply_offset(current_lut.lut[i],
						 comp.gate_offset); // Assuming source uses same as gate
	}
	// 158: VCOM register
	current_lut.lut[158] = apply_offset(current_lut.lut[158], comp.vcom_offset);

	return &current_lut;
}

int16_t linear_interpolate(int16_t a, int16_t b, float ratio) {
	return a + (int16_t)((b - a) * ratio + 0.5f);
}

uint8_t apply_offset(uint8_t base, int8_t offset) {
	int16_t result = base + offset;
	if (result < 0x00)
		return 0x00;
	if (result > 0xFF)
		return 0xFF;
	return (uint8_t)result;
}

temp_comp_t get_compensation_values(float temp) {
	// Clamp temperature to table range
	if (temp <= temp_comp_table[0].temp) {
		return temp_comp_table[0];
	}
	if (temp >= temp_comp_table[TEMP_COMP_COUNT - 1].temp) {
		return temp_comp_table[TEMP_COMP_COUNT - 1];
	}

	// Find the two closest temperature points
	int lower_idx = 0;
	for (int i = 0; i < TEMP_COMP_COUNT - 1; i++) {
		if (temp >= temp_comp_table[i].temp &&
			temp <= temp_comp_table[i + 1].temp) {
			lower_idx = i;
			break;
		}
	}
	int upper_idx = lower_idx + 1;

	// Linear interpolation between temperature points
	temp_comp_t result;
	float ratio =
		(temp - temp_comp_table[lower_idx].temp) /
		(temp_comp_table[upper_idx].temp - temp_comp_table[lower_idx].temp);

	result.vcom_offset =
		linear_interpolate(temp_comp_table[lower_idx].vcom_offset,
						   temp_comp_table[upper_idx].vcom_offset, ratio);

	result.gate_offset =
		linear_interpolate(temp_comp_table[lower_idx].gate_offset,
						   temp_comp_table[upper_idx].gate_offset, ratio);

	result.phase_mult = (uint8_t)linear_interpolate(
		(int16_t)temp_comp_table[lower_idx].phase_mult,
		(int16_t)temp_comp_table[upper_idx].phase_mult, ratio);

	result.extra_cycle = (temp < 15 || temp > 35) ? true : false;

	return result;
}

void generate_debug_lut(uint8_t *output, uint8_t white_vcom, uint8_t white_vgh,
						uint8_t white_vgl, bool aggressive_white) {

	memcpy(output, &lut_base_25c[0], 156);

	// Modify LUT1 (White→White) - BYTE OFFSET 12
	output[12] = white_vcom; // Byte 0 of LUT1
	output[13] = white_vgh;	 // Byte 1 of LUT1
	output[14] = white_vgl;	 // Byte 2 of LUT1

	// Also modify LUT3 (White→Black) to be consistent - BYTE OFFSET 36
	output[36] = white_vcom;
	output[37] = white_vgh;
	output[38] = white_vgl;

	if (aggressive_white) {
		// Make Group 0 active for white maintenance
		output[60] = 0x04; // Group 0: 4 phases of white refresh

		// Change phase control to use Group 0 + Group 1
		for (int i = 144; i < 150; i++) {
			output[i] = 0x26; // 0x26 = 00100110 = Groups 0,1,2 in some pattern
		}
	}
}

#if 0 // Placeholder - enable when load_and_test is implemented
// Test different combinations:
void test_white_preservation(void) {
    uint8_t test_lut[156];

    // Test 1: Moderate white voltage
    generate_debug_lut(test_lut, 0xA0, 0x60, 0x30, false);
    load_and_test(test_lut, "Test 1: A0/60/30");

    // Test 2: Strong white voltage  
    generate_debug_lut(test_lut, 0xC0, 0x70, 0x20, false);
    load_and_test(test_lut, "Test 2: C0/70/20");

    // Test 3: Very strong + aggressive
    generate_debug_lut(test_lut, 0xD0, 0x80, 0x10, true);
    load_and_test(test_lut, "Test 3: D0/80/10 + aggressive");
}
#endif

// SSD1680 LUT generator based on datasheet understanding
typedef struct {
	uint8_t voltages[5][3]; // 5 LUTs × 3 voltage bytes
	uint8_t timing[12][7];	// 12 groups × 7 timing bytes
	uint8_t phase_ctrl[9];	// Phase control
	uint8_t frame_count;	// Frame setting
} ssd168x_lut_t;

void generate_ssd1680_partial_lut(ssd168x_lut_t *lut, bool preserve_white) {
	// Initialize with zeros
	memset(lut, 0, sizeof(ssd168x_lut_t));

	// Set voltages
	// LUT0: BB
	lut->voltages[0][0] = 0x80;
	lut->voltages[0][1] = 0x48;
	lut->voltages[0][2] = 0x40;

	// LUT1: WW - key for white preservation
	if (preserve_white) {
		lut->voltages[1][0] = 0xA0; // Higher for better white
		lut->voltages[1][1] = 0x60;
		lut->voltages[1][2] = 0x30;
	} else {
		lut->voltages[1][0] = 0x40;
		lut->voltages[1][1] = 0x48;
		lut->voltages[1][2] = 0x80;
	}

	// LUT2: BW
	lut->voltages[2][0] = 0x80;
	lut->voltages[2][1] = 0x48;
	lut->voltages[2][2] = 0x40;

	// LUT3: WB
	if (preserve_white) {
		lut->voltages[3][0] = 0xA0; // Same as LUT1 for consistency
		lut->voltages[3][1] = 0x60;
		lut->voltages[3][2] = 0x30;
	} else {
		lut->voltages[3][0] = 0x40;
		lut->voltages[3][1] = 0x48;
		lut->voltages[3][2] = 0x80;
	}

	// LUT4: White refresh (optional)
	if (preserve_white) {
		lut->voltages[4][0] = 0xC0; // Strong white pulse
	}

	// Set timing for 2-cycle refresh
	// Group 0: First cycle
	lut->timing[0][0] = 0x08; // 8 phases
	lut->timing[0][6] = 0x01; // Repeat once

	// Group 1: Second cycle (white maintenance)
	lut->timing[1][0] = 0x08; // 8 phases
	lut->timing[1][1] = 0x02; // TP[*B]
	lut->timing[1][3] = 0x08; // TP[*C]
	lut->timing[1][4] = 0x02; // TP[*D]
	lut->timing[1][6] = 0x01; // Repeat once

	// Phase control: Use both groups
	for (int i = 0; i < 6; i++) {
		lut->phase_ctrl[i] = preserve_white ? 0x24 : 0x22;
	}

	// Frame count
	lut->frame_count = 0x00;
}

// Convert to 156-byte array
void lut_to_bytes(const ssd168x_lut_t *lut, uint8_t *output) {
	int idx = 0;

	// Voltages (60 bytes)
	for (int i = 0; i < 5; i++) {
		for (int j = 0; j < 3; j++) {
			output[idx++] = lut->voltages[i][j];
		}
		// Fill remaining 9 bytes with 0
		for (int j = 0; j < 9; j++) {
			output[idx++] = 0x00;
		}
	}

	// Timing (84 bytes)
	for (int i = 0; i < 12; i++) {
		for (int j = 0; j < 7; j++) {
			output[idx++] = lut->timing[i][j];
		}
	}

	// Phase control (9 bytes)
	for (int i = 0; i < 9; i++) {
		output[idx++] = lut->phase_ctrl[i];
	}

	// Register params (3 bytes fixed)
	output[idx++] = 0x22;
	output[idx++] = 0x17;
	output[idx++] = 0x41;
	output[idx++] = 0x00;
	output[idx++] = 0x32;
	output[idx++] = 0x20;
}

#if 0 // Placeholder - enable when helper functions are implemented
void test_partial_update(void) {
    // 1. First, do a full update with known good LUT
    load_lut(lut_full_refresh);
    set_display_update_control(0xF7);
    display_all_white();  // Clear screen
    trigger_update();

    // 2. Switch to partial mode WITHOUT reset
    set_display_update_control(0xFF);

    // 3. Test different LUTs
    const uint8_t* test_luts[] = {
        lut_partial_2cycle,
        lut_partial_enhanced,
        SSD1680_WAVESHARE_2IN13_V2_LUT_FAST_REFRESH_O  // Your original
    };

    for (int i = 0; i < 3; i++) {
        printf("Testing LUT %d\n", i);
        load_lut(test_luts[i]);

        // Draw test pattern
        draw_test_pattern();
        trigger_update();
        delay_ms(1000);

        // Check if white turned gray
        if (check_white_purity()) {
            printf("LUT %d: White preserved\n", i);
        } else {
            printf("LUT %d: White turned gray\n", i);
        }
    }
}
#endif
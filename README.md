# ESP LCD SSD168X E-Paper Panel Driver

An ESP-IDF LCD panel driver for SSD1680 and SSD1681 e-paper displays, providing comprehensive support for monochrome and dual-color e-paper panels with advanced refresh modes and bitmap manipulation.

## Features

### Display Support
- **SSD1680 Controller**: 2.13" e-paper displays (122x250 resolution)
- **SSD1681 Controller**: 1.54" e-paper displays (200x200 resolution)
- **Multiple Panel Models**:
  - DEPG0213BN (2.13" 122x250)
  - GDEY0213B74 (2.13" 122x250)
  - GDEY0154D67 (1.54" 200x200)

### Color Support
- **Monochrome**: Black and white display
- **Dual Color**: Black and red pixels (where supported)
- **Color Selection**: Runtime color selection for bitmaps
- **Independent VRAM**: Separate memory for black and red pixels

### Refresh Modes
- **Partial Refresh**: Fast updates for small changes
- **Fast Refresh**: Quick full-screen updates with minimal ghosting
- **Full Refresh**: Complete screen refresh for best image quality
- **Custom LUT**: Support for custom waveform lookup tables

### Advanced Features
- **Bitmap Rotation**: Hardware-accelerated 90°, 180°, 270° rotation
- **Pixel Conversion**: Callback-based pixel format conversion
- **Non-Copy Mode**: Direct buffer writing for memory efficiency
- **Busy Pin Monitoring**: Hardware busy signal detection
- **Event Callbacks**: Refresh completion notifications

## Installation

### ESP-IDF Integration
Add to your `main/CMakeLists.txt`:
```cmake
idf_component_register(SRCS "main.c"
                      INCLUDE_DIRS "."
                      REQUIRES esp_lcd_panel_ssd1680)
```

### PlatformIO
Add to your `platformio.ini`:
```ini
[env]
lib_deps =
    https://github.com/aivoprykk/esp-gps-logger.git#components/esp_lcd_panel_ssd1680
```

## Configuration

### Kconfig Options
Configure via `idf.py menuconfig`:

- **DISPLAY_DRIVER_SSD168X**: Enable SSD168X driver
- **LCD_IS_EPD**: Enable EPD-specific features
- **SSD168X_PANEL**: Select controller (SSD1680/SSD1681)
- **SSD168X_SCREEN**: Select specific screen model

### Supported Screen Models
- **SSD1680**:
  - DEPG0213BN: 2.13" 122x250 monochrome
  - GDEY0213B74: 2.13" 122x250 monochrome
- **SSD1681**:
  - GDEY0154D67: 1.54" 200x200 monochrome

## Usage

### Basic Initialization
```c
#include "esp_lcd_panel_ssd168x.h"
#include "esp_lcd_panel_io.h"

// Configure SPI bus
spi_bus_config_t bus_config = {
    .sclk_io_num = 18,
    .mosi_io_num = 23,
    .miso_io_num = -1,
    .quadwp_io_num = -1,
    .quadhd_io_num = -1,
};

// Configure panel IO
esp_lcd_panel_io_spi_config_t io_config = {
    .dc_gpio_num = 17,
    .cs_gpio_num = 5,
    .pclk_hz = 10000000,
    .lcd_cmd_bits = 8,
    .lcd_param_bits = 8,
    .spi_mode = 0,
    .trans_queue_depth = 10,
};

esp_lcd_panel_io_handle_t io_handle;
esp_lcd_new_panel_io_spi(&bus_config, &io_config, &io_handle);

// Configure panel
esp_lcd_ssd168x_config_t panel_config = {
    .busy_gpio_num = 4,
    .non_copy_mode = false,
    .width = 122,    // For 2.13" display
    .height = 250,
    .buffer_size = 122 * 250 / 8,
};

esp_lcd_panel_dev_config_t dev_config = {
    .reset_gpio_num = 16,
    .color_space = ESP_LCD_COLOR_SPACE_MONO,
    .bits_per_pixel = 1,
    .vendor_config = &panel_config,
};

esp_lcd_panel_handle_t panel_handle;
esp_lcd_new_panel_ssd168x(io_handle, &dev_config, &panel_handle);

// Initialize panel
esp_lcd_panel_init(panel_handle);
esp_lcd_panel_disp_on_off(panel_handle, true);
```

### Drawing Bitmaps
```c
// Set bitmap color (for dual-color displays)
epaper_panel_set_bitmap_color_ssd168x(panel_handle, SSD168X_EPAPER_BITMAP_BLACK);

// Draw bitmap (this automatically triggers refresh)
uint8_t *bitmap_data = get_bitmap_data();
esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, 122, 250, bitmap_data);
```

### Refresh Control
```c
// Manual refresh with specific mode
epaper_panel_refresh_screen_ssd168x(panel_handle, INIT_MODE_PARTIAL);  // Partial refresh
epaper_panel_refresh_screen_ssd168x(panel_handle, INIT_MODE_FAST_1);   // Fast refresh
epaper_panel_refresh_screen_ssd168x(panel_handle, INIT_MODE_FULL_1);   // Full refresh

// Wait for refresh completion (check busy pin)
while (gpio_get_level(CONFIG_EPAPER_BUSY_GPIO)) {
    vTaskDelay(10 / portTICK_PERIOD_MS);
}
```

### Event Callbacks
```c
// Register refresh completion callback
epaper_panel_callbacks_t callbacks = {
    .on_epaper_refresh_done = refresh_done_callback,
};

epaper_panel_register_event_callbacks_ssd168x(panel_handle, &callbacks, user_context);

static bool refresh_done_callback(const esp_lcd_panel_handle_t handle, const void *edata, void *user_data) {
    ESP_LOGI(TAG, "E-paper refresh completed");
    // Handle refresh completion (called in ISR context)
    return false;
}
```

### Bitmap Rotation
```c
// Rotate bitmap 90 degrees clockwise
uint8_t *src_bitmap = get_source_bitmap();
uint8_t *rotated_bitmap = malloc(122 * 250 / 8);

rotate_bitmap(src_bitmap, rotated_bitmap, 122, 250, 1, NULL, 0, NULL);

// Draw rotated bitmap
esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, 250, 122, rotated_bitmap);
```

### Custom Pixel Conversion
```c
// Pixel conversion callback for LVGL bitmaps
uint8_t lvgl_pixel_convert(const unsigned char *src_data, esp_lcd_ssd168x_area_t src_area,
                          int rotation, int data_format, void *user_data) {
    // Convert LVGL pixel format to monochrome
    int src_x = src_area.x1;
    int src_y = src_area.y1;

    // Extract pixel from LVGL bitmap
    uint8_t pixel = get_lvgl_pixel(src_data, src_x, src_y, data_format);

    // Convert to monochrome (0 = white, 1 = black)
    return (pixel > 128) ? 1 : 0;
}

// Rotate with custom conversion
rotate_bitmap(lvgl_bitmap, rotated_bitmap, width, height, 1,
             lvgl_pixel_convert, LVGL_VERSION, NULL);
```

### Custom LUT (Waveform)
```c
// Set custom waveform lookup table
const uint8_t *custom_lut = get_custom_lut();
epaper_panel_set_custom_lut_ssd168x(panel_handle, custom_lut, SSD168X_LUT_SIZE);

// Reset to built-in LUT
esp_lcd_panel_disp_on_off(panel_handle, true);
```

## API Reference

### Core Functions
- `esp_lcd_new_panel_ssd168x()`: Create SSD168x panel instance
- `epaper_panel_refresh_screen_ssd168x()`: Trigger screen refresh
- `epaper_panel_set_bitmap_color_ssd168x()`: Set bitmap color mode
- `epaper_panel_register_event_callbacks_ssd168x()`: Register event callbacks

### Advanced Functions
- `rotate_bitmap()`: Rotate bitmap data with optional conversion
- `epaper_panel_set_custom_lut_ssd168x()`: Set custom waveform LUT
- `epaper_panel_clear_screen_ssd168x()`: Clear screen with specific color
- `epaper_panel_init_screen_ssd168x()`: Initialize screen with specific mode

### Configuration Enums
- **Bitmap Colors**: `SSD168X_EPAPER_BITMAP_BLACK`, `SSD168X_EPAPER_BITMAP_RED`, `SSD168X_EPAPER_BITMAP_BOTH`
- **Init Modes**: `INIT_MODE_PARTIAL`, `INIT_MODE_FAST_1/2`, `INIT_MODE_FULL_1/2`
- **Sleep Modes**: `SLEEP_MODE_NORMAL`, `SLEEP_MODE_DEEP_1/2`

## Power Management

### Refresh Mode Selection
- **Partial**: Use for text updates, minimal power, some ghosting
- **Fast**: Use for image changes, moderate power, reduced ghosting
- **Full**: Use periodically to clear ghosting, highest power but best quality

### Busy Pin Handling
The driver monitors the BUSY pin to detect refresh completion:
```c
// Always wait for busy pin after refresh operations
epaper_panel_refresh_screen_ssd168x(panel_handle, INIT_MODE_FULL_1);
while (gpio_get_level(CONFIG_EPAPER_BUSY_GPIO)) {
    vTaskDelay(10 / portTICK_PERIOD_MS);
}
```

### Sleep Modes
```c
// Enter deep sleep
epaper_panel_shut_down(panel_handle);

// Wake from sleep
esp_lcd_panel_disp_on_off(panel_handle, true);
```

## Performance Considerations

### Memory Usage
- **Buffer Size**: 122×250÷8 = ~3.8KB for 2.13" display
- **Non-Copy Mode**: Reduces memory usage by writing directly to panel
- **Rotation**: Requires additional buffer for rotated data

### Timing
- **Partial Refresh**: ~0.3-0.5 seconds
- **Fast Refresh**: ~1-2 seconds
- **Full Refresh**: ~3-5 seconds

### Optimization Tips
- Use partial refresh for frequent small updates
- Batch multiple changes before refreshing
- Use fast refresh for better quality than partial
- Perform full refresh periodically to prevent ghosting

## Troubleshooting

### Common Issues
1. **Display Not Responding**: Check SPI connections and power supply
2. **Ghosting**: Use full refresh periodically or increase refresh frequency
3. **Slow Refresh**: Ensure proper BUSY pin monitoring
4. **Color Issues**: Verify bitmap format and color selection

### Debug Logging
Enable debug logging in Kconfig:
```kconfig
CONFIG_LCD_ENABLE_DEBUG_LOG=y
```

### Testing Commands
```c
// Check panel status
ESP_LOGI(TAG, "Panel initialized: %s", esp_lcd_panel_is_in_non_copy_mode_ssd168x(panel_handle) ? "non-copy" : "copy");

// Test refresh
epaper_panel_refresh_screen_ssd168x(panel_handle, INIT_MODE_FULL_1);
```

## Hardware Requirements

### Pin Connections
- **SPI**: MOSI, CLK, CS, DC pins
- **Control**: RST (reset), BUSY pins
- **Power**: 3.3V supply, proper decoupling capacitors

### Compatible Boards
- LilyGO T5 v2.13 (SSD1680)
- Waveshare 2.13" e-paper modules
- Waveshare 1.54" e-paper modules

## Dependencies

- ESP-IDF v4.4+
- esp_lcd component
- SPI driver

## License

See license.txt file in component directory.

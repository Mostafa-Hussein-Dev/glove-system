#include "drivers/display.h"
#include <string.h>
#include <math.h>
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "config/pin_definitions.h"
#include "util/debug.h"
#include "util/i2c_utils.h"
#include "driver/gpio.h"

// Display device handle
static i2c_master_bus_handle_t i2c_bus_handle = NULL;
static i2c_master_dev_handle_t display_dev_handle = NULL;

static const char *TAG = "DISPLAY";

// SSD1306 I2C address
#define SSD1306_ADDR 0x3C

// SSD1306 commands
#define SSD1306_COMMAND             0x00
#define SSD1306_DATA                0x40
#define SSD1306_CMD_SET_CONTRAST    0x81
#define SSD1306_CMD_DISPLAY_RAM     0xA4
#define SSD1306_CMD_DISPLAY_NORMAL  0xA6
#define SSD1306_CMD_DISPLAY_OFF     0xAE
#define SSD1306_CMD_DISPLAY_ON      0xAF
#define SSD1306_CMD_SET_MEM_ADDR    0x20
#define SSD1306_CMD_SET_COL_ADDR    0x21
#define SSD1306_CMD_SET_PAGE_ADDR   0x22
#define SSD1306_CMD_SET_START_LINE  0x40
#define SSD1306_CMD_SET_SEGMENT     0xA0
#define SSD1306_CMD_SET_MUX_RATIO   0xA8
#define SSD1306_CMD_SET_COM_SCAN    0xC0
#define SSD1306_CMD_SET_DISPLAY_OFFSET 0xD3
#define SSD1306_CMD_SET_COM_PINS    0xDA
#define SSD1306_CMD_SET_CHARGE_PUMP 0x8D
#define SSD1306_CMD_SET_PRECHARGE   0xD9
#define SSD1306_CMD_SET_VCOM_DESEL  0xDB
#define SSD1306_CMD_SET_SCROLL      0x2E
#define SSD1306_CMD_RIGHT_SCROLL    0x26
#define SSD1306_CMD_LEFT_SCROLL     0x27

// Display dimensions
#define SSD1306_WIDTH               128
#define SSD1306_HEIGHT              64
#define SSD1306_PAGES               8  // 64 pixels / 8 bits per page

// Display buffer
static uint8_t display_buffer[SSD1306_WIDTH * SSD1306_PAGES];
static bool display_initialized = false;
static bool display_powered_on = false;

// Simple 6x8 font (ASCII 32-127, reduced for brevity)
static const uint8_t font6x8[] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // Space
    0x00, 0x00, 0x5F, 0x00, 0x00, 0x00, // !
    0x00, 0x07, 0x00, 0x07, 0x00, 0x00, // "
    0x14, 0x7F, 0x14, 0x7F, 0x14, 0x00, // #
    // ... remainder of font data would be here
};

// Forward function declarations
static esp_err_t ssd1306_write_command(uint8_t command);
static esp_err_t ssd1306_write_data(uint8_t* data, size_t len);
static void ssd1306_set_pixel(uint8_t x, uint8_t y, uint8_t color);
static esp_err_t ssd1306_update_full();

esp_err_t display_init(void) {
    esp_err_t ret;
    
    // Initialize I2C master bus if not already done
    if (i2c_bus_handle == NULL) {
        i2c_master_bus_config_t i2c_mst_config = {
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .i2c_port = I2C_MASTER_NUM,
            .scl_io_num = I2C_MASTER_SCL_IO,
            .sda_io_num = I2C_MASTER_SDA_IO,
            .glitch_ignore_cnt = 7,
            .flags.enable_internal_pullup = true,
        };
        
        ret = i2c_new_master_bus(&i2c_mst_config, &i2c_bus_handle);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to create I2C master bus: %s", esp_err_to_name(ret));
            return ret;
        }
    }
    
    // Add display device to I2C bus
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = SSD1306_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };
    
    ret = i2c_master_bus_add_device(i2c_bus_handle, &dev_cfg, &display_dev_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add display device to I2C bus: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Reset display if reset pin defined
    if (DISPLAY_RST_PIN >= 0) {
        gpio_config_t io_conf = {
            .intr_type = GPIO_INTR_DISABLE,
            .mode = GPIO_MODE_OUTPUT,
            .pin_bit_mask = (1ULL << DISPLAY_RST_PIN),
            .pull_down_en = 0,
            .pull_up_en = 0
        };
        gpio_config(&io_conf);
        
        // Reset pulse
        gpio_set_level(DISPLAY_RST_PIN, 0);
        vTaskDelay(pdMS_TO_TICKS(10));
        gpio_set_level(DISPLAY_RST_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    // Initialize display command sequence
    ret = ssd1306_write_command(SSD1306_CMD_DISPLAY_OFF);  // Display off
    if (ret != ESP_OK) return ret;
    
    ret = ssd1306_write_command(SSD1306_CMD_SET_MUX_RATIO); // Set MUX ratio
    if (ret != ESP_OK) return ret;
    ret = ssd1306_write_command(0x3F);  // 64 lines
    if (ret != ESP_OK) return ret;
    
    ret = ssd1306_write_command(SSD1306_CMD_SET_DISPLAY_OFFSET); // Set display offset
    if (ret != ESP_OK) return ret;
    ret = ssd1306_write_command(0x00);  // No offset
    if (ret != ESP_OK) return ret;
    
    ret = ssd1306_write_command(SSD1306_CMD_SET_START_LINE | 0x00); // Set start line
    if (ret != ESP_OK) return ret;
    
    ret = ssd1306_write_command(SSD1306_CMD_SET_CHARGE_PUMP); // Set charge pump
    if (ret != ESP_OK) return ret;
    ret = ssd1306_write_command(0x14);  // Enable charge pump
    if (ret != ESP_OK) return ret;
    
    ret = ssd1306_write_command(SSD1306_CMD_SET_SEGMENT | 0x01); // Set segment remap
    if (ret != ESP_OK) return ret;
    
    ret = ssd1306_write_command(SSD1306_CMD_SET_COM_SCAN | 0x08); // Set COM scan direction
    if (ret != ESP_OK) return ret;
    
    ret = ssd1306_write_command(SSD1306_CMD_SET_COM_PINS); // Set COM pins hardware config
    if (ret != ESP_OK) return ret;
    ret = ssd1306_write_command(0x12);  // Alternative COM pin config
    if (ret != ESP_OK) return ret;
    
    ret = ssd1306_write_command(SSD1306_CMD_SET_CONTRAST); // Set contrast
    if (ret != ESP_OK) return ret;
    ret = ssd1306_write_command(0xCF);  // Medium contrast
    if (ret != ESP_OK) return ret;
    
    ret = ssd1306_write_command(SSD1306_CMD_SET_PRECHARGE); // Set pre-charge period
    if (ret != ESP_OK) return ret;
    ret = ssd1306_write_command(0xF1);  // Phase 1 = 15, Phase 2 = 1
    if (ret != ESP_OK) return ret;
    
    ret = ssd1306_write_command(SSD1306_CMD_SET_VCOM_DESEL); // Set VCOMH deselect level
    if (ret != ESP_OK) return ret;
    ret = ssd1306_write_command(0x40);  // 0.77 x VCC
    if (ret != ESP_OK) return ret;
    
    ret = ssd1306_write_command(SSD1306_CMD_DISPLAY_RAM); // Display from RAM
    if (ret != ESP_OK) return ret;
    
    ret = ssd1306_write_command(SSD1306_CMD_DISPLAY_NORMAL); // Normal display (not inverted)
    if (ret != ESP_OK) return ret;
    
    // Clear display buffer
    memset(display_buffer, 0, sizeof(display_buffer));
    
    // Send buffer to display
    ret = ssd1306_update_full();
    if (ret != ESP_OK) return ret;
    
    // Turn display on
    ret = ssd1306_write_command(SSD1306_CMD_DISPLAY_ON);
    if (ret != ESP_OK) return ret;
    
    display_initialized = true;
    display_powered_on = true;
    
    ESP_LOGI(TAG, "OLED display initialized successfully");
    
    // Display a splash screen
    display_show_splash_screen();
    
    return ESP_OK;
}

esp_err_t display_power_on(void) {
    if (!display_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (display_powered_on) {
        return ESP_OK;  // Already on
    }
    
    esp_err_t ret = ssd1306_write_command(SSD1306_CMD_DISPLAY_ON);
    if (ret != ESP_OK) {
        return ret;
    }
    
    display_powered_on = true;
    ESP_LOGI(TAG, "Display powered on");
    
    // Refresh display with current buffer
    return ssd1306_update_full();
}

esp_err_t display_power_off(void) {
    if (!display_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (!display_powered_on) {
        return ESP_OK;  // Already off
    }
    
    esp_err_t ret = ssd1306_write_command(SSD1306_CMD_DISPLAY_OFF);
    if (ret != ESP_OK) {
        return ret;
    }
    
    display_powered_on = false;
    ESP_LOGI(TAG, "Display powered off");
    
    return ESP_OK;
}

esp_err_t display_set_contrast(uint8_t contrast) {
    if (!display_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    esp_err_t ret = ssd1306_write_command(SSD1306_CMD_SET_CONTRAST);
    if (ret != ESP_OK) {
        return ret;
    }
    
    ret = ssd1306_write_command(contrast);
    if (ret != ESP_OK) {
        return ret;
    }
    
    ESP_LOGI(TAG, "Display contrast set to %d", contrast);
    
    return ESP_OK;
}

esp_err_t display_clear(void) {
    if (!display_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Clear buffer
    memset(display_buffer, 0, sizeof(display_buffer));
    
    // Send buffer to display
    return ssd1306_update_full();
}

esp_err_t display_update(void) {
    if (!display_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    return ssd1306_update_full();
}

esp_err_t display_draw_text(const char* text, uint8_t x, uint8_t y, display_font_t font, display_align_t align) {
    if (!display_initialized || text == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // For simplicity, we'll just use the 6x8 font for now
    uint8_t font_width = 6;
    //uint8_t font_height = 8;
    
    // Calculate text length in pixels
    size_t text_len = strlen(text);
    uint16_t text_width = text_len * font_width;
    
    // Adjust x coordinate based on alignment
    switch (align) {
        case DISPLAY_ALIGN_CENTER:
            if (text_width < SSD1306_WIDTH) {
                x = (SSD1306_WIDTH - text_width) / 2;
            }
            break;
        case DISPLAY_ALIGN_RIGHT:
            if (text_width < SSD1306_WIDTH) {
                x = SSD1306_WIDTH - text_width;
            }
            break;
        default:  // DISPLAY_ALIGN_LEFT (no adjustment needed)
            break;
    }
    
    // Draw each character
    uint8_t cursor_x = x;
    for (size_t i = 0; i < text_len; i++) {
        char c = text[i];
        
        // Skip non-printable characters
        if (c < 32 || c > 127) {
            continue;
        }
        
        // Get character index in font array
        uint16_t char_idx = (c - 32) * font_width;
        
        // Draw character (6x8 font only for simplicity)
        for (uint8_t col = 0; col < font_width; col++) {
            uint8_t x_pos = cursor_x + col;
            
            // Skip if out of bounds
            if (x_pos >= SSD1306_WIDTH) {
                break;
            }
            
            uint8_t font_byte = font6x8[char_idx + col];
            for (uint8_t row = 0; row < 8; row++) {
                if (y + row >= SSD1306_HEIGHT) {
                    break;
                }
                
                if (font_byte & (1 << row)) {
                    ssd1306_set_pixel(x_pos, y + row, 1);
                }
            }
        }
        
        // Move cursor to next character position
        cursor_x += font_width;
        
        // Break if outside display
        if (cursor_x >= SSD1306_WIDTH) {
            break;
        }
    }
    
    // No need to update display here, caller should call display_update() when needed
    return ESP_OK;
}

esp_err_t display_draw_pixel(uint8_t x, uint8_t y, uint8_t color) {
    if (!display_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (x >= SSD1306_WIDTH || y >= SSD1306_HEIGHT) {
        return ESP_ERR_INVALID_ARG;
    }
    
    ssd1306_set_pixel(x, y, color);
    
    // No need to update display here, caller should call display_update() when needed
    return ESP_OK;
}

esp_err_t display_draw_line(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t color) {
    if (!display_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Check bounds
    if (x1 >= SSD1306_WIDTH || y1 >= SSD1306_HEIGHT || 
        x2 >= SSD1306_WIDTH || y2 >= SSD1306_HEIGHT) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Bresenham's line algorithm
    int dx = abs(x2 - x1);
    int dy = abs(y2 - y1);
    int sx = (x1 < x2) ? 1 : -1;
    int sy = (y1 < y2) ? 1 : -1;
    int err = dx - dy;
    
    while (true) {
        ssd1306_set_pixel(x1, y1, color);
        
        if (x1 == x2 && y1 == y2) {
            break;
        }
        
        int e2 = 2 * err;
        if (e2 > -dy) {
            err -= dy;
            x1 += sx;
        }
        if (e2 < dx) {
            err += dx;
            y1 += sy;
        }
    }
    
    // No need to update display here, caller should call display_update() when needed
    return ESP_OK;
}

esp_err_t display_draw_rect(uint8_t x, uint8_t y, uint8_t width, uint8_t height, uint8_t color) {
    if (!display_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Check bounds
    if (x >= SSD1306_WIDTH || y >= SSD1306_HEIGHT || 
        x + width > SSD1306_WIDTH || y + height > SSD1306_HEIGHT) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Draw horizontal lines
    for (uint8_t i = 0; i < width; i++) {
        ssd1306_set_pixel(x + i, y, color);
        ssd1306_set_pixel(x + i, y + height - 1, color);
    }
    
    // Draw vertical lines
    for (uint8_t i = 0; i < height; i++) {
        ssd1306_set_pixel(x, y + i, color);
        ssd1306_set_pixel(x + width - 1, y + i, color);
    }
    
    // No need to update display here, caller should call display_update() when needed
    return ESP_OK;
}

esp_err_t display_fill_rect(uint8_t x, uint8_t y, uint8_t width, uint8_t height, uint8_t color) {
    if (!display_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Check bounds
    if (x >= SSD1306_WIDTH || y >= SSD1306_HEIGHT || 
        x + width > SSD1306_WIDTH || y + height > SSD1306_HEIGHT) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Fill the rectangle
    for (uint8_t i = 0; i < width; i++) {
        for (uint8_t j = 0; j < height; j++) {
            ssd1306_set_pixel(x + i, y + j, color);
        }
    }
    
    // No need to update display here, caller should call display_update() when needed
    return ESP_OK;
}

esp_err_t display_draw_circle(uint8_t x, uint8_t y, uint8_t radius, uint8_t color) {
    if (!display_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Check bounds
    if (x < radius || y < radius || 
        x + radius >= SSD1306_WIDTH || y + radius >= SSD1306_HEIGHT) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Bresenham's circle algorithm
    int f = 1 - radius;
    int ddF_x = 1;
    int ddF_y = -2 * radius;
    int x_pos = 0;
    int y_pos = radius;
    
    ssd1306_set_pixel(x, y + radius, color);
    ssd1306_set_pixel(x, y - radius, color);
    ssd1306_set_pixel(x + radius, y, color);
    ssd1306_set_pixel(x - radius, y, color);
    
    while (x_pos < y_pos) {
        if (f >= 0) {
            y_pos--;
            ddF_y += 2;
            f += ddF_y;
        }
        x_pos++;
        ddF_x += 2;
        f += ddF_x;
        
        ssd1306_set_pixel(x + x_pos, y + y_pos, color);
        ssd1306_set_pixel(x - x_pos, y + y_pos, color);
        ssd1306_set_pixel(x + x_pos, y - y_pos, color);
        ssd1306_set_pixel(x - x_pos, y - y_pos, color);
        ssd1306_set_pixel(x + y_pos, y + x_pos, color);
        ssd1306_set_pixel(x - y_pos, y + x_pos, color);
        ssd1306_set_pixel(x + y_pos, y - x_pos, color);
        ssd1306_set_pixel(x - y_pos, y - x_pos, color);
    }
    
    // No need to update display here, caller should call display_update() when needed
    return ESP_OK;
}

esp_err_t display_fill_circle(uint8_t x, uint8_t y, uint8_t radius, uint8_t color) {
    // Draw filled circle using horizontal lines
    for (int16_t i = -radius; i <= radius; i++) {
        // Calculate width at this height
        int16_t width = sqrt(radius * radius - i * i) * 2;
        int16_t start_x = x - width / 2;
        
        // Draw a horizontal line
        for (int16_t j = 0; j < width; j++) {
            if (start_x + j >= 0 && start_x + j < SSD1306_WIDTH && 
                y + i >= 0 && y + i < SSD1306_HEIGHT) {
                ssd1306_set_pixel(start_x + j, y + i, color);
            }
        }
    }
    
    return ESP_OK;
}

esp_err_t display_draw_bitmap(uint8_t x, uint8_t y, const uint8_t *bitmap, uint8_t width, uint8_t height, uint8_t color) {
    if (!display_initialized || bitmap == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Check bounds
    if (x >= SSD1306_WIDTH || y >= SSD1306_HEIGHT) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Calculate effective width and height
    uint8_t eff_width = (x + width <= SSD1306_WIDTH) ? width : (SSD1306_WIDTH - x);
    uint8_t eff_height = (y + height <= SSD1306_HEIGHT) ? height : (SSD1306_HEIGHT - y);
    
    // Draw bitmap (assumes 1 bit per pixel, row-major order)
    for (uint8_t j = 0; j < eff_height; j++) {
        for (uint8_t i = 0; i < eff_width; i++) {
            uint16_t byte_idx = (i + j * width) / 8;
            uint8_t bit_idx = 7 - ((i + j * width) % 8);  // MSB first
            
            if (bitmap[byte_idx] & (1 << bit_idx)) {
                ssd1306_set_pixel(x + i, y + j, color);
            }
        }
    }
    
    // No need to update display here, caller should call display_update() when needed
    return ESP_OK;
}

esp_err_t display_draw_progress_bar(uint8_t x, uint8_t y, uint8_t width, uint8_t height, uint8_t percentage) {
    if (!display_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Check bounds
    if (x >= SSD1306_WIDTH || y >= SSD1306_HEIGHT || 
        x + width > SSD1306_WIDTH || y + height > SSD1306_HEIGHT) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Clamp percentage
    if (percentage > 100) {
        percentage = 100;
    }
    
    // Calculate fill width
    uint8_t fill_width = (percentage * (width - 2)) / 100;
    
    // Draw outline rectangle
    display_draw_rect(x, y, width, height, 1);
    
    // Draw filled part
    if (fill_width > 0) {
        display_fill_rect(x + 1, y + 1, fill_width, height - 2, 1);
    }
    
    // No need to update display here, caller should call display_update() when needed
    return ESP_OK;
}

esp_err_t display_show_splash_screen(void) {
    if (!display_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Clear display buffer
    memset(display_buffer, 0, sizeof(display_buffer));
    
    // Draw a simple splash screen
    display_draw_text("Sign Language", 0, 16, DISPLAY_FONT_SMALL, DISPLAY_ALIGN_CENTER);
    display_draw_text("Glove", 0, 26, DISPLAY_FONT_SMALL, DISPLAY_ALIGN_CENTER);
    display_draw_text("v1.0", 0, 42, DISPLAY_FONT_SMALL, DISPLAY_ALIGN_CENTER);
    
    // Draw a border
    display_draw_rect(0, 0, SSD1306_WIDTH, SSD1306_HEIGHT, 1);
    
    // Update display
    esp_err_t ret = display_update();
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Wait a moment to show splash screen
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // Clear display
    return display_clear();
}

esp_err_t display_show_debug(const char *message) {
    if (!display_initialized || message == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Clear lower part of the screen for debug message
    display_fill_rect(0, SSD1306_HEIGHT - 9, SSD1306_WIDTH, 9, 0);
    
    // Draw debug message
    display_draw_text(message, 0, SSD1306_HEIGHT - 8, DISPLAY_FONT_SMALL, DISPLAY_ALIGN_LEFT);
    
    // Update display
    return display_update();
}

esp_err_t display_invert(bool invert) {
    if (!display_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    uint8_t command = invert ? 0xA7 : 0xA6;  // A7 = inverted, A6 = normal
    return ssd1306_write_command(command);
}

esp_err_t display_scroll(uint8_t start_line, uint8_t num_lines) {
    if (!display_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    esp_err_t ret;
    
    // Stop any current scrolling
    ret = display_stop_scroll();
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Enable vertical scroll
    ret = ssd1306_write_command(0x29);  // Vertical and right horizontal scroll
    if (ret != ESP_OK) return ret;
    
    ret = ssd1306_write_command(0x00);  // Dummy byte
    if (ret != ESP_OK) return ret;
    
    ret = ssd1306_write_command(start_line);  // Start page
    if (ret != ESP_OK) return ret;
    
    ret = ssd1306_write_command(0x00);  // Time interval
    if (ret != ESP_OK) return ret;
    
    ret = ssd1306_write_command(start_line + num_lines - 1);  // End page
    if (ret != ESP_OK) return ret;
    
    ret = ssd1306_write_command(0x01);  // Vertical offset
    if (ret != ESP_OK) return ret;
    
    // Start scrolling
    ret = ssd1306_write_command(0x2F);
    if (ret != ESP_OK) return ret;
    
    return ESP_OK;
}

esp_err_t display_stop_scroll(void) {
    if (!display_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    return ssd1306_write_command(0x2E);  // Deactivate scroll
}

esp_err_t display_flip_vertical(bool flip) {
    if (!display_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    uint8_t command = flip ? (SSD1306_CMD_SET_COM_SCAN | 0x00) : (SSD1306_CMD_SET_COM_SCAN | 0x08);
    return ssd1306_write_command(command);
}

esp_err_t display_flip_horizontal(bool flip) {
    if (!display_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    uint8_t command = flip ? (SSD1306_CMD_SET_SEGMENT | 0x00) : (SSD1306_CMD_SET_SEGMENT | 0x01);
    return ssd1306_write_command(command);
}

//---------------------- Private functions -----------------------

static esp_err_t ssd1306_write_command(uint8_t command) {
    uint8_t write_buf[2] = {SSD1306_COMMAND, command};
    return i2c_master_transmit(display_dev_handle, write_buf, sizeof(write_buf), pdMS_TO_TICKS(100));
}

static esp_err_t ssd1306_write_data(uint8_t* data, size_t len) {
    // Create temp buffer for data byte + control byte
    uint8_t *temp_buf = malloc(len + 1);
    if (temp_buf == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for I2C buffer");
        return ESP_ERR_NO_MEM;
    }
    
    // Set control byte for data
    temp_buf[0] = SSD1306_DATA;
    memcpy(temp_buf + 1, data, len);
    
    // Send data
    esp_err_t ret = i2c_master_transmit(display_dev_handle, temp_buf, len + 1, pdMS_TO_TICKS(100));
    
    // Free temp buffer
    free(temp_buf);
    
    return ret;
}

static void ssd1306_set_pixel(uint8_t x, uint8_t y, uint8_t color) {
    if (x >= SSD1306_WIDTH || y >= SSD1306_HEIGHT) {
        return;
    }
    
    // Calculate byte index and bit position
    uint16_t byte_idx = x + (y / 8) * SSD1306_WIDTH;
    uint8_t bit_pos = y % 8;
    
    // Set or clear the bit
    if (color) {
        display_buffer[byte_idx] |= (1 << bit_pos);
    } else {
        display_buffer[byte_idx] &= ~(1 << bit_pos);
    }
}

static esp_err_t ssd1306_update_full() {
    esp_err_t ret;
    
    // Set column address range
    ret = ssd1306_write_command(SSD1306_CMD_SET_COL_ADDR);
    if (ret != ESP_OK) return ret;
    ret = ssd1306_write_command(0);  // Start column
    if (ret != ESP_OK) return ret;
    ret = ssd1306_write_command(SSD1306_WIDTH - 1);  // End column
    if (ret != ESP_OK) return ret;
    
    // Set page address range
    ret = ssd1306_write_command(SSD1306_CMD_SET_PAGE_ADDR);
    if (ret != ESP_OK) return ret;
    ret = ssd1306_write_command(0);  // Start page
    if (ret != ESP_OK) return ret;
    ret = ssd1306_write_command(SSD1306_PAGES - 1);  // End page
    if (ret != ESP_OK) return ret;
    
    // Send the buffer
    return ssd1306_write_data(display_buffer, sizeof(display_buffer));
}
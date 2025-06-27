#include "drivers/display.h"
#include <string.h>
#include <math.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "config/pin_definitions.h"
#include "util/debug.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#include "driver/i2c_master.h"
#include "font6x8.inc"


#define SSD1306_ADDR 0x3C   // SSD1306 I2C address
#define SSD1306_WIDTH               128
#define SSD1306_HEIGHT              64
#define SSD1306_PAGES               8
#define FONT_WIDTH 6
#define FONT_HEIGHT 8

//static const char *TAG = "DISPLAY";

static esp_lcd_panel_handle_t ssd1306_panel = NULL;
static esp_lcd_panel_io_handle_t ssd1306_io = NULL;
static uint8_t display_buffer[SSD1306_WIDTH * SSD1306_PAGES];
static bool display_initialized = false;
static bool display_powered_on = false;

static void ssd1306_set_pixel(uint8_t x, uint8_t y, uint8_t color);

// Forward function declarations
esp_err_t ssd1306_write_command(uint8_t command);
esp_err_t ssd1306_write_data(uint8_t* data, size_t len);
static void ssd1306_set_pixel(uint8_t x, uint8_t y, uint8_t color);
esp_err_t ssd1306_update_full();

esp_err_t display_init(void) {
    esp_err_t ret;
    extern i2c_master_bus_handle_t i2c_master_bus;

    esp_lcd_panel_io_i2c_config_t io_config = {
        .dev_addr = 0x3C,
        .scl_speed_hz = 400000,
        .control_phase_bytes = 1,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
        .dc_bit_offset = 6,
    };

    ret = esp_lcd_new_panel_io_i2c(i2c_master_bus, &io_config, &ssd1306_io);
    if (ret != ESP_OK) return ret;

    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = -1,
        .bits_per_pixel = 1,
    };
    esp_lcd_panel_ssd1306_config_t ssd1306_cfg = {
        .height = SSD1306_HEIGHT,
    };
    panel_config.vendor_config = &ssd1306_cfg;

    ret = esp_lcd_new_panel_ssd1306(ssd1306_io, &panel_config, &ssd1306_panel);
    if (ret != ESP_OK) return ret;

    ESP_ERROR_CHECK(esp_lcd_panel_reset(ssd1306_panel));
    ESP_ERROR_CHECK(esp_lcd_panel_init(ssd1306_panel));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(ssd1306_panel, true));

    memset(display_buffer, 0, sizeof(display_buffer));
    display_initialized = true;
    display_powered_on = true;

    display_flip_vertical(false);
    display_flip_horizontal(false);

    display_show_splash_screen();

    return ESP_OK;
}

esp_err_t display_update(void) {
    if (!display_initialized) return ESP_ERR_INVALID_STATE;
    return esp_lcd_panel_draw_bitmap(ssd1306_panel, 0, 0, SSD1306_WIDTH, SSD1306_HEIGHT, display_buffer);
}

static void ssd1306_set_pixel(uint8_t x, uint8_t y, uint8_t color) {
    if (x >= SSD1306_WIDTH || y >= SSD1306_HEIGHT) return;
    uint16_t byte_idx = x + (y / 8) * SSD1306_WIDTH;
    uint8_t bit_pos = y % 8;
    if (color) {
        display_buffer[byte_idx] |= (1 << bit_pos);
    } else {
        display_buffer[byte_idx] &= ~(1 << bit_pos);
    }
}

esp_err_t display_draw_pixel(uint8_t x, uint8_t y, uint8_t color) {
    if (!display_initialized) return ESP_ERR_INVALID_STATE;
    if (x >= SSD1306_WIDTH || y >= SSD1306_HEIGHT) return ESP_ERR_INVALID_ARG;
    ssd1306_set_pixel(x, y, color);
    return ESP_OK;
}

esp_err_t display_draw_line(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t color) {
    int dx = abs(x2 - x1), sx = x1 < x2 ? 1 : -1;
    int dy = -abs(y2 - y1), sy = y1 < y2 ? 1 : -1;
    int err = dx + dy, e2;
    while (true) {
        ssd1306_set_pixel(x1, y1, color);
        if (x1 == x2 && y1 == y2) break;
        e2 = 2 * err;
        if (e2 >= dy) { err += dy; x1 += sx; }
        if (e2 <= dx) { err += dx; y1 += sy; }
    }
    return ESP_OK;
}

esp_err_t display_draw_rect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint8_t color) {
    for (uint8_t i = 0; i < w; i++) {
        ssd1306_set_pixel(x + i, y, color);
        ssd1306_set_pixel(x + i, y + h - 1, color);
    }
    for (uint8_t i = 0; i < h; i++) {
        ssd1306_set_pixel(x, y + i, color);
        ssd1306_set_pixel(x + w - 1, y + i, color);
    }
    return ESP_OK;
}

esp_err_t display_fill_rect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint8_t color) {
    for (uint8_t i = 0; i < w; i++) {
        for (uint8_t j = 0; j < h; j++) {
            ssd1306_set_pixel(x + i, y + j, color);
        }
    }
    return ESP_OK;
}

esp_err_t display_draw_circle(uint8_t x0, uint8_t y0, uint8_t r, uint8_t color) {
    int f = 1 - r, ddF_x = 1, ddF_y = -2 * r;
    int x = 0, y = r;
    ssd1306_set_pixel(x0, y0 + r, color);
    ssd1306_set_pixel(x0, y0 - r, color);
    ssd1306_set_pixel(x0 + r, y0, color);
    ssd1306_set_pixel(x0 - r, y0, color);
    while (x < y) {
        if (f >= 0) { y--; ddF_y += 2; f += ddF_y; }
        x++; ddF_x += 2; f += ddF_x;
        ssd1306_set_pixel(x0 + x, y0 + y, color);
        ssd1306_set_pixel(x0 - x, y0 + y, color);
        ssd1306_set_pixel(x0 + x, y0 - y, color);
        ssd1306_set_pixel(x0 - x, y0 - y, color);
        ssd1306_set_pixel(x0 + y, y0 + x, color);
        ssd1306_set_pixel(x0 - y, y0 + x, color);
        ssd1306_set_pixel(x0 + y, y0 - x, color);
        ssd1306_set_pixel(x0 - y, y0 - x, color);
    }
    return ESP_OK;
}

esp_err_t display_fill_circle(uint8_t x0, uint8_t y0, uint8_t r, uint8_t color) {
    for (int16_t y = -r; y <= r; y++) {
        int16_t x_span = sqrt(r * r - y * y);
        for (int16_t x = -x_span; x <= x_span; x++) {
            ssd1306_set_pixel(x0 + x, y0 + y, color);
        }
    }
    return ESP_OK;
}

esp_err_t display_draw_text(const char* text, uint8_t x, uint8_t y, display_font_t font, display_align_t align) {
    size_t len = strlen(text);
    uint16_t text_width = len * FONT_WIDTH;
    if (align == DISPLAY_ALIGN_CENTER && text_width < SSD1306_WIDTH)
        x = (SSD1306_WIDTH - text_width) / 2;
    else if (align == DISPLAY_ALIGN_RIGHT && text_width < SSD1306_WIDTH)
        x = SSD1306_WIDTH - text_width;

    uint8_t cursor_x = x;
    for (size_t i = 0; i < len; i++) {
        char c = text[i];
        if (c < 32 || c > 127) continue;
        uint16_t idx = (c - 32) * FONT_WIDTH;
        for (uint8_t col = 0; col < FONT_WIDTH; col++) {
            uint8_t byte = font6x8[idx + col];
            for (uint8_t row = 0; row < 8; row++) {
                if (byte & (1 << row))
                    ssd1306_set_pixel(cursor_x + col, y + row, 1);
            }
        }
        cursor_x += FONT_WIDTH;
        if (cursor_x >= SSD1306_WIDTH) break;
    }
    return ESP_OK;
}

esp_err_t display_draw_bitmap(uint8_t x, uint8_t y, const uint8_t *bitmap, uint8_t width, uint8_t height, uint8_t color) {
    for (uint8_t j = 0; j < height; j++) {
        for (uint8_t i = 0; i < width; i++) {
            uint16_t byte_idx = (i + j * width) / 8;
            uint8_t bit_idx = 7 - ((i + j * width) % 8);
            if (bitmap[byte_idx] & (1 << bit_idx))
                ssd1306_set_pixel(x + i, y + j, color);
        }
    }
    return ESP_OK;
}

esp_err_t display_draw_progress_bar(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint8_t pct) {
    if (pct > 100) pct = 100;
    uint8_t fill = (pct * (w - 2)) / 100;
    display_draw_rect(x, y, w, h, 1);
    if (fill > 0) display_fill_rect(x + 1, y + 1, fill, h - 2, 1);
    return ESP_OK;
}

esp_err_t display_show_splash_screen(void) {
    memset(display_buffer, 0, sizeof(display_buffer));
    display_draw_text("Sign Language", 0, 16, DISPLAY_FONT_SMALL, DISPLAY_ALIGN_CENTER);
    display_draw_text("Glove", 0, 26, DISPLAY_FONT_SMALL, DISPLAY_ALIGN_CENTER);
    display_draw_text("v1.0", 0, 42, DISPLAY_FONT_SMALL, DISPLAY_ALIGN_CENTER);
    display_draw_rect(0, 0, SSD1306_WIDTH, SSD1306_HEIGHT, 1);
    display_update();
    vTaskDelay(pdMS_TO_TICKS(1000));
    return display_clear();
}

esp_err_t display_show_debug(const char *msg) {
    display_fill_rect(0, SSD1306_HEIGHT - 9, SSD1306_WIDTH, 9, 0);
    display_draw_text(msg, 0, SSD1306_HEIGHT - 8, DISPLAY_FONT_SMALL, DISPLAY_ALIGN_LEFT);
    return display_update();
}

esp_err_t display_power_on(void) {
    if (!display_initialized) return ESP_ERR_INVALID_STATE;
    if (display_powered_on) return ESP_OK;
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(ssd1306_panel, true));
    display_powered_on = true;
    return display_update();
}

esp_err_t display_power_off(void) {
    if (!display_initialized) return ESP_ERR_INVALID_STATE;
    if (!display_powered_on) return ESP_OK;
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(ssd1306_panel, false));
    display_powered_on = false;
    return ESP_OK;
}

esp_err_t display_set_contrast(uint8_t contrast) {
    uint8_t cmd[] = {0x81, contrast};
    return esp_lcd_panel_io_tx_param(ssd1306_io, cmd[0], &cmd[1], 1);
}

esp_err_t display_clear(void) {
    memset(display_buffer, 0, sizeof(display_buffer));
    return display_update();
}

esp_err_t display_invert(bool invert) {
    uint8_t cmd = invert ? 0xA7 : 0xA6;
    return esp_lcd_panel_io_tx_param(ssd1306_io, cmd, NULL, 0);
}

esp_err_t display_flip_horizontal(bool flip) {
    uint8_t cmd = flip ? 0xA0 : 0xA1;
    return esp_lcd_panel_io_tx_param(ssd1306_io, cmd, NULL, 0);
}

esp_err_t display_flip_vertical(bool flip) {
    uint8_t cmd = flip ? 0xC0 : 0xC8;
    return esp_lcd_panel_io_tx_param(ssd1306_io, cmd, NULL, 0);
}

esp_err_t display_scroll(uint8_t start_line, uint8_t num_lines) {
    esp_err_t ret = display_stop_scroll();
    if (ret != ESP_OK) return ret;

    uint8_t cmds[] = {0x29, 0x00, start_line, 0x00, (uint8_t)(start_line + num_lines - 1), 0x01};
    for (int i = 0; i < sizeof(cmds); i++) {
        ret = esp_lcd_panel_io_tx_param(ssd1306_io, cmds[i], NULL, 0);
        if (ret != ESP_OK) return ret;
    }
    return esp_lcd_panel_io_tx_param(ssd1306_io, 0x2F, NULL, 0);
}

esp_err_t display_stop_scroll(void) {
    return esp_lcd_panel_io_tx_param(ssd1306_io, 0x2E, NULL, 0);
}

void get_font_dimensions(uint8_t *width, uint8_t *height) {
    *width = FONT_WIDTH;
    *height = FONT_HEIGHT;
}

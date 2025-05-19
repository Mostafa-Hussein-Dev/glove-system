#ifndef DRIVERS_DISPLAY_H
#define DRIVERS_DISPLAY_H

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Display text alignment
 */
typedef enum {
    DISPLAY_ALIGN_LEFT = 0,
    DISPLAY_ALIGN_CENTER,
    DISPLAY_ALIGN_RIGHT
} display_align_t;

/**
 * @brief Display font size
 */
typedef enum {
    DISPLAY_FONT_SMALL = 0,  // 6x8 pixels
    DISPLAY_FONT_MEDIUM,     // 8x16 pixels
    DISPLAY_FONT_LARGE       // 16x24 pixels
} display_font_t;

/**
 * @brief Initialize the OLED display
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t display_init(void);

/**
 * @brief Power on the display
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t display_power_on(void);

/**
 * @brief Power off the display
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t display_power_off(void);

/**
 * @brief Set display contrast
 * 
 * @param contrast Contrast value (0-255)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t display_set_contrast(uint8_t contrast);

/**
 * @brief Clear the display
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t display_clear(void);

/**
 * @brief Update the display with current buffer content
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t display_update(void);

/**
 * @brief Draw a text string at specified position
 * 
 * @param text Text string to display
 * @param x X coordinate (0-127)
 * @param y Y coordinate (0-63)
 * @param font Font size
 * @param align Text alignment
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t display_draw_text(const char* text, uint8_t x, uint8_t y, display_font_t font, display_align_t align);

/**
 * @brief Draw a pixel
 * 
 * @param x X coordinate (0-127)
 * @param y Y coordinate (0-63)
 * @param color 1 for white, 0 for black
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t display_draw_pixel(uint8_t x, uint8_t y, uint8_t color);

/**
 * @brief Draw a line
 * 
 * @param x1 Start X coordinate
 * @param y1 Start Y coordinate
 * @param x2 End X coordinate
 * @param y2 End Y coordinate
 * @param color 1 for white, 0 for black
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t display_draw_line(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t color);

/**
 * @brief Draw a rectangle
 * 
 * @param x X coordinate of top-left corner
 * @param y Y coordinate of top-left corner
 * @param width Width of rectangle
 * @param height Height of rectangle
 * @param color 1 for white, 0 for black
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t display_draw_rect(uint8_t x, uint8_t y, uint8_t width, uint8_t height, uint8_t color);

/**
 * @brief Draw a filled rectangle
 * 
 * @param x X coordinate of top-left corner
 * @param y Y coordinate of top-left corner
 * @param width Width of rectangle
 * @param height Height of rectangle
 * @param color 1 for white, 0 for black
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t display_fill_rect(uint8_t x, uint8_t y, uint8_t width, uint8_t height, uint8_t color);

/**
 * @brief Draw a circle
 * 
 * @param x X coordinate of center
 * @param y Y coordinate of center
 * @param radius Radius of circle
 * @param color 1 for white, 0 for black
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t display_draw_circle(uint8_t x, uint8_t y, uint8_t radius, uint8_t color);

/**
 * @brief Draw a filled circle
 * 
 * @param x X coordinate of center
 * @param y Y coordinate of center
 * @param radius Radius of circle
 * @param color 1 for white, 0 for black
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t display_fill_circle(uint8_t x, uint8_t y, uint8_t radius, uint8_t color);

/**
 * @brief Draw a bitmap
 * 
 * @param x X coordinate
 * @param y Y coordinate
 * @param bitmap Bitmap data
 * @param width Width of bitmap
 * @param height Height of bitmap
 * @param color 1 for white, 0 for black
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t display_draw_bitmap(uint8_t x, uint8_t y, const uint8_t *bitmap, uint8_t width, uint8_t height, uint8_t color);

/**
 * @brief Display progress bar
 * 
 * @param x X coordinate
 * @param y Y coordinate
 * @param width Width of progress bar
 * @param height Height of progress bar
 * @param percentage Progress percentage (0-100)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t display_draw_progress_bar(uint8_t x, uint8_t y, uint8_t width, uint8_t height, uint8_t percentage);

/**
 * @brief Display a splash screen on startup
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t display_show_splash_screen(void);

/**
 * @brief Display a debug message
 * 
 * @param message Debug message to display
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t display_show_debug(const char *message);

/**
 * @brief Invert display colors
 * 
 * @param invert True to invert colors, false for normal display
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t display_invert(bool invert);

/**
 * @brief Scroll text vertically
 * 
 * @param start_line Start line (0-7)
 * @param num_lines Number of lines to scroll
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t display_scroll(uint8_t start_line, uint8_t num_lines);

/**
 * @brief Stop scrolling
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t display_stop_scroll(void);

/**
 * @brief Flip the display vertically
 * 
 * @param flip True to flip vertically, false for normal orientation
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t display_flip_vertical(bool flip);

/**
 * @brief Flip the display horizontally
 * 
 * @param flip True to flip horizontally, false for normal orientation
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t display_flip_horizontal(bool flip);

#endif /* DRIVERS_DISPLAY_H */
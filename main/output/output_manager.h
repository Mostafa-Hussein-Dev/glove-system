#ifndef OUTPUT_OUTPUT_MANAGER_H
#define OUTPUT_OUTPUT_MANAGER_H

#include "esp_err.h"
#include "util/buffer.h"

/**
 * @brief Initialize output manager
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t output_manager_init(void);

/**
 * @brief Deinitialize output manager
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t output_manager_deinit(void);

/**
 * @brief Handle output command
 * 
 * @param command Output command to handle
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t output_manager_handle_command(output_command_t *command);

/**
 * @brief Set output mode
 * 
 * @param mode Output mode
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t output_manager_set_mode(output_mode_t mode);

/**
 * @brief Display text on screen
 * 
 * @param text Text to display
 * @param size Font size
 * @param line Line number (0-based)
 * @param clear_first Whether to clear screen first
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t output_manager_display_text(const char *text, uint8_t size, uint8_t line, bool clear_first);

/**
 * @brief Speak text using TTS
 * 
 * @param text Text to speak
 * @param priority Priority (0-highest)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t output_manager_speak_text(const char *text, uint8_t priority);

/**
 * @brief Generate haptic feedback
 * 
 * @param pattern Pattern ID (0-simple, 1-double, 2-success, 3-error)
 * @param intensity Intensity (0-100)
 * @param duration Duration in milliseconds
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t output_manager_haptic_feedback(uint8_t pattern, uint8_t intensity, uint16_t duration_ms);

/**
 * @brief Display battery status
 * 
 * @param percentage Battery percentage (0-100)
 * @param show_graphic Whether to show graphical representation
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t output_manager_show_battery(uint8_t percentage, bool show_graphic);

/**
 * @brief Display error message
 * 
 * @param error_code Error code
 * @param error_text Error description
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t output_manager_show_error(system_error_t error_code, const char *error_text);

/**
 * @brief Display system status
 * 
 * @param state System state
 * @param battery_level Battery level (0-100)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t output_manager_show_status(system_state_t state, uint8_t battery_level);

/**
 * @brief Clear all outputs
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t output_manager_clear(void);

#endif /* OUTPUT_OUTPUT_MANAGER_H */
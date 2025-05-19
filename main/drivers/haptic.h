#ifndef DRIVERS_HAPTIC_H
#define DRIVERS_HAPTIC_H

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Haptic pattern step definition
 */
typedef struct {
    uint8_t intensity;    // Intensity (0-100%)
    uint16_t duration_ms; // Duration in milliseconds
} haptic_pattern_t;

/**
 * @brief Initialize haptic driver
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t haptic_init(void);

/**
 * @brief Deinitialize haptic driver
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t haptic_deinit(void);

/**
 * @brief Set haptic feedback intensity
 * 
 * @param intensity Intensity (0-100%)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t haptic_set_intensity(uint8_t intensity);

/**
 * @brief Simple vibration for specified duration
 * 
 * @param duration_ms Duration in milliseconds
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t haptic_vibrate(uint16_t duration_ms);

/**
 * @brief Play a vibration pattern
 * 
 * @param pattern Array of haptic pattern steps
 * @param pattern_length Number of steps in the pattern
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t haptic_play_pattern(const haptic_pattern_t *pattern, uint8_t pattern_length);

/**
 * @brief Stop any active haptic feedback
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t haptic_stop(void);

/**
 * @brief Check if haptic feedback is active
 * 
 * @param active Pointer to store active state
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t haptic_is_active(bool *active);

#endif /* DRIVERS_HAPTIC_H */
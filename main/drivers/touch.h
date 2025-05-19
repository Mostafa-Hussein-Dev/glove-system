#ifndef DRIVERS_TOUCH_H
#define DRIVERS_TOUCH_H

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

// Number of touch sensors
#define TOUCH_SENSOR_COUNT 5

// Touch sensor IDs
#define TOUCH_SENSOR_THUMB  0
#define TOUCH_SENSOR_INDEX  1
#define TOUCH_SENSOR_MIDDLE 2
#define TOUCH_SENSOR_RING   3
#define TOUCH_SENSOR_PINKY  4

/**
 * @brief Touch event callback function type
 */
typedef void (*touch_callback_t)(bool *status);

/**
 * @brief Initialize touch sensors
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t touch_init(void);

/**
 * @brief Deinitialize touch sensors
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t touch_deinit(void);

/**
 * @brief Calibrate touch sensors
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t touch_calibrate(void);

/**
 * @brief Set touch sensor threshold
 * 
 * @param sensor_id Sensor ID (0-4)
 * @param threshold Threshold value
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t touch_set_threshold(uint8_t sensor_id, uint16_t threshold);

/**
 * @brief Set touch event callback
 * 
 * @param callback Callback function
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t touch_set_callback(touch_callback_t callback);

/**
 * @brief Enable/disable touch sensors
 * 
 * @param enable True to enable, false to disable
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t touch_enable(bool enable);

/**
 * @brief Get current touch status
 * 
 * @param status_array Array to store status (5 elements)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t touch_get_status(bool *status_array);

/**
 * @brief Update touch status (force read)
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t touch_update_status(void);

/**
 * @brief Get raw touch values
 * 
 * @param values_array Array to store values (5 elements)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t touch_get_values(uint16_t *values_array);

/**
 * @brief Check if a specific sensor is active
 * 
 * @param sensor_id Sensor ID (0-4)
 * @return true if active (touched), false otherwise
 */
bool touch_is_sensor_active(uint8_t sensor_id);

/**
 * @brief Touch interrupt handler
 * 
 * @param arg Handler argument (not used)
 */
void touch_intr_handler(void *arg);

#endif /* DRIVERS_TOUCH_H */
#ifndef DRIVERS_FLEX_SENSOR_H
#define DRIVERS_FLEX_SENSOR_H

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Finger identifiers (5 sensors total, 1 per finger)
 */
typedef enum {
    FINGER_THUMB = 0,        // Thumb finger
    FINGER_INDEX,            // Index finger
    FINGER_MIDDLE,           // Middle finger
    FINGER_RING,             // Ring finger
    FINGER_PINKY,            // Pinky finger
    FINGER_COUNT             // Total number of fingers (5)
} finger_t;

/**
 * @brief Calibration data for flex sensors
 */
typedef struct {
    uint16_t flat_value[FINGER_COUNT];    // ADC value when finger is flat (0 degrees)
    uint16_t bent_value[FINGER_COUNT];    // ADC value when finger is bent (90 degrees)
    float scale_factor[FINGER_COUNT];     // Scaling factor for angle calculation
    float offset[FINGER_COUNT];           // Offset for angle calculation
} flex_sensor_calibration_t;

/**
 * @brief Initialize flex sensors
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t flex_sensor_init(void);

/**
 * @brief Read raw ADC values from all flex sensors
 * 
 * @param raw_values Array to store raw ADC values (5 values)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t flex_sensor_read_raw(uint16_t* raw_values);

/**
 * @brief Read calibrated angle values from all flex sensors
 * 
 * @param angles Array to store angle values in degrees (5 values)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t flex_sensor_read_angles(float* angles);

/**
 * @brief Read raw and angle values for a specific finger
 * 
 * @param finger Finger identifier
 * @param raw_value Pointer to store raw ADC value
 * @param angle Pointer to store angle value in degrees
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t flex_sensor_read_finger(finger_t finger, uint16_t* raw_value, float* angle);

/**
 * @brief Calibrate flex sensors using current position as flat (0 degrees)
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t flex_sensor_calibrate_flat(void);

/**
 * @brief Calibrate flex sensors using current position as bent (90 degrees)
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t flex_sensor_calibrate_bent(void);

/**
 * @brief Save flex sensor calibration data to non-volatile storage
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t flex_sensor_save_calibration(void);

/**
 * @brief Load flex sensor calibration data from non-volatile storage
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t flex_sensor_load_calibration(void);

/**
 * @brief Reset flex sensor calibration to default values
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t flex_sensor_reset_calibration(void);

/**
 * @brief Get flex sensor calibration data
 * 
 * @param calibration Pointer to store calibration data
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t flex_sensor_get_calibration(flex_sensor_calibration_t* calibration);

/**
 * @brief Apply digital filtering to flex sensor readings
 * 
 * @param enable Enable or disable filtering
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t flex_sensor_set_filtering(bool enable);

#endif /* DRIVERS_FLEX_SENSOR_H */
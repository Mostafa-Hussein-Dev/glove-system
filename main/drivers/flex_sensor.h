#ifndef DRIVERS_FLEX_SENSOR_H
#define DRIVERS_FLEX_SENSOR_H

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Finger joint identifiers
 */
typedef enum {
    FINGER_THUMB_MCP = 0,    // Thumb metacarpophalangeal joint
    FINGER_THUMB_PIP,        // Thumb proximal interphalangeal joint
    FINGER_INDEX_MCP,        // Index finger metacarpophalangeal joint
    FINGER_INDEX_PIP,        // Index finger proximal interphalangeal joint
    FINGER_MIDDLE_MCP,       // Middle finger metacarpophalangeal joint
    FINGER_MIDDLE_PIP,       // Middle finger proximal interphalangeal joint
    FINGER_RING_MCP,         // Ring finger metacarpophalangeal joint
    FINGER_RING_PIP,         // Ring finger proximal interphalangeal joint
    FINGER_PINKY_MCP,        // Pinky finger metacarpophalangeal joint
    FINGER_PINKY_PIP,        // Pinky finger proximal interphalangeal joint
    FINGER_JOINT_COUNT       // Total number of finger joints
} finger_joint_t;

/**
 * @brief Calibration data for flex sensors
 */
typedef struct {
    uint16_t flat_value[FINGER_JOINT_COUNT];  // ADC value when finger is flat (0 degrees)
    uint16_t bent_value[FINGER_JOINT_COUNT];  // ADC value when finger is bent (90 degrees)
    float scale_factor[FINGER_JOINT_COUNT];   // Scaling factor for angle calculation
    float offset[FINGER_JOINT_COUNT];         // Offset for angle calculation
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
 * @param raw_values Array to store raw ADC values (10 values)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t flex_sensor_read_raw(uint16_t* raw_values);

/**
 * @brief Read calibrated angle values from all flex sensors
 * 
 * @param angles Array to store angle values in degrees (10 values)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t flex_sensor_read_angles(float* angles);

/**
 * @brief Read raw and angle values for a specific finger joint
 * 
 * @param joint Finger joint identifier
 * @param raw_value Pointer to store raw ADC value
 * @param angle Pointer to store angle value in degrees
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t flex_sensor_read_joint(finger_joint_t joint, uint16_t* raw_value, float* angle);

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
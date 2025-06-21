#ifndef DRIVERS_FLEX_SENSOR_H
#define DRIVERS_FLEX_SENSOR_H

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Finger identifiers (5 sensors total, 1 per finger)
 */
typedef enum {
    FINGER_THUMB = 0,        
    FINGER_INDEX,            
    FINGER_MIDDLE,           
    FINGER_RING,             
    FINGER_PINKY,            
    FINGER_COUNT             
} finger_t;

/**
 * @brief Sensor direction (increasing or decreasing with bend)
 */
typedef enum {
    SENSOR_DIRECTION_UNKNOWN = 0,
    SENSOR_DIRECTION_INCREASING,    // ADC value increases when bent
    SENSOR_DIRECTION_DECREASING     // ADC value decreases when bent
} sensor_direction_t;

/**
 * @brief Enhanced calibration data for flex sensors
 */
typedef struct {
    uint16_t flat_value[FINGER_COUNT];        // ADC value when flat (0 degrees)
    uint16_t bent_value[FINGER_COUNT];        // ADC value when bent (90 degrees)
    sensor_direction_t direction[FINGER_COUNT]; // Sensor direction
    float scale_factor[FINGER_COUNT];         // Scaling factor for angle calculation
    float offset[FINGER_COUNT];               // Offset for angle calculation
    uint16_t min_value[FINGER_COUNT];         // Minimum valid ADC value
    uint16_t max_value[FINGER_COUNT];         // Maximum valid ADC value
    bool calibrated[FINGER_COUNT];            // Per-sensor calibration status
} flex_sensor_calibration_t;

/**
 * @brief Sensor health status
 */
typedef struct {
    bool sensor_connected[FINGER_COUNT];      // Sensor connection status
    bool sensor_stable[FINGER_COUNT];         // Sensor stability status
    uint16_t noise_level[FINGER_COUNT];       // Estimated noise level
    uint32_t read_errors[FINGER_COUNT];       // Read error count
} flex_sensor_health_t;

// Function declarations
esp_err_t flex_sensor_init(void);
esp_err_t flex_sensor_deinit(void);
esp_err_t flex_sensor_read_raw(uint16_t* raw_values);
esp_err_t flex_sensor_read_raw_single(finger_t finger, uint16_t* raw_value);
esp_err_t flex_sensor_read_angles(float* angles);
esp_err_t flex_sensor_read_finger(finger_t finger, uint16_t* raw_value, float* angle);

// Enhanced calibration functions
esp_err_t flex_sensor_calibrate_auto(void);
esp_err_t flex_sensor_calibrate_flat(void);
esp_err_t flex_sensor_calibrate_bent(void);
esp_err_t flex_sensor_validate_calibration(void);

// Data persistence
esp_err_t flex_sensor_save_calibration(void);
esp_err_t flex_sensor_load_calibration(void);
esp_err_t flex_sensor_reset_calibration(void);
esp_err_t flex_sensor_get_calibration(flex_sensor_calibration_t* calibration);

// Advanced features
esp_err_t flex_sensor_set_filtering(bool enable);
esp_err_t flex_sensor_get_health_status(flex_sensor_health_t* health);
esp_err_t flex_sensor_test_connectivity(void);
esp_err_t flex_sensor_calibrate_noise_floor(void);
void flex_sensor_test_calibration_math(finger_t finger);


#endif /* DRIVERS_FLEX_SENSOR_H */
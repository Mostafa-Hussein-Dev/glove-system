#ifndef DRIVERS_IMU_H
#define DRIVERS_IMU_H

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

/**
 * @brief IMU accelerometer range settings
 */
typedef enum {
    IMU_ACCEL_RANGE_2G = 0,  // ±2g
    IMU_ACCEL_RANGE_4G,      // ±4g
    IMU_ACCEL_RANGE_8G,      // ±8g
    IMU_ACCEL_RANGE_16G      // ±16g
} imu_accel_range_t;

/**
 * @brief Load IMU calibration data from NVS
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t imu_load_calibration(void);

/**
 * @brief Save IMU calibration data to NVS
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t imu_save_calibration(void);

/**
 * @brief IMU gyroscope range settings
 */
typedef enum {
    IMU_GYRO_RANGE_250DPS = 0,  // ±250°/s
    IMU_GYRO_RANGE_500DPS,      // ±500°/s
    IMU_GYRO_RANGE_1000DPS,     // ±1000°/s
    IMU_GYRO_RANGE_2000DPS      // ±2000°/s
} imu_gyro_range_t;

/**
 * @brief IMU digital low-pass filter bandwidth settings
 */
typedef enum {
    IMU_DLPF_BW_256HZ = 0,  // 256Hz
    IMU_DLPF_BW_188HZ,      // 188Hz
    IMU_DLPF_BW_98HZ,       // 98Hz
    IMU_DLPF_BW_42HZ,       // 42Hz
    IMU_DLPF_BW_20HZ,       // 20Hz
    IMU_DLPF_BW_10HZ,       // 10Hz
    IMU_DLPF_BW_5HZ         // 5Hz
} imu_dlpf_bandwidth_t;

/**
 * @brief IMU sample rate divider
 * Sample rate = 1kHz / (1 + divider)
 */
typedef uint8_t imu_sample_rate_div_t;

/**
 * @brief IMU configuration
 */
typedef struct {
    imu_accel_range_t accel_range;
    imu_gyro_range_t gyro_range;
    imu_dlpf_bandwidth_t dlpf_bandwidth;
    imu_sample_rate_div_t sample_rate_div;
    bool use_dlpf;
} imu_config_t;

/**
 * @brief IMU raw data
 */
typedef struct {
    int16_t accel_raw[3];  // Raw accelerometer data (x, y, z)
    int16_t gyro_raw[3];   // Raw gyroscope data (x, y, z)
    int16_t temp_raw;      // Raw temperature data
} imu_raw_data_t;

/**
 * @brief IMU calibrated data
 */
typedef struct {
    float accel[3];      // Calibrated accelerometer data in g (x, y, z)
    float gyro[3];       // Calibrated gyroscope data in °/s (x, y, z)
    float temp;          // Calibrated temperature in °C
    float orientation[3]; // Roll, pitch, yaw in degrees
    uint32_t timestamp;  // Timestamp in milliseconds
} imu_data_t;

/**
 * @brief IMU motion detection configuration
 */
typedef struct {
    uint8_t threshold;       // Motion detection threshold (0-255)
    uint8_t duration;        // Motion detection duration (0-255)
    bool x_axis_enable;      // Enable motion detection on X axis
    bool y_axis_enable;      // Enable motion detection on Y axis
    bool z_axis_enable;      // Enable motion detection on Z axis
} imu_motion_detection_config_t;

/**
 * @brief Initialize IMU
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t imu_init(void);

/**
 * @brief Configure IMU with specified settings
 * 
 * @param config Configuration settings
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t imu_config(const imu_config_t* config);

/**
 * @brief Get current IMU configuration
 * 
 * @param config Pointer to store current configuration
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t imu_get_config(imu_config_t* config);

/**
 * @brief Read raw IMU data
 * 
 * @param raw_data Pointer to store raw data
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t imu_read_raw(imu_raw_data_t* raw_data);

/**
 * @brief Read calibrated IMU data
 * 
 * @param data Pointer to store calibrated data
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t imu_read(imu_data_t* data);

/**
 * @brief Perform IMU calibration
 * 
 * The device should be kept stationary during calibration.
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t imu_calibrate(void);

/**
 * @brief Reset IMU calibration to default values
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t imu_reset_calibration(void);

/**
 * @brief Configure motion detection
 * 
 * @param config Motion detection configuration
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t imu_config_motion_detection(const imu_motion_detection_config_t* config);

/**
 * @brief Enable motion detection
 * 
 * @param enable True to enable, false to disable
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t imu_enable_motion_detection(bool enable);

/**
 * @brief Check if motion is detected
 * 
 * @param detected Pointer to store detection status
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t imu_is_motion_detected(bool* detected);

/**
 * @brief Configure and use IMU interrupts (motion detection or data ready)
 * 
 * @param enable True to enable, false to disable
 * @param interrupt_type 0 for motion detection, 1 for data ready
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t imu_config_interrupts(bool enable, uint8_t interrupt_type);

/**
 * @brief Enter low power mode
 * 
 * @param enable True to enter low power mode, false to exit
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t imu_set_low_power_mode(bool enable);

/**
 * @brief Reset IMU hardware
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t imu_reset(void);

/**
 * @brief Apply complementary filter to calculate orientation
 * 
 * @param accel Accelerometer data in g
 * @param gyro Gyroscope data in °/s
 * @param dt Time difference in seconds
 * @param previous_orientation Previous orientation in degrees
 * @param new_orientation New orientation in degrees
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t imu_calculate_orientation(const float accel[3], const float gyro[3], 
                                   float dt, const float previous_orientation[3], 
                                   float new_orientation[3]);

#endif /* DRIVERS_IMU_H */
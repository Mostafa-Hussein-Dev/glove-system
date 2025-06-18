#include "drivers/imu.h"
#include <string.h>
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "esp_timer.h"
#include "math.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "config/pin_definitions.h"
#include "util/debug.h"
#include "util/i2c_utils.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static i2c_master_bus_handle_t i2c_bus_handle = NULL;
static i2c_master_dev_handle_t imu_dev_handle = NULL;

static const char *TAG = "IMU";

// MPU6050 I2C address
#define MPU6050_ADDR 0x68

// MPU6050 registers
#define MPU6050_REG_PWR_MGMT_1    0x6B
#define MPU6050_REG_SMPLRT_DIV    0x19
#define MPU6050_REG_CONFIG        0x1A
#define MPU6050_REG_GYRO_CONFIG   0x1B
#define MPU6050_REG_ACCEL_CONFIG  0x1C
#define MPU6050_REG_INT_ENABLE    0x38
#define MPU6050_REG_ACCEL_XOUT_H  0x3B
#define MPU6050_REG_TEMP_OUT_H    0x41
#define MPU6050_REG_GYRO_XOUT_H   0x43
#define MPU6050_REG_MOT_THR       0x1F
#define MPU6050_REG_MOT_DUR       0x20
#define MPU6050_REG_MOT_DETECT_CTRL 0x69
#define MPU6050_REG_INT_STATUS    0x3A
#define MPU6050_REG_WHO_AM_I      0x75

// MPU6050 constants
#define MPU6050_CLOCK_PLL_XGYRO   0x01
#define MPU6050_INT_ENABLE_DATA_RDY  0x01
#define MPU6050_INT_ENABLE_MOT    0x40
#define MPU6050_WHO_AM_I_VAL      0x68

// NVS namespace for IMU calibration
#define IMU_NVS_NAMESPACE "imu"
#define IMU_NVS_KEY "calibration"

// Gravitational acceleration constant
#define GRAVITY_EARTH 9.80665f

// Accelerometer scale factors for different ranges
static const float accel_scale_factor[] = {
    16384.0f,  // ±2g
    8192.0f,   // ±4g
    4096.0f,   // ±8g
    2048.0f    // ±16g
};

// Gyroscope scale factors for different ranges
static const float gyro_scale_factor[] = {
    131.0f,    // ±250°/s
    65.5f,     // ±500°/s
    32.8f,     // ±1000°/s
    16.4f      // ±2000°/s
};

// IMU calibration data
typedef struct {
    int16_t accel_offset[3];
    int16_t gyro_offset[3];
    float orientation_offset[3];
} imu_calibration_t;

// Current configuration
static imu_config_t current_config = {
    .accel_range = IMU_ACCEL_RANGE_2G,
    .gyro_range = IMU_GYRO_RANGE_500DPS,
    .dlpf_bandwidth = IMU_DLPF_BW_20HZ,
    .sample_rate_div = 9,   // 100Hz sample rate (1000/(1+9))
    .use_dlpf = true
};

// Calibration data
static imu_calibration_t calibration = {
    .accel_offset = {0, 0, 0},
    .gyro_offset = {0, 0, 0},
    .orientation_offset = {0.0f, 0.0f, 0.0f}
};

// Previous orientation for filtering
static float prev_orientation[3] = {0.0f, 0.0f, 0.0f};
static uint32_t prev_time_us = 0;

// Motion detection configuration
static imu_motion_detection_config_t motion_config = {
    .threshold = 20,        // Default threshold (0-255)
    .duration = 5,          // Default duration (0-255)
    .x_axis_enable = true,
    .y_axis_enable = true,
    .z_axis_enable = true
};

// I2C utilities for MPU6050
static esp_err_t mpu6050_write_byte(uint8_t reg_addr, uint8_t data) {
    uint8_t write_buf[2] = {reg_addr, data};
    return i2c_write_device(imu_dev_handle, write_buf, sizeof(write_buf), 100);
}

static esp_err_t mpu6050_read_bytes(uint8_t reg_addr, uint8_t *data, size_t len) {
    return i2c_write_read_device(imu_dev_handle, &reg_addr, 1, data, len, 100);
}

static esp_err_t calculate_calibration_factors(void) {
    // No additional calculation needed for calibration factors
    // Just log the current offsets
    ESP_LOGI(TAG, "Calibration factors - Accel offset: [%d, %d, %d], Gyro offset: [%d, %d, %d]",
             calibration.accel_offset[0], calibration.accel_offset[1], calibration.accel_offset[2],
             calibration.gyro_offset[0], calibration.gyro_offset[1], calibration.gyro_offset[2]);
    return ESP_OK;
}

static float normalize_angle(float angle) {
    while (angle > 180.0f) {
        angle -= 360.0f;
    }
    while (angle < -180.0f) {
        angle += 360.0f;
    }
    return angle;
}

esp_err_t imu_init(void) {
    esp_err_t ret;
    uint8_t who_am_i;
    
    // Initialize I2C master bus if not already done (shared with display)
    // Use the global I2C bus created in app_main
    extern i2c_master_bus_handle_t i2c_master_bus;
    i2c_bus_handle = i2c_master_bus;

    if (i2c_bus_handle == NULL) {
        ESP_LOGE(TAG, "Global I2C master bus not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    // Add IMU device to I2C bus
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = MPU6050_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };
    
    ret = i2c_master_bus_add_device(i2c_bus_handle, &dev_cfg, &imu_dev_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add IMU device to I2C bus: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Verify device identity
    ret = mpu6050_read_bytes(MPU6050_REG_WHO_AM_I, &who_am_i, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read WHO_AM_I register");
        return ret;
    }
    
    if (who_am_i != MPU6050_WHO_AM_I_VAL) {
        ESP_LOGE(TAG, "MPU6050 not found, WHO_AM_I = 0x%02x (expected 0x%02x)", who_am_i, MPU6050_WHO_AM_I_VAL);
        return ESP_ERR_NOT_FOUND;
    }
    
    // Wake up the MPU6050
    ret = mpu6050_write_byte(MPU6050_REG_PWR_MGMT_1, MPU6050_CLOCK_PLL_XGYRO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to wake up MPU6050");
        return ret;
    }
    
    // Configure with default settings
    ret = imu_config(&current_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure IMU");
        return ret;
    }
    
    // Load calibration data
    ret = imu_load_calibration();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to load calibration data, using defaults");
        // Calculate default calibration factors
        calculate_calibration_factors();
    }
    
    // Initialize timestamps
    prev_time_us = esp_timer_get_time();
    
    ESP_LOGI(TAG, "IMU initialized successfully");
    return ESP_OK;
}

esp_err_t imu_config(const imu_config_t* config) {
    if (config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    esp_err_t ret;
    
    // Set sample rate divider
    ret = mpu6050_write_byte(MPU6050_REG_SMPLRT_DIV, config->sample_rate_div);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Configure digital low-pass filter
    uint8_t dlpf_config = config->use_dlpf ? config->dlpf_bandwidth : 0;
    ret = mpu6050_write_byte(MPU6050_REG_CONFIG, dlpf_config);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Configure gyroscope range
    ret = mpu6050_write_byte(MPU6050_REG_GYRO_CONFIG, config->gyro_range << 3);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Configure accelerometer range
    ret = mpu6050_write_byte(MPU6050_REG_ACCEL_CONFIG, config->accel_range << 3);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Save current configuration
    memcpy(&current_config, config, sizeof(imu_config_t));
    
    ESP_LOGI(TAG, "IMU configured: accel_range=%d, gyro_range=%d, dlpf=%d, sample_rate_div=%d",
             config->accel_range, config->gyro_range, config->dlpf_bandwidth, config->sample_rate_div);
    
    return ESP_OK;
}

esp_err_t imu_get_config(imu_config_t* config) {
    if (config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Copy current configuration
    memcpy(config, &current_config, sizeof(imu_config_t));
    
    return ESP_OK;
}

esp_err_t imu_read_raw(imu_raw_data_t* raw_data) {
    if (raw_data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Read 14 bytes of data starting from ACCEL_XOUT_H register
    uint8_t buffer[14];
    esp_err_t ret = mpu6050_read_bytes(MPU6050_REG_ACCEL_XOUT_H, buffer, 14);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Combine high and low bytes
    raw_data->accel_raw[0] = (int16_t)((buffer[0] << 8) | buffer[1]);
    raw_data->accel_raw[1] = (int16_t)((buffer[2] << 8) | buffer[3]);
    raw_data->accel_raw[2] = (int16_t)((buffer[4] << 8) | buffer[5]);
    
    raw_data->temp_raw = (int16_t)((buffer[6] << 8) | buffer[7]);
    
    raw_data->gyro_raw[0] = (int16_t)((buffer[8] << 8) | buffer[9]);
    raw_data->gyro_raw[1] = (int16_t)((buffer[10] << 8) | buffer[11]);
    raw_data->gyro_raw[2] = (int16_t)((buffer[12] << 8) | buffer[13]);
    
    return ESP_OK;
}

esp_err_t imu_read(imu_data_t* data) {
    if (data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Read raw data
    imu_raw_data_t raw_data;
    esp_err_t ret = imu_read_raw(&raw_data);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Apply calibration offsets
    int16_t accel_calibrated[3];
    int16_t gyro_calibrated[3];
    
    for (int i = 0; i < 3; i++) {
        accel_calibrated[i] = raw_data.accel_raw[i] - calibration.accel_offset[i];
        gyro_calibrated[i] = raw_data.gyro_raw[i] - calibration.gyro_offset[i];
    }
    
    // Convert to physical units
    float accel_scale = accel_scale_factor[current_config.accel_range];
    float gyro_scale = gyro_scale_factor[current_config.gyro_range];
    
    for (int i = 0; i < 3; i++) {
        data->accel[i] = (float)accel_calibrated[i] / accel_scale;
        data->gyro[i] = (float)gyro_calibrated[i] / gyro_scale;
    }
    
    // Convert to m/s² (standard gravity units)
    for (int i = 0; i < 3; i++) {
        data->accel[i] *= GRAVITY_EARTH;
    }
    
    // Convert temperature
    data->temp = (float)raw_data.temp_raw / 340.0f + 36.53f;
    
    // Update timestamp
    uint32_t current_time_us = esp_timer_get_time();
    data->timestamp = current_time_us / 1000;  // Convert to milliseconds
    
    // Handle first reading - initialize orientation from accelerometer
    if (prev_time_us == 0) {
        // Calculate initial orientation from accelerometer only
        data->orientation[0] = atan2f(data->accel[1], data->accel[2]) * 180.0f / M_PI;  // Roll
        data->orientation[1] = atan2f(-data->accel[0], sqrtf(data->accel[1] * data->accel[1] + data->accel[2] * data->accel[2])) * 180.0f / M_PI;  // Pitch
        data->orientation[2] = 0.0f;  // Yaw starts at 0
        
        // Store for next iteration
        memcpy(prev_orientation, data->orientation, sizeof(prev_orientation));
        prev_time_us = current_time_us;
        
        ESP_LOGI("IMU", "Initial orientation: Roll=%.1f, Pitch=%.1f, Yaw=%.1f", 
                 data->orientation[0], data->orientation[1], data->orientation[2]);
        return ESP_OK;
    }
    
    // Calculate time delta
    float dt = (current_time_us - prev_time_us) / 1000000.0f;  // Convert to seconds
    prev_time_us = current_time_us;
    
    // Calculate orientation using complementary filter
    ret = imu_calculate_orientation(data->accel, data->gyro, dt, prev_orientation, data->orientation);
    if (ret != ESP_OK) {
        ESP_LOGE("IMU", "Failed to calculate orientation");
        return ret;
    }
    
    // Update previous orientation
    memcpy(prev_orientation, data->orientation, sizeof(prev_orientation));
    
    return ESP_OK;
}

esp_err_t imu_calibrate(void) {
    ESP_LOGI(TAG, "Starting IMU calibration (keep device still)...");
    
    vTaskDelay(pdMS_TO_TICKS(500));

    // Initialize accumulators
    int32_t accel_sum[3] = {0, 0, 0};
    int32_t gyro_sum[3] = {0, 0, 0};
    const int num_samples = 1000;
    
    // Collect samples with outlier rejection
    int valid_samples = 0;
    for (int i = 0; i < num_samples && valid_samples < 800; i++) {
        imu_raw_data_t raw_data;
        esp_err_t ret = imu_read_raw(&raw_data);
        if (ret != ESP_OK) continue;
        
        // Simple outlier rejection (reject samples too far from expected values)
        bool is_outlier = false;
        for (int j = 0; j < 3; j++) {
            if (abs(raw_data.gyro_raw[j]) > 1000) { // Reject high gyro values during calibration
                is_outlier = true;
                break;
            }
        }
        
        if (!is_outlier) {
            for (int j = 0; j < 3; j++) {
                accel_sum[j] += raw_data.accel_raw[j];
                gyro_sum[j] += raw_data.gyro_raw[j];
            }
            valid_samples++;
        }
        
        vTaskDelay(pdMS_TO_TICKS(5)); // 5ms between samples
    }
    
    // Calculate averages
    for (int i = 0; i < 3; i++) {
        calibration.accel_offset[i] = accel_sum[i] / valid_samples;
        calibration.gyro_offset[i] = gyro_sum[i] / valid_samples;
    }
    
    // For accelerometer, we only want to remove offset from X and Y axes
    // Z axis should be close to 1g when device is flat
    float accel_scale = accel_scale_factor[current_config.accel_range];
    float expected_z = accel_scale; // 1g in raw units
    
    // Adjust Z offset to keep gravity
    calibration.accel_offset[2] -= expected_z;
    
    // Reset orientation offset
    for (int i = 0; i < 3; i++) {
        calibration.orientation_offset[i] = 0.0f;
        prev_orientation[i] = 0.0f;
    }
    
    // Save calibration data
    esp_err_t ret = imu_save_calibration();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save calibration data");
        return ret;
    }
    
    // Log calibration results
    ESP_LOGI(TAG, "IMU calibration complete");
    calculate_calibration_factors();
    
    return ESP_OK;
}

esp_err_t imu_reset_calibration(void) {
    ESP_LOGI(TAG, "Resetting IMU calibration to defaults...");
    
    // Reset calibration data
    for (int i = 0; i < 3; i++) {
        calibration.accel_offset[i] = 0;
        calibration.gyro_offset[i] = 0;
        calibration.orientation_offset[i] = 0.0f;
        prev_orientation[i] = 0.0f;
    }
    
    // Save default calibration
    esp_err_t ret = imu_save_calibration();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save default calibration data");
        return ret;
    }
    
    ESP_LOGI(TAG, "IMU calibration reset to defaults");
    return ESP_OK;
}

esp_err_t imu_config_motion_detection(const imu_motion_detection_config_t* config) {
    if (config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Save configuration
    memcpy(&motion_config, config, sizeof(imu_motion_detection_config_t));
    
    // Set motion detection threshold (in mg, 1LSB = 32mg)
    esp_err_t ret = mpu6050_write_byte(MPU6050_REG_MOT_THR, config->threshold);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Set motion detection duration (in ms)
    ret = mpu6050_write_byte(MPU6050_REG_MOT_DUR, config->duration);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Configure motion detection control
    uint8_t mot_ctrl = 0;
    if (config->x_axis_enable) mot_ctrl |= 0x01;
    if (config->y_axis_enable) mot_ctrl |= 0x02;
    if (config->z_axis_enable) mot_ctrl |= 0x04;
    
    ret = mpu6050_write_byte(MPU6050_REG_MOT_DETECT_CTRL, mot_ctrl);
    if (ret != ESP_OK) {
        return ret;
    }
    
    ESP_LOGI(TAG, "Motion detection configured: threshold=%d, duration=%d, axes=%c%c%c",
             config->threshold, config->duration,
             config->x_axis_enable ? 'X' : '-',
             config->y_axis_enable ? 'Y' : '-',
             config->z_axis_enable ? 'Z' : '-');
    
    return ESP_OK;
}

esp_err_t imu_enable_motion_detection(bool enable) {
    // Configure interrupt
    uint8_t int_enable = enable ? MPU6050_INT_ENABLE_MOT : 0;
    
    esp_err_t ret = mpu6050_write_byte(MPU6050_REG_INT_ENABLE, int_enable);
    if (ret != ESP_OK) {
        return ret;
    }
    
    ESP_LOGI(TAG, "Motion detection %s", enable ? "enabled" : "disabled");
    return ESP_OK;
}

esp_err_t imu_is_motion_detected(bool* detected) {
    if (detected == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Read interrupt status register
    uint8_t int_status;
    esp_err_t ret = mpu6050_read_bytes(MPU6050_REG_INT_STATUS, &int_status, 1);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Check motion interrupt flag
    *detected = (int_status & MPU6050_INT_ENABLE_MOT) != 0;
    
    return ESP_OK;
}

esp_err_t imu_config_interrupts(bool enable, uint8_t interrupt_type) {
    uint8_t int_enable = 0;
    
    if (enable) {
        if (interrupt_type == 0) {
            // Motion detection interrupt
            int_enable = MPU6050_INT_ENABLE_MOT;
        } else if (interrupt_type == 1) {
            // Data ready interrupt
            int_enable = MPU6050_INT_ENABLE_DATA_RDY;
        } else {
            return ESP_ERR_INVALID_ARG;
        }
    }
    
    esp_err_t ret = mpu6050_write_byte(MPU6050_REG_INT_ENABLE, int_enable);
    if (ret != ESP_OK) {
        return ret;
    }
    
    ESP_LOGI(TAG, "%s interrupt %s", 
             interrupt_type == 0 ? "Motion detection" : "Data ready",
             enable ? "enabled" : "disabled");
    
    return ESP_OK;
}

esp_err_t imu_set_low_power_mode(bool enable) {
    // Read current power management register
    uint8_t pwr_mgmt;
    esp_err_t ret = mpu6050_read_bytes(MPU6050_REG_PWR_MGMT_1, &pwr_mgmt, 1);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Set or clear sleep bit
    if (enable) {
        pwr_mgmt |= 0x40;  // Set sleep bit
    } else {
        pwr_mgmt &= ~0x40; // Clear sleep bit
    }
    
    // Write back to register
    ret = mpu6050_write_byte(MPU6050_REG_PWR_MGMT_1, pwr_mgmt);
    if (ret != ESP_OK) {
        return ret;
    }
    
    ESP_LOGI(TAG, "Low power mode %s", enable ? "enabled" : "disabled");
    return ESP_OK;
}

esp_err_t imu_reset(void) {
    // Set reset bit in power management register
    esp_err_t ret = mpu6050_write_byte(MPU6050_REG_PWR_MGMT_1, 0x80);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Wait for reset to complete
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Re-initialize the IMU
    ret = imu_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to re-initialize IMU after reset");
        return ret;
    }
    
    ESP_LOGI(TAG, "IMU reset and re-initialized");
    return ESP_OK;
}

esp_err_t imu_calculate_orientation(const float accel[3], const float gyro[3], float dt, const float previous_orientation[3], float new_orientation[3]) {
    if (accel == NULL || gyro == NULL || previous_orientation == NULL || new_orientation == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Validate time delta - if invalid, calculate orientation from accelerometer only
    if (dt <= 0.0f || dt > 0.1f) {
        // Invalid time delta, calculate orientation from accelerometer only
        new_orientation[0] = atan2f(accel[1], accel[2]) * 180.0f / M_PI;  // Roll from accelerometer
        new_orientation[1] = atan2f(-accel[0], sqrtf(accel[1] * accel[1] + accel[2] * accel[2])) * 180.0f / M_PI; // Pitch from accelerometer  
        new_orientation[2] = previous_orientation[2]; // Keep previous yaw
        return ESP_OK;
    }
    
    // Complementary filter constants
    const float alpha = 0.98f;  // Weight for gyro data
    
    // FIX 1: Correct pitch calculation (add negative sign)
    float accel_pitch = atan2f(-accel[0], sqrtf(accel[1] * accel[1] + accel[2] * accel[2])) * 180.0f / M_PI;
    float accel_roll = atan2f(accel[1], accel[2]) * 180.0f / M_PI;
    
    // FIX 3: Dynamic gyro bias compensation for yaw
    static float gyro_z_bias_estimate = 0.0f;
    static float gyro_z_history[10] = {0}; // Rolling average
    static int history_index = 0;
    
    // Update rolling average of gyro Z
    gyro_z_history[history_index] = gyro[2];
    history_index = (history_index + 1) % 10;
    
    // Calculate average
    float gyro_z_avg = 0.0f;
    for (int i = 0; i < 10; i++) {
        gyro_z_avg += gyro_z_history[i];
    }
    gyro_z_avg /= 10.0f;
    
    // If gyro readings are consistently small (likely bias), update bias estimate
    if (fabs(gyro_z_avg) < 2.0f && fabs(gyro[0]) < 5.0f && fabs(gyro[1]) < 5.0f) {
        gyro_z_bias_estimate = 0.95f * gyro_z_bias_estimate + 0.05f * gyro_z_avg;
    }
    
    // Apply bias compensation to yaw calculation
    float corrected_gyro_z = gyro[2] - gyro_z_bias_estimate;
    new_orientation[2] = previous_orientation[2] + corrected_gyro_z * dt;
    
    // Apply complementary filter to roll and pitch only
    new_orientation[0] = alpha * (previous_orientation[0] + gyro[0] * dt) + (1.0f - alpha) * accel_roll;
    new_orientation[1] = alpha * (previous_orientation[1] + gyro[1] * dt) + (1.0f - alpha) * accel_pitch;
    
    // FIX 2: Normalize all angles to prevent drift
    new_orientation[0] = normalize_angle(new_orientation[0]);  // Roll
    new_orientation[1] = normalize_angle(new_orientation[1]);  // Pitch
    new_orientation[2] = normalize_angle(new_orientation[2]);  // Yaw
    
    // FIX 3: Yaw drift compensation
    static uint32_t last_yaw_reset = 0;
    uint32_t current_time = esp_timer_get_time() / 1000; // Convert to ms
    
    // Check if hand is stationary (low angular velocity)
    float angular_motion = sqrtf(gyro[0]*gyro[0] + gyro[1]*gyro[1] + gyro[2]*gyro[2]);
    
    // Reset yaw when stationary for 3 seconds
    if (angular_motion < 3.0f && (current_time - last_yaw_reset) > 3000) {
        new_orientation[2] = 0.0f; // Reset yaw to reference
        last_yaw_reset = current_time;
        ESP_LOGI("IMU", "Yaw drift reset (stationary)");
    }
    
    return ESP_OK;
}

esp_err_t imu_save_calibration(void) {
    ESP_LOGI(TAG, "Saving IMU calibration...");
    
    nvs_handle_t nvs_handle;
    esp_err_t ret;
    
    ret = nvs_open(IMU_NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error opening NVS handle: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Write calibration data to NVS
    ret = nvs_set_blob(nvs_handle, IMU_NVS_KEY, &calibration, sizeof(imu_calibration_t));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error writing to NVS: %s", esp_err_to_name(ret));
        nvs_close(nvs_handle);
        return ret;
    }
    
    // Commit the changes
    ret = nvs_commit(nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error committing NVS changes: %s", esp_err_to_name(ret));
    }
    
    nvs_close(nvs_handle);
    return ret;
}

esp_err_t imu_load_calibration(void) {
    ESP_LOGI(TAG, "Loading IMU calibration...");
    
    nvs_handle_t nvs_handle;
    esp_err_t ret;
    
    ret = nvs_open(IMU_NVS_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Error opening NVS handle: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Read calibration data from NVS
    size_t required_size = sizeof(imu_calibration_t);
    ret = nvs_get_blob(nvs_handle, IMU_NVS_KEY, &calibration, &required_size);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Error reading from NVS: %s", esp_err_to_name(ret));
        nvs_close(nvs_handle);
        return ret;
    }
    
    nvs_close(nvs_handle);
    
    // Reset previous orientation
    memcpy(prev_orientation, calibration.orientation_offset, sizeof(prev_orientation));
    
    ESP_LOGI(TAG, "IMU calibration loaded successfully");
    return ESP_OK;
}
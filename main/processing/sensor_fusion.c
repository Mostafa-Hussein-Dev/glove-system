#include "processing/sensor_fusion.h"
#include <string.h>
#include <math.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "util/buffer.h"
#include "util/debug.h"
#include "drivers/flex_sensor.h" 

static const char *TAG = "SENSOR_FUSION";

// Complementary filter coefficients for sensor fusion
#define ALPHA_FLEX_SENSOR   0.7f    // Flex sensor weight in fusion
#define ALPHA_IMU           0.2f    // IMU weight in fusion
#define ALPHA_CAMERA        0.1f    // Camera weight in fusion (small due to higher latency)

// Last fused sensor data for reference
static sensor_data_t last_fused_data;

// Initialized flag
static bool sensor_fusion_initialized = false;

esp_err_t sensor_fusion_init(void) {
    // Initialize fusion state
    memset(&last_fused_data, 0, sizeof(sensor_data_t));
    
    sensor_fusion_initialized = true;
    ESP_LOGI(TAG, "Sensor fusion initialized");
    
    return ESP_OK;
}

esp_err_t sensor_fusion_deinit(void) {
    sensor_fusion_initialized = false;
    ESP_LOGI(TAG, "Sensor fusion deinitialized");
    
    return ESP_OK;
}

esp_err_t sensor_fusion_process(sensor_data_t *new_data, sensor_data_buffer_t *data_buffer) {
    if (!sensor_fusion_initialized || new_data == NULL || data_buffer == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Simple complementary filter-based fusion
    // In a complete implementation, this would be more sophisticated
    
    // If we have both flex and IMU data valid, perform fusion
    if (new_data->flex_data_valid && new_data->imu_data_valid) {
        // For this basic version, we simply use the data as-is
        // A full implementation would fuse the data using more complex algorithms
        
        // Example fusion (placeholder): adjust flex sensor readings based on hand orientation
        // This is a simplistic approach - in reality, you'd use a more sophisticated model
        
        // Get hand orientation from IMU
        float roll = new_data->imu_data.orientation[0];
        float pitch = new_data->imu_data.orientation[1];
        
        // Apply small corrections to flex sensor readings based on orientation
        // This is just an illustrative example - real fusion would be more complex
        for (int i = 0; i < FINGER_COUNT; i++) {
            // Small adjustment based on gravity's effect on sensor when hand tilted
            float orientation_factor = 1.0f - (fabs(roll) + fabs(pitch)) * 0.001f; // Very small correction
            
            // Apply correction to flex sensor angle
            new_data->flex_data.angles[i] *= orientation_factor;
            
            // Clamp to valid range
            if (new_data->flex_data.angles[i] < 0.0f) {
                new_data->flex_data.angles[i] = 0.0f;
            } else if (new_data->flex_data.angles[i] > 90.0f) {
                new_data->flex_data.angles[i] = 90.0f;
            }
        }
        
        // Temporal fusion: if we have historical data, smooth the changes
        size_t buffer_size = buffer_get_size(data_buffer);
        if (buffer_size > 1) {
            // Get the previous data point for temporal smoothing
            sensor_data_t prev_data;
            esp_err_t ret = buffer_get(data_buffer, buffer_size - 2, &prev_data);
            if (ret == ESP_OK && prev_data.flex_data_valid) {
                // Apply temporal smoothing (simple low-pass filter)
                const float temporal_alpha = 0.8f; // Smoothing factor
                
                for (int i = 0; i < FINGER_COUNT; i++) {
                    new_data->flex_data.angles[i] = temporal_alpha * new_data->flex_data.angles[i] +
                                                   (1.0f - temporal_alpha) * prev_data.flex_data.angles[i];
                }
            }
        }
    }
    
    // Perform IMU data fusion if we have multiple IMU samples
    if (new_data->imu_data_valid) {
        size_t buffer_size = buffer_get_size(data_buffer);
        if (buffer_size > 0) {
            // Simple complementary filter for orientation
            sensor_data_t prev_data;
            esp_err_t ret = buffer_get(data_buffer, buffer_size - 1, &prev_data);
            if (ret == ESP_OK && prev_data.imu_data_valid) {
                // Time difference for integration
                float dt = (new_data->imu_data.timestamp - prev_data.imu_data.timestamp) / 1000.0f; // Convert to seconds
                
                if (dt > 0 && dt < 0.1f) { // Reasonable time delta (less than 100ms)
                    // Integrate gyroscope data for orientation estimate
                    float gyro_roll = prev_data.imu_data.orientation[0] + new_data->imu_data.gyro[0] * dt;
                    float gyro_pitch = prev_data.imu_data.orientation[1] + new_data->imu_data.gyro[1] * dt;
                    float gyro_yaw = prev_data.imu_data.orientation[2] + new_data->imu_data.gyro[2] * dt;
                    
                    // Complementary filter: combine gyro integration with accelerometer
                    const float gyro_weight = 0.98f; // Trust gyro more for short-term
                    const float accel_weight = 0.02f; // Trust accelerometer for long-term
                    
                    // Calculate accelerometer-based roll and pitch (yaw cannot be determined from accel alone)
                    float accel_roll = atan2(new_data->imu_data.accel[1], new_data->imu_data.accel[2]) * 180.0f / M_PI;
                    float accel_pitch = atan2(-new_data->imu_data.accel[0], 
                                             sqrt(new_data->imu_data.accel[1] * new_data->imu_data.accel[1] + 
                                                  new_data->imu_data.accel[2] * new_data->imu_data.accel[2])) * 180.0f / M_PI;
                    
                    // Fuse gyro and accelerometer data
                    new_data->imu_data.orientation[0] = gyro_weight * gyro_roll + accel_weight * accel_roll;
                    new_data->imu_data.orientation[1] = gyro_weight * gyro_pitch + accel_weight * accel_pitch;
                    new_data->imu_data.orientation[2] = gyro_yaw; // Yaw relies purely on gyro integration
                }
            }
        }
    }
    
    // Update last fused data
    memcpy(&last_fused_data, new_data, sizeof(sensor_data_t));
    
    return ESP_OK;
}

esp_err_t sensor_fusion_get_latest(sensor_data_t *data) {
    if (!sensor_fusion_initialized || data == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Copy the latest fused data
    memcpy(data, &last_fused_data, sizeof(sensor_data_t));
    
    return ESP_OK;
}
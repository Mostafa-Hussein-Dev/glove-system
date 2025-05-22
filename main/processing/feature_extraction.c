#include "processing/feature_extraction.h"
#include <string.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "util/buffer.h"
#include "util/debug.h"
#include "math.h"
#include "drivers/flex_sensor.h"

static const char *TAG = "FEATURE_EXTRACT";

// Feature extraction state
static bool feature_extraction_initialized = false;

esp_err_t feature_extraction_init(void) {
    feature_extraction_initialized = true;
    ESP_LOGI(TAG, "Feature extraction initialized");
    
    return ESP_OK;
}

esp_err_t feature_extraction_deinit(void) {
    feature_extraction_initialized = false;
    ESP_LOGI(TAG, "Feature extraction deinitialized");
    
    return ESP_OK;
}

esp_err_t feature_extraction_process(sensor_data_t *sensor_data, 
                                    sensor_data_buffer_t *data_buffer, 
                                    feature_vector_t *feature_vector) {
    if (!feature_extraction_initialized || sensor_data == NULL || 
        data_buffer == NULL || feature_vector == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Reset feature vector
    memset(feature_vector, 0, sizeof(feature_vector_t));
    
    // Set timestamp
    feature_vector->timestamp = esp_timer_get_time() / 1000;
    
    // Extract features from flex sensor data
    if (sensor_data->flex_data_valid) {
        // Direct features: finger joint angles
        for (int i = 0; i < FINGER_JOINT_COUNT; i++) {
            // Each joint angle is a feature
            feature_vector->features[i] = sensor_data->flex_data.angles[i];
        }
        
        // Derived features: angle differences between adjacent fingers
        for (int i = 0; i < 4; i++) {  // 4 pairs of adjacent fingers
            // MCP joints
            feature_vector->features[10 + i] = fabsf(sensor_data->flex_data.angles[i*2] - 
                                                  sensor_data->flex_data.angles[(i+1)*2]);
            // PIP joints
            feature_vector->features[14 + i] = fabsf(sensor_data->flex_data.angles[i*2+1] - 
                                                  sensor_data->flex_data.angles[(i+1)*2+1]);
        }
        
        // Feature count update
        feature_vector->feature_count = 18;  // 10 joint angles + 8 difference features
    }
    
    // Extract features from IMU data
    if (sensor_data->imu_data_valid) {
        // Hand orientation features
        feature_vector->features[18] = sensor_data->imu_data.orientation[0];  // Roll
        feature_vector->features[19] = sensor_data->imu_data.orientation[1];  // Pitch
        feature_vector->features[20] = sensor_data->imu_data.orientation[2];  // Yaw
        
        // Hand acceleration features
        feature_vector->features[21] = sensor_data->imu_data.accel[0];  // X acceleration
        feature_vector->features[22] = sensor_data->imu_data.accel[1];  // Y acceleration
        feature_vector->features[23] = sensor_data->imu_data.accel[2];  // Z acceleration
        
        // Angular velocity features
        feature_vector->features[24] = sensor_data->imu_data.gyro[0];  // X angular velocity
        feature_vector->features[25] = sensor_data->imu_data.gyro[1];  // Y angular velocity
        feature_vector->features[26] = sensor_data->imu_data.gyro[2];  // Z angular velocity
        
        // Feature count update
        feature_vector->feature_count = 27;
    }
    
    // Extract features from touch sensor data
    if (sensor_data->touch_data_valid) {
        // Touch status as features (binary)
        for (int i = 0; i < TOUCH_SENSOR_COUNT; i++) {
            feature_vector->features[27 + i] = sensor_data->touch_data.touch_status[i] ? 1.0f : 0.0f;
        }
        
        // Feature count update
        feature_vector->feature_count = 32;
    }
    
    // Temporal features (if we have enough historical data)
    if (buffer_get_size(data_buffer) >= 5) {
        // Extract motion trajectories from buffer
        // This is a simplified version; a real implementation would do more complex analysis
        
        // Get past samples for temporal analysis
        sensor_data_t past_data[5];
        for (int i = 0; i < 5; i++) {
            sensor_data_t temp_data;
            if (i == 0) {
                // Current data is first
                memcpy(&past_data[0], sensor_data, sizeof(sensor_data_t));
            } else {
                // Get older samples from buffer (if available)
                if (buffer_get_size(data_buffer) > i) {
                    // This is a simplification; in practice, you'd need to access
                    // specific items in the buffer based on their index
                    // For now, we're just reusing the current data
                    memcpy(&past_data[i], sensor_data, sizeof(sensor_data_t));
                } else {
                    // Not enough historical data
                    break;
                }
            }
        }
        
        // Calculate motion velocity features (simplified)
        // For a real implementation, you'd compute these from the actual historical data
        if (sensor_data->imu_data_valid) {
            // Example: Compute average acceleration over last few samples
            float avg_accel_x = 0.0f;
            float avg_accel_y = 0.0f;
            float avg_accel_z = 0.0f;
            
            for (int i = 0; i < 5; i++) {
                if (past_data[i].imu_data_valid) {
                    avg_accel_x += past_data[i].imu_data.accel[0];
                    avg_accel_y += past_data[i].imu_data.accel[1];
                    avg_accel_z += past_data[i].imu_data.accel[2];
                }
            }
            
            avg_accel_x /= 5.0f;
            avg_accel_y /= 5.0f;
            avg_accel_z /= 5.0f;
            
            // Store as features
            feature_vector->features[32] = avg_accel_x;
            feature_vector->features[33] = avg_accel_y;
            feature_vector->features[34] = avg_accel_z;
            
            // Feature count update
            feature_vector->feature_count = 35;
        }
    }
    
    // In a complete implementation, you'd also extract features from camera data
    // and perform more sophisticated temporal analysis
    
    ESP_LOGV(TAG, "Extracted %d features", feature_vector->feature_count);
    
    return ESP_OK;
}
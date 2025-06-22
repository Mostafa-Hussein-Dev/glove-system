#include "data_preprocessor.h"
#include "esp_log.h"
#include "drivers/flex_sensor.h"
#include <math.h>
#include <string.h>

static const char* TAG = "DATA_PREPROCESSOR";

// Normalization parameters (these would be learned from training data)
static const float feature_means[64] = {
    // Flex sensor means (degrees)
    45.0f, 45.0f, 45.0f, 45.0f, 45.0f, 45.0f, 45.0f, 45.0f, 45.0f, 45.0f,
    // IMU means
    0.0f, 0.0f, 0.0f,      // acceleration
    0.0f, 0.0f, 0.0f,      // gyroscope
    0.0f, 0.0f, 0.0f,      // orientation
    // Derived features
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 0.0f
};

static const float feature_stds[64] = {
    // Flex sensor stds (degrees)
    30.0f, 30.0f, 30.0f, 30.0f, 30.0f, 30.0f, 30.0f, 30.0f, 30.0f, 30.0f,
    // IMU stds
    2.0f, 2.0f, 2.0f,      // acceleration
    50.0f, 50.0f, 50.0f,   // gyroscope
    180.0f, 180.0f, 180.0f, // orientation
    // Derived features
    15.0f, 15.0f, 15.0f, 15.0f, 15.0f, 15.0f, 15.0f, 15.0f, 15.0f, 15.0f,
    5.0f, 5.0f, 5.0f, 5.0f, 5.0f, 5.0f, 5.0f, 5.0f, 5.0f, 5.0f,
    1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f,
    1.0f, 1.0f, 1.0f, 1.0f
};

esp_err_t data_preprocessor_init(void) {
    ESP_LOGI(TAG, "Data preprocessor initialized");
    return ESP_OK;
}

esp_err_t data_preprocessor_static(const sensor_data_t* sensor_data, ml_input_t* ml_input) {
    if (!sensor_data || !ml_input) {
        return ESP_ERR_INVALID_ARG;
    }
    
    memset(ml_input, 0, sizeof(ml_input_t));
    
    uint32_t feature_idx = 0;
    
    // Add flex sensor features (joint angles)
    if (sensor_data->flex_data_valid) {
        for (int i = 0; i < FINGER_COUNT && feature_idx < 64; i++) {
            ml_input->features[feature_idx++] = sensor_data->flex_data.angles[i];
        }
    }
    
    // Add IMU features
    if (sensor_data->imu_data_valid) {
        // Acceleration
        for (int i = 0; i < 3 && feature_idx < 64; i++) {
            ml_input->features[feature_idx++] = sensor_data->imu_data.accel[i];
        }
        // Gyroscope
        for (int i = 0; i < 3 && feature_idx < 64; i++) {
            ml_input->features[feature_idx++] = sensor_data->imu_data.gyro[i];
        }
        // Orientation
        for (int i = 0; i < 3 && feature_idx < 64; i++) {
            ml_input->features[feature_idx++] = sensor_data->imu_data.orientation[i];
        }
    }
    
    // Add derived features (finger differences, velocity estimates, etc.)
    if (sensor_data->flex_data_valid && feature_idx < 56) {
        // Adjacent finger angle differences
        for (int i = 0; i < FINGER_COUNT - 1; i++) {
            ml_input->features[feature_idx++] = fabsf(
                sensor_data->flex_data.angles[i*2] - sensor_data->flex_data.angles[(i+1)*2]
            );
        }
        
        // MCP-PIP joint differences for each finger
        for (int i = 0; i < FINGER_COUNT; i++) {
            ml_input->features[feature_idx++] = fabsf(
                sensor_data->flex_data.angles[i*2] - sensor_data->flex_data.angles[i*2+1]
            );
        }
    }
    
    ml_input->feature_count = feature_idx;
    ml_input->timestamp = sensor_data->timestamp;
    ml_input->is_sequence_start = true;
    ml_input->is_sequence_end = true;
    
    // Normalize features
    data_preprocessor_normalize(ml_input->features, ml_input->feature_count);
    
    return ESP_OK;
}

esp_err_t data_preprocessor_dynamic(const sensor_data_buffer_t* sensor_buffer, ml_input_t* ml_input) {
    if (!sensor_buffer || !ml_input) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // For dynamic gestures, we'll use temporal features
    // This is a simplified version - real implementation would be more sophisticated
    
    size_t buffer_size = buffer_get_size(sensor_buffer);
    if (buffer_size < 2) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Get recent samples for velocity and acceleration calculation
    sensor_data_t current, previous;
    esp_err_t ret1 = buffer_get(sensor_buffer, buffer_size - 1, &current);
    esp_err_t ret2 = buffer_get(sensor_buffer, buffer_size - 2, &previous);
    
    if (ret1 != ESP_OK || ret2 != ESP_OK) {
        return ESP_FAIL;
    }
    
    memset(ml_input, 0, sizeof(ml_input_t));
    
    uint32_t feature_idx = 0;
    float dt = (current.timestamp - previous.timestamp) / 1000.0f; // Convert to seconds
    
    if (dt <= 0 || dt > 0.1f) dt = 0.02f; // Default 50Hz sampling
    
    // Current pose features (similar to static)
    if (current.flex_data_valid) {
        for (int i = 0; i < FINGER_COUNT && feature_idx < 32; i++) {
            ml_input->features[feature_idx++] = current.flex_data.angles[i];
        }
    }
    
    // Velocity features (change in joint angles)
    if (current.flex_data_valid && previous.flex_data_valid) {
        for (int i = 0; i < FINGER_COUNT * 2 && feature_idx < 48; i++) {
            float velocity = (current.flex_data.angles[i] - previous.flex_data.angles[i]) / dt;
            ml_input->features[feature_idx++] = velocity;
        }
    }
    
    // IMU motion features
    if (current.imu_data_valid) {
        // Current IMU state
        for (int i = 0; i < 3 && feature_idx < 64; i++) {
            ml_input->features[feature_idx++] = current.imu_data.accel[i];
            ml_input->features[feature_idx++] = current.imu_data.gyro[i];
        }
    }
    
    ml_input->feature_count = feature_idx;
    ml_input->timestamp = current.timestamp;
    ml_input->is_sequence_start = false;  // Part of ongoing sequence
    ml_input->is_sequence_end = false;
    
    // Normalize features
    data_preprocessor_normalize(ml_input->features, ml_input->feature_count);
    
    return ESP_OK;
}

esp_err_t data_preprocessor_normalize(float* features, uint32_t count) {
    if (!features || count == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Z-score normalization: (x - mean) / std
    for (uint32_t i = 0; i < count && i < 64; i++) {
        features[i] = (features[i] - feature_means[i]) / feature_stds[i];
        
        // Clamp to reasonable range
        if (features[i] > 3.0f) features[i] = 3.0f;
        if (features[i] < -3.0f) features[i] = -3.0f;
    }
    
    return ESP_OK;
}
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
        for (int i = 0; i < FINGER_COUNT; i++) {  // Changed from FINGER_JOINT_COUNT to FINGER_COUNT
            // Small adjustment based on gravity's effect on sensor when hand tilted
            float orientation_factor = 1.0f - (fabs(roll) + fabs(pitch)) * 0.001f;
            orientation_factor = fmaxf(0.95f, fminf(1.05f, orientation_factor));
            
            // Apply orientation correction
            new_data->flex_data.angles[i] *= orientation_factor;
            
            // Ensure angles stay within valid range
            new_data->flex_data.angles[i] = fmaxf(0.0f, fminf(90.0f, new_data->flex_data.angles[i]));
        }
        
        ESP_LOGD(TAG, "Applied orientation correction: roll=%.2f, pitch=%.2f", roll, pitch);
    }
    
    // Apply temporal smoothing using historical data
    if (data_buffer->count > 1) {
        sensor_data_t prev_data;
        if (buffer_get(data_buffer, data_buffer->count - 2, &prev_data) == ESP_OK) {
            // Simple temporal smoothing for flex sensors
            if (new_data->flex_data_valid && prev_data.flex_data_valid) {
                for (int i = 0; i < FINGER_COUNT; i++) {  // Changed from FINGER_JOINT_COUNT to FINGER_COUNT
                    float smoothed_angle = 0.8f * new_data->flex_data.angles[i] + 
                                         0.2f * prev_data.flex_data.angles[i];
                    new_data->flex_data.angles[i] = smoothed_angle;
                }
            }
        }
    }
    
    // Store current data as last fused data
    memcpy(&last_fused_data, new_data, sizeof(sensor_data_t));
    
    ESP_LOGD(TAG, "Sensor fusion processed - Flex angles: [%.1f, %.1f, %.1f, %.1f, %.1f]",
             new_data->flex_data.angles[0], new_data->flex_data.angles[1], 
             new_data->flex_data.angles[2], new_data->flex_data.angles[3], 
             new_data->flex_data.angles[4]);
    
    return ESP_OK;
}

esp_err_t sensor_fusion_get_last_data(sensor_data_t *data) {
    if (!sensor_fusion_initialized || data == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    
    memcpy(data, &last_fused_data, sizeof(sensor_data_t));
    return ESP_OK;
}
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
        for (int i = 0; i < FINGER_JOINT_COUNT; i++) {
            // Small adjustment based on gravity's effect on sensor when hand tilted
            float orientation_factor = 1.0f - (fabs(roll) + fabs(pitch)) / 180.0f * 0.1f;
            
            // Apply the correction factor (limited effect for demonstration)
            new_data->flex_data.angles[i] *= orientation_factor;
        }
    }
    
    // If we have camera data, we could use it to validate/correct the other sensors
    // This is just a placeholder example
    if (new_data->camera_data_valid) {
        // In a real implementation, computer vision would extract hand pose
        // and could be used to correct other sensor readings
        
        // This is where you'd add computer vision processing
        // For now, we just log that we have camera data
        ESP_LOGV(TAG, "Camera data available for fusion (frame size: %dx%d)", 
                new_data->camera_data.width, new_data->camera_data.height);
    }
    
    // Store the current data as the last fused data for next iteration
    memcpy(&last_fused_data, new_data, sizeof(sensor_data_t));
    
    return ESP_OK;
}

esp_err_t sensor_fusion_get_latest(sensor_data_t *data) {
    if (!sensor_fusion_initialized || data == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Copy the last fused data
    memcpy(data, &last_fused_data, sizeof(sensor_data_t));
    
    return ESP_OK;
}
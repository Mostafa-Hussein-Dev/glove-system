#include "drivers/flex_sensor.h"
#include <string.h>
#include "esp_log.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h" 
#include "config/pin_definitions.h"
#include "util/debug.h"

static const char *TAG = "FLEX_SENSOR";

// NVS namespace for flex sensor calibration
#define FLEX_SENSOR_NVS_NAMESPACE "flex_sensor"
#define FLEX_SENSOR_NVS_KEY "calibration"

// Filter buffer size for moving average
#define FILTER_BUFFER_SIZE 5

// Flex sensor calibration data
static flex_sensor_calibration_t sensor_calibration = {
    .flat_value = {2000, 2000, 2000, 2000, 2000, 2000, 2000, 2000, 2000, 2000},  // Default values when flat (0 degrees)
    .bent_value = {3500, 3500, 3500, 3500, 3500, 3500, 3500, 3500, 3500, 3500},  // Default values when bent (90 degrees)
    .scale_factor = {1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f},
    .offset = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}
};

// ADC channel mapping to finger joints
static const adc1_channel_t adc_channels[FINGER_JOINT_COUNT] = {
    FLEX_SENSOR_THUMB_MCP_ADC_CHANNEL,
    FLEX_SENSOR_THUMB_PIP_ADC_CHANNEL,
    FLEX_SENSOR_INDEX_MCP_ADC_CHANNEL,
    FLEX_SENSOR_INDEX_PIP_ADC_CHANNEL,
    FLEX_SENSOR_MIDDLE_MCP_ADC_CHANNEL,
    FLEX_SENSOR_MIDDLE_PIP_ADC_CHANNEL,
    FLEX_SENSOR_RING_MCP_ADC_CHANNEL,
    FLEX_SENSOR_RING_PIP_ADC_CHANNEL,
    FLEX_SENSOR_PINKY_MCP_ADC_CHANNEL,
    FLEX_SENSOR_PINKY_PIP_ADC_CHANNEL
};

// ADC calibration
static esp_adc_cal_characteristics_t adc_chars;

// Filter buffers for each sensor
static uint16_t filter_buffers[FINGER_JOINT_COUNT][FILTER_BUFFER_SIZE];
static uint8_t filter_index[FINGER_JOINT_COUNT] = {0};
static bool filtering_enabled = true;

// Function to calculate calibration scaling factors
static void calculate_calibration_factors(void) {
    for (int i = 0; i < FINGER_JOINT_COUNT; i++) {
        // Ensure the difference between flat and bent values is significant
        if (abs((int)sensor_calibration.bent_value[i] - (int)sensor_calibration.flat_value[i]) < 100) {
            ESP_LOGW(TAG, "Calibration values for joint %d are too close", i);
            // Set default values
            sensor_calibration.flat_value[i] = 2000;
            sensor_calibration.bent_value[i] = 3500;
        }
        
        // Calculate scale factor and offset for angle calculation
        // Angle = scale_factor * raw_value + offset
        sensor_calibration.scale_factor[i] = 90.0f / (sensor_calibration.bent_value[i] - sensor_calibration.flat_value[i]);
        sensor_calibration.offset[i] = -sensor_calibration.scale_factor[i] * sensor_calibration.flat_value[i];
        
        ESP_LOGI(TAG, "Joint %d calibration: flat=%u, bent=%u, scale=%.4f, offset=%.4f", 
            i, sensor_calibration.flat_value[i], sensor_calibration.bent_value[i], 
            sensor_calibration.scale_factor[i], sensor_calibration.offset[i]);
    }
}

// Function to apply moving average filter
static uint16_t apply_filter(finger_joint_t joint, uint16_t raw_value) {
    if (!filtering_enabled) {
        return raw_value;
    }
    
    // Add current value to filter buffer
    filter_buffers[joint][filter_index[joint]] = raw_value;
    filter_index[joint] = (filter_index[joint] + 1) % FILTER_BUFFER_SIZE;
    
    // Calculate mean
    uint32_t sum = 0;
    for (int i = 0; i < FILTER_BUFFER_SIZE; i++) {
        sum += filter_buffers[joint][i];
    }
    
    return (uint16_t)(sum / FILTER_BUFFER_SIZE);
}

esp_err_t flex_sensor_init(void) {
    esp_err_t ret;
    
    // Configure ADC
    adc1_config_width(ADC_WIDTH_BIT_12);
    
    // Configure channels
    for (int i = 0; i < FINGER_JOINT_COUNT; i++) {
        adc1_config_channel_atten(adc_channels[i], FLEX_SENSOR_ADC_ATTENUATION);
    }
    
    // Characterize ADC
    esp_adc_cal_characterize(FLEX_SENSOR_ADC_UNIT, FLEX_SENSOR_ADC_ATTENUATION, 
                            FLEX_SENSOR_ADC_BIT_WIDTH, 0, &adc_chars);
    
    // Initialize filter buffers
    for (int i = 0; i < FINGER_JOINT_COUNT; i++) {
        for (int j = 0; j < FILTER_BUFFER_SIZE; j++) {
            filter_buffers[i][j] = 0;
        }
        filter_index[i] = 0;
    }
    
    // Load calibration data
    ret = flex_sensor_load_calibration();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to load calibration data, using defaults");
        calculate_calibration_factors();
    }
    
    // Read initial values to fill filter buffers
    uint16_t raw_values[FINGER_JOINT_COUNT];
    for (int i = 0; i < 10; i++) {
        flex_sensor_read_raw(raw_values);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    ESP_LOGI(TAG, "Flex sensors initialized");
    return ESP_OK;
}

esp_err_t flex_sensor_read_raw(uint16_t* raw_values) {
    if (raw_values == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Read raw values and apply filtering
    for (int i = 0; i < FINGER_JOINT_COUNT; i++) {
        uint16_t raw = adc1_get_raw(adc_channels[i]);
        raw_values[i] = apply_filter(i, raw);
    }
    
    return ESP_OK;
}

esp_err_t flex_sensor_read_angles(float* angles) {
    if (angles == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    uint16_t raw_values[FINGER_JOINT_COUNT];
    esp_err_t ret = flex_sensor_read_raw(raw_values);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Calculate angles using calibration data
    for (int i = 0; i < FINGER_JOINT_COUNT; i++) {
        angles[i] = sensor_calibration.scale_factor[i] * raw_values[i] + sensor_calibration.offset[i];
        
        // Constrain angles to 0-90 degrees
        if (angles[i] < 0.0f) {
            angles[i] = 0.0f;
        } else if (angles[i] > 90.0f) {
            angles[i] = 90.0f;
        }
    }
    
    return ESP_OK;
}

esp_err_t flex_sensor_read_joint(finger_joint_t joint, uint16_t* raw_value, float* angle) {
    if (joint >= FINGER_JOINT_COUNT || raw_value == NULL || angle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Read raw value
    *raw_value = apply_filter(joint, adc1_get_raw(adc_channels[joint]));
    
    // Calculate angle
    *angle = sensor_calibration.scale_factor[joint] * (*raw_value) + sensor_calibration.offset[joint];
    
    // Constrain angle to 0-90 degrees
    if (*angle < 0.0f) {
        *angle = 0.0f;
    } else if (*angle > 90.0f) {
        *angle = 90.0f;
    }
    
    return ESP_OK;
}

esp_err_t flex_sensor_calibrate_flat(void) {
    ESP_LOGI(TAG, "Calibrating flat position...");
    
    // Read current values as flat position
    return flex_sensor_read_raw(sensor_calibration.flat_value);
}

esp_err_t flex_sensor_calibrate_bent(void) {
    ESP_LOGI(TAG, "Calibrating bent position...");
    
    // Read current values as bent position
    esp_err_t ret = flex_sensor_read_raw(sensor_calibration.bent_value);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Calculate calibration factors
    calculate_calibration_factors();
    
    return ESP_OK;
}

esp_err_t flex_sensor_save_calibration(void) {
    ESP_LOGI(TAG, "Saving flex sensor calibration...");
    
    nvs_handle_t nvs_handle;
    esp_err_t ret;
    
    ret = nvs_open(FLEX_SENSOR_NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error opening NVS handle: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Write calibration data to NVS
    ret = nvs_set_blob(nvs_handle, FLEX_SENSOR_NVS_KEY, &sensor_calibration, sizeof(flex_sensor_calibration_t));
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

esp_err_t flex_sensor_load_calibration(void) {
    ESP_LOGI(TAG, "Loading flex sensor calibration...");
    
    nvs_handle_t nvs_handle;
    esp_err_t ret;
    
    ret = nvs_open(FLEX_SENSOR_NVS_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Error opening NVS handle: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Read calibration data from NVS
    size_t required_size = sizeof(flex_sensor_calibration_t);
    ret = nvs_get_blob(nvs_handle, FLEX_SENSOR_NVS_KEY, &sensor_calibration, &required_size);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Error reading from NVS: %s", esp_err_to_name(ret));
        nvs_close(nvs_handle);
        return ret;
    }
    
    nvs_close(nvs_handle);
    
    // Calculate calibration factors
    calculate_calibration_factors();
    
    return ESP_OK;
}

esp_err_t flex_sensor_reset_calibration(void) {
    ESP_LOGI(TAG, "Resetting flex sensor calibration to defaults...");
    
    // Set default calibration values
    for (int i = 0; i < FINGER_JOINT_COUNT; i++) {
        sensor_calibration.flat_value[i] = 2000;
        sensor_calibration.bent_value[i] = 3500;
    }
    
    // Calculate calibration factors
    calculate_calibration_factors();
    
    // Save to NVS
    return flex_sensor_save_calibration();
}

esp_err_t flex_sensor_get_calibration(flex_sensor_calibration_t* calibration) {
    if (calibration == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Copy calibration data
    memcpy(calibration, &sensor_calibration, sizeof(flex_sensor_calibration_t));
    
    return ESP_OK;
}

esp_err_t flex_sensor_set_filtering(bool enable) {
    filtering_enabled = enable;
    
    // If filtering is being enabled, reset filter buffers
    if (enable) {
        uint16_t raw_values[FINGER_JOINT_COUNT];
        flex_sensor_read_raw(raw_values);
        
        for (int i = 0; i < FINGER_JOINT_COUNT; i++) {
            for (int j = 0; j < FILTER_BUFFER_SIZE; j++) {
                filter_buffers[i][j] = raw_values[i];
            }
            filter_index[i] = 0;
        }
    }
    
    ESP_LOGI(TAG, "Flex sensor filtering %s", enable ? "enabled" : "disabled");
    return ESP_OK;
}
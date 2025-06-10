#include "drivers/flex_sensor.h"
#include <string.h>
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
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

// Flex sensor calibration data (5 sensors instead of 10)
static flex_sensor_calibration_t sensor_calibration = {
    .flat_value = {2000, 2000, 2000, 2000, 2000},  // Default values when flat (0 degrees)
    .bent_value = {3500, 3500, 3500, 3500, 3500},  // Default values when bent (90 degrees)
    .scale_factor = {1.0f, 1.0f, 1.0f, 1.0f, 1.0f},
    .offset = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f}
};

// ADC channel mapping to fingers (5 channels instead of 10)
static const adc_channel_t adc_channels[FINGER_COUNT] = {
    FLEX_SENSOR_THUMB_ADC_CHANNEL,
    FLEX_SENSOR_INDEX_ADC_CHANNEL,
    FLEX_SENSOR_MIDDLE_ADC_CHANNEL,
    FLEX_SENSOR_RING_ADC_CHANNEL,
    FLEX_SENSOR_PINKY_ADC_CHANNEL
};

// ADC calibration
static adc_cali_handle_t adc_cali_handle = NULL;
static adc_oneshot_unit_handle_t adc_handle = NULL;

// Filter buffers for each sensor (5 sensors instead of 10)
static uint16_t filter_buffers[FINGER_COUNT][FILTER_BUFFER_SIZE];
static uint8_t filter_index[FINGER_COUNT] = {0};
static bool filtering_enabled = true;

// Function to calculate calibration scaling factors
static void calculate_calibration_factors(void) {
    for (int i = 0; i < FINGER_COUNT; i++) {
        // Ensure the difference between flat and bent values is significant
        if (abs((int)sensor_calibration.bent_value[i] - (int)sensor_calibration.flat_value[i]) > 200) {
            // Calculate linear scaling factors
            sensor_calibration.scale_factor[i] = 90.0f / 
                (sensor_calibration.bent_value[i] - sensor_calibration.flat_value[i]);
            sensor_calibration.offset[i] = -sensor_calibration.scale_factor[i] * 
                sensor_calibration.flat_value[i];
        } else {
            // Use default values if calibration range is too small
            sensor_calibration.scale_factor[i] = 0.06f;  // 90 degrees / 1500 ADC units
            sensor_calibration.offset[i] = -120.0f;
            ESP_LOGW(TAG, "Finger %d calibration range too small, using defaults", i);
        }
    }
}

// Apply digital filter to sensor readings
static uint16_t apply_filter(int sensor_idx, uint16_t raw_value) {
    if (!filtering_enabled) {
        return raw_value;
    }
    
    // Add new value to filter buffer
    filter_buffers[sensor_idx][filter_index[sensor_idx]] = raw_value;
    filter_index[sensor_idx] = (filter_index[sensor_idx] + 1) % FILTER_BUFFER_SIZE;
    
    // Calculate moving average
    uint32_t sum = 0;
    for (int i = 0; i < FILTER_BUFFER_SIZE; i++) {
        sum += filter_buffers[sensor_idx][i];
    }
    
    return (uint16_t)(sum / FILTER_BUFFER_SIZE);
}

esp_err_t flex_sensor_init(void) {
    ESP_LOGI(TAG, "Initializing flex sensors (5 sensors)...");
    
    esp_err_t ret;
    
    // Configure ADC oneshot
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = FLEX_SENSOR_ADC_UNIT,
    };
    ret = adc_oneshot_new_unit(&init_config, &adc_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize ADC unit: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Configure ADC channels for all 5 flex sensors
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = FLEX_SENSOR_ADC_BIT_WIDTH,
        .atten = FLEX_SENSOR_ADC_ATTENUATION,
    };
    
    for (int i = 0; i < FINGER_COUNT; i++) {
        ret = adc_oneshot_config_channel(adc_handle, adc_channels[i], &config);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to configure ADC channel %d: %s", 
                     adc_channels[i], esp_err_to_name(ret));
            return ret;
        }
    }
    
    // Initialize ADC calibration
    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = FLEX_SENSOR_ADC_UNIT,
        .atten = FLEX_SENSOR_ADC_ATTENUATION,
        .bitwidth = FLEX_SENSOR_ADC_BIT_WIDTH,
    };
    adc_cali_create_scheme_curve_fitting(&cali_config, &adc_cali_handle);

    // Initialize filter buffers
    for (int i = 0; i < FINGER_COUNT; i++) {
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
    uint16_t raw_values[FINGER_COUNT];
    for (int i = 0; i < 10; i++) {
        flex_sensor_read_raw(raw_values);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    ESP_LOGI(TAG, "Flex sensors initialized (5 sensors)");
    return ESP_OK;
}

esp_err_t flex_sensor_read_raw(uint16_t* raw_values) {
    if (raw_values == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Read raw values and apply filtering
    for (int i = 0; i < FINGER_COUNT; i++) {
        int raw_value = 0;
        adc_oneshot_read(adc_handle, adc_channels[i], &raw_value);
        uint16_t raw = (uint16_t)raw_value;
        raw_values[i] = apply_filter(i, raw);
    }
    
    return ESP_OK;
}

esp_err_t flex_sensor_read_angles(float* angles) {
    if (angles == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    uint16_t raw_values[FINGER_COUNT];
    esp_err_t ret = flex_sensor_read_raw(raw_values);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Calculate angles using calibration data
    for (int i = 0; i < FINGER_COUNT; i++) {
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

esp_err_t flex_sensor_read_finger(finger_t finger, uint16_t* raw_value, float* angle) {
    if (finger >= FINGER_COUNT || raw_value == NULL || angle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Read raw value for the specific finger
    int raw;
    adc_oneshot_read(adc_handle, adc_channels[finger], &raw);
    *raw_value = apply_filter(finger, (uint16_t)raw);
    
    // Calculate angle
    *angle = sensor_calibration.scale_factor[finger] * (*raw_value) + sensor_calibration.offset[finger];
    
    // Constrain angle to 0-90 degrees
    if (*angle < 0.0f) {
        *angle = 0.0f;
    } else if (*angle > 90.0f) {
        *angle = 90.0f;
    }
    
    return ESP_OK;
}

esp_err_t flex_sensor_calibrate_flat(void) {
    ESP_LOGI(TAG, "Calibrating flat position for 5 flex sensors...");
    
    uint16_t raw_values[FINGER_COUNT];
    
    // Take multiple readings and average them
    uint32_t sum[FINGER_COUNT] = {0};
    for (int i = 0; i < 20; i++) {
        flex_sensor_read_raw(raw_values);
        for (int j = 0; j < FINGER_COUNT; j++) {
            sum[j] += raw_values[j];
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    
    // Store averaged flat values
    for (int i = 0; i < FINGER_COUNT; i++) {
        sensor_calibration.flat_value[i] = sum[i] / 20;
        ESP_LOGI(TAG, "Finger %d flat value: %d", i, sensor_calibration.flat_value[i]);
    }
    
    // Recalculate calibration factors
    calculate_calibration_factors();
    
    return ESP_OK;
}

esp_err_t flex_sensor_calibrate_bent(void) {
    ESP_LOGI(TAG, "Calibrating bent position for 5 flex sensors...");
    
    uint16_t raw_values[FINGER_COUNT];
    
    // Take multiple readings and average them
    uint32_t sum[FINGER_COUNT] = {0};
    for (int i = 0; i < 20; i++) {
        flex_sensor_read_raw(raw_values);
        for (int j = 0; j < FINGER_COUNT; j++) {
            sum[j] += raw_values[j];
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    
    // Store averaged bent values
    for (int i = 0; i < FINGER_COUNT; i++) {
        sensor_calibration.bent_value[i] = sum[i] / 20;
        ESP_LOGI(TAG, "Finger %d bent value: %d", i, sensor_calibration.bent_value[i]);
    }
    
    // Recalculate calibration factors
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
    
    // Save calibration data to NVS
    ret = nvs_set_blob(nvs_handle, FLEX_SENSOR_NVS_KEY, &sensor_calibration, sizeof(flex_sensor_calibration_t));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error writing to NVS: %s", esp_err_to_name(ret));
        nvs_close(nvs_handle);
        return ret;
    }
    
    ret = nvs_commit(nvs_handle);
    nvs_close(nvs_handle);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error committing NVS: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "Flex sensor calibration saved");
    }
    
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
    
    // Set default calibration values for 5 sensors
    for (int i = 0; i < FINGER_COUNT; i++) {
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
        uint16_t raw_values[FINGER_COUNT];
        flex_sensor_read_raw(raw_values);
        
        for (int i = 0; i < FINGER_COUNT; i++) {
            for (int j = 0; j < FILTER_BUFFER_SIZE; j++) {
                filter_buffers[i][j] = raw_values[i];
            }
            filter_index[i] = 0;
        }
    }
    
    ESP_LOGI(TAG, "Flex sensor filtering %s", enable ? "enabled" : "disabled");
    return ESP_OK;
}
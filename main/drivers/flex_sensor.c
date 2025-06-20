// File: main/drivers/flex_sensor.h - IMPROVED VERSION
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

#endif /* DRIVERS_FLEX_SENSOR_H */

// File: main/drivers/flex_sensor.c - IMPROVED VERSION
#include "drivers/flex_sensor.h"
#include <string.h>
#include <math.h>
#include <stdlib.h>
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

// Configuration constants
#define FLEX_SENSOR_NVS_NAMESPACE "flex_sensor"
#define FLEX_SENSOR_NVS_KEY "calibration"
#define FILTER_BUFFER_SIZE 10
#define SAMPLES_PER_READ 8
#define CALIBRATION_SAMPLES 50
#define MIN_CALIBRATION_RANGE 300
#define MAX_NOISE_THRESHOLD 4095  // Increased from 100 to allow larger changes
#define SENSOR_TIMEOUT_MS 100
#define STABILITY_THRESHOLD 20

// Global variables
static flex_sensor_calibration_t sensor_calibration;
static flex_sensor_health_t sensor_health;
static adc_cali_handle_t adc_cali_handle = NULL;
static adc_oneshot_unit_handle_t adc_handle = NULL;
static bool driver_initialized = false;
static bool filtering_enabled = true;

// Filter buffers
static uint16_t filter_buffers[FINGER_COUNT][FILTER_BUFFER_SIZE];
static uint8_t filter_index[FINGER_COUNT] = {0};
static bool filter_filled[FINGER_COUNT] = {false};

// ADC channel mapping
static const adc_channel_t adc_channels[FINGER_COUNT] = {
    FLEX_SENSOR_THUMB_ADC_CHANNEL,
    FLEX_SENSOR_INDEX_ADC_CHANNEL,
    FLEX_SENSOR_MIDDLE_ADC_CHANNEL,
    FLEX_SENSOR_RING_ADC_CHANNEL,
    FLEX_SENSOR_PINKY_ADC_CHANNEL
};

// Static function declarations
static esp_err_t init_adc_unit(void);
static esp_err_t init_calibration_defaults(void);
static uint16_t apply_filter(int sensor_idx, uint16_t raw_value);
static esp_err_t read_adc_with_validation(adc_channel_t channel, uint16_t* value);
static esp_err_t detect_sensor_direction(finger_t finger);
static void calculate_calibration_factors(void);
static bool validate_adc_value(finger_t finger, uint16_t value);
static esp_err_t check_sensor_connectivity(finger_t finger);

esp_err_t flex_sensor_init(void) {
    if (driver_initialized) {
        ESP_LOGW(TAG, "Flex sensor driver already initialized");
        return ESP_OK;
    }
    
    ESP_LOGI(TAG, "Initializing enhanced flex sensors (5 sensors)...");
    
    esp_err_t ret;
    
    // Initialize ADC unit
    ret = init_adc_unit();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize ADC unit: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Initialize calibration with defaults
    ret = init_calibration_defaults();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize calibration: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Initialize filter buffers with actual readings
    ESP_LOGI(TAG, "Initializing filter buffers...");
    for (int i = 0; i < FINGER_COUNT; i++) {
        uint16_t initial_value;
        if (read_adc_with_validation(adc_channels[i], &initial_value) == ESP_OK) {
            for (int j = 0; j < FILTER_BUFFER_SIZE; j++) {
                filter_buffers[i][j] = initial_value;
            }
        } else {
            // Use default value if read fails
            for (int j = 0; j < FILTER_BUFFER_SIZE; j++) {
                filter_buffers[i][j] = 2000;
            }
        }
        filter_index[i] = 0;
        filter_filled[i] = true;
    }
    
    // Load saved calibration or use defaults
    ret = flex_sensor_load_calibration();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to load calibration data, using defaults");
        calculate_calibration_factors();
    }
    
    // Test sensor connectivity
    ret = flex_sensor_test_connectivity();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Sensor connectivity test failed");
    }
    
    // Enable filtering by default
    flex_sensor_set_filtering(true);
    
    // Stabilization period
    vTaskDelay(pdMS_TO_TICKS(200));
    
    driver_initialized = true;
    ESP_LOGI(TAG, "Flex sensors initialized successfully");
    return ESP_OK;
}

static esp_err_t init_adc_unit(void) {
    esp_err_t ret;
    
    // Configure ADC oneshot unit
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = FLEX_SENSOR_ADC_UNIT,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ret = adc_oneshot_new_unit(&init_config, &adc_handle);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Configure ADC channels with improved settings
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = FLEX_SENSOR_ADC_BIT_WIDTH,
        .atten = ADC_ATTEN_DB_6,  // Better SNR than DB_12
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
        .atten = ADC_ATTEN_DB_6,
        .bitwidth = FLEX_SENSOR_ADC_BIT_WIDTH,
    };
    ret = adc_cali_create_scheme_curve_fitting(&cali_config, &adc_cali_handle);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "ADC calibration failed, continuing without calibration");
        adc_cali_handle = NULL;
    }
    
    return ESP_OK;
}

static esp_err_t init_calibration_defaults(void) {
    // Initialize with reasonable defaults
    for (int i = 0; i < FINGER_COUNT; i++) {
        sensor_calibration.flat_value[i] = 1100;
        sensor_calibration.bent_value[i] = 2450;
        sensor_calibration.direction[i] = SENSOR_DIRECTION_INCREASING;
        sensor_calibration.scale_factor[i] = 90.0f / 4095.0f;
        sensor_calibration.offset[i] = 0.0f;
        sensor_calibration.min_value[i] = 800;
        sensor_calibration.max_value[i] = 4095;
        sensor_calibration.calibrated[i] = true;
        
        // Initialize health status
        sensor_health.sensor_connected[i] = false;
        sensor_health.sensor_stable[i] = false;
        sensor_health.noise_level[i] = 0;
        sensor_health.read_errors[i] = 0;
    }
    
    return ESP_OK;
}

static esp_err_t read_adc_with_validation(adc_channel_t channel, uint16_t* value) {
    if (value == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    uint32_t sum = 0;
    uint8_t valid_samples = 0;
    
    // Take multiple samples and validate
    for (int i = 0; i < SAMPLES_PER_READ; i++) {
        int raw_value = 0;
        esp_err_t ret = adc_oneshot_read(adc_handle, channel, &raw_value);
        
        if (ret == ESP_OK && raw_value >= 0 && raw_value <= 4095) {
            sum += raw_value;
            valid_samples++;
        }
        
        vTaskDelay(pdMS_TO_TICKS(1)); // Small delay between samples
    }
    
    if (valid_samples < (SAMPLES_PER_READ / 2)) {
        return ESP_ERR_INVALID_RESPONSE;
    }
    
    *value = (uint16_t)(sum / valid_samples);
    return ESP_OK;
}

static uint16_t apply_filter(int sensor_idx, uint16_t raw_value) {
    if (!filtering_enabled || sensor_idx >= FINGER_COUNT) {
        return raw_value;
    }
    
    // Adaptive outlier rejection
    if (filter_filled[sensor_idx]) {
        uint32_t sum = 0;
        uint16_t min_val = 4095, max_val = 750;
        
        // Calculate average and range of recent samples
        for (int i = 0; i < FILTER_BUFFER_SIZE; i++) {
            uint16_t sample = filter_buffers[sensor_idx][i];
            sum += sample;
            if (sample < min_val) min_val = sample;
            if (sample > max_val) max_val = sample;
        }
        uint16_t avg = sum / FILTER_BUFFER_SIZE;
        uint16_t range = max_val - min_val;
        
        // Adaptive threshold based on recent range and calibration
        uint16_t adaptive_threshold = MAX_NOISE_THRESHOLD;
        if (sensor_calibration.calibrated[sensor_idx]) {
            uint16_t sensor_range = abs((int)sensor_calibration.bent_value[sensor_idx] - 
                                      (int)sensor_calibration.flat_value[sensor_idx]);
            adaptive_threshold = fmax(MAX_NOISE_THRESHOLD, sensor_range * 0.3f); // 30% of sensor range
        }
        
        // If recent samples show movement, increase threshold
        if (range > 200) {
            adaptive_threshold *= 2; // Double threshold during movement
        }
        
        // Check if this is really an outlier
        uint16_t deviation = abs((int)raw_value - (int)avg);
        if (deviation > adaptive_threshold) {
            // Additional check: is this a consistent trend?
            uint16_t recent_avg = 0;
            int recent_samples = fmin(3, FILTER_BUFFER_SIZE);
            for (int i = 0; i < recent_samples; i++) {
                int idx = (filter_index[sensor_idx] - 1 - i + FILTER_BUFFER_SIZE) % FILTER_BUFFER_SIZE;
                recent_avg += filter_buffers[sensor_idx][idx];
            }
            recent_avg /= recent_samples;
            
            // If the new value is closer to recent trend, allow it
            uint16_t trend_deviation = abs((int)raw_value - (int)recent_avg);
            if (trend_deviation < deviation) {
                // This follows the recent trend, probably legitimate
                goto accept_value;
            }
            
            ESP_LOGW(TAG, "Outlier rejected on sensor %d: %d (avg: %d, thresh: %d)", 
                     sensor_idx, raw_value, avg, adaptive_threshold);
            return avg; // Return filtered average instead
        }
    }
    
accept_value:
    // Add new value to filter buffer
    filter_buffers[sensor_idx][filter_index[sensor_idx]] = raw_value;
    filter_index[sensor_idx] = (filter_index[sensor_idx] + 1) % FILTER_BUFFER_SIZE;
    
    // Calculate weighted moving average
    uint32_t weighted_sum = 0;
    uint32_t weight_total = 0;
    
    for (int i = 0; i < FILTER_BUFFER_SIZE; i++) {
        int sample_index = (filter_index[sensor_idx] - 1 - i + FILTER_BUFFER_SIZE) % FILTER_BUFFER_SIZE;
        uint32_t weight = FILTER_BUFFER_SIZE - i; // More weight to recent samples
        weighted_sum += filter_buffers[sensor_idx][sample_index] * weight;
        weight_total += weight;
    }
    
    return (uint16_t)(weighted_sum / weight_total);
}

esp_err_t flex_sensor_read_raw(uint16_t* raw_values) {
   if (!driver_initialized || raw_values == NULL) {
       return ESP_ERR_INVALID_STATE;
   }
   
   esp_err_t overall_result = ESP_OK;

   /*
   for (int i = 0; i < FINGER_COUNT; i++) {
        uint16_t raw_value;
        esp_err_t ret = read_adc_with_validation(adc_channels[i], &raw_value);
        
        if (ret == ESP_OK) {
            if (validate_adc_value(i, raw_value)) {
                raw_values[i] = apply_filter(i, raw_value);
                sensor_health.sensor_connected[i] = true;
            } else {
                sensor_health.read_errors[i]++;
                overall_result = ESP_ERR_INVALID_RESPONSE;
            }
        } else {
            sensor_health.read_errors[i]++;
            sensor_health.sensor_connected[i] = false;
            raw_values[i] = filter_filled[i] ?
                apply_filter(i, filter_buffers[i][filter_index[i]]) : 
                sensor_calibration.flat_value[i];
            overall_result = ESP_FAIL;
        }
    }
    */
    
    for (int i = 0; i < FINGER_COUNT; i++) {
       int temp_value;
       esp_err_t ret = adc_oneshot_read(adc_handle, adc_channels[i], &temp_value);
       
       if (ret == ESP_OK) {
           raw_values[i] = (uint16_t)temp_value;
           sensor_health.sensor_connected[i] = true;
       } else {
           raw_values[i] = 0;
           sensor_health.read_errors[i]++;
           sensor_health.sensor_connected[i] = false;
           overall_result = ESP_FAIL;
       }
   }
   
   return overall_result;
}

esp_err_t flex_sensor_read_angles(float* angles) {
    if (!driver_initialized || angles == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    
    uint16_t raw_values[FINGER_COUNT];
    esp_err_t ret = flex_sensor_read_raw(raw_values);
    
    // Convert to angles even if some sensors failed
    for (int i = 0; i < FINGER_COUNT; i++) {
        if (sensor_calibration.calibrated[i]) {
            float raw_angle = (float)raw_values[i] * 90.0f / 4095.0f;
            
            // Constrain to valid range
            angles[i] = fmaxf(0.0f, fminf(90.0f, raw_angle));
        } else {
            angles[i] = 0.0f; // Default to flat if not calibrated
        }
    }
    
    return ret;
}

static bool validate_adc_value(finger_t finger, uint16_t value) {
    if (finger >= FINGER_COUNT) {
        return false;
    }
    
    return (value >= sensor_calibration.min_value[finger] && 
            value <= sensor_calibration.max_value[finger]);
}

esp_err_t flex_sensor_calibrate_auto(void) {
    ESP_LOGI(TAG, "Starting automatic calibration...");
    
    for (int finger = 0; finger < FINGER_COUNT; finger++) {
        esp_err_t ret = detect_sensor_direction(finger);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to detect direction for finger %d", finger);
        }
    }
    
    calculate_calibration_factors();
    return flex_sensor_save_calibration();
}

static esp_err_t detect_sensor_direction(finger_t finger) {
    ESP_LOGI(TAG, "Detecting sensor direction for finger %d", finger);
    ESP_LOGI(TAG, "Please move finger from flat to bent position slowly...");
    
    uint16_t readings[20];
    bool direction_determined = false;
    
    // Take readings over time to determine direction
    for (int i = 0; i < 20; i++) {
        read_adc_with_validation(adc_channels[finger], &readings[i]);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    
    // Analyze trend
    uint16_t min_val = readings[0], max_val = readings[0];
    for (int i = 1; i < 20; i++) {
        if (readings[i] < min_val) min_val = readings[i];
        if (readings[i] > max_val) max_val = readings[i];
    }
    
    // Set calibration values and direction
    if (max_val - min_val > MIN_CALIBRATION_RANGE) {
        sensor_calibration.flat_value[finger] = min_val;
        sensor_calibration.bent_value[finger] = max_val;
        sensor_calibration.direction[finger] = SENSOR_DIRECTION_INCREASING;
        sensor_calibration.calibrated[finger] = true;
        direction_determined = true;
    }
    
    if (!direction_determined) {
        ESP_LOGW(TAG, "Could not determine direction for finger %d", finger);
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "Finger %d: flat=%d, bent=%d, direction=%s", 
             finger, 
             sensor_calibration.flat_value[finger],
             sensor_calibration.bent_value[finger],
             sensor_calibration.direction[finger] == SENSOR_DIRECTION_INCREASING ? "INC" : "DEC");
    
    return ESP_OK;
}

static void calculate_calibration_factors(void) {
    for (int i = 0; i < FINGER_COUNT; i++) {
        if (sensor_calibration.calibrated[i]) {
            uint16_t range = abs((int)sensor_calibration.bent_value[i] - 
                               (int)sensor_calibration.flat_value[i]);
            
            if (range > MIN_CALIBRATION_RANGE) {
                sensor_calibration.scale_factor[i] = 90.0f / range;
                sensor_calibration.offset[i] = -sensor_calibration.scale_factor[i] * 
                                              sensor_calibration.flat_value[i];
                
                // Set valid range with some margin
                sensor_calibration.min_value[i] = fmin(sensor_calibration.flat_value[i], 
                                                      sensor_calibration.bent_value[i]) - 200;
                sensor_calibration.max_value[i] = fmax(sensor_calibration.flat_value[i], 
                                                      sensor_calibration.bent_value[i]) + 200;
            } else {
                ESP_LOGW(TAG, "Finger %d calibration range too small (%d), using defaults", 
                         i, range);
                sensor_calibration.calibrated[i] = false;
            }
        }
    }
}

esp_err_t flex_sensor_test_connectivity(void) {
    ESP_LOGI(TAG, "Testing sensor connectivity...");
    
    for (int i = 0; i < FINGER_COUNT; i++) {
        uint16_t value;
        esp_err_t ret = read_adc_with_validation(adc_channels[i], &value);
        
        if (ret == ESP_OK) {
            sensor_health.sensor_connected[i] = true;
            ESP_LOGI(TAG, "Sensor %d: Connected, value=%d", i, value);
        } else {
            sensor_health.sensor_connected[i] = false;
            ESP_LOGW(TAG, "Sensor %d: NOT CONNECTED", i);
        }
    }
    
    return ESP_OK;
}

esp_err_t flex_sensor_get_health_status(flex_sensor_health_t* health) {
    if (!driver_initialized || health == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    
    memcpy(health, &sensor_health, sizeof(flex_sensor_health_t));
    return ESP_OK;
}

esp_err_t flex_sensor_read_raw_single(finger_t finger, uint16_t* raw_value) {
    if (!driver_initialized || finger >= FINGER_COUNT || raw_value == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    esp_err_t ret = read_adc_with_validation(adc_channels[finger], raw_value);
    if (ret == ESP_OK) {
        *raw_value = apply_filter(finger, *raw_value);
        sensor_health.sensor_connected[finger] = true;
    } else {
        sensor_health.read_errors[finger]++;
        sensor_health.sensor_connected[finger] = false;
    }
    
    return ret;
}

esp_err_t flex_sensor_read_finger(finger_t finger, uint16_t* raw_value, float* angle) {
    if (!driver_initialized || finger >= FINGER_COUNT || 
        raw_value == NULL || angle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    esp_err_t ret = flex_sensor_read_raw_single(finger, raw_value);
    
    if (ret == ESP_OK && sensor_calibration.calibrated[finger]) {
        float raw_angle = sensor_calibration.scale_factor[finger] * (*raw_value) + 
                         sensor_calibration.offset[finger];
        
        // Apply direction correction
        if (sensor_calibration.direction[finger] == SENSOR_DIRECTION_DECREASING) {
            raw_angle = 90.0f - raw_angle;
        }
        
        // Constrain to valid range
        *angle = fmaxf(0.0f, fminf(90.0f, raw_angle));
    } else {
        *angle = 0.0f; // Default to flat if read failed or not calibrated
    }
    
    return ret;
}

esp_err_t flex_sensor_calibrate_flat(void) {
    if (!driver_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    ESP_LOGI(TAG, "Calibrating flat position for all sensors...");
    ESP_LOGI(TAG, "Keep all fingers straight and flat...");
    
    // Wait for user to position fingers
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    uint32_t sum[FINGER_COUNT] = {0};
    uint8_t valid_samples[FINGER_COUNT] = {0};
    
    // Take multiple readings
    for (int sample = 0; sample < CALIBRATION_SAMPLES; sample++) {
        for (int finger = 0; finger < FINGER_COUNT; finger++) {
            uint16_t raw_value;
            if (read_adc_with_validation(adc_channels[finger], &raw_value) == ESP_OK) {
                sum[finger] += raw_value;
                valid_samples[finger]++;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    
    // Calculate averages and validate
    for (int finger = 0; finger < FINGER_COUNT; finger++) {
        if (valid_samples[finger] >= (CALIBRATION_SAMPLES * 0.8f)) {
            uint16_t avg_value = sum[finger] / valid_samples[finger];
            sensor_calibration.flat_value[finger] = avg_value;
            ESP_LOGI(TAG, "Finger %d flat value: %d (%d samples)", 
                     finger, avg_value, valid_samples[finger]);
        } else {
            ESP_LOGW(TAG, "Insufficient valid samples for finger %d: %d/%d", 
                     finger, valid_samples[finger], CALIBRATION_SAMPLES);
            return ESP_ERR_INVALID_RESPONSE;
        }
    }
    
    // Recalculate calibration factors if bent values exist
    bool has_bent_values = true;
    for (int finger = 0; finger < FINGER_COUNT; finger++) {
        if (sensor_calibration.bent_value[finger] == 0) {
            has_bent_values = false;
            break;
        }
    }
    
    if (has_bent_values) {
        calculate_calibration_factors();
    }
    
    ESP_LOGI(TAG, "Flat calibration completed");
    return ESP_OK;
}

esp_err_t flex_sensor_calibrate_bent(void) {
    if (!driver_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    ESP_LOGI(TAG, "Calibrating bent position for all sensors...");
    ESP_LOGI(TAG, "Bend all fingers to 90 degrees...");
    
    // Wait for user to position fingers
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    uint32_t sum[FINGER_COUNT] = {0};
    uint8_t valid_samples[FINGER_COUNT] = {0};
    
    // Take multiple readings
    for (int sample = 0; sample < CALIBRATION_SAMPLES; sample++) {
        for (int finger = 0; finger < FINGER_COUNT; finger++) {
            uint16_t raw_value;
            if (read_adc_with_validation(adc_channels[finger], &raw_value) == ESP_OK) {
                sum[finger] += raw_value;
                valid_samples[finger]++;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    
    // Calculate averages and validate
    for (int finger = 0; finger < FINGER_COUNT; finger++) {
        if (valid_samples[finger] >= (CALIBRATION_SAMPLES * 0.8f)) {
            uint16_t avg_value = sum[finger] / valid_samples[finger];
            sensor_calibration.bent_value[finger] = avg_value;
            
            // Determine sensor direction
            if (avg_value > sensor_calibration.flat_value[finger]) {
                sensor_calibration.direction[finger] = SENSOR_DIRECTION_INCREASING;
            } else {
                sensor_calibration.direction[finger] = SENSOR_DIRECTION_DECREASING;
            }
            
            ESP_LOGI(TAG, "Finger %d bent value: %d, direction: %s (%d samples)", 
                     finger, avg_value, 
                     sensor_calibration.direction[finger] == SENSOR_DIRECTION_INCREASING ? "INC" : "DEC",
                     valid_samples[finger]);
        } else {
            ESP_LOGW(TAG, "Insufficient valid samples for finger %d: %d/%d", 
                     finger, valid_samples[finger], CALIBRATION_SAMPLES);
            return ESP_ERR_INVALID_RESPONSE;
        }
    }
    
    // Calculate calibration factors and mark as calibrated
    calculate_calibration_factors();
    
    // Mark sensors as calibrated if they have valid range
    for (int finger = 0; finger < FINGER_COUNT; finger++) {
        uint16_t range = abs((int)sensor_calibration.bent_value[finger] - 
                           (int)sensor_calibration.flat_value[finger]);
        sensor_calibration.calibrated[finger] = (range >= MIN_CALIBRATION_RANGE);
        
        if (!sensor_calibration.calibrated[finger]) {
            ESP_LOGW(TAG, "Finger %d calibration range too small: %d", finger, range);
        }
    }
    
    ESP_LOGI(TAG, "Bent calibration completed");
    return ESP_OK;
}

esp_err_t flex_sensor_validate_calibration(void) {
    if (!driver_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    ESP_LOGI(TAG, "Validating sensor calibration...");
    
    bool all_valid = true;
    
    for (int finger = 0; finger < FINGER_COUNT; finger++) {
        bool finger_valid = true;
        
        // Check if calibrated
        if (!sensor_calibration.calibrated[finger]) {
            ESP_LOGW(TAG, "Finger %d: Not calibrated", finger);
            finger_valid = false;
        }
        
        // Check calibration range
        uint16_t range = abs((int)sensor_calibration.bent_value[finger] - 
                           (int)sensor_calibration.flat_value[finger]);
        if (range < MIN_CALIBRATION_RANGE) {
            ESP_LOGW(TAG, "Finger %d: Calibration range too small (%d)", finger, range);
            finger_valid = false;
        }
        
        // Check sensor connectivity
        if (!sensor_health.sensor_connected[finger]) {
            ESP_LOGW(TAG, "Finger %d: Sensor not connected", finger);
            finger_valid = false;
        }
        
        // Check for excessive errors
        if (sensor_health.read_errors[finger] > 100) {
            ESP_LOGW(TAG, "Finger %d: Too many read errors (%lu)", 
                     finger, sensor_health.read_errors[finger]);
            finger_valid = false;
        }
        
        if (finger_valid) {
            ESP_LOGI(TAG, "Finger %d: Valid (range=%d, flat=%d, bent=%d)", 
                     finger, range, 
                     sensor_calibration.flat_value[finger],
                     sensor_calibration.bent_value[finger]);
        } else {
            all_valid = false;
        }
    }
    
    return all_valid ? ESP_OK : ESP_ERR_INVALID_STATE;
}

esp_err_t flex_sensor_save_calibration(void) {
    if (!driver_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    ESP_LOGI(TAG, "Saving flex sensor calibration...");
    
    nvs_handle_t nvs_handle;
    esp_err_t ret;
    
    ret = nvs_open(FLEX_SENSOR_NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error opening NVS handle: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Save calibration data with version info
    ret = nvs_set_blob(nvs_handle, FLEX_SENSOR_NVS_KEY, &sensor_calibration, 
                       sizeof(flex_sensor_calibration_t));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error writing calibration to NVS: %s", esp_err_to_name(ret));
        nvs_close(nvs_handle);
        return ret;
    }
    
    // Save health data as well
    ret = nvs_set_blob(nvs_handle, "health", &sensor_health, 
                       sizeof(flex_sensor_health_t));
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Warning: Could not save health data: %s", esp_err_to_name(ret));
    }
    
    ret = nvs_commit(nvs_handle);
    nvs_close(nvs_handle);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error committing NVS: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "Flex sensor calibration saved successfully");
    
    // Log saved values for verification
    for (int i = 0; i < FINGER_COUNT; i++) {
        if (sensor_calibration.calibrated[i]) {
            ESP_LOGI(TAG, "Saved finger %d: flat=%d, bent=%d, dir=%s", 
                     i, sensor_calibration.flat_value[i], 
                     sensor_calibration.bent_value[i],
                     sensor_calibration.direction[i] == SENSOR_DIRECTION_INCREASING ? "INC" : "DEC");
        }
    }
    
    return ESP_OK;
}

esp_err_t flex_sensor_load_calibration(void) {
    if (!driver_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    ESP_LOGI(TAG, "Loading flex sensor calibration...");
    
    nvs_handle_t nvs_handle;
    esp_err_t ret;
    
    ret = nvs_open(FLEX_SENSOR_NVS_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Error opening NVS handle for read: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Load calibration data
    size_t required_size = sizeof(flex_sensor_calibration_t);
    ret = nvs_get_blob(nvs_handle, FLEX_SENSOR_NVS_KEY, &sensor_calibration, &required_size);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Error reading calibration from NVS: %s", esp_err_to_name(ret));
        nvs_close(nvs_handle);
        return ret;
    }
    
    // Validate loaded data
    bool data_valid = true;
    for (int i = 0; i < FINGER_COUNT; i++) {
        if (sensor_calibration.flat_value[i] > 4095 || 
            sensor_calibration.bent_value[i] > 4095 ||
            sensor_calibration.direction[i] > SENSOR_DIRECTION_DECREASING) {
            ESP_LOGW(TAG, "Invalid calibration data for finger %d", i);
            data_valid = false;
            break;
        }
    }
    
    if (!data_valid) {
        ESP_LOGW(TAG, "Loaded calibration data is corrupted, using defaults");
        nvs_close(nvs_handle);
        return ESP_ERR_INVALID_CRC;
    }
    
    // Try to load health data (optional)
    required_size = sizeof(flex_sensor_health_t);
    ret = nvs_get_blob(nvs_handle, "health", &sensor_health, &required_size);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Could not load health data, initializing defaults");
        // Reset error counters
        for (int i = 0; i < FINGER_COUNT; i++) {
            sensor_health.read_errors[i] = 0;
        }
    }
    
    nvs_close(nvs_handle);
    
    // Recalculate calibration factors
    calculate_calibration_factors();
    
    ESP_LOGI(TAG, "Flex sensor calibration loaded successfully");
    
    // Log loaded values for verification
    for (int i = 0; i < FINGER_COUNT; i++) {
        if (sensor_calibration.calibrated[i]) {
            ESP_LOGI(TAG, "Loaded finger %d: flat=%d, bent=%d, dir=%s", 
                     i, sensor_calibration.flat_value[i], 
                     sensor_calibration.bent_value[i],
                     sensor_calibration.direction[i] == SENSOR_DIRECTION_INCREASING ? "INC" : "DEC");
        }
    }
    
    return ESP_OK;
}

esp_err_t flex_sensor_reset_calibration(void) {
    if (!driver_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    ESP_LOGI(TAG, "Resetting flex sensor calibration to defaults...");
    
    // Reset to default values
    esp_err_t ret = init_calibration_defaults();
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Clear NVS data
    nvs_handle_t nvs_handle;
    ret = nvs_open(FLEX_SENSOR_NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (ret == ESP_OK) {
        nvs_erase_key(nvs_handle, FLEX_SENSOR_NVS_KEY);
        nvs_erase_key(nvs_handle, "health");
        nvs_commit(nvs_handle);
        nvs_close(nvs_handle);
        ESP_LOGI(TAG, "NVS calibration data cleared");
    } else {
        ESP_LOGW(TAG, "Could not clear NVS data: %s", esp_err_to_name(ret));
    }
    
    // Reset error counters
    for (int i = 0; i < FINGER_COUNT; i++) {
        sensor_health.read_errors[i] = 0;
        sensor_health.sensor_connected[i] = false;
        sensor_health.sensor_stable[i] = false;
        sensor_health.noise_level[i] = 0;
    }
    
    ESP_LOGI(TAG, "Flex sensor calibration reset completed");
    return ESP_OK;
}

esp_err_t flex_sensor_get_calibration(flex_sensor_calibration_t* calibration) {
    if (!driver_initialized || calibration == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    memcpy(calibration, &sensor_calibration, sizeof(flex_sensor_calibration_t));
    return ESP_OK;
}

esp_err_t flex_sensor_set_filtering(bool enable) {
    if (!driver_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    filtering_enabled = enable;
    
    if (enable) {
        // Re-initialize filter buffers with current readings
        ESP_LOGI(TAG, "Enabling filtering and reinitializing buffers...");
        
        for (int i = 0; i < FINGER_COUNT; i++) {
            uint16_t current_value;
            if (read_adc_with_validation(adc_channels[i], &current_value) == ESP_OK) {
                for (int j = 0; j < FILTER_BUFFER_SIZE; j++) {
                    filter_buffers[i][j] = current_value;
                }
            }
            filter_index[i] = 0;
            filter_filled[i] = true;
        }
        
        // Wait for filters to stabilize
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    
    ESP_LOGI(TAG, "Flex sensor filtering %s", enable ? "enabled" : "disabled");
    return ESP_OK;
}

esp_err_t flex_sensor_calibrate_noise_floor(void) {
    if (!driver_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    ESP_LOGI(TAG, "Calibrating noise floor - keep sensors still for 5 seconds...");
    
    uint16_t readings[FINGER_COUNT][100];
    uint8_t valid_samples[FINGER_COUNT] = {0};
    
    // Disable filtering temporarily for raw noise measurement
    bool prev_filtering = filtering_enabled;
    filtering_enabled = false;
    
    // Collect samples
    for (int sample = 0; sample < 100; sample++) {
        for (int finger = 0; finger < FINGER_COUNT; finger++) {
            uint16_t value;
            if (read_adc_with_validation(adc_channels[finger], &value) == ESP_OK) {
                readings[finger][valid_samples[finger]] = value;
                valid_samples[finger]++;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    
    // Restore filtering
    filtering_enabled = prev_filtering;
    
    // Calculate noise statistics
    for (int finger = 0; finger < FINGER_COUNT; finger++) {
        if (valid_samples[finger] > 10) {
            uint32_t sum = 0;
            uint32_t sum_sq = 0;
            
            for (int i = 0; i < valid_samples[finger]; i++) {
                sum += readings[finger][i];
                sum_sq += readings[finger][i] * readings[finger][i];
            }
            
            float mean = (float)sum / valid_samples[finger];
            float variance = ((float)sum_sq / valid_samples[finger]) - (mean * mean);
            float std_dev = sqrtf(variance);
            
            sensor_health.noise_level[finger] = (uint16_t)std_dev;
            sensor_health.sensor_stable[finger] = (std_dev < STABILITY_THRESHOLD);
            
            ESP_LOGI(TAG, "Finger %d: Mean=%.1f, StdDev=%.1f, Samples=%d, Stable=%s", 
                     finger, mean, std_dev, valid_samples[finger],
                     sensor_health.sensor_stable[finger] ? "YES" : "NO");
        } else {
            ESP_LOGW(TAG, "Finger %d: Insufficient samples for noise analysis", finger);
            sensor_health.noise_level[finger] = 9999;
            sensor_health.sensor_stable[finger] = false;
        }
    }
    
    ESP_LOGI(TAG, "Noise floor calibration completed");
    return ESP_OK;
}

esp_err_t flex_sensor_deinit(void) {
    if (!driver_initialized) {
        return ESP_OK;
    }
    
    // Save current calibration before deinitializing
    flex_sensor_save_calibration();
    
    if (adc_cali_handle) {
        adc_cali_delete_scheme_curve_fitting(adc_cali_handle);
        adc_cali_handle = NULL;
    }
    
    if (adc_handle) {
        adc_oneshot_del_unit(adc_handle);
        adc_handle = NULL;
    }
    
    driver_initialized = false;
    ESP_LOGI(TAG, "Flex sensor driver deinitialized");
    return ESP_OK;
}
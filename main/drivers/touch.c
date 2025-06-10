#include "drivers/touch.h"
#include <string.h>
#include "esp_log.h"
#include "driver/touch_pad.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "config/pin_definitions.h"
#include "util/debug.h"

static const char *TAG = "TOUCH";

// Touch sensor pins array
static const touch_pad_t touch_pins[TOUCH_SENSOR_COUNT] = {
    TOUCH_THUMB_PIN,
    TOUCH_INDEX_PIN,
    TOUCH_MIDDLE_PIN,
    TOUCH_RING_PIN,
    TOUCH_PINKY_PIN
};

// Touch sensor state
static bool touch_initialized = false;
static bool touch_enabled = true;
static uint32_t touch_thresholds[TOUCH_SENSOR_COUNT] = {0};
static uint32_t touch_baseline[TOUCH_SENSOR_COUNT] = {0};
static bool touch_status[TOUCH_SENSOR_COUNT] = {false};

// Callback function pointer for touch events
static touch_callback_t touch_callback = NULL;

esp_err_t touch_init(void) {
    esp_err_t ret;
    
    if (touch_initialized) {
        return ESP_OK;  // Already initialized
    }
    
    ESP_LOGI(TAG, "Initializing touch sensors...");
    
    // Initialize touch pad peripheral
    ret = touch_pad_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize touch pad: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Set touch pad FSM mode
    ret = touch_pad_set_fsm_mode(TOUCH_FSM_MODE_TIMER);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set touch pad FSM mode: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Configure touch pads
    for (int i = 0; i < TOUCH_SENSOR_COUNT; i++) {
        // FIXED: Configure touch pad correctly
        ret = touch_pad_config(touch_pins[i]);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to configure touch pad %d: %s", i, esp_err_to_name(ret));
            return ret;
        }
        
        // Set touch pad measurement (filter period)
        touch_pad_set_cnt_mode(touch_pins[i], TOUCH_PAD_SLOPE_7, TOUCH_PAD_TIE_OPT_LOW);
        
        ESP_LOGI(TAG, "Configured touch pad %d (pin %d)", i, touch_pins[i]);
    }

    // FIXED: Start touch sensing system
    ret = touch_pad_fsm_start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start touch pad: %s", esp_err_to_name(ret));
        return ret;
    }
    
    touch_initialized = true;
    
    // Wait a moment for touch system to stabilize
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Initialize and start calibration
    ret = touch_calibrate();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to calibrate touch sensors: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "Touch sensor system initialized successfully");
    return ESP_OK;
}

esp_err_t touch_deinit(void) {
    if (!touch_initialized) {
        return ESP_OK;  // Already deinitialized
    }
    
    // Stop touch sensing
    touch_pad_fsm_stop();
    
    // Deinitialize touch pad peripheral
    touch_pad_deinit();
    
    touch_initialized = false;
    ESP_LOGI(TAG, "Touch sensor system deinitialized");
    
    return ESP_OK;
}

esp_err_t touch_calibrate(void) {
    esp_err_t ret;

    if (!touch_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    ESP_LOGI(TAG, "Calibrating touch sensors... Please don't touch the sensors!");
    
    // Wait for sensors to stabilize
    vTaskDelay(pdMS_TO_TICKS(500));
    
    // Measure baseline values for each sensor
    for (int i = 0; i < TOUCH_SENSOR_COUNT; i++) {
        uint32_t val;
        
        // Read multiple samples to get a stable baseline
        uint32_t sum = 0;
        const int samples = 20;  // More samples for better accuracy
        
        for (int j = 0; j < samples; j++) {
            // FIXED: Use correct API for reading touch values
            ret = touch_pad_read_raw_data(touch_pins[i], &val);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to read touch pad %d during calibration: %s", 
                         i, esp_err_to_name(ret));
                return ret;
            }
            sum += val;
            vTaskDelay(pdMS_TO_TICKS(10));  // Short delay between readings
        }
        
        // Calculate average baseline
        touch_baseline[i] = sum / samples;
        
        // Set threshold at 80% of baseline value (touch = lower value)
        touch_thresholds[i] = (touch_baseline[i] * 120) / 100;
        
        // Set the threshold for the touch pad
        ret = touch_pad_set_thresh(touch_pins[i], touch_thresholds[i]);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to set threshold for touch pad %d: %s", 
                     i, esp_err_to_name(ret));
            return ret;
        }
        
        ESP_LOGI(TAG, "Touch sensor %d: baseline=%lu, threshold=%lu", 
                 i, touch_baseline[i], touch_thresholds[i]);
    }
    
    ESP_LOGI(TAG, "Touch calibration complete - you can now test the sensors!");
    return ESP_OK;
}

esp_err_t touch_set_threshold(uint8_t sensor_id, uint16_t threshold) {
    if (!touch_initialized || sensor_id >= TOUCH_SENSOR_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }
    
    touch_thresholds[sensor_id] = threshold;
    
    // Set the threshold for interrupt
    esp_err_t ret = touch_pad_set_thresh(touch_pins[sensor_id], threshold);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set touch threshold for sensor %d: %s", 
                 sensor_id, esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "Touch sensor %d threshold set to %u", sensor_id, threshold);
    return ESP_OK;
}

esp_err_t touch_set_callback(touch_callback_t callback) {
    if (!touch_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    touch_callback = callback;
    return ESP_OK;
}

esp_err_t touch_enable(bool enable) {
    if (!touch_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (touch_enabled == enable) {
        return ESP_OK;  // Already in the requested state
    }
    
    esp_err_t ret;
    if (enable) {
        // Start touch sensing
        ret = touch_pad_fsm_start();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to start touch pad: %s", esp_err_to_name(ret));
            return ret;
        }
    } else {
        // Stop touch sensing
        ret = touch_pad_fsm_stop();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to stop touch pad: %s", esp_err_to_name(ret));
            return ret;
        }
    }
    
    touch_enabled = enable;
    ESP_LOGI(TAG, "Touch sensors %s", enable ? "enabled" : "disabled");
    
    return ESP_OK;
}

esp_err_t touch_get_status(bool *status_array) {
    if (!touch_initialized || status_array == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Read current status
    touch_update_status();
    
    // Copy status to output array
    memcpy(status_array, touch_status, sizeof(touch_status));
    
    return ESP_OK;
}

esp_err_t touch_update_status(void) {
    if (!touch_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Read current status for each sensor
    bool new_status[TOUCH_SENSOR_COUNT] = {false};
    bool status_changed = false;
    
    for (int i = 0; i < TOUCH_SENSOR_COUNT; i++) {
        uint32_t val;
        esp_err_t ret = touch_pad_read_raw_data(touch_pins[i], &val);
        
        if (ret == ESP_OK) {
            // Touch detected if value is below threshold
            new_status[i] = (val > touch_thresholds[i]);
            
            // Check if status changed
            if (new_status[i] != touch_status[i]) {
                status_changed = true;
            }
        } else {
            ESP_LOGW(TAG, "Failed to read touch pad %d: %s", i, esp_err_to_name(ret));
            new_status[i] = false;  // Default to not touched on error
        }
    }
    
    // Update status and notify callback if changed
    if (status_changed) {
        memcpy(touch_status, new_status, sizeof(touch_status));
        
        // Call callback if registered
        if (touch_callback != NULL) {
            touch_callback(touch_status);
        }
    }
    
    return ESP_OK;
}

esp_err_t touch_get_values(uint16_t *values_array) {
    if (!touch_initialized || values_array == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Read raw values for each sensor
    for (int i = 0; i < TOUCH_SENSOR_COUNT; i++) {
        uint32_t raw_value;
        esp_err_t ret = touch_pad_read_raw_data(touch_pins[i], &raw_value);
        
        if (ret == ESP_OK) {
            // FIXED: Proper conversion with bounds checking
            if (raw_value > 65535) {
                values_array[i] = 65535;  // Clamp to uint16_t max
            } else {
                values_array[i] = (uint16_t)raw_value;
            }
        } else {
            ESP_LOGW(TAG, "Failed to read touch pad %d: %s", i, esp_err_to_name(ret));
            values_array[i] = 65535;  // Max value indicates error
        }
    }
    
    return ESP_OK;
}

bool touch_is_sensor_active(uint8_t sensor_id) {
    if (!touch_initialized || sensor_id >= TOUCH_SENSOR_COUNT) {
        return false;
    }
    
    return touch_status[sensor_id];
}

void touch_intr_handler(void *arg) {
    // Update touch status
    touch_update_status();
    
    // Clear touch interrupt
    touch_pad_clear_status();
}
#include "util/debug.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include "esp_system.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/uart.h"
#include "drivers/display.h"
#include "communication/ble_service.h"
#include "ml/model_manager.h"
#include "ml/ml_test.h"
#include "util/buffer.h"
#include "drivers/flex_sensor.h"
#include "ml/data_preprocessor.h"
#include "ml/ml_inference.h"

static const char *TAG = "DEBUG";

static debug_level_t current_debug_level = DEBUG_LEVEL_INFO;
static uint8_t current_debug_mode = DEBUG_MODE_UART;

// Debug display buffer (for OLED display)
static char debug_display_buffer[128] = {0};

esp_err_t debug_init(debug_level_t level, uint8_t mode) {
    current_debug_level = level;
    current_debug_mode = mode;
    
    ESP_LOGI(TAG, "Debug subsystem initialized with level %d and mode %d", level, mode);
    return ESP_OK;
}

void debug_set_level(debug_level_t level) {
    current_debug_level = level;
}

void debug_set_mode(uint8_t mode) {
    current_debug_mode = mode;
}

void debug_log(debug_level_t level, const char* tag, const char* format, ...) {
    if (level > current_debug_level) {
        return;
    }
    
    va_list args;
    va_start(args, format);
    
    // Format message
    char buffer[256];
    vsnprintf(buffer, sizeof(buffer), format, args);
    
    // Output to UART using ESP logging
    if (current_debug_mode & DEBUG_MODE_UART) {
        switch (level) {
            case DEBUG_LEVEL_ERROR:
                ESP_LOGE(tag, "%s", buffer);
                break;
            case DEBUG_LEVEL_WARNING:
                ESP_LOGW(tag, "%s", buffer);
                break;
            case DEBUG_LEVEL_INFO:
                ESP_LOGI(tag, "%s", buffer);
                break;
            case DEBUG_LEVEL_DEBUG:
                ESP_LOGD(tag, "%s", buffer);
                break;
            case DEBUG_LEVEL_VERBOSE:
                ESP_LOGV(tag, "%s", buffer);
                break;
            default:
                break;
        }
    }
    
    // Output to display
    if (current_debug_mode & DEBUG_MODE_DISPLAY) {
        // Keep only the latest message for display
        snprintf(debug_display_buffer, sizeof(debug_display_buffer), "[%s] %s", tag, buffer);
        
        // Only display error and warning messages to avoid cluttering the display
        if (level <= DEBUG_LEVEL_WARNING) {
            display_show_debug(debug_display_buffer);
        }
    }
    
    // Output over BLE
    if (current_debug_mode & DEBUG_MODE_BLUETOOTH) {
        // Format with level and tag
        char ble_buffer[128];
        char level_char = 'I';
        switch (level) {
            case DEBUG_LEVEL_ERROR:   level_char = 'E'; break;
            case DEBUG_LEVEL_WARNING: level_char = 'W'; break;
            case DEBUG_LEVEL_INFO:    level_char = 'I'; break;
            case DEBUG_LEVEL_DEBUG:   level_char = 'D'; break;
            case DEBUG_LEVEL_VERBOSE: level_char = 'V'; break;
            default: break;
        }
        
        snprintf(ble_buffer, sizeof(ble_buffer), "[%c][%s] %s", level_char, tag, buffer);
        ble_service_send_debug(ble_buffer);
    }
    
    va_end(args);
}

void debug_buffer(debug_level_t level, const char* tag, const char* prefix, const void* buffer, size_t length) {
    if (level > current_debug_level || !buffer) {
        return;
    }
    
    // Only support UART for buffer dumps
    if (current_debug_mode & DEBUG_MODE_UART) {
        const uint8_t* data = (const uint8_t*)buffer;
        char hex_buffer[16*3 + 1]; // Each byte takes 3 chars (2 hex + 1 space)
        
        ESP_LOGI(tag, "%s (%d bytes):", prefix, length);
        
        for (size_t i = 0; i < length; i += 16) {
            size_t row_size = (i + 16 <= length) ? 16 : (length - i);
            
            // Format hex representation
            memset(hex_buffer, 0, sizeof(hex_buffer));
            for (size_t j = 0; j < row_size; j++) {
                sprintf(hex_buffer + j*3, "%02x ", data[i + j]);
            }
            
            ESP_LOGI(tag, "  %04x: %s", i, hex_buffer);
        }
    }
}

void debug_get_time_str(char* buffer, size_t size) {
    int64_t time = esp_timer_get_time();
    int seconds = (int)(time / 1000000);
    int millis = (int)((time % 1000000) / 1000);
    
    snprintf(buffer, size, "%d.%03d", seconds, millis);
}

esp_err_t debug_assert(bool condition, const char* tag, const char* message, int line, const char* file) {
    if (!condition) {
        const char* filename = strrchr(file, '/');
        if (filename) {
            filename++; // Skip the '/'
        } else {
            filename = file;
        }
        
        ESP_LOGE(tag, "Assertion failed: %s (%s:%d)", message, filename, line);
        
        // Also output to display if enabled
        if (current_debug_mode & DEBUG_MODE_DISPLAY) {
            char assert_msg[64];
            snprintf(assert_msg, sizeof(assert_msg), "ASSERT: %s (%s:%d)", message, filename, line);
            display_show_debug(assert_msg);
        }
        
        return ESP_FAIL;
    }
    
    return ESP_OK;
}

void debug_ml_status(void) {
    ESP_LOGI(TAG, "=== ML System Status ===");
    
    // Check model status
    bool static_ready = model_manager_is_ready(MODEL_TYPE_STATIC_CNN);
    bool dynamic_ready = model_manager_is_ready(MODEL_TYPE_DYNAMIC_LSTM);
    
    ESP_LOGI(TAG, "Models loaded:");
    ESP_LOGI(TAG, "  Static CNN: %s", static_ready ? "YES" : "NO");
    ESP_LOGI(TAG, "  Dynamic LSTM: %s", dynamic_ready ? "YES" : "NO");
    
    // Memory usage
    ESP_LOGI(TAG, "Memory usage:");
    ESP_LOGI(TAG, "  Free heap: %u bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "  Free SPIRAM: %zu bytes", heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
    
    // Run performance test if models are loaded
    if (static_ready) {
        ml_test_performance();
    }
}

void debug_ml_inference_test(void) {
    ESP_LOGI(TAG, "Running ML inference test...");
    
    // Create dummy sensor data
    sensor_data_t sensor_data = {0};
    sensor_data.flex_data_valid = true;
    sensor_data.imu_data_valid = true;
    sensor_data.timestamp = esp_timer_get_time() / 1000;
    
    // Simulate fist gesture
    for (int i = 0; i < FINGER_COUNT; i++) {
        sensor_data.flex_data.angles[i] = 80.0f + (rand() % 20) - 10;  // 70-90 degrees
    }
    
    // Simulate IMU data
    sensor_data.imu_data.accel[0] = 0.0f;
    sensor_data.imu_data.accel[1] = 0.0f;
    sensor_data.imu_data.accel[2] = 9.8f;
    
    // Preprocess and run inference
    ml_input_t ml_input;
    esp_err_t ret = data_preprocessor_static(&sensor_data, &ml_input);
    if (ret == ESP_OK) {
        ml_result_t result;
        ret = ml_inference_run(&ml_input, MODEL_TYPE_STATIC_CNN, &result);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Inference result: %s (%.2f confidence, %u μs)", 
                    result.gesture_name, result.confidence, result.inference_time_us);
        } else {
            ESP_LOGE(TAG, "Inference failed");
        }
    } else {
        ESP_LOGE(TAG, "Preprocessing failed");
    }
}
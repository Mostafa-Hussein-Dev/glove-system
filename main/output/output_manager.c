#include "output/output_manager.h"
#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "drivers/display.h"
#include "drivers/audio.h"
#include "drivers/haptic.h"
#include "communication/ble_service.h"
#include "config/system_config.h"
#include "util/debug.h"

static const char *TAG = "OUTPUT_MANAGER";

// Predefined haptic patterns
static const haptic_pattern_t HAPTIC_PATTERN_SIMPLE[] = {
    {intensity: 100, duration_ms: 100}
};

static const haptic_pattern_t HAPTIC_PATTERN_DOUBLE[] = {
    {intensity: 100, duration_ms: 50},
    {intensity: 0, duration_ms: 50},
    {intensity: 100, duration_ms: 50}
};

static const haptic_pattern_t HAPTIC_PATTERN_SUCCESS[] = {
    {intensity: 60, duration_ms: 50},
    {intensity: 80, duration_ms: 50},
    {intensity: 100, duration_ms: 100}
};

static const haptic_pattern_t HAPTIC_PATTERN_ERROR[] = {
    {intensity: 100, duration_ms: 100},
    {intensity: 0, duration_ms: 50},
    {intensity: 100, duration_ms: 100},
    {intensity: 0, duration_ms: 50},
    {intensity: 100, duration_ms: 100}
};

// Output manager state
static bool output_manager_initialized = false;
static SemaphoreHandle_t output_mutex = NULL;

// Font size mapping
static const display_font_t font_size_map[] = {
    DISPLAY_FONT_SMALL,
    DISPLAY_FONT_MEDIUM,
    DISPLAY_FONT_LARGE
};

// System state text representation
static const char *state_text[] = {
    "Initializing",
    "Idle",
    "Active",
    "Standby",
    "Sleep",
    "Charging",
    "Low Battery",
    "Error"
};

esp_err_t output_manager_init(void) {
    if (output_manager_initialized) {
        return ESP_OK;  // Already initialized
    }
    
    // Create mutex for thread-safe access to output devices
    output_mutex = xSemaphoreCreateMutex();
    if (output_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create output mutex");
        return ESP_ERR_NO_MEM;
    }
    
    output_manager_initialized = true;
    ESP_LOGI(TAG, "Output manager initialized");
    
    return ESP_OK;
}

esp_err_t output_manager_deinit(void) {
    if (!output_manager_initialized) {
        return ESP_OK;  // Already deinitialized
    }
    
    // Delete mutex
    if (output_mutex != NULL) {
        vSemaphoreDelete(output_mutex);
        output_mutex = NULL;
    }
    
    output_manager_initialized = false;
    ESP_LOGI(TAG, "Output manager deinitialized");
    
    return ESP_OK;
}

esp_err_t output_manager_handle_command(output_command_t *command) {
    if (!output_manager_initialized || command == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    
    esp_err_t ret = ESP_OK;
    
    // Take mutex to ensure thread-safe access to output devices
    if (xSemaphoreTake(output_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGW(TAG, "Failed to take output mutex");
        return ESP_ERR_TIMEOUT;
    }
    
    // Handle command based on type
    switch (command->type) {
        case OUTPUT_CMD_DISPLAY_TEXT:
            ret = output_manager_display_text(
                command->data.display.text,
                command->data.display.size,
                command->data.display.line,
                command->data.display.clear_first
            );
            break;
            
        case OUTPUT_CMD_SPEAK_TEXT:
            ret = output_manager_speak_text(
                command->data.speak.text,
                command->data.speak.priority
            );
            break;
            
        case OUTPUT_CMD_HAPTIC_FEEDBACK:
            ret = output_manager_haptic_feedback(
                command->data.haptic.pattern,
                command->data.haptic.intensity,
                command->data.haptic.duration_ms
            );
            break;
            
        case OUTPUT_CMD_SET_MODE:
            ret = output_manager_set_mode(command->data.set_mode.mode);
            break;
            
        case OUTPUT_CMD_CLEAR:
            ret = output_manager_clear();
            break;
            
        case OUTPUT_CMD_SHOW_BATTERY:
            ret = output_manager_show_battery(
                command->data.battery.percentage,
                command->data.battery.show_graphic
            );
            break;
            
        case OUTPUT_CMD_SHOW_ERROR:
            ret = output_manager_show_error(
                command->data.error.error_code,
                command->data.error.error_text
            );
            break;
            
        case OUTPUT_CMD_SHOW_STATUS:
            // Implementation would be here
            break;
            
        default:
            ESP_LOGW(TAG, "Unknown output command type: %d", command->type);
            ret = ESP_ERR_INVALID_ARG;
            break;
    }
    
    // Release mutex
    xSemaphoreGive(output_mutex);
    
    return ret;
}

esp_err_t output_manager_set_mode(output_mode_t mode) {
    ESP_LOGI(TAG, "Setting output mode to %d", mode);
    
    // Update system configuration feature flags based on mode
    switch (mode) {
        case OUTPUT_MODE_TEXT_ONLY:
            g_system_config.audio_feedback_enabled = false;
            g_system_config.haptic_feedback_enabled = false;
            // Display is always enabled for text
            break;
            
        case OUTPUT_MODE_AUDIO_ONLY:
            g_system_config.audio_feedback_enabled = true;
            g_system_config.haptic_feedback_enabled = false;
            // Keep display enabled for status
            break;
            
        case OUTPUT_MODE_TEXT_AND_AUDIO:
            g_system_config.audio_feedback_enabled = true;
            g_system_config.haptic_feedback_enabled = true;  // Enable haptic too for full experience
            break;
            
        case OUTPUT_MODE_MINIMAL:
            g_system_config.audio_feedback_enabled = false;
            g_system_config.haptic_feedback_enabled = false;
            // Minimal display updates only
            break;
            
        default:
            ESP_LOGW(TAG, "Unknown output mode %d, defaulting to TEXT_AND_AUDIO", mode);
            g_system_config.audio_feedback_enabled = true;
            g_system_config.haptic_feedback_enabled = true;
            mode = OUTPUT_MODE_TEXT_AND_AUDIO;
            break;
    }
    
    ESP_LOGI(TAG, "Output mode set - Audio: %s, Haptic: %s", 
        g_system_config.audio_feedback_enabled ? "ON" : "OFF",
        g_system_config.haptic_feedback_enabled ? "ON" : "OFF");
    
    // Display current mode on screen
    char mode_text[32];
    switch (mode) {
        case OUTPUT_MODE_TEXT_ONLY:
            strcpy(mode_text, "Text Only");
            break;
        case OUTPUT_MODE_AUDIO_ONLY:
            strcpy(mode_text, "Audio Only");
            break;
        case OUTPUT_MODE_TEXT_AND_AUDIO:
            strcpy(mode_text, "Text & Audio");
            break;
        case OUTPUT_MODE_MINIMAL:
            strcpy(mode_text, "Minimal");
            break;
        default:
            strcpy(mode_text, "Unknown");
            break;
    }
    
    // Display mode information
    display_clear();
    display_draw_text("Output Mode:", 0, 16, DISPLAY_FONT_SMALL, DISPLAY_ALIGN_CENTER);
    display_draw_text(mode_text, 0, 32, DISPLAY_FONT_MEDIUM, DISPLAY_ALIGN_CENTER);
    
    // Show feature status
    char status_text[64];
    snprintf(status_text, sizeof(status_text), "Audio:%s Haptic:%s", 
        g_system_config.audio_feedback_enabled ? "ON" : "OFF",
        g_system_config.haptic_feedback_enabled ? "ON" : "OFF");
    display_draw_text(status_text, 0, 48, DISPLAY_FONT_SMALL, DISPLAY_ALIGN_CENTER);
    
    display_update();
    
    // Generate feedback only if audio is enabled
    if (g_system_config.audio_feedback_enabled) {
        audio_play_beep(1000, 100);
    }
    
    // Save configuration to NVS
    esp_err_t ret = system_config_save();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to save configuration: %s", esp_err_to_name(ret));
    }
    
    return ESP_OK;
}

esp_err_t output_manager_display_text(const char *text, uint8_t size, uint8_t line, bool clear_first) {
    if (text == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Map size to display font (with bounds checking)
    display_font_t font = DISPLAY_FONT_SMALL;
    if (size < sizeof(font_size_map) / sizeof(font_size_map[0])) {
        font = font_size_map[size];
    }
    
    // Calculate y position based on line and font size
    uint8_t y = line * 10;  // Approximate, adjust based on actual font sizes
    if (font == DISPLAY_FONT_MEDIUM) {
        y = line * 16;
    } else if (font == DISPLAY_FONT_LARGE) {
        y = line * 24;
    }
    
    // Clear display if requested
    if (clear_first) {
        display_clear();
    }
    
    // Draw text and update display
    display_draw_text(text, 0, y, font, DISPLAY_ALIGN_LEFT);
    display_update();
    
    // Also send to BLE if connected
    bool connected = false;
    if (ble_service_is_connected(&connected) == ESP_OK && connected) {
        ble_service_send_text(text);
    }
    
    ESP_LOGI(TAG, "Displayed text: '%s'", text);
    
    return ESP_OK;
}

esp_err_t output_manager_speak_text(const char *text, uint8_t priority) {
    if (text == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Priority not implemented yet, just speak the text
    esp_err_t ret = audio_speak(text);
    
    ESP_LOGI(TAG, "Speaking text: '%s'", text);
    
    return ret;
}

esp_err_t output_manager_haptic_feedback(uint8_t pattern, uint8_t intensity, uint16_t duration_ms) {
    esp_err_t ret = ESP_OK;
    
    // Apply pattern based on pattern ID
    switch (pattern) {
        case 0: // Simple
            if (sizeof(HAPTIC_PATTERN_SIMPLE) > 0) {
                ret = haptic_play_pattern(HAPTIC_PATTERN_SIMPLE, sizeof(HAPTIC_PATTERN_SIMPLE) / sizeof(haptic_pattern_t));
            } else {
                ret = haptic_vibrate(duration_ms);
            }
            break;
            
        case 1: // Double
            ret = haptic_play_pattern(HAPTIC_PATTERN_DOUBLE, sizeof(HAPTIC_PATTERN_DOUBLE) / sizeof(haptic_pattern_t));
            break;
            
        case 2: // Success
            ret = haptic_play_pattern(HAPTIC_PATTERN_SUCCESS, sizeof(HAPTIC_PATTERN_SUCCESS) / sizeof(haptic_pattern_t));
            break;
            
        case 3: // Error
            ret = haptic_play_pattern(HAPTIC_PATTERN_ERROR, sizeof(HAPTIC_PATTERN_ERROR) / sizeof(haptic_pattern_t));
            break;
            
        default:
            // Use simple vibration with specified duration
            ret = haptic_vibrate(duration_ms);
            break;
    }
    
    ESP_LOGI(TAG, "Haptic feedback: pattern=%d, intensity=%d, duration=%d", 
             pattern, intensity, duration_ms);
    
    return ret;
}

esp_err_t output_manager_show_battery(uint8_t percentage, bool show_graphic) {
    // Ensure percentage is in range
    if (percentage > 100) {
        percentage = 100;
    }
    
    // Format battery text
    char battery_text[16];
    sprintf(battery_text, "Battery: %d%%", percentage);
    
    // Display battery text
    if (show_graphic) {
        // Clear top part of screen for battery display
        display_fill_rect(0, 0, DISPLAY_WIDTH, 16, 0);
        
        // Draw battery text
        display_draw_text(battery_text, 0, 2, DISPLAY_FONT_SMALL, DISPLAY_ALIGN_LEFT);
        
        // Draw battery graphic
        uint8_t battery_x = DISPLAY_WIDTH - 30;
        uint8_t battery_y = 3;
        uint8_t battery_width = 25;
        uint8_t battery_height = 10;
        
        // Draw battery outline
        display_draw_rect(battery_x, battery_y, battery_width, battery_height, 1);
        
        // Draw battery tip
        display_draw_rect(battery_x + battery_width, battery_y + 2, 2, battery_height - 4, 1);
        
        // Draw battery level
        uint8_t level_width = (percentage * (battery_width - 4)) / 100;
        if (level_width > 0) {
            display_fill_rect(battery_x + 2, battery_y + 2, level_width, battery_height - 4, 1);
        }
        
        // Update display
        display_update();
    } else {
        // Just display text
        display_draw_text(battery_text, 0, 0, DISPLAY_FONT_SMALL, DISPLAY_ALIGN_LEFT);
        display_update();
    }
    
    ESP_LOGI(TAG, "Battery status: %d%%", percentage);
    
    return ESP_OK;
}

esp_err_t output_manager_show_error(system_error_t error_code, const char *error_text) {
    // Clear display
    display_clear();
    
    // Draw error header
    display_draw_text("ERROR", 0, 0, DISPLAY_FONT_MEDIUM, DISPLAY_ALIGN_CENTER);
    
    // Draw error code
    char error_code_text[16];
    sprintf(error_code_text, "Code: %d", error_code);
    display_draw_text(error_code_text, 0, 20, DISPLAY_FONT_SMALL, DISPLAY_ALIGN_CENTER);
    
    // Draw error description
    if (error_text != NULL) {
        display_draw_text(error_text, 0, 35, DISPLAY_FONT_SMALL, DISPLAY_ALIGN_CENTER);
    } else {
        // Default error messages based on error code
        const char *default_error = "Unknown error";
        switch (error_code) {
            case SYSTEM_ERROR_FLEX_SENSOR:
                default_error = "Flex sensor error";
                break;
            case SYSTEM_ERROR_IMU:
                default_error = "IMU error";
                break;
            case SYSTEM_ERROR_CAMERA:
                default_error = "Camera error";
                break;
            case SYSTEM_ERROR_DISPLAY:
                default_error = "Display error";
                break;
            case SYSTEM_ERROR_AUDIO:
                default_error = "Audio error";
                break;
            case SYSTEM_ERROR_BLUETOOTH:
                default_error = "Bluetooth error";
                break;
            case SYSTEM_ERROR_MEMORY:
                default_error = "Memory error";
                break;
            case SYSTEM_ERROR_BATTERY:
                default_error = "Battery error";
                break;
            default:
                break;
        }
        display_draw_text(default_error, 0, 35, DISPLAY_FONT_SMALL, DISPLAY_ALIGN_CENTER);
    }
    
    // Update display
    display_update();
    
    // Generate error sound and haptic feedback
    audio_play_beep(2000, 200);
    haptic_play_pattern(HAPTIC_PATTERN_ERROR, sizeof(HAPTIC_PATTERN_ERROR) / sizeof(haptic_pattern_t));
    
    ESP_LOGE(TAG, "Error displayed: code=%d, text=%s", error_code, error_text != NULL ? error_text : "NULL");
    
    return ESP_OK;
}

esp_err_t output_manager_show_status(system_state_t state, uint8_t battery_level) {
    // Clear display
    display_clear();
    
    // Draw status header
    display_draw_text("System Status", 0, 0, DISPLAY_FONT_SMALL, DISPLAY_ALIGN_CENTER);
    
    // Draw horizontal line
    display_draw_line(0, 10, DISPLAY_WIDTH - 1, 10, 1);
    
    // Draw state
    char state_line[32];
    sprintf(state_line, "State: %s", state_text[state]);
    display_draw_text(state_line, 2, 15, DISPLAY_FONT_SMALL, DISPLAY_ALIGN_LEFT);
    
    // Draw battery level
    char battery_line[32];
    sprintf(battery_line, "Battery: %d%%", battery_level);
    display_draw_text(battery_line, 2, 25, DISPLAY_FONT_SMALL, DISPLAY_ALIGN_LEFT);
    
    // Draw battery level indicator
    display_draw_progress_bar(2, 35, DISPLAY_WIDTH - 4, 8, battery_level);
    
    // Update display
    display_update();
    
    ESP_LOGI(TAG, "Status displayed: state=%s, battery=%d%%", 
             state_text[state], battery_level);
    
    return ESP_OK;
}

esp_err_t output_manager_clear(void) {
    // Clear display
    display_clear();
    display_update();
    
    // Stop audio playback
    audio_stop();
    
    // Stop haptic feedback
    haptic_stop();
    
    ESP_LOGI(TAG, "All outputs cleared");
    
    return ESP_OK;
}
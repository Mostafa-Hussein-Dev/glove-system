#include "tasks/communication_task.h"
#include <string.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "communication/ble_service.h"
#include "app_main.h"
#include "core/power_management.h"
#include "config/system_config.h"
#include "util/debug.h"
#include "util/buffer.h"
#include "esp_heap_caps.h"

static const char *TAG = "COMM_TASK";

// Task handle
static TaskHandle_t communication_task_handle = NULL;

// Last status update time
static uint32_t last_status_update_ms = 0;
#define STATUS_UPDATE_INTERVAL_MS 5000  // Update status every 5 seconds

// Forward declarations
static void communication_task(void *arg);
static void ble_command_handler(const uint8_t *data, size_t length);
static void print_real_task_stats(void);

esp_err_t communication_task_init(void) {
    ESP_LOGI(TAG, "Initializing communication task with enhanced architecture...");
    ESP_LOGI(TAG, "  Core: %d, Priority: %d, Stack: %d bytes", 
        COMMUNICATION_TASK_CORE, COMMUNICATION_TASK_PRIORITY, COMMUNICATION_TASK_STACK_SIZE);
    
    // Create the communication task with new configuration
    BaseType_t ret = xTaskCreatePinnedToCore(
        communication_task,
        "comm_task",
        COMMUNICATION_TASK_STACK_SIZE,  // UPDATED: Increased from 6144 to 8192
        NULL,
        COMMUNICATION_TASK_PRIORITY,    // SAME: Priority 7
        &communication_task_handle,
        COMMUNICATION_TASK_CORE         // UPDATED: Moved from Core 0 to Core 1
    );
    
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create communication task");
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "Communication task initialized on core %d (moved from core 0)", 
        COMMUNICATION_TASK_CORE);
    ESP_LOGI(TAG, "This avoids I2C conflicts with sensor task");
    return ESP_OK;
}

static void communication_task(void *arg) {
    ESP_LOGI(TAG, "Communication task started");
    
    // Set communication task as ready
    xEventGroupSetBits(g_system_event_group, SYSTEM_EVENT_BLE_READY);
    
    // Wait for system initialization to complete
    xEventGroupWaitBits(g_system_event_group, 
                        SYSTEM_EVENT_INIT_COMPLETE, 
                        pdFALSE, pdTRUE, portMAX_DELAY);
    
    // Register BLE command callback
    ble_service_register_command_callback(ble_command_handler);
    
    // Enable BLE if configured
    if (g_system_config.ble_enabled) {
        ble_service_enable();
    }
    
    // Initialize last status update time
    last_status_update_ms = esp_timer_get_time() / 1000;
    
    // System command processing
    system_command_t system_cmd;
    
    while (1) {
        // Process any incoming system commands
        if (xQueueReceive(g_system_command_queue, &system_cmd, 0) == pdTRUE) {
            // Handle system commands
            switch (system_cmd.type) {
                case SYS_CMD_ENABLE_FEATURE:
                    ble_service_enable();
                    g_system_config.ble_enabled = true;
                    break;
                    
                case SYS_CMD_DISABLE_FEATURE:
                    ble_service_disable();
                    g_system_config.ble_enabled = false;
                    break;
                    
                default:
                    // Forward to other subsystems or tasks if needed
                    if (xQueueSend(g_system_command_queue, &system_cmd, 0) != pdTRUE) {
                        ESP_LOGW(TAG, "Failed to forward system command (queue full)");
                    }
                    break;
            }
        }
        
        // Periodically send status updates over BLE if connected
        uint32_t current_time_ms = esp_timer_get_time() / 1000;
        if (current_time_ms - last_status_update_ms >= STATUS_UPDATE_INTERVAL_MS) {
            bool connected = false;
            if (ble_service_is_connected(&connected) == ESP_OK && connected) {
                // Get battery status
                battery_status_t battery_status;
                if (power_management_get_battery_status(&battery_status) == ESP_OK) {
                    // Send status update
                    ble_service_send_status(
                        battery_status.percentage,
                        (uint8_t)g_system_config.system_state,
                        (uint8_t)g_system_config.error_count
                    );
                }
            }
            
            last_status_update_ms = current_time_ms;
        }
        
        
    

        // Short delay to prevent CPU hogging
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void communication_task_deinit(void) {
    // Cleanup resources when task is deleted
    if (communication_task_handle != NULL) {
        vTaskDelete(communication_task_handle);
        communication_task_handle = NULL;
    }
    
    ESP_LOGI(TAG, "Communication task deinitialized");
}

// Handle BLE commands from mobile app
static void ble_command_handler(const uint8_t *data, size_t length) {
    if (data == NULL || length < 1) {
        return;
    }
    
    // First byte is command ID
    uint8_t cmd_id = data[0];
    
    ESP_LOGI(TAG, "Received BLE command: 0x%02x, length: %d", cmd_id, length);
    
    // Process commands based on ID
    switch (cmd_id) {
        case 0x01: // Set output mode
            if (length >= 2) {
                uint8_t mode = data[1];
                if (mode <= OUTPUT_MODE_MINIMAL) {
                    // Update system config using new enhanced feature flags
                    switch ((output_mode_t)mode) {
                        case OUTPUT_MODE_TEXT_ONLY:
                            g_system_config.audio_feedback_enabled = false;
                            g_system_config.haptic_feedback_enabled = false;
                            ESP_LOGI(TAG, "BLE: Set output mode to TEXT_ONLY");
                            break;
                        case OUTPUT_MODE_AUDIO_ONLY:
                            g_system_config.audio_feedback_enabled = true;
                            g_system_config.haptic_feedback_enabled = false;
                            ESP_LOGI(TAG, "BLE: Set output mode to AUDIO_ONLY");
                            break;
                        case OUTPUT_MODE_TEXT_AND_AUDIO:
                            g_system_config.audio_feedback_enabled = true;
                            g_system_config.haptic_feedback_enabled = true;
                            ESP_LOGI(TAG, "BLE: Set output mode to TEXT_AND_AUDIO");
                            break;
                        case OUTPUT_MODE_MINIMAL:
                            g_system_config.audio_feedback_enabled = false;
                            g_system_config.haptic_feedback_enabled = false;
                            ESP_LOGI(TAG, "BLE: Set output mode to MINIMAL");
                            break;
                        default:
                            ESP_LOGW(TAG, "BLE: Unknown output mode %d", mode);
                            break;
                    }
                    
                    // Create output command
                    output_command_t cmd = {
                        .type = OUTPUT_CMD_SET_MODE,
                        .data.set_mode.mode = (output_mode_t)mode
                    };
                    
                    // Send to output queue
                    if (xQueueSend(g_output_command_queue, &cmd, 0) != pdTRUE) {
                        ESP_LOGW(TAG, "Failed to send output mode command (queue full)");
                    }
                }
            }
            break;
            
        case 0x02: // Calibration command
            {
                // Create system command
                system_command_t cmd = {
                    .type = SYS_CMD_CALIBRATE
                };
                
                // Send to system command queue
                if (xQueueSend(g_system_command_queue, &cmd, 0) != pdTRUE) {
                    ESP_LOGW(TAG, "Failed to send calibration command (queue full)");
                }
            }
            break;
            
        case 0x03: // Power mode command
            if (length >= 2) {
                uint8_t power_mode = data[1];
                if (power_mode <= POWER_MODE_MAX_POWER_SAVE) {
                    // Create system command
                    system_command_t cmd = {
                        .type = SYS_CMD_SET_POWER_MODE,
                        .parameter = power_mode,
                        .timestamp = esp_timer_get_time() / 1000,
                        .source_task = xTaskGetCurrentTaskHandle()
                    };
                    
                    // Send to system command queue
                    if (xQueueSend(g_system_command_queue, &cmd, 0) != pdTRUE) {
                        ESP_LOGW(TAG, "Failed to send power mode command (queue full)");
                    }
                }
            }
            break;
            
        case 0x04: // System state command
            if (length >= 2) {
                uint8_t state = data[1];
                if (state <= SYSTEM_STATE_ERROR) {
                    // Create system command
                    system_command_t cmd = {
                        .type = SYS_CMD_CHANGE_STATE,
                        .parameter = state,
                        .timestamp = esp_timer_get_time() / 1000,
                        .source_task = xTaskGetCurrentTaskHandle()
                    };
                    
                    // Send to system command queue
                    if (xQueueSend(g_system_command_queue, &cmd, 0) != pdTRUE) {
                        ESP_LOGW(TAG, "Failed to send state change command (queue full)");
                    }
                }
            }
            break;
            
        case 0x05: // Sleep command
            if (length >= 3) {
                uint16_t sleep_duration = (data[1] << 8) | data[2];
                
                // Create system command
                system_command_t cmd = {
                    .type = SYS_CMD_SLEEP,
                    .parameter = sleep_duration,
                    .data = NULL,
                    .timestamp = esp_timer_get_time() / 1000,
                    .source_task = xTaskGetCurrentTaskHandle()
                };
                
                // Send to system command queue
                if (xQueueSend(g_system_command_queue, &cmd, 0) != pdTRUE) {
                    ESP_LOGW(TAG, "Failed to send sleep command (queue full)");
                }
            }
            break;
            
        case 0x06: // Restart command
            {
                // Create system command
                system_command_t cmd = {
                    .type = SYS_CMD_RESTART
                };
                
                // Send to system command queue
                if (xQueueSend(g_system_command_queue, &cmd, 0) != pdTRUE) {
                    ESP_LOGW(TAG, "Failed to send restart command (queue full)");
                }
            }
            break;
            
        case 0x07: // Factory reset command
            {
                // Create system command
                system_command_t cmd = {
                    .type = SYS_CMD_FACTORY_RESET
                };
                
                // Send to system command queue
                if (xQueueSend(g_system_command_queue, &cmd, 0) != pdTRUE) {
                    ESP_LOGW(TAG, "Failed to send factory reset command (queue full)");
                }
            }
            break;
            
        case 0x08: // Display text command
            if (length >= 3) {
                uint8_t text_len = data[1];
                if (text_len > 0 && length >= 2 + text_len) {
                    // Create output command
                    output_command_t cmd = {
                        .type = OUTPUT_CMD_DISPLAY_TEXT,
                        .data.display.size = 0,  // Small font
                        .data.display.line = 1,  // Line 1
                        .data.display.clear_first = true  // Clear first
                    };
                    
                    // Copy text (with null termination)
                    size_t copy_len = text_len;
                    if (copy_len > sizeof(cmd.data.display.text) - 1) {
                        copy_len = sizeof(cmd.data.display.text) - 1;
                    }
                    memcpy(cmd.data.display.text, &data[2], copy_len);
                    cmd.data.display.text[copy_len] = '\0';
                    
                    // Send to output queue
                    if (xQueueSend(g_output_command_queue, &cmd, 0) != pdTRUE) {
                        ESP_LOGW(TAG, "Failed to send display text command (queue full)");
                    }
                }
            }
            break;
            
        case 0x09: // Speak text command
            if (length >= 3) {
                uint8_t text_len = data[1];
                if (text_len > 0 && length >= 2 + text_len) {
                    // Create output command
                    output_command_t cmd = {
                        .type = OUTPUT_CMD_SPEAK_TEXT,
                        .data.speak.priority = 0  // Highest priority
                    };
                    
                    // Copy text (with null termination)
                    size_t copy_len = text_len;
                    if (copy_len > sizeof(cmd.data.speak.text) - 1) {
                        copy_len = sizeof(cmd.data.speak.text) - 1;
                    }
                    memcpy(cmd.data.speak.text, &data[2], copy_len);
                    cmd.data.speak.text[copy_len] = '\0';
                    
                    // Send to output queue
                    if (xQueueSend(g_output_command_queue, &cmd, 0) != pdTRUE) {
                        ESP_LOGW(TAG, "Failed to send speak text command (queue full)");
                    }
                }
            }
            break;
            
        case 0x0A: // Haptic feedback command
            if (length >= 4) {
                uint8_t pattern = data[1];
                uint8_t intensity = data[2];
                uint8_t duration = data[3];
                
                // Create output command
                output_command_t cmd = {
                    .type = OUTPUT_CMD_HAPTIC_FEEDBACK,
                    .data.haptic.pattern = pattern,
                    .data.haptic.intensity = intensity,
                    .data.haptic.duration_ms = duration * 10  // Convert to ms (0-2550ms)
                };
                
                // Send to output queue
                if (xQueueSend(g_output_command_queue, &cmd, 0) != pdTRUE) {
                    ESP_LOGW(TAG, "Failed to send haptic feedback command (queue full)");
                }
            }
            break;
            
        default:
            ESP_LOGW(TAG, "Unknown BLE command: 0x%02x", cmd_id);
            break;
    }
}


void* communication_task_get_handle(void) {
    extern TaskHandle_t communication_task_handle;  // Declare external reference
    return (void*)communication_task_handle;
}
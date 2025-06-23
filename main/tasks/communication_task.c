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

static void check_system_health(void) {
    // Check queue health
    UBaseType_t sensor_items = uxQueueMessagesWaiting(g_sensor_data_queue);
    UBaseType_t processing_items = uxQueueMessagesWaiting(g_processing_result_queue);
    UBaseType_t output_items = uxQueueMessagesWaiting(g_output_command_queue);
    UBaseType_t system_items = uxQueueMessagesWaiting(g_system_command_queue);
    
    // Check memory health
    size_t free_heap = heap_caps_get_free_size(MALLOC_CAP_DEFAULT);
    
    // Log warnings if queues are getting full
    if (sensor_items > (SENSOR_QUEUE_SIZE * 0.8)) {
        ESP_LOGW(TAG, "Sensor queue high: %u/%d", (unsigned int)sensor_items, SENSOR_QUEUE_SIZE);
    }
    if (processing_items > (PROCESSING_QUEUE_SIZE * 0.8)) {
        ESP_LOGW(TAG, "Processing queue high: %u/%d", (unsigned int)processing_items, PROCESSING_QUEUE_SIZE);
    }
    if (output_items > (OUTPUT_QUEUE_SIZE * 0.8)) {
        ESP_LOGW(TAG, "Output queue high: %u/%d", (unsigned int)output_items, OUTPUT_QUEUE_SIZE);
    }
    if (system_items > (COMMAND_QUEUE_SIZE * 0.8)) {
        ESP_LOGW(TAG, "System queue high: %u/%d", (unsigned int)system_items, COMMAND_QUEUE_SIZE);
    }
    
    // Log memory warnings
    if (free_heap < 50000) {  // Less than 50KB
        ESP_LOGW(TAG, "Low memory warning: %zu bytes free", free_heap);
        if (free_heap < 20000) {  // Less than 20KB - critical
            ESP_LOGE(TAG, "CRITICAL: Very low memory: %zu bytes", free_heap);
            buffer_emergency_cleanup();  // Emergency cleanup
        }
    }
}

esp_err_t communication_task_init(void) {
    // Create the communication task
    BaseType_t ret = xTaskCreatePinnedToCore(
        communication_task,
        "communication_task",
        COMMUNICATION_TASK_STACK_SIZE,
        NULL,
        COMMUNICATION_TASK_PRIORITY,
        &communication_task_handle,
        COMMUNICATION_TASK_CORE
    );
    
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create communication task");
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "Communication task initialized on core %d", COMMUNICATION_TASK_CORE);
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
    if (g_system_config.bluetooth_enabled) {
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
                case SYS_CMD_ENABLE_BLE:
                    ble_service_enable();
                    g_system_config.bluetooth_enabled = true;
                    break;
                    
                case SYS_CMD_DISABLE_BLE:
                    ble_service_disable();
                    g_system_config.bluetooth_enabled = false;
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
                        (uint8_t)g_system_config.last_error
                    );
                }
            }
            
            // Periodic system health check (every 10 seconds)
            uint32_t current_time_ms = esp_timer_get_time() / 1000;
            if (current_time_ms - last_status_update_ms >= 10000) {
                check_system_health();
                last_status_update_ms = current_time_ms;
            }
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
                    // Update system config
                    g_system_config.output_mode = (output_mode_t)mode;
                    
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
                        .data.power_mode.enable_power_save = (power_mode != POWER_MODE_PERFORMANCE)
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
                        .data.change_state.new_state = (system_state_t)state
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
                    .data.sleep.sleep_duration_sec = sleep_duration
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
    return (void*)communication_task_handle;
}
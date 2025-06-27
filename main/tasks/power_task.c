#include "tasks/power_task.h"
#include <string.h>
#include "esp_log.h"
#include "esp_system.h"
#include "esp_sleep.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "core/power_management.h"
#include "core/system_monitor.h"
#include "app_main.h"
#include "config/system_config.h"
#include "util/debug.h"
#include "output/output_manager.h"
#include "esp_heap_caps.h"
#include "util/buffer.h"

static const char *TAG = "POWER_TASK";

// Task handle
static TaskHandle_t power_task_handle = NULL;

// Monitoring intervals
#define BATTERY_CHECK_INTERVAL_MS    30000   // Check battery every 30 seconds
#define STATUS_DISPLAY_INTERVAL_MS   60000   // Display status every 60 seconds

// Forward declarations
static void power_task(void *arg);
static void handle_system_command(system_command_t *cmd);
static void check_battery_and_power(void);
static void handle_sleep_logic(void);


esp_err_t power_task_init(void) {
    ESP_LOGI(TAG, "Initializing simplified power task...");
    ESP_LOGI(TAG, "Core: %d, Priority: %d, Stack: %d bytes", 
        POWER_TASK_CORE, POWER_TASK_PRIORITY, POWER_TASK_STACK_SIZE);
    
    // Create the power task
    BaseType_t ret = xTaskCreatePinnedToCore(
        power_task,
        "power_task",
        POWER_TASK_STACK_SIZE,
        NULL,
        POWER_TASK_PRIORITY,
        &power_task_handle,
        POWER_TASK_CORE
    );
    
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create power task");
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "Power task initialized successfully");
    return ESP_OK;
}
static void power_task(void *arg) {
    ESP_LOGI(TAG, "Power task started - simplified implementation");
    
    // Wait for system initialization
    xEventGroupWaitBits(g_system_event_group, 
                        SYSTEM_EVENT_INIT_COMPLETE, 
                        pdFALSE, pdTRUE, portMAX_DELAY);
    
    // Initialize timestamps
    uint32_t last_battery_check_ms = 0;
    uint32_t last_status_display_ms = 0;
    
    while (1) {
        uint32_t current_time_ms = esp_timer_get_time() / 1000;
        
        // 1. Handle system commands (high priority)
        system_command_t system_cmd;
        if (xQueueReceive(g_system_command_queue, &system_cmd, 0) == pdTRUE) {
            handle_system_command(&system_cmd);
        }
        
        // 2. Check battery status periodically
        if (current_time_ms - last_battery_check_ms >= BATTERY_CHECK_INTERVAL_MS) {
            check_battery_and_power();
            last_battery_check_ms = current_time_ms;
        }
        
        // 3. Display system status when idle
        if (g_system_config.system_state == SYSTEM_STATE_IDLE && 
            current_time_ms - last_status_display_ms >= STATUS_DISPLAY_INTERVAL_MS) {
            
            battery_status_t battery_status;
            if (power_management_get_battery_status(&battery_status) == ESP_OK) {
                output_command_t cmd = {
                    .type = OUTPUT_CMD_SHOW_STATUS
                };
                xQueueSend(g_output_command_queue, &cmd, 0);
            }
            last_status_display_ms = current_time_ms;
        }
        
        // 4. Handle sleep logic (simplified)
        handle_sleep_logic();
        
        // 5. Simple memory check
        size_t free_heap = esp_get_free_heap_size();
        if (free_heap < 10000) {  // 10KB threshold
            ESP_LOGW(TAG, "Low memory: %d bytes", free_heap);
            // Switch to power save mode
            if (power_management_get_mode() != POWER_MODE_POWER_SAVE) {
                power_management_set_mode(POWER_MODE_POWER_SAVE);
            }
        }
        
        // Sleep for 1 second
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

static void handle_system_command(system_command_t *cmd) {
    switch (cmd->type) {
        case SYS_CMD_SET_POWER_MODE:
            ESP_LOGI(TAG, "Setting power mode: %d", (power_mode_t)cmd->parameter);
            power_management_set_mode((power_mode_t)cmd->parameter);
            break;
            
        case SYS_CMD_SLEEP:
            ESP_LOGI(TAG, "Manual sleep command received");
            power_management_light_sleep(30000);  // 30 second sleep
            break;
            
        case SYS_CMD_CHANGE_STATE:
            ESP_LOGD(TAG, "User activity detected");
            power_management_reset_activity();
            
            // If we're in power save mode and battery isn't critical, go to balanced
            battery_status_t battery;
            if (power_management_get_battery_status(&battery) == ESP_OK) {
                if (!battery.is_critical && power_management_get_mode() == POWER_MODE_POWER_SAVE) {
                    power_management_set_mode(POWER_MODE_BALANCED);
                }
            }
            break;
            
        default:
            ESP_LOGW(TAG, "Unknown system command: %d", cmd->type);
            break;
    }
}

static void check_battery_and_power(void) {
    battery_status_t battery_status;
    esp_err_t ret = power_management_get_battery_status(&battery_status);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get battery status");
        return;
    }
    
    ESP_LOGI(TAG, "Battery: %.2fV (%d%%), %s%s%s", 
             battery_status.voltage_mv / 1000.0f,
             battery_status.percentage,
             battery_status.is_charging ? "CHARGING" : "NOT_CHARGING",
             battery_status.is_low ? ", LOW" : "",
             battery_status.is_critical ? ", CRITICAL" : "");
    
    // Handle battery-based power management
    if (battery_status.is_critical) {
        ESP_LOGE(TAG, "CRITICAL BATTERY - Entering power save mode");
        g_system_config.system_state = SYSTEM_STATE_LOW_BATTERY;
        power_management_set_mode(POWER_MODE_POWER_SAVE);
        
        // Set low battery event
        xEventGroupSetBits(g_system_event_group, SYSTEM_EVENT_LOW_BATTERY);
        
    } else if (battery_status.is_low && !battery_status.is_charging) {
        ESP_LOGW(TAG, "Low battery - Switching to power save mode");
        if (g_system_config.system_state != SYSTEM_STATE_LOW_BATTERY) {
            g_system_config.system_state = SYSTEM_STATE_LOW_BATTERY;
            power_management_set_mode(POWER_MODE_POWER_SAVE);
        }
        
    } else if (battery_status.is_charging) {
        // Handle charging state
        if (g_system_config.system_state != SYSTEM_STATE_CHARGING) {
            ESP_LOGI(TAG, "Device is charging");
            g_system_config.system_state = SYSTEM_STATE_CHARGING;
            
            // Display charging status
            output_command_t cmd = {
                .type = OUTPUT_CMD_SHOW_BATTERY,
                .data.battery.percentage = battery_status.percentage,
                .data.battery.show_graphic = true
            };
            xQueueSend(g_output_command_queue, &cmd, 0);
        }
        
    } else if (!battery_status.is_low && !battery_status.is_critical) {
        // Battery is good - normal operation
        if (g_system_config.system_state == SYSTEM_STATE_LOW_BATTERY || 
            g_system_config.system_state == SYSTEM_STATE_CHARGING) {
            
            ESP_LOGI(TAG, "Battery recovered - Normal operation");
            g_system_config.system_state = SYSTEM_STATE_IDLE;
            xEventGroupClearBits(g_system_event_group, SYSTEM_EVENT_LOW_BATTERY);
            
            // Return to balanced mode
            power_management_set_mode(POWER_MODE_BALANCED);
        }
    }
}

static void handle_sleep_logic(void) {
    // Get inactive time
    uint32_t inactive_time_ms = power_management_get_inactive_time();
    
    // Get current battery status
    battery_status_t battery;
    if (power_management_get_battery_status(&battery) != ESP_OK) {
        return;  // Can't check battery, skip sleep logic
    }
    
    // Skip sleep if charging
    if (battery.is_charging) {
        return;
    }
    
    // Skip sleep if system is busy
    if (g_system_config.system_state != SYSTEM_STATE_IDLE) {
        return;
    }
    
    // Check what sleep action to take
    sleep_action_t sleep_action = power_management_check_sleep(inactive_time_ms, battery.is_low);
    
    switch (sleep_action) {
        case SLEEP_ACTION_LIGHT:
            ESP_LOGI(TAG, "Entering light sleep due to inactivity (%d ms)", inactive_time_ms);
            power_management_light_sleep(30000);  // 30 seconds
            break;
            
        case SLEEP_ACTION_DEEP:
            ESP_LOGI(TAG, "Entering deep sleep due to long inactivity (%d ms)", inactive_time_ms);
            power_management_deep_sleep(300000);  // 5 minutes
            break;
            
        case SLEEP_ACTION_NONE:
        default:
            // Stay awake
            break;
    }
}

void power_task_deinit(void) {
    if (power_task_handle != NULL) {
        vTaskDelete(power_task_handle);
        power_task_handle = NULL;
    }
    ESP_LOGI(TAG, "Power task deinitialized");
}

void* power_task_get_handle(void) {
    extern TaskHandle_t power_task_handle;  // Declare external reference
    return (void*)power_task_handle;
}
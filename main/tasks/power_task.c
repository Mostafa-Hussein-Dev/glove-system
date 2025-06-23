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

// Last battery check time
static uint32_t last_battery_check_ms = 0;
#define BATTERY_CHECK_INTERVAL_MS 30000  // Check battery every 30 seconds

// Last system status display time
static uint32_t last_status_display_ms = 0;
#define STATUS_DISPLAY_INTERVAL_MS 60000  // Display status every 60 seconds

// Inactivity tracking
static uint32_t last_activity_time_ms = 0;

// Forward declarations
static void power_task(void *arg);
static void handle_system_command(system_command_t *cmd);
static void enter_power_save_mode(void);
static void exit_power_save_mode(void);
static void check_battery_status(void);

esp_err_t power_task_init(void) {
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
    
    ESP_LOGI(TAG, "Power task initialized on core %d", POWER_TASK_CORE);
    return ESP_OK;
}

static void power_task(void *arg) {
    ESP_LOGI(TAG, "Power task started");
    
    // Wait for system initialization to complete
    xEventGroupWaitBits(g_system_event_group, 
                        SYSTEM_EVENT_INIT_COMPLETE, 
                        pdFALSE, pdTRUE, portMAX_DELAY);
    
    // Initialize timestamps
    uint32_t current_time_ms = esp_timer_get_time() / 1000;
    last_battery_check_ms = current_time_ms;
    last_status_display_ms = current_time_ms;
    last_activity_time_ms = current_time_ms;
    
    // System command processing
    system_command_t system_cmd;
    
    while (1) {
        current_time_ms = esp_timer_get_time() / 1000;
        
        // Check for system commands
        if (xQueueReceive(g_system_command_queue, &system_cmd, 0) == pdTRUE) {
            handle_system_command(&system_cmd);
        }
        
        // Check battery status periodically
        if (current_time_ms - last_battery_check_ms >= BATTERY_CHECK_INTERVAL_MS) {
            check_battery_status();
            last_battery_check_ms = current_time_ms;
        }
        
        // Periodically display system status (if in idle state)
        if (g_system_config.system_state == SYSTEM_STATE_IDLE && 
            current_time_ms - last_status_display_ms >= STATUS_DISPLAY_INTERVAL_MS) {
            
            // Get battery status
            battery_status_t battery_status;
            if (power_management_get_battery_status(&battery_status) == ESP_OK) {
                // Create output command for status display
                output_command_t cmd = {
                    .type = OUTPUT_CMD_SHOW_STATUS
                };
                
                // Send to output queue
                if (xQueueSend(g_output_command_queue, &cmd, 0) != pdTRUE) {
                    ESP_LOGW(TAG, "Failed to send status display command (queue full)");
                }
            }
            
            last_status_display_ms = current_time_ms;
        }
        
        // Check for inactivity and handle power management
        if (g_system_config.power_save_enabled) {
            // Process inactivity timeout
            power_management_process_inactivity(current_time_ms);
        }
        
        // Check system health with direct heap check
        system_metrics_t metrics;
        size_t direct_free_heap = esp_get_free_heap_size();  // Direct read
        
        if (system_monitor_get_metrics(&metrics) == ESP_OK) {
            // Validate metrics - if system monitor hasn't run yet, use direct values
            if (metrics.free_heap == 0 || metrics.uptime_ms == 0) {
                ESP_LOGD(TAG, "System monitor not ready, using direct heap reading");
                metrics.free_heap = direct_free_heap;
            }
            
            // Perform automated actions based on validated metrics
            if (metrics.free_heap < 50000) {  // Increased threshold to 50KB (was 10KB)
                ESP_LOGW(TAG, "Low memory detected: %u bytes (direct: %zu bytes)", 
                         metrics.free_heap, direct_free_heap);
                
                // Only take action if both readings are low
                if (direct_free_heap < 50000) {
                    ESP_LOGW(TAG, "Confirmed low memory condition");
                    // Take some actions to free memory
                    buffer_emergency_cleanup();  // Emergency buffer cleanup
                }
            } else {
                ESP_LOGD(TAG, "Memory OK: %u bytes (direct: %zu bytes)", 
                         metrics.free_heap, direct_free_heap);
            }
            
            if (metrics.cpu_usage_percent > 80) {  // High CPU usage
                ESP_LOGW(TAG, "High CPU usage detected: %u%%", metrics.cpu_usage_percent);
                
                // Consider throttling some non-essential tasks
                if (power_management_get_mode() != POWER_MODE_PERFORMANCE) {
                    // Temporarily boost performance
                    power_management_set_mode(POWER_MODE_BALANCED);
                }
            }
        }
        
        // Short delay to prevent CPU hogging
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void power_task_deinit(void) {
    // Cleanup resources when task is deleted
    if (power_task_handle != NULL) {
        vTaskDelete(power_task_handle);
        power_task_handle = NULL;
    }
    
    ESP_LOGI(TAG, "Power task deinitialized");
}

static void handle_system_command(system_command_t *cmd) {
    if (cmd == NULL) return;
    
    switch (cmd->type) {
        case SYS_CMD_CHANGE_STATE:
            // Update system state
            ESP_LOGI(TAG, "Changing system state from %d to %d", 
                    g_system_config.system_state, cmd->data.change_state.new_state);
            
            g_system_config.system_state = cmd->data.change_state.new_state;
            
            // Handle state-specific actions
            switch (g_system_config.system_state) {
                case SYSTEM_STATE_SLEEP:
                    power_management_set_mode(POWER_MODE_MAX_POWER_SAVE);
                    break;
                    
                case SYSTEM_STATE_STANDBY:
                    power_management_set_mode(POWER_MODE_POWER_SAVE);
                    break;
                    
                case SYSTEM_STATE_ACTIVE:
                    power_management_set_mode(POWER_MODE_BALANCED);
                    break;
                    
                default:
                    break;
            }
            
            // Reset inactivity timer
            power_management_reset_inactivity_timer();
            last_activity_time_ms = esp_timer_get_time() / 1000;
            break;
            
        case SYS_CMD_CALIBRATE:
            ESP_LOGI(TAG, "Executing calibration command");
            
            // Set system state to calibration
            g_system_config.system_state = SYSTEM_STATE_CALIBRATION;
            
            // Create output command for calibration instructions
            output_command_t out_cmd = {
                .type = OUTPUT_CMD_DISPLAY_TEXT,
                .data.display.clear_first = true,
                .data.display.line = 0,
                .data.display.size = 0
            };
            
            strcpy(out_cmd.data.display.text, "Calibration Mode");
            xQueueSend(g_output_command_queue, &out_cmd, 0);
            
            // Execute calibration sequence here
            // This would involve flex sensor calibration, IMU calibration, etc.
            // For now, this is just a placeholder
            
            // Reset inactivity timer
            power_management_reset_inactivity_timer();
            last_activity_time_ms = esp_timer_get_time() / 1000;
            break;
            
        case SYS_CMD_SET_POWER_MODE:
            if (cmd->data.power_mode.enable_power_save) {
                enter_power_save_mode();
            } else {
                exit_power_save_mode();
            }
            break;
            
        case SYS_CMD_RESTART:
            ESP_LOGI(TAG, "System restart requested");
            
            // Display restart message
            output_command_t restart_cmd = {
                .type = OUTPUT_CMD_DISPLAY_TEXT,
                .data.display.clear_first = true,
                .data.display.line = 0,
                .data.display.size = 0
            };
            
            strcpy(restart_cmd.data.display.text, "Restarting...");
            xQueueSend(g_output_command_queue, &restart_cmd, 0);
            
            // Give some time for the message to be displayed
            vTaskDelay(pdMS_TO_TICKS(1000));
            
            // Restart the system
            esp_restart();
            break;
            
        case SYS_CMD_SLEEP:
            ESP_LOGI(TAG, "Sleep command received: %d seconds", 
                    cmd->data.sleep.sleep_duration_sec);
            
            // Display sleep message
            output_command_t sleep_cmd = {
                .type = OUTPUT_CMD_DISPLAY_TEXT,
                .data.display.clear_first = true,
                .data.display.line = 0,
                .data.display.size = 0
            };
            
            sprintf(sleep_cmd.data.display.text, "Sleeping for %d sec...", 
                    cmd->data.sleep.sleep_duration_sec);
            xQueueSend(g_output_command_queue, &sleep_cmd, 0);
            
            // Give some time for the message to be displayed
            vTaskDelay(pdMS_TO_TICKS(1000));
            
            // Prepare for sleep
            g_system_config.system_state = SYSTEM_STATE_SLEEP;
            
            // Enter deep sleep
            power_management_deep_sleep(cmd->data.sleep.sleep_duration_sec * 1000);
            break;
            
        case SYS_CMD_FACTORY_RESET:
            ESP_LOGI(TAG, "Factory reset requested");
            
            // Display factory reset message
            output_command_t reset_cmd = {
                .type = OUTPUT_CMD_DISPLAY_TEXT,
                .data.display.clear_first = true,
                .data.display.line = 0,
                .data.display.size = 0
            };
            
            strcpy(reset_cmd.data.display.text, "Factory reset...");
            xQueueSend(g_output_command_queue, &reset_cmd, 0);
            
            // Give some time for the message to be displayed
            vTaskDelay(pdMS_TO_TICKS(1000));
            
            // Perform factory reset actions here
            // This would involve clearing NVS, resetting calibration, etc.
            
            // Restart the system after reset
            esp_restart();
            break;
            
        default:
            ESP_LOGW(TAG, "Unknown system command: %d", cmd->type);
            break;
    }
}

static void enter_power_save_mode(void) {
    ESP_LOGI(TAG, "Entering power save mode");
    
    // Enable power save mode
    g_system_config.power_save_enabled = true;
    
    // Choose appropriate power mode based on system state
    power_mode_t target_mode;
    switch (g_system_config.system_state) {
        case SYSTEM_STATE_SLEEP:
            target_mode = POWER_MODE_MAX_POWER_SAVE;
            break;
        case SYSTEM_STATE_STANDBY:
            target_mode = POWER_MODE_POWER_SAVE;
            break;
        case SYSTEM_STATE_IDLE:
            target_mode = POWER_MODE_POWER_SAVE;
            break;
        default:
            target_mode = POWER_MODE_BALANCED;
            break;
    }
    
    // Apply the power mode
    power_management_set_mode(target_mode);
    
    // Display power save mode enabled
    output_command_t cmd = {
        .type = OUTPUT_CMD_DISPLAY_TEXT,
        .data.display.clear_first = false,
        .data.display.line = 5,
        .data.display.size = 0
    };
    
    strcpy(cmd.data.display.text, "Power Save: ON");
    xQueueSend(g_output_command_queue, &cmd, 0);
}

static void exit_power_save_mode(void) {
    ESP_LOGI(TAG, "Exiting power save mode");
    
    // Disable power save mode
    g_system_config.power_save_enabled = false;
    
    // Set to performance mode
    power_management_set_mode(POWER_MODE_PERFORMANCE);
    
    // Display power save mode disabled
    output_command_t cmd = {
        .type = OUTPUT_CMD_DISPLAY_TEXT,
        .data.display.clear_first = false,
        .data.display.line = 5,
        .data.display.size = 0
    };
    
    strcpy(cmd.data.display.text, "Power Save: OFF");
    xQueueSend(g_output_command_queue, &cmd, 0);
}

static void check_battery_status(void) {
    battery_status_t battery_status;
    
    if (power_management_get_battery_status(&battery_status) != ESP_OK) {
        ESP_LOGW(TAG, "Failed to get battery status");
        return;
    }
    
    ESP_LOGI(TAG, "Battery status: %d%% (%d mV), charging: %s, low: %s, critical: %s",
             battery_status.percentage, battery_status.voltage_mv,
             battery_status.is_charging ? "yes" : "no",
             battery_status.is_low ? "yes" : "no",
             battery_status.is_critical ? "yes" : "no");
    
    // Handle low battery
    if (battery_status.is_critical && g_system_config.system_state != SYSTEM_STATE_CHARGING) {
        ESP_LOGW(TAG, "Battery critically low, entering power save mode");
        
        // Set error state
        g_system_config.system_state = SYSTEM_STATE_LOW_BATTERY;
        g_system_config.last_error = SYSTEM_ERROR_BATTERY;
        
        // Set max power save mode
        power_management_set_mode(POWER_MODE_MAX_POWER_SAVE);
        
        // Display low battery warning
        output_command_t cmd = {
            .type = OUTPUT_CMD_SHOW_ERROR,
            .data.error.error_code = SYSTEM_ERROR_BATTERY,
            .data.error.error_text = "Battery critically low!"
        };
        
        xQueueSend(g_output_command_queue, &cmd, 0);
        
        // Set low battery event bit
        xEventGroupSetBits(g_system_event_group, SYSTEM_EVENT_LOW_BATTERY);
    }
    else if (battery_status.is_low && g_system_config.system_state != SYSTEM_STATE_LOW_BATTERY && 
             g_system_config.system_state != SYSTEM_STATE_CHARGING) {
        ESP_LOGW(TAG, "Battery low, entering power save mode");
        
        // Display low battery warning
        output_command_t cmd = {
            .type = OUTPUT_CMD_SHOW_BATTERY,
            .data.battery.percentage = battery_status.percentage,
            .data.battery.show_graphic = true
        };
        
        xQueueSend(g_output_command_queue, &cmd, 0);
        
        // Enter power save mode
        if (g_system_config.power_save_enabled == false) {
            enter_power_save_mode();
        }
        else {
            // Already in power save mode, use a more aggressive mode
            power_management_set_mode(POWER_MODE_POWER_SAVE);
        }
    }
    else if (battery_status.is_charging) {
        // Update system state if charging
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
    }
    else if (g_system_config.system_state == SYSTEM_STATE_CHARGING && !battery_status.is_charging) {
        // Was charging, but not anymore
        ESP_LOGI(TAG, "Charging complete or disconnected");
        
        // Return to idle state
        g_system_config.system_state = SYSTEM_STATE_IDLE;
        
        // Display battery status
        output_command_t cmd = {
            .type = OUTPUT_CMD_SHOW_BATTERY,
            .data.battery.percentage = battery_status.percentage,
            .data.battery.show_graphic = true
        };
        
        xQueueSend(g_output_command_queue, &cmd, 0);
    }
    else if (g_system_config.system_state == SYSTEM_STATE_LOW_BATTERY && 
             !battery_status.is_low && !battery_status.is_critical) {
        // Battery was low but now recovered
        ESP_LOGI(TAG, "Battery level recovered");
        
        // Return to idle state
        g_system_config.system_state = SYSTEM_STATE_IDLE;
        g_system_config.last_error = SYSTEM_ERROR_NONE;
        
        // Clear low battery event bit
        xEventGroupClearBits(g_system_event_group, SYSTEM_EVENT_LOW_BATTERY);
        
        // Display battery status
        output_command_t cmd = {
            .type = OUTPUT_CMD_SHOW_BATTERY,
            .data.battery.percentage = battery_status.percentage,
            .data.battery.show_graphic = true
        };
        
        xQueueSend(g_output_command_queue, &cmd, 0);
        
        // Return to balanced power mode
        power_management_set_mode(POWER_MODE_BALANCED);
    }
}
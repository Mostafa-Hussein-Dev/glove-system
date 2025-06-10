#include "core/system_monitor.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "util/debug.h"

static const char *TAG = "SYS_MONITOR";

// Task handle for the system monitor task
static TaskHandle_t monitor_task_handle = NULL;

// Last captured metrics
static system_metrics_t last_metrics = {0};

// The monitoring interval in milliseconds
#define MONITOR_INTERVAL_MS 5000

// Forward declarations
static void system_monitor_task(void *pvParameters);

esp_err_t system_monitor_init(void) {
    // Create the system monitor task
    BaseType_t xReturned = xTaskCreate(
        system_monitor_task,
        "system_monitor",
        4096,  // Stack size increased from 2048 to 4096
        NULL,
        2,     // Priority (low)
        &monitor_task_handle);
        
    if (xReturned != pdPASS) {
        ESP_LOGE(TAG, "Failed to create system monitor task");
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "System monitor initialized with 4KB stack");
    return ESP_OK;
}

esp_err_t system_monitor_get_metrics(system_metrics_t* metrics) {
    if (metrics == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Copy the last captured metrics
    memcpy(metrics, &last_metrics, sizeof(system_metrics_t));
    
    return ESP_OK;
}

esp_err_t system_monitor_print_metrics(void) {
    system_metrics_t metrics;
    esp_err_t ret = system_monitor_get_metrics(&metrics);
    if (ret != ESP_OK) {
        return ret;
    }
    
    ESP_LOGI(TAG, "System Metrics:");
    ESP_LOGI(TAG, "  Free Heap: %u bytes", metrics.free_heap);
    ESP_LOGI(TAG, "  Min Free Heap: %u bytes", metrics.min_free_heap);
    ESP_LOGI(TAG, "  CPU Usage: %u%%", metrics.cpu_usage_percent);
    ESP_LOGI(TAG, "  CPU Temperature: %.1f°C", metrics.cpu_temperature);
    ESP_LOGI(TAG, "  Task Count: %u", metrics.task_count);
    ESP_LOGI(TAG, "  Stack High-Water: Core 0: %u, Core 1: %u", 
        metrics.stack_high_water[0], metrics.stack_high_water[1]);
    ESP_LOGI(TAG, "  Uptime: %" PRIu64 " ms", metrics.uptime_ms);
    
    return ESP_OK;
}

esp_err_t system_monitor_health_check(void) {
    system_metrics_t metrics;
    esp_err_t ret = system_monitor_get_metrics(&metrics);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Check heap memory
    if (metrics.free_heap < 10000) {  // 10KB as a critical threshold
        ESP_LOGW(TAG, "Low heap memory: %u bytes", metrics.free_heap);
        return ESP_ERR_NO_MEM;
    }
    
    // Check CPU usage (if extremely high for this kind of system)
    if (metrics.cpu_usage_percent > 90) {
        ESP_LOGW(TAG, "High CPU usage: %u%%", metrics.cpu_usage_percent);
        return ESP_FAIL;
    }
    
    // Check temperature (if supported and if too high)
    if (metrics.cpu_temperature > 65.0f) {  // 65°C as a critical threshold
        ESP_LOGW(TAG, "High CPU temperature: %.1f°C", metrics.cpu_temperature);
        return ESP_FAIL;
    }
    
    return ESP_OK;
}

void* system_monitor_get_task_handle(void) {
    return (void*)monitor_task_handle;
}

// System monitor task function - FIXED VERSION
static void system_monitor_task(void *pvParameters) {
    uint32_t idle_run_time_prev[2] = {0, 0};  // Previous idle task run time for cores
    uint32_t total_run_time_prev = 0;         // Previous total run time
    
    // Wait for initial stabilization
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    while (1) {
        // Get current metrics
        last_metrics.free_heap = esp_get_free_heap_size();
        last_metrics.min_free_heap = esp_get_minimum_free_heap_size();
        last_metrics.task_count = uxTaskGetNumberOfTasks();
        last_metrics.uptime_ms = esp_timer_get_time() / 1000;
        
        // CPU usage calculation with proper bounds checking
        uint32_t idle_run_time[2] = {0, 0};  // Run time for idle tasks per core
        uint32_t total_run_time = 0;         // Total run time
        
        // Get runtime stats if available
        UBaseType_t task_count = uxTaskGetNumberOfTasks();
        TaskStatus_t *pxTaskStatusArray = pvPortMalloc(task_count * sizeof(TaskStatus_t));
        
        if (pxTaskStatusArray != NULL) {
            UBaseType_t actual_tasks = uxTaskGetSystemState(pxTaskStatusArray, task_count, &total_run_time);
            
            // Process each task to find IDLE tasks
            for (UBaseType_t i = 0; i < actual_tasks; i++) {
                TaskStatus_t *task = &pxTaskStatusArray[i];
                
                // Check for IDLE tasks and update stack high water marks
                if (strcmp(task->pcTaskName, "IDLE") == 0 || 
                    strcmp(task->pcTaskName, "IDLE0") == 0) {
                    idle_run_time[0] = task->ulRunTimeCounter;
                } else if (strcmp(task->pcTaskName, "IDLE1") == 0) {
                    idle_run_time[1] = task->ulRunTimeCounter;
                }
                
                // Update stack high water marks
                if (task->usStackHighWaterMark < last_metrics.stack_high_water[0] || 
                    last_metrics.stack_high_water[0] == 0) {
                    last_metrics.stack_high_water[0] = task->usStackHighWaterMark;
                }
            }
            
            // Free the task status array
            vPortFree(pxTaskStatusArray);
            
            // **FIXED CPU CALCULATION WITH PROPER BOUNDS CHECKING**
            if (total_run_time_prev > 0 && total_run_time > total_run_time_prev) {
                uint32_t delta_time = total_run_time - total_run_time_prev;
                
                if (delta_time > 0) {  // Prevent division by zero
                    uint32_t delta_idle0 = (idle_run_time[0] > idle_run_time_prev[0]) ? 
                                          (idle_run_time[0] - idle_run_time_prev[0]) : 0;
                    
                    // Calculate CPU usage with overflow protection
                    if (delta_idle0 <= delta_time) {  // Ensure idle time doesn't exceed total time
                        uint32_t idle_percentage = (delta_idle0 * 100) / delta_time;
                        if (idle_percentage > 100) idle_percentage = 100;  // Clamp to 100%
                        
                        last_metrics.cpu_usage_percent = 100 - idle_percentage;
                        
                        // Additional bounds check
                        if (last_metrics.cpu_usage_percent > 100) {
                            last_metrics.cpu_usage_percent = 0;  // Handle any remaining overflow
                        }
                    } else {
                        // If idle time somehow exceeds total time, set CPU usage to 0
                        last_metrics.cpu_usage_percent = 0;
                    }
                } else {
                    last_metrics.cpu_usage_percent = 0;  // No time passed
                }
            } else {
                last_metrics.cpu_usage_percent = 0;  // First run or invalid data
            }
        } else {
            // If malloc failed, set safe defaults
            last_metrics.cpu_usage_percent = 0;
            ESP_LOGW(TAG, "Failed to allocate memory for task status array");
        }
        
        // Store current values for next calculation
        idle_run_time_prev[0] = idle_run_time[0];
        idle_run_time_prev[1] = idle_run_time[1];
        total_run_time_prev = total_run_time;
        
        // Set fixed temperature placeholder
        last_metrics.cpu_temperature = 45.0f;
        
        // Periodically log the metrics (every 30 seconds)
        static uint32_t log_counter = 0;
        if (++log_counter >= 6) {  // 6 * 5000ms = 30 seconds
            system_monitor_print_metrics();
            log_counter = 0;
        }
        
        // Run health check with fixed CPU usage calculation
        esp_err_t health_result = system_monitor_health_check();
        if (health_result != ESP_OK) {
            ESP_LOGW(TAG, "Health check failed with error %s", esp_err_to_name(health_result));
        }
        
        // Sleep until next check
        vTaskDelay(pdMS_TO_TICKS(MONITOR_INTERVAL_MS));
    }
}
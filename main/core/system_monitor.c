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
        2048,  // Stack size
        NULL,
        2,     // Priority (low)
        &monitor_task_handle);
        
    if (xReturned != pdPASS) {
        ESP_LOGE(TAG, "Failed to create system monitor task");
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "System monitor initialized");
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
    ESP_LOGI(TAG, "  Uptime: %llu ms", metrics.uptime_ms);
    
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

// System monitor task function
static void system_monitor_task(void *pvParameters) {
    uint32_t task_run_time_prev[2] = {0, 0};  // Previous run time for cores
    uint32_t idle_run_time_prev[2] = {0, 0};  // Previous idle task run time for cores
    uint32_t total_run_time_prev = 0;         // Previous total run time
    
    while (1) {
        // Get current metrics
        
        // Heap metrics
        last_metrics.free_heap = esp_get_free_heap_size();
        last_metrics.min_free_heap = esp_get_minimum_free_heap_size();
        
        // Task count
        last_metrics.task_count = uxTaskGetNumberOfTasks();
        
        // Uptime
        last_metrics.uptime_ms = esp_timer_get_time() / 1000;
        
        // CPU usage calculation
        uint32_t task_run_time[2] = {0, 0};       // Run time for all tasks per core
        uint32_t idle_run_time[2] = {0, 0};       // Run time for idle tasks per core
        uint32_t total_run_time = 0;             // Total run time
        
        // Get runtime stats
        TaskStatus_t *pxTaskStatusArray;
        volatile UBaseType_t uxArraySize, x;
        uint32_t ulTotalRunTime;
        
        // Get the number of tasks
        uxArraySize = uxTaskGetNumberOfTasks();
        
        // Allocate a TaskStatus_t structure for each task
        pxTaskStatusArray = pvPortMalloc(uxArraySize * sizeof(TaskStatus_t));
        
        if (pxTaskStatusArray != NULL) {
            // Generate raw status information about each task
            uxArraySize = uxTaskGetSystemState(pxTaskStatusArray, uxArraySize, &ulTotalRunTime);
            total_run_time = ulTotalRunTime;
            
            // Calculate run time for each core
            for (x = 0; x < uxArraySize; x++) {
                TaskStatus_t *task = &pxTaskStatusArray[x];
                
                // Check if it's the idle task
                if (strcmp(task->pcTaskName, "IDLE") == 0) {
                    // Core 0 idle task
                    if (task->xCoreID == 0) {
                        idle_run_time[0] = task->ulRunTimeCounter;
                    }
                    // Core 1 idle task
                    else if (task->xCoreID == 1) {
                        idle_run_time[1] = task->ulRunTimeCounter;
                    }
                } else {
                    // Add to the appropriate core's total
                    if (task->xCoreID == 0) {
                        task_run_time[0] += task->ulRunTimeCounter;
                    } else if (task->xCoreID == 1) {
                        task_run_time[1] += task->ulRunTimeCounter;
                    }
                }
                
                // Check stack high water mark for tasks on specific cores
                if (task->xCoreID == 0 || task->xCoreID == 1) {
                    if (task->usStackHighWaterMark < last_metrics.stack_high_water[task->xCoreID] || 
                        last_metrics.stack_high_water[task->xCoreID] == 0) {
                        last_metrics.stack_high_water[task->xCoreID] = task->usStackHighWaterMark;
                    }
                }
            }
            
            // Free the status array
            vPortFree(pxTaskStatusArray);
            
            // Calculate CPU usage if we have previous values
            if (total_run_time_prev > 0) {
                uint32_t delta_time = total_run_time - total_run_time_prev;
                uint32_t delta_idle0 = idle_run_time[0] - idle_run_time_prev[0];
                uint32_t delta_idle1 = idle_run_time[1] - idle_run_time_prev[1];
                
                // Calculate the CPU usage for each core and average them
                uint32_t usage0 = 100 - ((delta_idle0 * 100) / (delta_time / 2));
                uint32_t usage1 = 100 - ((delta_idle1 * 100) / (delta_time / 2));
                last_metrics.cpu_usage_percent = (usage0 + usage1) / 2;
            }
            
            // Store current values for next calculation
            task_run_time_prev[0] = task_run_time[0];
            task_run_time_prev[1] = task_run_time[1];
            idle_run_time_prev[0] = idle_run_time[0];
            idle_run_time_prev[1] = idle_run_time[1];
            total_run_time_prev = total_run_time;
        }
        
        // CPU temperature (example - not supported on all ESP32 versions/boards)
        // This is a placeholder - the actual implementation would depend on hardware support
        last_metrics.cpu_temperature = 45.0f;  // Fixed placeholder value
        
        // Periodically log the metrics (every 30 seconds)
        static uint32_t log_counter = 0;
        if (++log_counter >= 6) {  // 6 * 5000ms = 30 seconds
            system_monitor_print_metrics();
            log_counter = 0;
        }
        
        // Run health check
        esp_err_t health_result = system_monitor_health_check();
        if (health_result != ESP_OK) {
            ESP_LOGW(TAG, "Health check failed with error %s", esp_err_to_name(health_result));
        }
        
        // Sleep until next check
        vTaskDelay(pdMS_TO_TICKS(MONITOR_INTERVAL_MS));
    }
}
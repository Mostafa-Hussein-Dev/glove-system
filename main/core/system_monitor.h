#ifndef CORE_SYSTEM_MONITOR_H
#define CORE_SYSTEM_MONITOR_H

#include "esp_err.h"

/**
 * @brief System performance metrics
 */
typedef struct {
    uint32_t free_heap;            // Free heap memory in bytes
    uint32_t min_free_heap;        // Minimum free heap size since boot
    uint32_t cpu_usage_percent;    // CPU usage percentage (0-100)
    float cpu_temperature;         // CPU temperature in Celsius
    uint32_t task_count;           // Number of tasks running
    uint32_t stack_high_water[2];  // Minimum free stack for core 0 and 1
    uint64_t uptime_ms;            // System uptime in milliseconds
} system_metrics_t;

/**
 * @brief Initialize system monitor
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t system_monitor_init(void);

/**
 * @brief Get system metrics
 * 
 * @param metrics Pointer to store system metrics
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t system_monitor_get_metrics(system_metrics_t* metrics);

/**
 * @brief Print system metrics to log
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t system_monitor_print_metrics(void);

/**
 * @brief Run a system health check
 * 
 * @return ESP_OK if all checks pass, error code otherwise
 */
esp_err_t system_monitor_health_check(void);

/**
 * @brief Get system monitor task handle
 * 
 * @return Task handle for the system monitor task
 */
void* system_monitor_get_task_handle(void);

#endif /* CORE_SYSTEM_MONITOR_H */
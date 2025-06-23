#ifndef CORE_SYSTEM_MONITOR_H
#define CORE_SYSTEM_MONITOR_H

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/**
 * @brief System health levels
 */
typedef enum {
    SYSTEM_HEALTH_OK = 0,       // 🟢 All systems normal
    SYSTEM_HEALTH_WARNING = 1,  // 🟡 Attention needed
    SYSTEM_HEALTH_CRITICAL = 2  // 🔴 Take action now
} system_health_level_t;

/**
 * @brief Task health information
 */
typedef struct {
    char task_name[16];         // Task name
    uint32_t stack_free;        // Free stack bytes
    uint32_t stack_size;        // Total stack size
    uint32_t last_alive_ms;     // Last time task was seen alive
    bool is_alive;              // Task health status
    bool restart_requested;     // Recovery restart requested
} task_health_t;

/**
 * @brief Enhanced system performance metrics
 */
typedef struct {
    // Memory metrics
    uint32_t free_heap;            // Free heap memory in bytes
    uint32_t min_free_heap;        // Minimum free heap size since boot
    uint32_t total_heap;           // Total heap size
    uint32_t free_psram;           // Free PSRAM (if available)
    uint32_t total_psram;          // Total PSRAM (if available)
    
    // CPU and performance
    uint32_t cpu_usage_percent;    // CPU usage percentage (0-100)
    float cpu_temperature;         // CPU temperature in Celsius
    uint32_t cpu_frequency;        // CPU frequency in MHz
    
    // Task information
    uint32_t task_count;           // Number of tasks running
    task_health_t critical_tasks[8]; // Health of critical tasks
    uint32_t critical_task_count;  // Number of critical tasks monitored
    
    // System status
    uint64_t uptime_ms;            // System uptime in milliseconds
    system_health_level_t health_level; // Overall system health
    uint32_t error_count;          // Total error count since boot
    uint32_t recovery_count;       // Total recovery actions taken
    
    // Queue health (basic)
    uint32_t queue_overflows;      // Queue overflow events
    uint32_t queue_usage_max;      // Maximum queue usage percentage
    
    // Flash and storage
    uint32_t flash_operations;     // Number of flash operations
    uint32_t flash_errors;         // Flash operation errors
} system_metrics_t;

/**
 * @brief Recovery action types  
 */
typedef enum {
    RECOVERY_NONE = 0,
    RECOVERY_TASK_RESTART,
    RECOVERY_MEMORY_CLEANUP, 
    RECOVERY_SYSTEM_REBALANCE,
    RECOVERY_EMERGENCY_MODE
} recovery_action_t;

/**
 * @brief Initialize system monitor
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t system_monitor_init(void);

/**
 * @brief Get enhanced system metrics
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
 * @brief Run enhanced system health check
 * 
 * @return ESP_OK if all checks pass, error code otherwise
 */
esp_err_t system_monitor_health_check(void);

/**
 * @brief Get system health level
 * 
 * @return Current system health level
 */
system_health_level_t system_monitor_get_health_level(void);

/**
 * @brief Register a task for critical monitoring
 * 
 * @param task_handle Task handle to monitor
 * @param task_name Task name for identification
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t system_monitor_register_task(TaskHandle_t task_handle, const char* task_name);

/**
 * @brief Trigger a recovery action
 * 
 * @param action Recovery action to perform
 * @param target_task Task handle if action is task-specific (can be NULL)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t system_monitor_recovery_action(recovery_action_t action, TaskHandle_t target_task);

/**
 * @brief Update queue health statistics
 * 
 * @param queue_usage_percent Current queue usage percentage (0-100)
 * @param overflow_occurred Set to true if overflow just occurred
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t system_monitor_update_queue_health(uint32_t queue_usage_percent, bool overflow_occurred);

/**
 * @brief Get system monitor task handle
 * 
 * @return Task handle for the system monitor task
 */
void* system_monitor_get_task_handle(void);

/**
 * @brief Enable/disable adaptive monitoring intervals
 * 
 * @param enable true to enable adaptive intervals, false for fixed 5s
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t system_monitor_set_adaptive_intervals(bool enable);

#endif /* CORE_SYSTEM_MONITOR_H */
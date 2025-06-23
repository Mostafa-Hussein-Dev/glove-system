#ifndef QUEUE_MONITOR_H
#define QUEUE_MONITOR_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_err.h"

/**
 * @brief Queue health status
 */
typedef enum {
    QUEUE_HEALTH_GOOD = 0,
    QUEUE_HEALTH_WARNING,
    QUEUE_HEALTH_CRITICAL,
    QUEUE_HEALTH_FULL
} queue_health_t;

/**
 * @brief Queue statistics
 */
typedef struct {
    const char* name;
    QueueHandle_t handle;
    UBaseType_t max_items;
    UBaseType_t current_items;
    UBaseType_t peak_items;
    uint32_t overflow_count;
    uint32_t total_sends;
    uint32_t failed_sends;
    queue_health_t health;
} queue_stats_t;

/**
 * @brief Initialize queue monitoring system
 */
esp_err_t queue_monitor_init(void);

/**
 * @brief Register a queue for monitoring
 */
esp_err_t queue_monitor_register(const char* name, QueueHandle_t queue, UBaseType_t max_items);

/**
 * @brief Update queue statistics
 */
void queue_monitor_update(void);

/**
 * @brief Get queue statistics
 */
queue_stats_t* queue_monitor_get_stats(const char* name);

/**
 * @brief Print all queue statistics
 */
void queue_monitor_print_all(void);

#endif /* QUEUE_MONITOR_H */
#ifndef TASKS_POWER_TASK_H
#define TASKS_POWER_TASK_H

#include "esp_err.h"

/**
 * @brief Initialize the power task
 * 
 * This task handles power management, battery monitoring, and system 
 * state transitions for the sign language glove.
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t power_task_init(void);

/**
 * @brief Deinitialize the power task
 * 
 * Frees resources allocated by the power task.
 */
void power_task_deinit(void);

#endif /* TASKS_POWER_TASK_H */
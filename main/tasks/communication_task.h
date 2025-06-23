#ifndef TASKS_COMMUNICATION_TASK_H
#define TASKS_COMMUNICATION_TASK_H

#include "esp_err.h"

/**
 * @brief Initialize the communication task
 * 
 * This task handles the Bluetooth communication with the mobile app.
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t communication_task_init(void);

/**
 * @brief Deinitialize the communication task
 * 
 * Frees resources allocated by the communication task.
 */
void communication_task_deinit(void);

void* communication_task_get_handle(void);

#endif /* TASKS_COMMUNICATION_TASK_H */
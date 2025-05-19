#ifndef TASKS_PROCESSING_TASK_H
#define TASKS_PROCESSING_TASK_H

#include "esp_err.h"

/**
 * @brief Initialize the processing task
 * 
 * This task handles the processing of sensor data, feature extraction,
 * and gesture recognition.
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t processing_task_init(void);

/**
 * @brief Deinitialize the processing task
 * 
 * Frees resources allocated by the processing task.
 */
void processing_task_deinit(void);

#endif /* TASKS_PROCESSING_TASK_H */
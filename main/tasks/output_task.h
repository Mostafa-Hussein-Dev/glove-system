#ifndef TASKS_OUTPUT_TASK_H
#define TASKS_OUTPUT_TASK_H

#include "esp_err.h"

/**
 * @brief Initialize the output task
 * 
 * This task handles the output generation (display, audio, haptic)
 * based on recognition results.
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t output_task_init(void);

/**
 * @brief Deinitialize the output task
 * 
 * Frees resources allocated by the output task.
 */
void output_task_deinit(void);

void* output_task_get_handle(void);

#endif /* TASKS_OUTPUT_TASK_H */
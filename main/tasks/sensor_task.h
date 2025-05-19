#ifndef TASKS_SENSOR_TASK_H
#define TASKS_SENSOR_TASK_H

#include "esp_err.h"

/**
 * @brief Initialize the sensor task
 * 
 * This task handles the sampling of all sensors (flex, IMU, camera, touch)
 * and sends the data to the processing task.
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t sensor_task_init(void);

/**
 * @brief Deinitialize the sensor task
 * 
 * Frees resources allocated by the sensor task.
 */
void sensor_task_deinit(void);

#endif /* TASKS_SENSOR_TASK_H */
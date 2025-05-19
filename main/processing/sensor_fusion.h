#ifndef PROCESSING_SENSOR_FUSION_H
#define PROCESSING_SENSOR_FUSION_H

#include "esp_err.h"
#include "util/buffer.h"

/**
 * @brief Initialize sensor fusion module
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t sensor_fusion_init(void);

/**
 * @brief Deinitialize sensor fusion module
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t sensor_fusion_deinit(void);

/**
 * @brief Process sensor data through sensor fusion
 * 
 * @param new_data Pointer to current sensor data
 * @param data_buffer Buffer containing historical sensor data
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t sensor_fusion_process(sensor_data_t *new_data, sensor_data_buffer_t *data_buffer);

/**
 * @brief Get the latest fused sensor data
 * 
 * @param data Pointer to store fused sensor data
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t sensor_fusion_get_latest(sensor_data_t *data);

#endif /* PROCESSING_SENSOR_FUSION_H */
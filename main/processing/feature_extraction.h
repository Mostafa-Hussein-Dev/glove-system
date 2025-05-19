#ifndef PROCESSING_FEATURE_EXTRACTION_H
#define PROCESSING_FEATURE_EXTRACTION_H

#include "esp_err.h"
#include "util/buffer.h"

/**
 * @brief Initialize feature extraction module
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t feature_extraction_init(void);

/**
 * @brief Deinitialize feature extraction module
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t feature_extraction_deinit(void);

/**
 * @brief Extract features from sensor data
 * 
 * @param sensor_data Current sensor data
 * @param data_buffer Buffer containing historical sensor data
 * @param feature_vector Output feature vector
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t feature_extraction_process(sensor_data_t *sensor_data, 
                                   sensor_data_buffer_t *data_buffer, 
                                   feature_vector_t *feature_vector);

#endif /* PROCESSING_FEATURE_EXTRACTION_H */
#ifndef ML_DATA_PREPROCESSOR_H
#define ML_DATA_PREPROCESSOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "ml_types.h"
#include "util/buffer.h"
#include "esp_err.h"

/**
 * @brief Initialize data preprocessor
 * @return ESP_OK on success
 */
esp_err_t data_preprocessor_init(void);

/**
 * @brief Preprocess sensor data for static gesture model
 * @param sensor_data Current sensor reading
 * @param ml_input Output ML input structure
 * @return ESP_OK on success
 */
esp_err_t data_preprocessor_static(const sensor_data_t* sensor_data, ml_input_t* ml_input);

/**
 * @brief Preprocess sensor data for dynamic gesture model
 * @param sensor_buffer Buffer of historical sensor data
 * @param ml_input Output ML input structure
 * @return ESP_OK on success
 */
esp_err_t data_preprocessor_dynamic(const sensor_data_buffer_t* sensor_buffer, ml_input_t* ml_input);

/**
 * @brief Normalize feature vector
 * @param features Input/output feature array
 * @param count Number of features
 * @return ESP_OK on success
 */
esp_err_t data_preprocessor_normalize(float* features, uint32_t count);

#ifdef __cplusplus
}
#endif

#endif /* ML_DATA_PREPROCESSOR_H */
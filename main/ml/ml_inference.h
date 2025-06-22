#ifndef ML_ML_INFERENCE_H
#define ML_ML_INFERENCE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "ml_types.h"
#include "esp_err.h"

/**
 * @brief Initialize ML inference engine
 * @return ESP_OK on success
 */
esp_err_t ml_inference_init(void);

/**
 * @brief Run inference on preprocessed data
 * @param ml_input Preprocessed input data
 * @param model_type Type of model to use
 * @param result Output inference result
 * @return ESP_OK on success
 */
esp_err_t ml_inference_run(const ml_input_t* ml_input, model_type_t model_type, ml_result_t* result);

/**
 * @brief Run hybrid inference (combine static and dynamic models)
 * @param static_input Static gesture features
 * @param dynamic_input Dynamic gesture features
 * @param result Combined inference result
 * @return ESP_OK on success
 */
esp_err_t ml_inference_hybrid(const ml_input_t* static_input, 
                              const ml_input_t* dynamic_input, 
                              ml_result_t* result);

/**
 * @brief Get gesture name by ID
 * @param gesture_id Gesture ID
 * @return Gesture name string
 */
const char* ml_inference_get_gesture_name(uint32_t gesture_id);

#ifdef __cplusplus
}
#endif

#endif /* ML_ML_INFERENCE_H */
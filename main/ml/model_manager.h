#ifndef ML_MODEL_MANAGER_H
#define ML_MODEL_MANAGER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "ml_types.h"
#include "esp_err.h"

/**
 * @brief Initialize the ML model manager
 * @return ESP_OK on success
 */
esp_err_t model_manager_init(void);

/**
 * @brief Load a model from SPIFFS
 * @param model_name Name of the model file
 * @param model_type Type of model
 * @return ESP_OK on success
 */
esp_err_t model_manager_load_model(const char* model_name, model_type_t model_type);

/**
 * @brief Get loaded model by type
 * @param type Model type
 * @return Pointer to model or NULL if not loaded
 */
ml_model_t* model_manager_get_model(model_type_t type);

/**
 * @brief Check if model is ready for inference
 * @param type Model type
 * @return true if ready
 */
bool model_manager_is_ready(model_type_t type);

/**
 * @brief Unload all models and free memory
 * @return ESP_OK on success
 */
esp_err_t model_manager_cleanup(void);

#ifdef __cplusplus
}
#endif

#endif /* ML_MODEL_MANAGER_H */
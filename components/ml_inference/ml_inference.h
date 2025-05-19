#ifndef ML_INFERENCE_H
#define ML_INFERENCE_H

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

/**
 * @brief ML model types for different gesture recognition approaches
 */
typedef enum {
    ML_MODEL_STATIC_GESTURES = 0,  // For static hand poses
    ML_MODEL_DYNAMIC_GESTURES,     // For motion-based gestures
    ML_MODEL_COUNT                 // Number of model types
} ml_model_type_t;

/**
 * @brief Input feature structure for ML inference
 */
typedef struct {
    float features[100];     // Input feature vector
    uint16_t feature_count;  // Number of valid features
} ml_input_features_t;

/**
 * @brief Result structure from ML inference
 */
typedef struct {
    uint8_t gesture_id;      // Recognized gesture ID
    char gesture_name[32];   // Gesture name
    float confidence;        // Confidence score (0-1)
    bool is_valid;           // Whether the result is valid
} ml_result_t;

/**
 * @brief Initialize ML inference subsystem
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t ml_inference_init(void);

/**
 * @brief Deinitialize ML inference subsystem
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t ml_inference_deinit(void);

/**
 * @brief Run inference on input features
 * 
 * @param model_type Type of model to use for inference
 * @param features Input feature vector
 * @param result Pointer to store inference result
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t ml_inference_run(ml_model_type_t model_type, const ml_input_features_t* features, ml_result_t* result);

/**
 * @brief Load a model from storage
 * 
 * @param model_type Type of model to load
 * @param path Path to model file
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t ml_inference_load_model(ml_model_type_t model_type, const char* path);

/**
 * @brief Get inference statistics
 * 
 * @param model_type Type of model
 * @param inference_time_ms Pointer to store average inference time
 * @param accuracy Pointer to store accuracy (if available)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t ml_inference_get_stats(ml_model_type_t model_type, float* inference_time_ms, float* accuracy);

/**
 * @brief Set inference parameters
 * 
 * @param model_type Type of model
 * @param confidence_threshold Minimum confidence threshold (0-1)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t ml_inference_set_params(ml_model_type_t model_type, float confidence_threshold);

#endif /* ML_INFERENCE_H */
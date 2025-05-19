#ifndef PROCESSING_GESTURE_DETECTION_H
#define PROCESSING_GESTURE_DETECTION_H

#include "esp_err.h"
#include "util/buffer.h"

/**
 * @brief Initialize gesture detection module
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t gesture_detection_init(void);

/**
 * @brief Deinitialize gesture detection module
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t gesture_detection_deinit(void);

/**
 * @brief Process feature vector to detect gestures
 * 
 * @param feature_vector Input feature vector
 * @param result Output processing result
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t gesture_detection_process(feature_vector_t *feature_vector, processing_result_t *result);

/**
 * @brief Add a new gesture template
 * 
 * @param name Gesture name
 * @param features Feature vector template
 * @param is_dynamic Whether this is a dynamic gesture
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t gesture_detection_add_template(const char *name, feature_vector_t *features, bool is_dynamic);

#endif /* PROCESSING_GESTURE_DETECTION_H */
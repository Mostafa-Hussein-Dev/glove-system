#ifndef GESTURE_TEMPLATES_H
#define GESTURE_TEMPLATES_H

#include "esp_err.h"
#include "util/buffer.h"

// Define the maximum number of gesture templates
#define MAX_GESTURE_TEMPLATES 50

/**
 * @brief Gesture template structure
 */
typedef struct {
    char name[32];               // Gesture name
    float features[FEATURE_BUFFER_SIZE];  // Template feature vector
    uint16_t feature_count;      // Number of features used
    bool is_dynamic;             // Static vs dynamic gesture
    float confidence_threshold;  // Minimum confidence for detection
} gesture_template_t;

/**
 * @brief Initialize the gesture templates
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t gesture_templates_init(void);

/**
 * @brief Load gesture templates from storage
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t gesture_templates_load(void);

/**
 * @brief Save gesture templates to storage
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t gesture_templates_save(void);

/**
 * @brief Add or update a gesture template
 * 
 * @param name Gesture name
 * @param features Feature vector
 * @param feature_count Number of features
 * @param is_dynamic Whether this is a dynamic gesture
 * @param confidence_threshold Confidence threshold for this gesture
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t gesture_templates_add(const char* name, const float* features, 
                              uint16_t feature_count, bool is_dynamic,
                              float confidence_threshold);

/**
 * @brief Get a gesture template by name
 * 
 * @param name Gesture name
 * @param template Pointer to store the template
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t gesture_templates_get_by_name(const char* name, gesture_template_t* template);

/**
 * @brief Get a gesture template by index
 * 
 * @param index Template index
 * @param template Pointer to store the template
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t gesture_templates_get_by_index(uint8_t index, gesture_template_t* template);

/**
 * @brief Get the number of loaded templates
 * 
 * @return Number of templates
 */
uint8_t gesture_templates_get_count(void);

/**
 * @brief Reset all templates to default
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t gesture_templates_reset(void);

#endif /* GESTURE_TEMPLATES_H */
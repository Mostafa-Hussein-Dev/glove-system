#ifndef OUTPUT_TEXT_GENERATION_H
#define OUTPUT_TEXT_GENERATION_H

#include "esp_err.h"
#include <stddef.h>
#include "util/buffer.h"

/**
 * @brief Initialize text generation module
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t text_generation_init(void);

/**
 * @brief Deinitialize text generation module
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t text_generation_deinit(void);

/**
 * @brief Generate text from gesture recognition result
 * 
 * @param result Processing result from gesture detection
 * @param output_text Buffer to store generated text
 * @param max_length Maximum length of output text
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t text_generation_generate_text(processing_result_t *result, char *output_text, size_t max_length);

/**
 * @brief Get the current text
 * 
 * @param output_text Buffer to store current text
 * @param max_length Maximum length of output text
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t text_generation_get_current_text(char *output_text, size_t max_length);

/**
 * @brief Clear the current text
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t text_generation_clear_text(void);

#endif /* OUTPUT_TEXT_GENERATION_H */
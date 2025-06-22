#ifndef UTIL_FILE_UPLOAD_H
#define UTIL_FILE_UPLOAD_H

#include "esp_err.h"

/**
 * @brief Initialize file upload handler
 * @return ESP_OK on success
 */
esp_err_t file_upload_init(void);

/**
 * @brief Process serial commands for file upload
 * @param line Serial input line
 * @return ESP_OK if command was processed
 */
esp_err_t file_upload_process_command(const char* line);

#endif /* UTIL_FILE_UPLOAD_H */
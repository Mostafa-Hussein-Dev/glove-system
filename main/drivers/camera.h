#ifndef DRIVERS_CAMERA_H
#define DRIVERS_CAMERA_H

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Camera resolution
 */
typedef enum {
    CAMERA_RESOLUTION_QQVGA = 0,  // 160x120
    CAMERA_RESOLUTION_QVGA,       // 320x240
    CAMERA_RESOLUTION_VGA         // 640x480
} camera_resolution_t;

/**
 * @brief Camera pixel format
 */
typedef enum {
    CAMERA_FORMAT_RGB565 = 0,
    CAMERA_FORMAT_JPEG
} camera_format_t;

/**
 * @brief Camera frame structure
 */
typedef struct {
    uint8_t *buffer;        // Pointer to image data
    uint32_t buffer_size;   // Size of the buffer
    uint16_t width;         // Image width
    uint16_t height;        // Image height
    camera_format_t format; // Pixel format
    uint32_t timestamp;     // Acquisition timestamp (ms)
} camera_frame_t;

/**
 * @brief Camera information structure
 */
typedef struct {
    camera_resolution_t resolution; // Current resolution
    uint16_t width;                // Current width
    uint16_t height;               // Current height
} camera_info_t;

/**
 * @brief Initialize camera
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t camera_init(void);

/**
 * @brief Deinitialize camera
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t camera_deinit(void);

/**
 * @brief Set camera resolution
 * 
 * @param resolution Desired resolution
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t camera_set_resolution(camera_resolution_t resolution);

/**
 * @brief Capture a single frame
 * 
 * @param frame Pointer to store frame information
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t camera_capture_frame(camera_frame_t *frame);

/**
 * @brief Release the current frame
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t camera_release_frame(void);

/**
 * @brief Get camera information
 * 
 * @param info Pointer to store camera information
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t camera_get_info(camera_info_t *info);

/**
 * @brief Start camera streaming mode
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t camera_start_streaming(void);

/**
 * @brief Stop camera streaming mode
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t camera_stop_streaming(void);

/**
 * @brief Check if camera is in streaming mode
 * 
 * @return true if streaming, false otherwise
 */
bool camera_is_streaming(void);

#endif /* DRIVERS_CAMERA_H */
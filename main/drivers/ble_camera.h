#ifndef DRIVERS_BLE_CAMERA_H
#define DRIVERS_BLE_CAMERA_H

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

/**
 * @brief BLE Camera frame structure
 */
typedef struct {
    uint8_t *buffer;        // Pointer to image data  
    uint32_t buffer_size;   // Size of the buffer
    uint16_t width;         // Image width
    uint16_t height;        // Image height
    uint8_t format;         // Pixel format (0=RGB565, 1=JPEG)
    uint32_t timestamp;     // Acquisition timestamp (ms)
    uint32_t sequence;      // Frame sequence number
    bool valid;             // Data validity flag
} ble_camera_frame_t;

/**
 * @brief BLE Camera status
 */
typedef enum {
    BLE_CAMERA_STATUS_DISCONNECTED = 0,
    BLE_CAMERA_STATUS_CONNECTING,
    BLE_CAMERA_STATUS_CONNECTED,
    BLE_CAMERA_STATUS_STREAMING,
    BLE_CAMERA_STATUS_ERROR
} ble_camera_status_t;

/**
 * @brief BLE Camera statistics
 */
typedef struct {
    uint32_t frames_received;
    uint32_t frames_dropped;
    uint32_t bytes_received;
    uint32_t last_frame_time;
    float frame_rate;
    int8_t signal_strength; // RSSI
} ble_camera_stats_t;

// Function declarations
esp_err_t ble_camera_init(void);
esp_err_t ble_camera_deinit(void);
esp_err_t ble_camera_connect(const char* device_name);
esp_err_t ble_camera_disconnect(void);
esp_err_t ble_camera_start_streaming(void);
esp_err_t ble_camera_stop_streaming(void);
esp_err_t ble_camera_capture_frame(ble_camera_frame_t *frame);
esp_err_t ble_camera_release_frame(void);
esp_err_t ble_camera_get_status(ble_camera_status_t *status);
esp_err_t ble_camera_get_stats(ble_camera_stats_t *stats);
esp_err_t ble_camera_set_resolution(uint8_t width, uint8_t height);
esp_err_t ble_camera_set_quality(uint8_t quality); // 0-63 for JPEG
bool ble_camera_is_connected(void);
bool ble_camera_frame_available(void);

#endif /* DRIVERS_BLE_CAMERA_H */
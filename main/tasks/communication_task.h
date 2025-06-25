#ifndef TASKS_COMMUNICATION_TASK_H
#define TASKS_COMMUNICATION_TASK_H

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Initialize the communication task
 * 
 * This task handles the Bluetooth communication with the mobile app.
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t communication_task_init(void);

/**
 * @brief Deinitialize the communication task
 * 
 * Frees resources allocated by the communication task.
 */
esp_err_t  communication_task_deinit(void);

/**
 * @brief Start communication services
 * 
 * Enables BLE advertising and starts camera connection attempts.
 * Call this after system initialization is complete.
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t communication_task_start(void);

/**
 * @brief Stop communication services
 * 
 * Disables BLE advertising and disconnects camera.
 * Call this for clean shutdown or power saving.
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t communication_task_stop(void);

/**
 * @brief Send gesture information via BLE
 * 
 * Queues a gesture notification to be sent to connected BLE clients.
 * 
 * @param gesture_id Unique identifier for the gesture (0-255)
 * @param gesture_name Human-readable name of the gesture (max 31 chars)
 * @param confidence Confidence level of recognition (0.0-1.0)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t communication_send_gesture(uint8_t gesture_id, const char *gesture_name, float confidence);

/**
 * @brief Send text message via BLE
 * 
 * Queues a text notification to be sent to connected BLE clients.
 * Useful for sending recognized gesture sequences or system messages.
 * 
 * @param text Text message to send (max 255 chars)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t communication_send_text(const char *text);

/**
 * @brief Send system status via BLE
 * 
 * Queues a status notification with battery level, system state, and error codes.
 * 
 * @param battery_level Battery percentage (0-100)
 * @param state System state (0=idle, 1=active, 2=error, etc.)
 * @param error Error code (0=no error, >0=specific error codes)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t communication_send_status(uint8_t battery_level, uint8_t state, uint8_t error);

/**
 * @brief Send debug message via BLE
 * 
 * Queues a debug notification for development and troubleshooting.
 * Only sent if debug notifications are enabled by the client.
 * 
 * @param message Debug message to send (max 127 chars)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t communication_send_debug(const char *message);

/**
 * @brief Request camera frame capture
 * 
 * Triggers the ESP32-CAM to capture a single frame for gesture recognition.
 * The frame will be processed asynchronously when received.
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t communication_capture_camera_frame(void);

/**
 * @brief Check if BLE service is connected to a client
 * 
 * @return true if a BLE client is connected, false otherwise
 */
bool communication_is_ble_connected(void);

/**
 * @brief Check if ESP32-CAM is connected
 * 
 * @return true if ESP32-CAM is connected via BLE, false otherwise
 */
bool communication_is_camera_connected(void);

/**
 * @brief Get camera connection statistics
 * 
 * Retrieves statistics about the camera connection including frame rates,
 * dropped frames, and signal strength.
 * 
 * @param frames_received Pointer to store total frames received
 * @param frames_dropped Pointer to store total frames dropped
 * @param frame_rate Pointer to store current frame rate (FPS)
 * @param signal_strength Pointer to store RSSI in dBm
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t communication_get_camera_stats(uint32_t *frames_received, uint32_t *frames_dropped, 
                                        float *frame_rate, int8_t *signal_strength);

/**
 * @brief Force camera reconnection
 * 
 * Disconnects from the current ESP32-CAM (if connected) and attempts
 * to establish a new connection. Useful for error recovery.
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t communication_reconnect_camera(void);

/**
 * @brief Enable/disable camera streaming mode
 * 
 * When enabled, the ESP32-CAM will continuously stream frames at ~5 FPS.
 * When disabled, frames are only captured on demand.
 * 
 * @param enable true to enable streaming, false to disable
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t communication_set_camera_streaming(bool enable);

/**
 * @brief Set camera capture quality
 * 
 * Adjusts the JPEG quality of captured frames. Lower values = higher compression.
 * 
 * @param quality JPEG quality (0-63, where 12 is typical for gesture recognition)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t communication_set_camera_quality(uint8_t quality);

/**
 * @brief Set camera resolution
 * 
 * Changes the capture resolution of the ESP32-CAM.
 * Note: Higher resolutions require more bandwidth and processing time.
 * 
 * @param width Image width (e.g., 320 for QVGA)
 * @param height Image height (e.g., 240 for QVGA)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t communication_set_camera_resolution(uint8_t width, uint8_t height);

/**
 * @brief Register callback for received camera frames
 * 
 * Registers a callback function that will be called whenever a complete
 * camera frame is received from the ESP32-CAM.
 * 
 * @param callback Function pointer to frame processing callback
 * @return ESP_OK on success, error code otherwise
 */
typedef void (*camera_frame_callback_t)(const uint8_t *frame_data, uint32_t frame_size, 
                                        uint16_t width, uint16_t height, uint32_t timestamp);
esp_err_t communication_register_frame_callback(camera_frame_callback_t callback);

/**
 * @brief Register callback for BLE command processing
 * 
 * Registers a callback function that will be called whenever a command
 * is received from a connected BLE client. This allows the application
 * to extend the command set beyond the built-in commands.
 * 
 * @param callback Function pointer to command processing callback
 * @return ESP_OK on success, error code otherwise
 */
typedef void (*communication_ble_callback_t)(uint8_t command_id, const uint8_t *data, size_t length);
esp_err_t communication_register_ble_callback(communication_ble_callback_t  callback);

/**
 * @brief Communication task configuration structure
 */
typedef struct {
    bool auto_connect_camera;       // Automatically connect to ESP32-CAM on startup
    bool enable_debug_notifications; // Enable debug message notifications
    uint16_t camera_retry_interval; // Retry interval for camera connection (ms)
    uint8_t default_camera_quality; // Default JPEG quality (0-63)
    const char *camera_device_name; // ESP32-CAM device name to connect to
} communication_config_t;

/**
 * @brief Set communication task configuration
 * 
 * Updates the configuration for the communication task. Should be called
 * before communication_task_start() for settings to take effect.
 * 
 * @param config Pointer to configuration structure
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t communication_set_config(const communication_config_t *config);

/**
 * @brief Get current communication task configuration
 * 
 * Retrieves the current configuration settings.
 * 
 * @param config Pointer to configuration structure to fill
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t communication_get_config(communication_config_t *config);


void* communication_task_get_handle(void);

#endif /* TASKS_COMMUNICATION_TASK_H */
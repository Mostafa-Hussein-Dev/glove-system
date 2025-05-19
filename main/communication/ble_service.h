#ifndef COMMUNICATION_BLE_SERVICE_H
#define COMMUNICATION_BLE_SERVICE_H

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

/**
 * @brief BLE service notification types
 */
typedef enum {
    BLE_NOTIFY_GESTURE, // Recognized gesture notification
    BLE_NOTIFY_TEXT,    // Generated text notification
    BLE_NOTIFY_STATUS,  // System status notification
    BLE_NOTIFY_DEBUG    // Debug information notification
} ble_notification_type_t;

/**
 * @brief Initialize BLE service
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t ble_service_init(void);

/**
 * @brief Deinitialize BLE service
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t ble_service_deinit(void);

/**
 * @brief Enable BLE service
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t ble_service_enable(void);

/**
 * @brief Disable BLE service
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t ble_service_disable(void);

/**
 * @brief Check if BLE service is connected
 * 
 * @param connected Pointer to store connection status
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t ble_service_is_connected(bool *connected);

/**
 * @brief Send gesture data over BLE
 * 
 * @param gesture_id Gesture ID
 * @param gesture_name Gesture name
 * @param confidence Confidence level (0-1)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t ble_service_send_gesture(uint8_t gesture_id, const char *gesture_name, float confidence);

/**
 * @brief Send recognized text over BLE
 * 
 * @param text Text to send
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t ble_service_send_text(const char *text);

/**
 * @brief Send system status over BLE
 * 
 * @param battery_level Battery level (0-100)
 * @param state System state
 * @param error Error code
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t ble_service_send_status(uint8_t battery_level, uint8_t state, uint8_t error);

/**
 * @brief Send debug information over BLE
 * 
 * @param data Debug message
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t ble_service_send_debug(const char *data);

/**
 * @brief Process received BLE command
 * 
 * @param data Command data
 * @param length Command data length
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t ble_service_process_command(const uint8_t *data, size_t length);

/**
 * @brief Register callback for BLE commands
 * 
 * @param callback Function pointer to callback
 * @return ESP_OK on success, error code otherwise
 */
typedef void (*ble_command_callback_t)(const uint8_t *data, size_t length);
esp_err_t ble_service_register_command_callback(ble_command_callback_t callback);

#endif /* COMMUNICATION_BLE_SERVICE_H */
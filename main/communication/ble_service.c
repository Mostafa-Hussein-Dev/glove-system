// Replace the entire ble_service.c content with this stub implementation:

#include "communication/ble_service.h"
#include "esp_log.h"

static const char *TAG = "BLE_SERVICE";

esp_err_t ble_service_init(void) {
    ESP_LOGI(TAG, "BLE service disabled (stub implementation)");
    return ESP_OK;
}

esp_err_t ble_service_deinit(void) {
    return ESP_OK;
}

esp_err_t ble_service_enable(void) {
    return ESP_OK;
}

esp_err_t ble_service_disable(void) {
    return ESP_OK;
}

esp_err_t ble_service_is_connected(bool *connected) {
    if (connected) *connected = false;
    return ESP_OK;
}

esp_err_t ble_service_send_gesture(uint8_t gesture_id, const char *gesture_name, float confidence) {
    return ESP_OK;
}

esp_err_t ble_service_send_text(const char *text) {
    return ESP_OK;
}

esp_err_t ble_service_send_status(uint8_t battery_level, uint8_t state, uint8_t error) {
    return ESP_OK;
}

esp_err_t ble_service_send_debug(const char *data) {
    return ESP_OK;
}

esp_err_t ble_service_process_command(const uint8_t *data, size_t length) {
    return ESP_OK;
}

esp_err_t ble_service_register_command_callback(ble_command_callback_t callback) {
    return ESP_OK;
}
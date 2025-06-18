#include "util/i2c_utils.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static const char *TAG = "I2C_UTILS";

esp_err_t i2c_create_device(uint8_t device_address, uint32_t scl_speed_hz, i2c_master_dev_handle_t *dev_handle) {
    if (dev_handle == NULL || i2c_master_bus == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = device_address,
        .scl_speed_hz = scl_speed_hz,
    };
    
    esp_err_t ret = i2c_master_bus_add_device(i2c_master_bus, &dev_cfg, dev_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add I2C device 0x%02x: %s", device_address, esp_err_to_name(ret));
    }
    
    return ret;
}

esp_err_t i2c_write_device(i2c_master_dev_handle_t dev_handle, const uint8_t *write_buffer, size_t write_size, int timeout_ms) {
    esp_err_t ret;
    
    if (g_i2c_mutex == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Take I2C mutex with timeout
    if (xSemaphoreTake(g_i2c_mutex, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    
    return i2c_master_transmit(dev_handle, write_buffer, write_size, timeout_ms);

    // Always release mutex
    xSemaphoreGive(g_i2c_mutex);
    return ret;
}

esp_err_t i2c_read_device(i2c_master_dev_handle_t dev_handle, uint8_t *read_buffer, size_t read_size, int timeout_ms) {
    esp_err_t ret;
    
    if (g_i2c_mutex == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Take I2C mutex with timeout
    if (xSemaphoreTake(g_i2c_mutex, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    return i2c_master_receive(dev_handle, read_buffer, read_size, timeout_ms);

    // Always release mutex
    xSemaphoreGive(g_i2c_mutex);
    return ret;
}

esp_err_t i2c_write_read_device(i2c_master_dev_handle_t dev_handle, const uint8_t *write_buffer, size_t write_size, uint8_t *read_buffer, size_t read_size, int timeout_ms) {
    if (dev_handle == NULL || write_buffer == NULL || read_buffer == NULL || write_size == 0 || read_size == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    
    return i2c_master_transmit_receive(dev_handle, write_buffer, write_size, read_buffer, read_size, timeout_ms);
}
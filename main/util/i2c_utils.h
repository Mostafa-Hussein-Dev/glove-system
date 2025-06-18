#ifndef UTIL_I2C_UTILS_H
#define UTIL_I2C_UTILS_H

#include "esp_err.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

// Global I2C master bus handle
extern i2c_master_bus_handle_t i2c_master_bus;

//Global I2C mutex
extern SemaphoreHandle_t g_i2c_mutex;

/**
 * @brief Create an I2C device handle for a specific device
 * 
 * @param device_address I2C device address (7-bit)
 * @param scl_speed_hz SCL frequency for this device
 * @param dev_handle Pointer to store the device handle
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t i2c_create_device(uint8_t device_address, uint32_t scl_speed_hz, i2c_master_dev_handle_t *dev_handle);

/**
 * @brief Write data to I2C device
 * 
 * @param dev_handle Device handle
 * @param write_buffer Data to write
 * @param write_size Number of bytes to write
 * @param timeout_ms Timeout in milliseconds
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t i2c_write_device(i2c_master_dev_handle_t dev_handle, const uint8_t *write_buffer, size_t write_size, int timeout_ms);

/**
 * @brief Read data from I2C device
 * 
 * @param dev_handle Device handle
 * @param read_buffer Buffer to store read data
 * @param read_size Number of bytes to read
 * @param timeout_ms Timeout in milliseconds
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t i2c_read_device(i2c_master_dev_handle_t dev_handle, uint8_t *read_buffer, size_t read_size, int timeout_ms);

/**
 * @brief Write then read from I2C device
 * 
 * @param dev_handle Device handle
 * @param write_buffer Data to write
 * @param write_size Number of bytes to write
 * @param read_buffer Buffer to store read data
 * @param read_size Number of bytes to read
 * @param timeout_ms Timeout in milliseconds
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t i2c_write_read_device(i2c_master_dev_handle_t dev_handle, const uint8_t *write_buffer, size_t write_size, uint8_t *read_buffer, size_t read_size, int timeout_ms);

#endif /* UTIL_I2C_UTILS_H */
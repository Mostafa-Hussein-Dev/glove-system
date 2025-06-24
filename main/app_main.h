#ifndef APP_MAIN_H
#define APP_MAIN_H

#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "esp_mac.h"
#include "config/system_config.h"
#include "driver/i2c_master.h"

/**
 * @brief Global system configuration
 */
extern system_config_t g_system_config;

/**
 * @brief Global I2C master bus handle
 */
extern i2c_master_bus_handle_t i2c_master_bus;

/**
 * @brief Global queue handlers
 */
extern QueueHandle_t g_sensor_data_queue;          // Queue for sensor data
extern QueueHandle_t g_processing_result_queue;    // Queue for processing results
extern QueueHandle_t g_output_command_queue;       // Queue for output commands
extern QueueHandle_t g_system_command_queue;       // Queue for system commands

/**
 * @brief Event group for system synchronization
 */
extern EventGroupHandle_t g_system_event_group;

/**
 * @brief Initialize the application
 * 
 * This function initializes all subsystems, creates queues and tasks
 * and starts the system.
 * 
 * @return ESP_OK on success, or an error code on failure
 */
esp_err_t app_init(void);

/**
 * @brief Check if the system is running in debug mode
 * 
 * @return true if debug mode is enabled, false otherwise
 */
bool app_is_debug_mode(void);

/**
 * @brief Enable debug mode at runtime
 * 
 * This function can be called to switch to debug mode during runtime.
 * Note: This will stop all running tasks and switch to debug mode.
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t app_enable_debug_mode(void);

/**
 * @brief Disable debug mode and start normal operation
 * 
 * This function switches from debug mode to normal operation mode.
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t app_disable_debug_mode(void);

#endif /* APP_MAIN_H */
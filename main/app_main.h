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
 * @brief System event bits for the event group
 */
#define SYSTEM_EVENT_INIT_COMPLETE   (1 << 0)
#define SYSTEM_EVENT_SENSOR_READY    (1 << 1)
#define SYSTEM_EVENT_PROCESSING_READY (1 << 2)
#define SYSTEM_EVENT_OUTPUT_READY    (1 << 3)
#define SYSTEM_EVENT_BLE_READY       (1 << 4)
#define SYSTEM_EVENT_POWER_READY     (1 << 5)
#define SYSTEM_EVENT_LOW_BATTERY     (1 << 6)
#define SYSTEM_EVENT_ERROR           (1 << 7)
#define SYSTEM_EVENT_CALIBRATION     (1 << 8)

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

#endif /* APP_MAIN_H */
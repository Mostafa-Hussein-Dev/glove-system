#ifndef CORE_POWER_MANAGEMENT_H
#define CORE_POWER_MANAGEMENT_H

#include "esp_err.h"
#include "config/system_config.h"

/**
 * @brief Power modes for the system
 */
typedef enum {
    POWER_MODE_PERFORMANCE,    // Maximum performance, no power saving
    POWER_MODE_BALANCED,       // Balance between performance and power saving
    POWER_MODE_POWER_SAVE,     // Prioritize power saving
    POWER_MODE_MAX_POWER_SAVE  // Maximum power saving
} power_mode_t;

/**
 * @brief Battery status
 */
typedef struct {
    uint16_t voltage_mv;       // Battery voltage in millivolts
    uint8_t percentage;        // Battery percentage (0-100)
    bool is_charging;          // Whether the battery is charging
    bool is_low;               // Whether the battery is low
    bool is_critical;          // Whether the battery is critically low
} battery_status_t;

/**
 * @brief Initialize power management subsystem
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t power_management_init(void);

/**
 * @brief Set power mode
 * 
 * @param mode Power mode
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t power_management_set_mode(power_mode_t mode);

/**
 * @brief Get current power mode
 * 
 * @return Current power mode
 */
power_mode_t power_management_get_mode(void);

/**
 * @brief Get battery status
 * 
 * @param status Pointer to store battery status
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t power_management_get_battery_status(battery_status_t* status);

/**
 * @brief Enter light sleep mode
 * 
 * @param sleep_duration_ms Sleep duration in milliseconds (0 for indefinite)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t power_management_light_sleep(uint32_t sleep_duration_ms);

/**
 * @brief Enter deep sleep mode
 * 
 * @param sleep_duration_ms Sleep duration in milliseconds (0 for indefinite)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t power_management_deep_sleep(uint32_t sleep_duration_ms);

/**
 * @brief Wake up from sleep mode
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t power_management_wake_up(void);

/**
 * @brief Set CPU frequency
 * 
 * @param frequency_mhz Frequency in MHz
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t power_management_set_cpu_frequency(uint32_t frequency_mhz);

/**
 * @brief Enable/disable peripheral power
 * 
 * @param peripheral Peripheral type
 * @param enable Whether to enable or disable
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t power_management_set_peripheral_power(uint8_t peripheral, bool enable);

/**
 * @brief Process inactivity
 * 
 * This function should be called periodically to check for inactivity
 * and enter power saving mode if needed.
 * 
 * @param current_time_ms Current system time in milliseconds
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t power_management_process_inactivity(uint32_t current_time_ms);

/**
 * @brief Reset inactivity timer
 * 
 * This function should be called when user activity is detected.
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t power_management_reset_inactivity_timer(void);

#endif /* CORE_POWER_MANAGEMENT_H */
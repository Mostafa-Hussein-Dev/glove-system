#ifndef CORE_POWER_MANAGEMENT_H
#define CORE_POWER_MANAGEMENT_H

#include "esp_err.h"
#include "config/system_config.h"

/**
 * @brief Simplified Power modes for the system
 */
typedef enum {
    POWER_MODE_PERFORMANCE,    // Maximum performance - 240MHz, all peripherals ON
    POWER_MODE_BALANCED,       // Balance mode - 160MHz, smart peripheral control
    POWER_MODE_POWER_SAVE      // Power saving - 80MHz, minimal peripherals
} power_mode_t;

/**
 * @brief Battery status
 */
typedef struct {
    uint16_t voltage_mv;       // Battery voltage in millivolts
    uint8_t percentage;        // Battery percentage (0-100)
    bool is_charging;          // Whether USB is connected and charging
    bool is_low;               // Below 20% (3.4V)
    bool is_critical;          // Below 10% (3.3V)
} battery_status_t;

/**
 * @brief Sleep detection result
 */
typedef enum {
    SLEEP_ACTION_NONE,         // Stay awake
    SLEEP_ACTION_LIGHT,        // Enter light sleep
    SLEEP_ACTION_DEEP          // Enter deep sleep
} sleep_action_t;

/**
 * @brief Initialize power management subsystem
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t power_management_init(void);

/**
 * @brief Set power mode (simplified)
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
 * @brief Get battery status with proper ADC reading
 * 
 * @param status Pointer to store battery status
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t power_management_get_battery_status(battery_status_t* status);

/**
 * @brief Check if system should sleep (simplified logic)
 * 
 * @param inactive_time_ms Time since last activity
 * @param low_battery True if battery is low
 * @return Sleep action to take
 */
sleep_action_t power_management_check_sleep(uint32_t inactive_time_ms, bool low_battery);

/**
 * @brief Enter light sleep mode
 * 
 * @param wakeup_time_ms Wake up after this time (0 = wait for interrupt)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t power_management_light_sleep(uint32_t wakeup_time_ms);

/**
 * @brief Enter deep sleep mode
 * 
 * @param wakeup_time_ms Wake up after this time (0 = indefinite)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t power_management_deep_sleep(uint32_t wakeup_time_ms);

/**
 * @brief Reset activity timer (call when user activity detected)
 */
void power_management_reset_activity(void);

/**
 * @brief Get time since last activity
 * 
 * @return Time in milliseconds since last activity
 */
uint32_t power_management_get_inactive_time(void);

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
#ifndef DEFAULT_CONFIG_H
#define DEFAULT_CONFIG_H

#include "config/system_config.h"

/**
 * @brief Default system configuration
 * 
 * This struct contains default values for system configuration.
 * It is used to initialize the system when no saved configuration is found.
 */
static const system_config_t DEFAULT_SYSTEM_CONFIG = {
    .system_state = SYSTEM_STATE_IDLE,
    .last_error = SYSTEM_ERROR_NONE,
    .output_mode = OUTPUT_MODE_TEXT_AND_AUDIO,
    .display_brightness = 100,
    .audio_volume = 80,
    .haptic_intensity = 80,
    .bluetooth_enabled = true,
    .power_save_enabled = true,
    .touch_enabled = true,
    .calibration_required = true
};

/**
 * @brief Default flex sensor calibration values
 * 
 * These are the default values used when no calibration data is available.
 */
static const flex_sensor_calibration_t DEFAULT_FLEX_CALIBRATION = {
    .flat_value = {2450, 2450, 2450, 2450, 2450},
    .bent_value = {3400, 3400, 3400, 3400, 3400},
    .scale_factor = {1.0f, 1.0f, 1.0f, 1.0f, 1.0f},
    .offset = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}
};

/**
 * @brief Default IMU calibration values
 */
static const struct {
    int16_t accel_offset[3];
    int16_t gyro_offset[3];
    float orientation_offset[3];
} DEFAULT_IMU_CALIBRATION = {
    .accel_offset = {0, 0, 0},
    .gyro_offset = {0, 0, 0},
    .orientation_offset = {0.0f, 0.0f, 0.0f}
};

/**
 * @brief Default timeout values (in milliseconds)
 */
static const struct {
    uint32_t display_timeout_ms;
    uint32_t inactivity_timeout_ms;
    uint32_t deep_sleep_timeout_ms;
} DEFAULT_TIMEOUTS = {
    .display_timeout_ms = 30000,    // 30 seconds
    .inactivity_timeout_ms = 60000, // 1 minute
    .deep_sleep_timeout_ms = 300000 // 5 minutes
};

#endif /* DEFAULT_CONFIG_H */
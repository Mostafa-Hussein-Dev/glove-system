#ifndef SYSTEM_CONFIG_H
#define SYSTEM_CONFIG_H

#include "esp_err.h"
#include <stdbool.h>
#include <stdio.h>

/**
 * @brief System configuration parameters
 */

/* Task priorities */
#define SENSOR_TASK_PRIORITY        (10)
#define PROCESSING_TASK_PRIORITY    (9)
#define OUTPUT_TASK_PRIORITY        (8)
#define COMMUNICATION_TASK_PRIORITY (7)
#define POWER_TASK_PRIORITY         (6)

/* Task stack sizes */
#define SENSOR_TASK_STACK_SIZE        (4096)
#define PROCESSING_TASK_STACK_SIZE    (8192)
#define OUTPUT_TASK_STACK_SIZE        (4096)
#define COMMUNICATION_TASK_STACK_SIZE (4096)
#define POWER_TASK_STACK_SIZE         (2048)

/* Core assignments */
#define SENSOR_TASK_CORE           (0)
#define PROCESSING_TASK_CORE       (1)
#define OUTPUT_TASK_CORE           (1)
#define COMMUNICATION_TASK_CORE    (0)
#define POWER_TASK_CORE            (0)

/* Sampling rates */
#define FLEX_SENSOR_SAMPLE_RATE_HZ  (50)
#define IMU_SAMPLE_RATE_HZ          (100)
#define CAMERA_FRAME_RATE_HZ        (15)
#define TOUCH_SAMPLE_RATE_HZ        (20)

/* Queue sizes */
#define SENSOR_QUEUE_SIZE           (10)
#define PROCESSING_QUEUE_SIZE       (5)
#define OUTPUT_QUEUE_SIZE           (5)
#define COMMAND_QUEUE_SIZE          (5)

/* Buffer sizes */
#define FLEX_SENSOR_BUFFER_SIZE     (10)
#define IMU_BUFFER_SIZE             (20)
#define FEATURE_BUFFER_SIZE         (100)

/* Power management */
#define BATTERY_LOW_THRESHOLD_MV    (3300)
#define BATTERY_CRITICAL_MV         (3100)
#define INACTIVITY_TIMEOUT_SEC      (60)
#define DEEP_SLEEP_TIMEOUT_SEC      (300)

/* Display parameters */
#define DISPLAY_TIMEOUT_SEC         (30)
#define DISPLAY_WIDTH               (128)
#define DISPLAY_HEIGHT              (64)

/* Audio parameters */
#define AUDIO_SAMPLE_RATE           (16000)
#define AUDIO_BUFFER_SIZE           (1024)

/* Bluetooth LE */
#define BLE_DEVICE_NAME             "SignLangGlove"
#define BLE_MAX_CONNECTIONS         (1)

/* Gesture recognition */
#define MAX_GESTURES                (50)
#define CONFIDENCE_THRESHOLD        (0.7f)
#define MAX_GESTURE_DURATION_MS     (2000)
#define MIN_GESTURE_DURATION_MS     (200)

/* System states */
typedef enum {
    SYSTEM_STATE_INIT,
    SYSTEM_STATE_IDLE,
    SYSTEM_STATE_ACTIVE,
    SYSTEM_STATE_STANDBY,
    SYSTEM_STATE_SLEEP,
    SYSTEM_STATE_CHARGING,
    SYSTEM_STATE_LOW_BATTERY,
    SYSTEM_STATE_ERROR,
    SYSTEM_STATE_CALIBRATION
} system_state_t;

/* Error codes */
typedef enum {
    SYSTEM_ERROR_NONE,
    SYSTEM_ERROR_FLEX_SENSOR,
    SYSTEM_ERROR_IMU,
    SYSTEM_ERROR_CAMERA,
    SYSTEM_ERROR_DISPLAY,
    SYSTEM_ERROR_AUDIO,
    SYSTEM_ERROR_BLUETOOTH,
    SYSTEM_ERROR_MEMORY,
    SYSTEM_ERROR_BATTERY,
    SYSTEM_ERROR_UNKNOWN
} system_error_t;

/* Output modes */
typedef enum {
    OUTPUT_MODE_TEXT_ONLY,
    OUTPUT_MODE_AUDIO_ONLY,
    OUTPUT_MODE_TEXT_AND_AUDIO,
    OUTPUT_MODE_MINIMAL
} output_mode_t;

/* System configuration structure */
typedef struct {
    system_state_t system_state;
    system_error_t last_error;
    output_mode_t output_mode;
    uint8_t display_brightness;
    uint8_t audio_volume;
    uint8_t haptic_intensity;
    bool bluetooth_enabled;
    bool power_save_enabled;
    bool touch_enabled;
    bool camera_enabled;
    bool calibration_required;
    bool imu_enabled;
} system_config_t;

/* Global configuration variable (declared in app_main.c) */
extern system_config_t g_system_config;

/* Function declarations */
esp_err_t system_config_init(void);
esp_err_t system_config_save(void);
esp_err_t system_config_load(void);
esp_err_t system_config_reset_to_default(void);

#endif /* SYSTEM_CONFIG_H */

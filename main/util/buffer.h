#ifndef UTIL_BUFFER_H
#define UTIL_BUFFER_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "config/system_config.h"
#include "drivers/imu.h"       // Add this include for imu_data_t
#include "drivers/camera.h"    // Add this include for camera_frame_t
#include "drivers/touch.h"     // Include if you have touch_sensor_data_t defined here

/**
 * @brief Structure to hold flex sensor data (5 sensors instead of 10)
 */
typedef struct {
    uint16_t raw_values[5];   // Raw ADC values (5 sensors)
    float angles[5];          // Calculated bend angles (5 sensors)
    uint32_t timestamp;       // Acquisition timestamp
} flex_sensor_data_t;


/**
 * @brief Structure to hold IMU data
 */
typedef struct {
    float accel[3];          // X, Y, Z acceleration
    float gyro[3];           // X, Y, Z rotation rates
    float orientation[3];    // Roll, pitch, yaw
    uint32_t timestamp;      // Acquisition timestamp
} imu_data_t;

/**
 * @brief Structure to hold camera frame data
 */
typedef struct {
    uint8_t* frame_buffer;   // Pointer to image data
    uint32_t buffer_size;    // Size of the buffer
   uint16_t width;          // Image width
   uint16_t height;         // Image height
    uint32_t timestamp;      // Acquisition timestamp
} camera_frame_t;


/**
 * @brief Structure to hold touch sensor data
 */
typedef struct {
    bool touch_status[5];     // Status of 5 touch sensors
    uint32_t timestamp;       // Acquisition timestamp
} touch_sensor_data_t;

/**
 * @brief Structure to hold all sensor data
 */
typedef struct sensor_data_s {
    flex_sensor_data_t flex_data;
    imu_data_t imu_data;
    camera_frame_t camera_data;
    touch_sensor_data_t touch_data;
    bool flex_data_valid;
    bool imu_data_valid;
    bool camera_data_valid;
    bool touch_data_valid;
    uint32_t sequence_number;  // For tracking data packets
    uint32_t timestamp;        // Global timestamp for this dataset
} sensor_data_t;

/**
 * @brief Structure to hold feature vector
 */
typedef struct {
    float features[FEATURE_BUFFER_SIZE];
    uint16_t feature_count;
    uint32_t timestamp;
} feature_vector_t;

/**
 * @brief Structure to hold processing results
 */
typedef struct processing_result_s {
    uint8_t gesture_id;       // Recognized gesture ID
    char gesture_name[32];    // Text representation
    float confidence;         // Recognition confidence (0-1)
    bool is_dynamic;          // Static vs dynamic gesture
    uint32_t duration_ms;     // Gesture duration
    uint32_t timestamp;       // Processing timestamp
} processing_result_t;

/**
 * @brief Output command types
 */
typedef enum {
    OUTPUT_CMD_DISPLAY_TEXT,
    OUTPUT_CMD_SPEAK_TEXT,
    OUTPUT_CMD_HAPTIC_FEEDBACK,
    OUTPUT_CMD_SET_MODE,
    OUTPUT_CMD_CLEAR,
    OUTPUT_CMD_SHOW_BATTERY,
    OUTPUT_CMD_SHOW_ERROR,
    OUTPUT_CMD_SHOW_STATUS
} output_command_type_t;

/**
 * @brief Structure for output commands
 */
typedef struct output_command_s {
    output_command_type_t type;
    union {
        struct {
            char text[64];
            uint8_t size;
            uint8_t x;
            uint8_t y;
        } display_text;
        struct {
            char text[128];
            float volume;
        } speak_text;
        struct {
            uint16_t duration_ms;
            uint8_t intensity;
        } haptic;
        struct {
            uint8_t mode;
        } set_mode;
        struct {
            uint8_t battery_percent;
            float voltage;
        } battery_info;
        struct {
            char error_message[64];
            uint8_t error_code;
        } error_info;
        struct {
            char status_text[64];
            uint8_t status_code;
        } status_info;
    };
    uint32_t timestamp;
} output_command_t;

/**
 * @brief System command types
 */
typedef enum {
    SYSTEM_CMD_SLEEP,
    SYSTEM_CMD_WAKE,
    SYSTEM_CMD_CALIBRATE,
    SYSTEM_CMD_RESET,
    SYSTEM_CMD_POWER_SAVE,
    SYSTEM_CMD_NORMAL_MODE,
    SYSTEM_CMD_DEBUG_MODE
} system_command_type_t;

/**
 * @brief Structure for system commands
 */
typedef struct system_command_s {
    system_command_type_t type;
    union {
        struct {
            uint8_t calibration_type;  // 0: flat, 1: bent, 2: full
        } calibrate;
        struct {
            uint8_t power_level;       // 0-100
        } power_save;
    };
    uint32_t timestamp;
} system_command_t;

/**
 * @brief Circular buffer for sensor data history
 */
typedef struct {
    sensor_data_t *data;      // Buffer array
    uint16_t size;            // Buffer size
    uint16_t head;            // Head index
    uint16_t tail;            // Tail index
    uint16_t count;           // Current element count
    bool full;                // Buffer full flag
} sensor_data_buffer_t;

/**
 * @brief Initialize a sensor data buffer
 * 
 * @param buffer Pointer to buffer structure
 * @param size Maximum number of elements
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t buffer_init(sensor_data_buffer_t *buffer, uint16_t size);

/**
 * @brief Free a sensor data buffer
 * 
 * @param buffer Pointer to buffer structure
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t buffer_free(sensor_data_buffer_t *buffer);

/**
 * @brief Push data to buffer
 * 
 * @param buffer Pointer to buffer structure
 * @param data Pointer to data to push
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t buffer_push(sensor_data_buffer_t *buffer, const sensor_data_t *data);

/**
 * @brief Pop data from buffer
 * 
 * @param buffer Pointer to buffer structure
 * @param data Pointer to store popped data
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t buffer_pop(sensor_data_buffer_t *buffer, sensor_data_t *data);

/**
 * @brief Get data at specific index without removing it
 * 
 * @param buffer Pointer to buffer structure
 * @param index Index (0 = oldest, count-1 = newest)
 * @param data Pointer to store data
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t buffer_get(sensor_data_buffer_t *buffer, uint16_t index, sensor_data_t *data);

/**
 * @brief Check if buffer is empty
 * 
 * @param buffer Pointer to buffer structure
 * @return true if empty, false otherwise
 */
bool buffer_is_empty(sensor_data_buffer_t *buffer);

/**
 * @brief Check if buffer is full
 * 
 * @param buffer Pointer to buffer structure
 * @return true if full, false otherwise
 */
bool buffer_is_full(sensor_data_buffer_t *buffer);

/**
 * @brief Get current element count
 * 
 * @param buffer Pointer to buffer structure
 * @return Number of elements in buffer
 */
uint16_t buffer_count(sensor_data_buffer_t *buffer);

/**
 * @brief Clear buffer
 * 
 * @param buffer Pointer to buffer structure
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t buffer_clear(sensor_data_buffer_t *buffer);

#endif /* UTIL_BUFFER_H */
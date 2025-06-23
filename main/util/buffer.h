#ifndef UTIL_BUFFER_H
#define UTIL_BUFFER_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"
#include "config/system_config.h"
#include "drivers/imu.h"       // For imu_data_t
#include "drivers/ble_camera.h"    // For ble_camera_frame_t

/**
 * @brief Structure to hold flex sensor data
 */
typedef struct {
    uint16_t raw_values[5];  // Raw ADC values (10 sensors)
    float angles[5];         // Calculated bend angles
    uint32_t timestamp;       // Acquisition timestamp
} flex_sensor_data_t;

/**
 * @brief Structure to hold touch sensor data
 */
typedef struct {
    bool touch_status[5];    // Status of 5 touch sensors
    uint32_t timestamp;      // Acquisition timestamp
} touch_sensor_data_t;

/**
 * @brief Structure to hold all sensor data
 */
typedef struct sensor_data_s {
    flex_sensor_data_t flex_data;
    imu_data_t imu_data;
    ble_camera_frame_t camera_data;
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
            uint8_t line;
            bool clear_first;
        } display;
        
        struct {
            char text[128];
            uint8_t priority;
        } speak;
        
        struct {
            uint8_t pattern;
            uint8_t intensity;
            uint16_t duration_ms;
        } haptic;
        
        struct {
            output_mode_t mode;
        } set_mode;
        
        struct {
            uint8_t percentage;
            bool show_graphic;
        } battery;
        
        struct {
            system_error_t error_code;
            char error_text[64];
        } error;
    } data;
} output_command_t;

/**
 * @brief System command types
 */
typedef enum {
    SYS_CMD_CHANGE_STATE,
    SYS_CMD_CALIBRATE,
    SYS_CMD_SET_POWER_MODE,
    SYS_CMD_ENABLE_BLE,
    SYS_CMD_DISABLE_BLE,
    SYS_CMD_RESTART,
    SYS_CMD_SLEEP,
    SYS_CMD_FACTORY_RESET
} system_command_type_t;

/**
 * @brief Structure for system commands
 */
typedef struct system_command_s {
    system_command_type_t type;
    union {
        struct {
            system_state_t new_state;
        } change_state;
        
        struct {
            bool enable_power_save;
        } power_mode;
        
        struct {
            uint16_t sleep_duration_sec;
        } sleep;
    } data;
} system_command_t;

/**
 * @brief Circular buffer for sensor data storage
 */
typedef struct {
    sensor_data_t* buffer;
    size_t capacity;
    size_t size;
    size_t head;
    size_t tail;
} sensor_data_buffer_t;

/**
 * @brief Initialize a circular buffer for sensor data
 * 
 * @param buffer Pointer to buffer structure
 * @param capacity Maximum number of elements in buffer
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t buffer_init(sensor_data_buffer_t* buffer, size_t capacity);

/**
 * @brief Free and cleanup circular buffer
 * 
 * @param buffer Pointer to buffer structure
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t buffer_free(sensor_data_buffer_t* buffer);

/**
 * @brief Push data into the circular buffer
 * 
 * @param buffer Pointer to buffer structure
 * @param data Pointer to data to be stored
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t buffer_push(sensor_data_buffer_t* buffer, const sensor_data_t* data);

/**
 * @brief Pop data from the circular buffer
 * 
 * @param buffer Pointer to buffer structure
 * @param data Pointer to store retrieved data
 * @return ESP_OK on success, ESP_ERR_NOT_FOUND if buffer is empty
 */
esp_err_t buffer_pop(sensor_data_buffer_t* buffer, sensor_data_t* data);

/**
 * @brief Check if buffer is empty
 * 
 * @param buffer Pointer to buffer structure
 * @return true if empty, false otherwise
 */
bool buffer_is_empty(const sensor_data_buffer_t* buffer);

/**
 * @brief Check if buffer is full
 * 
 * @param buffer Pointer to buffer structure
 * @return true if full, false otherwise
 */
bool buffer_is_full(const sensor_data_buffer_t* buffer);

/**
 * @brief Get current size of buffer
 * 
 * @param buffer Pointer to buffer structure
 * @return Current number of elements in buffer
 */
size_t buffer_get_size(const sensor_data_buffer_t* buffer);

/**
 * @brief Clear all data from buffer
 * 
 * @param buffer Pointer to buffer structure
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t buffer_clear(sensor_data_buffer_t* buffer);

/**
 * @brief Get data at a specific index without removing it
 * 
 * @param buffer Pointer to buffer structure
 * @param index Index of data to retrieve (0 = oldest, size-1 = newest)
 * @param data Pointer to store retrieved data
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG if index is out of bounds
 */
esp_err_t buffer_get(const sensor_data_buffer_t* buffer, size_t index, sensor_data_t* data);

/**
 * @brief Buffer statistics structure for monitoring
 */
typedef struct {
    size_t total_allocations;
    size_t total_deallocations;
    size_t current_camera_buffers;
    size_t max_camera_buffers;
    size_t overflow_count;
    size_t bytes_allocated;
    size_t bytes_freed;
    size_t peak_memory_usage;
    size_t memory_leaked;
} buffer_stats_t;

/**
 * @brief Get buffer statistics for monitoring
 * 
 * @param stats Pointer to structure to fill with statistics
 */
void buffer_get_stats(buffer_stats_t* stats);

/**
 * @brief Print buffer statistics for debugging
 */
void buffer_print_stats(void);

/**
 * @brief Emergency cleanup - force free all camera buffers
 */
void buffer_emergency_cleanup(void);

#endif /* UTIL_BUFFER_H */
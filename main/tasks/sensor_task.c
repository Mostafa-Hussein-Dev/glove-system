#include "tasks/sensor_task.h"
#include <string.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "drivers/flex_sensor.h"
#include "drivers/imu.h"
#include "drivers/ble_camera.h"
#include "drivers/touch.h"
#include "app_main.h"
#include "config/system_config.h"
#include "config/pin_definitions.h"
#include "util/debug.h"
#include "util/buffer.h"

static const char *TAG = "SENSOR_TASK";

// Task handle
static TaskHandle_t sensor_task_handle = NULL;

// Sensor sampling rates (in milliseconds)
#define FLEX_SENSOR_SAMPLE_INTERVAL  (1000 / FLEX_SENSOR_SAMPLE_RATE_HZ)
#define IMU_SAMPLE_INTERVAL          (1000 / IMU_SAMPLE_RATE_HZ)
#define CAMERA_SAMPLE_INTERVAL       (1000 / CAMERA_FRAME_RATE_HZ)
#define TOUCH_SAMPLE_INTERVAL        (1000 / TOUCH_SAMPLE_RATE_HZ)

// Last sampling timestamps
static uint32_t last_flex_sample_time = 0;
static uint32_t last_imu_sample_time = 0;
static uint32_t last_camera_sample_time = 0;
static uint32_t last_touch_sample_time = 0;

// Sensor data storage
static sensor_data_t current_sensor_data;
static uint32_t sequence_number = 0;

// Forward declarations for sampling functions
static esp_err_t sample_flex_sensors(void);
static esp_err_t sample_imu(void);
static esp_err_t sample_camera(void);
static esp_err_t sample_touch_sensors(void);
static void touch_callback(bool *status);

// Sensor task function
static void sensor_task(void *arg);

esp_err_t sensor_task_init(void) {
    // Create the sensor task
    BaseType_t ret = xTaskCreatePinnedToCore(
        sensor_task,
        "sensor_task",
        SENSOR_TASK_STACK_SIZE,
        NULL,
        SENSOR_TASK_PRIORITY,
        &sensor_task_handle,
        SENSOR_TASK_CORE
    );
    
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create sensor task");
        return ESP_FAIL;
    }
    
    // Register touch callback
    touch_set_callback(touch_callback);
    
    // Initialize sensor data structure
    memset(&current_sensor_data, 0, sizeof(sensor_data_t));
    
    ESP_LOGI(TAG, "Sensor task initialized on core %d", SENSOR_TASK_CORE);
    return ESP_OK;
}

static void sensor_task(void *arg) {
    ESP_LOGI(TAG, "Sensor task started");
    
    // Set sensor task as ready
    xEventGroupSetBits(g_system_event_group, SYSTEM_EVENT_SENSOR_READY);
    
    // Wait for system initialization to complete
    xEventGroupWaitBits(g_system_event_group, 
                        SYSTEM_EVENT_INIT_COMPLETE, 
                        pdFALSE, pdTRUE, portMAX_DELAY);
    
    // Initialize timestamps to current time
    uint32_t current_time = esp_timer_get_time() / 1000;  // Convert to milliseconds
    last_flex_sample_time = current_time;
    last_imu_sample_time = current_time;
    last_camera_sample_time = current_time;
    last_touch_sample_time = current_time;
    
    while (1) {
        current_time = esp_timer_get_time() / 1000;  // Get current time in milliseconds
        bool data_updated = false;
        
        // Check if it's time to sample flex sensors
        if (current_time - last_flex_sample_time >= FLEX_SENSOR_SAMPLE_INTERVAL) {
            if (sample_flex_sensors() == ESP_OK) {
                last_flex_sample_time = current_time;
                data_updated = true;
            }
        }
        
        // Check if it's time to sample IMU
        if (current_time - last_imu_sample_time >= IMU_SAMPLE_INTERVAL) {
            if (sample_imu() == ESP_OK) {
                last_imu_sample_time = current_time;
                data_updated = true;
            }
        }
        
        // Check if it's time to sample camera (if enabled)
        if (current_time - last_camera_sample_time >= CAMERA_SAMPLE_INTERVAL) {
            if (sample_camera() == ESP_OK) {
                last_camera_sample_time = current_time;
                data_updated = true;
            }
        }
        
        // Check if it's time to sample touch sensors
        if (g_system_config.touch_enabled && 
            current_time - last_touch_sample_time >= TOUCH_SAMPLE_INTERVAL) {
            if (sample_touch_sensors() == ESP_OK) {
                last_touch_sample_time = current_time;
                data_updated = true;
            }
        }
        
        // If any data was updated, send it to the processing task
        if (data_updated) {
            // Update timestamp and sequence number
            current_sensor_data.timestamp = current_time;
            current_sensor_data.sequence_number = sequence_number++;
            
            // Send data to processing task
            if (xQueueSend(g_sensor_data_queue, &current_sensor_data, 0) != pdTRUE) {
                ESP_LOGW(TAG, "Failed to send sensor data to queue (queue full)");
            }
        }
        
        // Short delay to prevent CPU hogging
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

static esp_err_t sample_flex_sensors(void) {
    esp_err_t ret;
    
    // Read raw values
    ret = flex_sensor_read_raw(current_sensor_data.flex_data.raw_values);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to read flex sensor raw values: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Read angles
    ret = flex_sensor_read_angles(current_sensor_data.flex_data.angles);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to read flex sensor angles: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Set timestamp
    current_sensor_data.flex_data.timestamp = esp_timer_get_time() / 1000;
    current_sensor_data.flex_data_valid = true;
    
    return ESP_OK;
}

static esp_err_t sample_imu(void) {
    esp_err_t ret;
    
    // Read IMU data using a temporary imu_data_t structure
    imu_data_t imu_data;
    ret = imu_read(&imu_data);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to read IMU data: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Copy data to sensor data structure
    memcpy(&current_sensor_data.imu_data, &imu_data, sizeof(imu_data_t));
    current_sensor_data.imu_data_valid = true;
    
    return ESP_OK;
}

static esp_err_t sample_camera(void) {
    esp_err_t ret;
    
    // First release any previous frame
    if (current_sensor_data.camera_data_valid && 
        current_sensor_data.camera_data.buffer != NULL) {
        ble_camera_release_frame();
        current_sensor_data.camera_data.buffer = NULL;
        current_sensor_data.camera_data_valid = false;
    }
    
    // Capture new frame using a temporary ble_camera_frame_t structure
    ble_camera_frame_t frame;
    ret = ble_camera_capture_frame(&frame);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to capture camera frame: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Copy the captured frame data to our sensor data structure
    current_sensor_data.camera_data.buffer = frame.buffer;
    current_sensor_data.camera_data.buffer_size = frame.buffer_size;
    current_sensor_data.camera_data.width = frame.width;
    current_sensor_data.camera_data.height = frame.height;
    current_sensor_data.camera_data.timestamp = frame.timestamp;
    current_sensor_data.camera_data_valid = true;
    
    return ESP_OK;
}

static esp_err_t sample_touch_sensors(void) {
    esp_err_t ret;
    
    // Read touch status
    ret = touch_get_status(current_sensor_data.touch_data.touch_status);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to read touch status: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Set timestamp
    current_sensor_data.touch_data.timestamp = esp_timer_get_time() / 1000;
    current_sensor_data.touch_data_valid = true;
    
    return ESP_OK;
}

// Callback for touch events
static void touch_callback(bool *status) {
    // Copy touch status to sensor data
    memcpy(current_sensor_data.touch_data.touch_status, status, sizeof(bool) * TOUCH_SENSOR_COUNT);
    
    // Set timestamp
    current_sensor_data.touch_data.timestamp = esp_timer_get_time() / 1000;
    current_sensor_data.touch_data_valid = true;
    
    // Send data immediately since this is an event-driven update
    current_sensor_data.timestamp = current_sensor_data.touch_data.timestamp;
    current_sensor_data.sequence_number = sequence_number++;
    
    // Send data to processing task
    if (xQueueSend(g_sensor_data_queue, &current_sensor_data, 0) != pdTRUE) {
        ESP_LOGW(TAG, "Failed to send touch event data to queue (queue full)");
    }
}
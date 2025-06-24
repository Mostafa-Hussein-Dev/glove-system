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
#include "cJSON.h"
#include "esp_heap_caps.h"
#include "core/system_monitor.h"

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
static void output_sensor_data_for_collection(const sensor_data_t* data);
static void print_real_task_stats(void);

// Sensor task declaration
static void sensor_task(void *arg);

esp_err_t sensor_task_init(void) {
    ESP_LOGI(TAG, "Initializing sensor task with enhanced architecture...");
    ESP_LOGI(TAG, "  Core: %d, Priority: %d, Stack: %d bytes",
        SENSOR_TASK_CORE, SENSOR_TASK_PRIORITY, SENSOR_TASK_STACK_SIZE); 

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
    ESP_LOGI(TAG, "Sensor task started on core %d", xPortGetCoreID());
    
    // Wait for system initialization
    xEventGroupWaitBits(g_system_event_group, 
                       SYSTEM_EVENT_INIT_COMPLETE, 
                       pdFALSE, pdTRUE, portMAX_DELAY);
    
    // Set sensor ready event
    xEventGroupSetBits(g_system_event_group, SYSTEM_EVENT_SENSOR_READY);
    
    while (1) {
        uint32_t current_time_ms = esp_timer_get_time() / 1000;
        bool data_collected = false;
        
        // Sample sensors based on their individual rates
        if (current_time_ms - last_flex_sample_time >= FLEX_SENSOR_SAMPLE_INTERVAL) {
            if (sample_flex_sensors() == ESP_OK) {
                data_collected = true;
            }
            last_flex_sample_time = current_time_ms;
        }
        
        if (current_time_ms - last_imu_sample_time >= IMU_SAMPLE_INTERVAL) {
            if (sample_imu() == ESP_OK) {
                data_collected = true;
            }
            last_imu_sample_time = current_time_ms;
        }
        
        if (current_time_ms - last_camera_sample_time >= CAMERA_SAMPLE_INTERVAL) {
            if (sample_camera() == ESP_OK) {
                data_collected = true;
            }
            last_camera_sample_time = current_time_ms;
        }
        
        if (current_time_ms - last_touch_sample_time >= TOUCH_SAMPLE_INTERVAL) {
            if (sample_touch_sensors() == ESP_OK) {
                data_collected = true;
            }
            last_touch_sample_time = current_time_ms;
        }
        
        // Send data to processing queue if any data was collected
        if (data_collected) {
            current_sensor_data.timestamp = current_time_ms;
            current_sensor_data.sequence_number = sequence_number++;
            
            // ENHANCED QUEUE HANDLING WITH OVERFLOW PROTECTION
            if (QUEUE_OVERFLOW_PROTECTION) {
                // Check queue usage before sending
                UBaseType_t waiting = uxQueueMessagesWaiting(g_sensor_data_queue);
                UBaseType_t spaces = uxQueueSpacesAvailable(g_sensor_data_queue);
                
                if (waiting + spaces > 0) {
                    uint32_t usage_percent = (waiting * 100) / (waiting + spaces);
                    
                    if (usage_percent > QUEUE_HIGH_WATERMARK_PERCENT) {
                        ESP_LOGW(TAG, "Sensor queue high usage: %u%%", usage_percent);
                    }
                    
                    // Report queue health to system monitor
                    system_monitor_update_queue_health(usage_percent, false);
                }
                
                // Try to send with retry mechanism
                for (int retry = 0; retry < QUEUE_FULL_RETRY_COUNT; retry++) {
                    if (xQueueSend(g_sensor_data_queue, &current_sensor_data, 0) == pdTRUE) {
                        break;  // Successfully sent
                    } else if (retry < QUEUE_FULL_RETRY_COUNT - 1) {
                        // Queue full, wait and retry
                        vTaskDelay(pdMS_TO_TICKS(QUEUE_FULL_RETRY_DELAY_MS));
                    } else {
                        // Final retry failed - report overflow
                        ESP_LOGW(TAG, "Sensor queue overflow - data dropped");
                        system_monitor_update_queue_health(100, true);
                    }
                }
            } else {
                // Standard queue send without protection
                if (xQueueSend(g_sensor_data_queue, &current_sensor_data, 0) != pdTRUE) {
                    ESP_LOGW(TAG, "Failed to send sensor data - queue full");
                }
            }
        }

                
        // High-frequency task - minimal delay for real-time response
        vTaskDelay(pdMS_TO_TICKS(50));  // 10ms = 100Hz loop rate
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

static void output_sensor_data_for_collection(const sensor_data_t* data) {
    if (!data) return;
    
    // Create JSON object for sensor data
    cJSON *json = cJSON_CreateObject();
    
    // Add flex sensor data
    if (data->flex_data_valid) {
        cJSON *flex_array = cJSON_CreateArray();
        for (int i = 0; i < FINGER_COUNT * 2; i++) {
            cJSON_AddItemToArray(flex_array, cJSON_CreateNumber(data->flex_data.angles[i]));
        }
        cJSON_AddItemToObject(json, "flex_data", flex_array);
    }
    
    // Add IMU data
    if (data->imu_data_valid) {
        cJSON *imu_obj = cJSON_CreateObject();
        
        cJSON *accel_array = cJSON_CreateArray();
        cJSON *gyro_array = cJSON_CreateArray();
        cJSON *orient_array = cJSON_CreateArray();
        
        for (int i = 0; i < 3; i++) {
            cJSON_AddItemToArray(accel_array, cJSON_CreateNumber(data->imu_data.accel[i]));
            cJSON_AddItemToArray(gyro_array, cJSON_CreateNumber(data->imu_data.gyro[i]));
            cJSON_AddItemToArray(orient_array, cJSON_CreateNumber(data->imu_data.orientation[i]));
        }
        
        cJSON_AddItemToObject(imu_obj, "accel", accel_array);
        cJSON_AddItemToObject(imu_obj, "gyro", gyro_array);
        cJSON_AddItemToObject(imu_obj, "orientation", orient_array);
        cJSON_AddItemToObject(json, "imu_data", imu_obj);
    }
    
    // Add timestamp
    cJSON_AddItemToObject(json, "timestamp", cJSON_CreateNumber(data->timestamp));
    
    // Output JSON string
    char *json_string = cJSON_Print(json);
    if (json_string) {
        printf("SENSOR_DATA:%s\n", json_string);
        free(json_string);
    }
    
    cJSON_Delete(json);
}

static void touch_callback(bool *status) {
    // Copy touch status to sensor data
    memcpy(current_sensor_data.touch_data.touch_status, status, sizeof(bool) * TOUCH_SENSOR_COUNT);
    
    // Set timestamp
    current_sensor_data.touch_data.timestamp = esp_timer_get_time() / 1000;
    current_sensor_data.touch_data_valid = true;
    
    // Send data immediately since this is an event-driven update
    current_sensor_data.timestamp = current_sensor_data.touch_data.timestamp;
    current_sensor_data.sequence_number = sequence_number++;
    
    // Send touch event data with high priority (touch is critical)
    if (xQueueSend(g_sensor_data_queue, &current_sensor_data, pdMS_TO_TICKS(5)) != pdTRUE) {
        ESP_LOGW(TAG, "Failed to send touch event data - queue critically full");
        // Touch events are critical, try to force by removing oldest non-touch data
        sensor_data_t dummy;
        if (xQueueReceive(g_sensor_data_queue, &dummy, 0) == pdTRUE) {
            xQueueSend(g_sensor_data_queue, &current_sensor_data, 0);
        }
    }
}



void* sensor_task_get_handle(void) {
    extern TaskHandle_t sensor_task_handle;  // Declare external reference
    return (void*)sensor_task_handle;
}
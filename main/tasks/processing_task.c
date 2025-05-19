#include "tasks/processing_task.h"
#include <string.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "processing/sensor_fusion.h"
#include "processing/feature_extraction.h"
#include "processing/gesture_detection.h"
#include "app_main.h"
#include "config/system_config.h"
#include "config/pin_definitions.h"
#include "util/debug.h"
#include "util/buffer.h"

static const char *TAG = "PROCESSING_TASK";

// Task handle
static TaskHandle_t processing_task_handle = NULL;

// Buffer for sensor data history
static sensor_data_buffer_t sensor_data_buffer;

// Processing task function
static void processing_task(void *arg);

esp_err_t processing_task_init(void) {
    // Initialize sensor data buffer
    esp_err_t ret = buffer_init(&sensor_data_buffer, 20);  // Buffer for 20 samples
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize sensor data buffer: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Create the processing task
    BaseType_t xReturned = xTaskCreatePinnedToCore(
        processing_task,
        "processing_task",
        PROCESSING_TASK_STACK_SIZE,
        NULL,
        PROCESSING_TASK_PRIORITY,
        &processing_task_handle,
        PROCESSING_TASK_CORE
    );
    
    if (xReturned != pdPASS) {
        ESP_LOGE(TAG, "Failed to create processing task");
        buffer_free(&sensor_data_buffer);
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "Processing task initialized on core %d", PROCESSING_TASK_CORE);
    return ESP_OK;
}

static void processing_task(void *arg) {
    ESP_LOGI(TAG, "Processing task started");
    
    // Set processing task as ready
    xEventGroupSetBits(g_system_event_group, SYSTEM_EVENT_PROCESSING_READY);
    
    // Wait for system initialization to complete
    xEventGroupWaitBits(g_system_event_group, 
                        SYSTEM_EVENT_INIT_COMPLETE, 
                        pdFALSE, pdTRUE, portMAX_DELAY);
    
    // Sensor data and feature vector
    sensor_data_t sensor_data;
    feature_vector_t feature_vector;
    
    // Processing result
    processing_result_t result;
    
    while (1) {
        // Wait for sensor data from queue
        if (xQueueReceive(g_sensor_data_queue, &sensor_data, pdMS_TO_TICKS(100)) == pdTRUE) {
            // Store sensor data in buffer for temporal analysis
            buffer_push(&sensor_data_buffer, &sensor_data);
            
            // Perform sensor fusion
            sensor_fusion_process(&sensor_data, &sensor_data_buffer);
            
            // Extract features from sensor data
            if (feature_extraction_process(&sensor_data, &sensor_data_buffer, &feature_vector) == ESP_OK) {
                // Detect gesture based on features - no ML, just basic algorithm
                if (gesture_detection_process(&feature_vector, &result) == ESP_OK) {
                    // If a gesture was detected with sufficient confidence
                    if (result.confidence >= CONFIDENCE_THRESHOLD) {
                        // Add timestamp to result
                        result.timestamp = esp_timer_get_time() / 1000;
                        
                        ESP_LOGI(TAG, "Gesture detected: %s (confidence: %.2f)", 
                                result.gesture_name, result.confidence);
                        
                        // Send result to output task
                        if (xQueueSend(g_processing_result_queue, &result, 0) != pdTRUE) {
                            ESP_LOGW(TAG, "Failed to send processing result to queue (queue full)");
                        }
                    }
                }
            }
        }
        
        // Check system events or commands if any (could add here)
    }
}

void processing_task_deinit(void) {
    // Cleanup resources when task is deleted
    if (processing_task_handle != NULL) {
        vTaskDelete(processing_task_handle);
        processing_task_handle = NULL;
    }
    
    buffer_free(&sensor_data_buffer);
    
    ESP_LOGI(TAG, "Processing task deinitialized");
}
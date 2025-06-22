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
#include "ml/ml_inference.h"
#include "ml/model_manager.h"
#include "ml/data_preprocessor.h"

static const char *TAG = "PROCESSING_TASK";

// Task handle
static TaskHandle_t processing_task_handle = NULL;

// Buffer for sensor data history
static sensor_data_buffer_t sensor_data_buffer;

// Processing task function
static void processing_task(void *arg);

static esp_err_t process_ml_inference(sensor_data_t* sensor_data, sensor_data_buffer_t* data_buffer, processing_result_t* result) {
    ml_input_t static_input, dynamic_input;
    ml_result_t ml_result;
    
    // Preprocess data for static gesture recognition
    esp_err_t ret = data_preprocessor_static(sensor_data, &static_input);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to preprocess static data");
        return ret;
    }
    
    // Check if we have enough data for dynamic gesture recognition
    size_t buffer_size = buffer_get_size(data_buffer);
    bool use_dynamic = (buffer_size >= 5); // Need at least 5 samples for temporal analysis
    
    if (use_dynamic) {
        ret = data_preprocessor_dynamic(data_buffer, &dynamic_input);
        if (ret == ESP_OK) {
            // Use hybrid inference
            ret = ml_inference_hybrid(&static_input, &dynamic_input, &ml_result);
        } else {
            // Fall back to static only
            ret = ml_inference_run(&static_input, MODEL_TYPE_STATIC_CNN, &ml_result);
        }
    } else {
        // Use static model only
        ret = ml_inference_run(&static_input, MODEL_TYPE_STATIC_CNN, &ml_result);
    }
    
    if (ret != ESP_OK || !ml_result.is_valid) {
        return ESP_FAIL;
    }
    
    // Convert ML result to processing result
    result->gesture_id = ml_result.gesture_id;
    strncpy(result->gesture_name, ml_result.gesture_name, sizeof(result->gesture_name) - 1);
    result->gesture_name[sizeof(result->gesture_name) - 1] = '\0';
    result->confidence = ml_result.confidence;
    result->is_dynamic = use_dynamic;
    result->duration_ms = 0; // Could be calculated from buffer timestamps
    result->timestamp = sensor_data->timestamp;
    
    ESP_LOGI(TAG, "ML Gesture detected: %s (confidence: %.2f, inference: %u μs)", 
             result->gesture_name, result->confidence, ml_result.inference_time_us);
    
    return ESP_OK;
}

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
            // In the main processing loop, replace the existing gesture detection with:
            if (feature_extraction_process(&sensor_data, &sensor_data_buffer, &feature_vector) == ESP_OK) {
                // Try ML-based gesture detection first
                if (process_ml_inference(&sensor_data, &sensor_data_buffer, &result) == ESP_OK) {
                    // ML inference successful
                    if (result.confidence >= CONFIDENCE_THRESHOLD) {
                        result.timestamp = esp_timer_get_time() / 1000;
                        
                        ESP_LOGI(TAG, "ML Gesture detected: %s (confidence: %.2f)", 
                                result.gesture_name, result.confidence);
                        
                        // Send result to output task
                        if (xQueueSend(g_processing_result_queue, &result, 0) != pdTRUE) {
                            ESP_LOGW(TAG, "Failed to send ML processing result to queue (queue full)");
                        }
                    }
                } else {
                    // Fall back to template-based detection
                    if (gesture_detection_process(&feature_vector, &result) == ESP_OK) {
                        if (result.confidence >= CONFIDENCE_THRESHOLD) {
                            result.timestamp = esp_timer_get_time() / 1000;
                            
                            ESP_LOGI(TAG, "Template Gesture detected: %s (confidence: %.2f)", 
                                    result.gesture_name, result.confidence);
                            
                            // Send result to output task
                            if (xQueueSend(g_processing_result_queue, &result, 0) != pdTRUE) {
                                ESP_LOGW(TAG, "Failed to send template processing result to queue (queue full)");
                            }
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
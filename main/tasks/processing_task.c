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
#include "util/buffer.h" 

static const char *TAG = "PROCESSING_TASK";

// Task handle
static TaskHandle_t processing_task_handle = NULL;

// Buffer for sensor data history
static sensor_data_buffer_t sensor_data_buffer;

// Processing task function
static void processing_task(void *arg);
static void print_real_task_stats(void);

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
    ESP_LOGI(TAG, "Initializing processing task with enhanced architecture...");
    ESP_LOGI(TAG, "  Core: %d, Priority: %d, Stack: %d bytes", 
        PROCESSING_TASK_CORE, PROCESSING_TASK_PRIORITY, PROCESSING_TASK_STACK_SIZE);
    
    // === KEEP ALL YOUR EXISTING INITIALIZATION ===
    
    // Initialize sensor data buffer
    esp_err_t ret = buffer_init(&sensor_data_buffer, 20);  // Buffer for 20 samples
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize sensor data buffer: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = sensor_fusion_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize sensor fusion");
        return ret;
    }

    ret = feature_extraction_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize advanced feature extraction");
        return ret;
    }

    ret = gesture_detection_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize advanced gesture detection");
        return ret;
    }

    // === ONLY UPDATE THE TASK CREATION PART ===
    // Create the processing task with NEW ENHANCED CONFIGURATION
    BaseType_t xReturned = xTaskCreatePinnedToCore(
        processing_task,
        "processing_task",
        PROCESSING_TASK_STACK_SIZE,  // UPDATED: Now 16384 instead of 12288
        NULL,
        PROCESSING_TASK_PRIORITY,    // UPDATED: Now 10 instead of 9
        &processing_task_handle,
        PROCESSING_TASK_CORE         // SAME: Still Core 1
    );
    
    if (xReturned != pdPASS) {
        ESP_LOGE(TAG, "Failed to create processing task");
        buffer_free(&sensor_data_buffer);
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "Processing task initialized on core %d with priority %d (enhanced)", 
        PROCESSING_TASK_CORE, PROCESSING_TASK_PRIORITY);
    return ESP_OK;
}

static void processing_task(void *arg) {
    ESP_LOGI(TAG, "Processing task started on core %d", xPortGetCoreID());
    
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
        // Wait for sensor data from queue with adaptive timeout
        TickType_t timeout = pdMS_TO_TICKS(100);
        UBaseType_t queue_items = uxQueueMessagesWaiting(g_sensor_data_queue);
        
        // Use shorter timeout if queue is getting full
        if (queue_items > (SENSOR_QUEUE_SIZE * 0.8)) {
            timeout = pdMS_TO_TICKS(10);  // Process faster when queue filling up
        }
        
        if (xQueueReceive(g_sensor_data_queue, &sensor_data, timeout) == pdTRUE) {            // Store sensor data in buffer for temporal analysis
            buffer_push(&sensor_data_buffer, &sensor_data);
            
            // Perform sensor fusion
            sensor_fusion_process(&sensor_data, &sensor_data_buffer);
            
            fusion_stats_t stats;
            if (sensor_fusion_get_stats(&stats) == ESP_OK) {
                ESP_LOGI(TAG, "Fusion Stats:");
                ESP_LOGI(TAG, "Fusions performed: %u", stats.fusion_count);
                ESP_LOGI(TAG, "Kalman filters: %u active", stats.flex_filter_count);
                ESP_LOGI(TAG, "IMU filter: %s", stats.imu_filter_active ? "Active" : "Inactive");
                ESP_LOGI(TAG, "Average filter error: %.3f", stats.avg_filter_error);
            }

            // Extract features from sensor data
            // In the main processing loop, replace the existing gesture detection with:
            if (feature_extraction_process(&sensor_data, &sensor_data_buffer, &feature_vector) == ESP_OK) {
                
                // Add this to your debug code for treasure status:
                feature_extraction_stats_t features_stats;
                if (feature_extraction_get_stats(&features_stats) == ESP_OK) {
                    ESP_LOGI(TAG, "Feature Extraction Treasure Status:");
                    ESP_LOGI(TAG, "Extractions performed: %u", features_stats.extraction_count);
                    ESP_LOGI(TAG, "Total features: %u", features_stats.total_features);
                    ESP_LOGI(TAG, "Window size: %u samples", features_stats.window_size);
                    ESP_LOGI(TAG, "Current samples: %u", features_stats.current_samples);
                }

                // Feature vector inspection:
                ESP_LOGI(TAG, "Sample Features:");
                ESP_LOGI(TAG, "Flex current: %.1f°, %.1f°, %.1f°, %.1f°, %.1f°", 
                        feature_vector.features[0], feature_vector.features[1], 
                        feature_vector.features[2], feature_vector.features[3], feature_vector.features[4]);
                ESP_LOGI(TAG, "Fist closure: %.2f", feature_vector.features[136]); // Gesture-specific
                ESP_LOGI(TAG, "Pointing detected: %.1f", feature_vector.features[139]);
                ESP_LOGI(TAG, "Hand stability: %.2f", feature_vector.features[142]);
                    
                if (gesture_detection_process(&feature_vector, &result) == ESP_OK) {
                    // Now using advanced DTW, boundary detection, confidence scoring!
                    
                    if (result.confidence > 0.0f) {
                        ESP_LOGI(TAG, "GESTURE: %s (%.1f%%, %ums, %s)", 
                                result.gesture_name, 
                                result.confidence * 100.0f,
                                result.duration_ms,
                                result.is_dynamic ? "dynamic" : "static");
                        
                        // Send to output task
                        xQueueSend(g_processing_result_queue, &result, 0);
                    }

                    gesture_detection_stats_t stats;
                    if (gesture_detection_get_stats(&stats) == ESP_OK) {
                        ESP_LOGI(TAG, "Gesture Detection Treasure Status:");
                        ESP_LOGI(TAG, "Total detections: %u", stats.total_detections);
                        ESP_LOGI(TAG, "Templates loaded: %u", stats.template_count);
                        ESP_LOGI(TAG, "Current state: %u", stats.current_state);
                        ESP_LOGI(TAG, "Sequence length: %u", stats.sequence_length);
                        ESP_LOGI(TAG, "False positives: %u", stats.false_positive_count);
                        ESP_LOGI(TAG, "Sequence timeouts: %u", stats.sequence_timeouts);
                    }

                    // Real-time gesture monitoring:
                    ESP_LOGI(TAG, "Gesture State: %s", 
                            (stats.current_state == 0) ? "IDLE" :
                            (stats.current_state == 1) ? "STARTING" :
                            (stats.current_state == 2) ? "ACTIVE" :
                            (stats.current_state == 3) ? "HOLDING" :
                            (stats.current_state == 4) ? "ENDING" : "UNKNOWN");
                }



                
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
                            
                            // Send result to output task with overflow protection
                            if (xQueueSend(g_processing_result_queue, &result, 0) != pdTRUE) {
                                UBaseType_t spaces = uxQueueSpacesAvailable(g_processing_result_queue);
                                ESP_LOGW(TAG, "Processing result queue full (%u spaces), dropping result", 
                                        (unsigned int)spaces);
                            }

                        }
                    }
                }
            }
        }
        
        // Periodic queue health check (every 100 iterations ≈ 10 seconds)
        static uint32_t health_check_counter = 0;
        if (++health_check_counter >= 100) {
            health_check_counter = 0;
            
            UBaseType_t sensor_waiting = uxQueueMessagesWaiting(g_sensor_data_queue);
            UBaseType_t processing_waiting = uxQueueMessagesWaiting(g_processing_result_queue);
            
            if (sensor_waiting > (SENSOR_QUEUE_SIZE * 0.8)) {
                ESP_LOGW(TAG, "Sensor queue high usage: %u/%d items", 
                         (unsigned int)sensor_waiting, SENSOR_QUEUE_SIZE);
            }
            
            if (processing_waiting > (PROCESSING_QUEUE_SIZE * 0.8)) {
                ESP_LOGW(TAG, "Processing queue high usage: %u/%d items", 
                         (unsigned int)processing_waiting, PROCESSING_QUEUE_SIZE);
            }
            
            // Print buffer statistics periodically
            buffer_print_stats();
        }


        vTaskDelay(pdMS_TO_TICKS(10));
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


void* processing_task_get_handle(void) {
    extern TaskHandle_t processing_task_handle;  // Declare external reference
    return (void*)processing_task_handle;
}
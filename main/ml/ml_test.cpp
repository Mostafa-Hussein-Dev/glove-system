#include "esp_system.h"
#include "ml_test.h"
#include "ml_inference.h"
#include "model_manager.h"
#include "data_preprocessor.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <string.h>

static const char* TAG = "ML_TEST";

// Test data for known gestures
static const struct {
    const char* name;
    float features[32];
} test_gestures[] = {
    // Fist - all fingers bent
    {"FIST", {80, 85, 75, 80, 70, 75, 85, 90, 80, 85, 0, 0, 9.8, 0, 0, 0, 0, 0, 0}},
    
    // Open hand - all fingers straight
    {"OPEN_HAND", {5, 10, 0, 5, 10, 15, 5, 0, 10, 5, 0, 0, 9.8, 0, 0, 0, 0, 0, 0}},
    
    // Thumbs up - thumb straight, others bent
    {"THUMBS_UP", {10, 15, 80, 85, 75, 80, 85, 90, 80, 85, 0, 0, 9.8, 0, 0, 0, 0, 0, 0}},
    
    // Peace sign - index and middle straight, others bent
    {"PEACE", {5, 10, 10, 15, 80, 85, 75, 80, 85, 90, 0, 0, 9.8, 0, 0, 0, 0, 0, 0}},
};

esp_err_t ml_test_performance(void) {
    ESP_LOGI(TAG, "Starting ML performance tests...");
    
    if (!model_manager_is_ready(MODEL_TYPE_STATIC_CNN)) {
        ESP_LOGW(TAG, "Static CNN model not loaded, skipping tests");
        return ESP_ERR_INVALID_STATE;
    }
    
    ml_test_accuracy();
    ml_test_benchmark();
    
    ESP_LOGI(TAG, "ML performance tests complete");
    return ESP_OK;
}

esp_err_t ml_test_accuracy(void) {
    ESP_LOGI(TAG, "Testing model accuracy...");
    
    int correct_predictions = 0;
    int total_tests = sizeof(test_gestures) / sizeof(test_gestures[0]);
    
    for (int i = 0; i < total_tests; i++) {
        ml_input_t input = {0};
        ml_result_t result = {0};
        
        // Prepare test input
        memcpy(input.features, test_gestures[i].features, sizeof(test_gestures[i].features));
        input.feature_count = 32;
        input.timestamp = esp_timer_get_time() / 1000;
        
        // Normalize features
        data_preprocessor_normalize(input.features, input.feature_count);
        
        // Run inference
        esp_err_t ret = ml_inference_run(&input, MODEL_TYPE_STATIC_CNN, &result);
        if (ret == ESP_OK && result.is_valid) {
            ESP_LOGI(TAG, "Test %d: Expected '%s', Got '%s' (%.2f confidence)", 
                    i+1, test_gestures[i].name, result.gesture_name, result.confidence);
            
            if (strcmp(test_gestures[i].name, result.gesture_name) == 0) {
                correct_predictions++;
            }
        } else {
            ESP_LOGW(TAG, "Test %d: Inference failed for %s", i+1, test_gestures[i].name);
        }
    }
    
    float accuracy = (float)correct_predictions / total_tests * 100.0f;
    ESP_LOGI(TAG, "Accuracy: %d/%d (%.1f%%)", correct_predictions, total_tests, accuracy);
    
    return ESP_OK;
}

esp_err_t ml_test_benchmark(void) {
    ESP_LOGI(TAG, "Benchmarking inference speed...");
    
    ml_input_t input = {0};
    ml_result_t result = {0};
    
    // Use first test gesture
    memcpy(input.features, test_gestures[0].features, sizeof(test_gestures[0].features));
    input.feature_count = 32;
    
    // Normalize features
    data_preprocessor_normalize(input.features, input.feature_count);
    
    const int num_iterations = 100;
    uint64_t total_time = 0;
    int successful_inferences = 0;
    
    for (int i = 0; i < num_iterations; i++) {
        input.timestamp = esp_timer_get_time() / 1000;
        
        uint64_t start_time = esp_timer_get_time();
        esp_err_t ret = ml_inference_run(&input, MODEL_TYPE_STATIC_CNN, &result);
        uint64_t end_time = esp_timer_get_time();
        
        if (ret == ESP_OK) {
            total_time += (end_time - start_time);
            successful_inferences++;
        }
    }
    
    if (successful_inferences > 0) {
        uint32_t avg_time_us = (uint32_t)(total_time / successful_inferences);
        float fps = 1000000.0f / avg_time_us;
        
        ESP_LOGI(TAG, "Benchmark results:");
        ESP_LOGI(TAG, "  Successful inferences: %d/%d", successful_inferences, num_iterations);
        ESP_LOGI(TAG, "  Average inference time: %u μs", avg_time_us);
        ESP_LOGI(TAG, "  Estimated FPS: %.1f", fps);
        ESP_LOGI(TAG, "  Memory usage: %u bytes free heap", esp_get_free_heap_size());
    } else {
        ESP_LOGE(TAG, "No successful inferences in benchmark");
    }
    
    return ESP_OK;
}
#include "ml_inference.h"
#include "model_manager.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <string.h>
#include <math.h>

// TensorFlow Lite includes
#include "tensorflow/lite/micro/micro_interpreter.h"

static const char* TAG = "ML_INFERENCE";

// Gesture name mappings
static const char* gesture_names[MAX_GESTURES] = {
    "UNKNOWN",      // 0
    "FIST",         // 1
    "OPEN_HAND",    // 2
    "THUMBS_UP",    // 3
    "THUMBS_DOWN",  // 4
    "PEACE",        // 5
    "OK",           // 6
    "POINTING",     // 7
    "ROCK",         // 8
    "PAPER",        // 9
    "SCISSORS",     // 10
    "HELLO",        // 11
    "GOODBYE",      // 12
    "YES",          // 13
    "NO",           // 14
    "PLEASE",       // 15
    "THANK_YOU",    // 16
    "SORRY",        // 17
    "HELP",         // 18
    "STOP"          // 19
};

// Confidence threshold for valid predictions
#define CONFIDENCE_THRESHOLD 0.6f

esp_err_t ml_inference_init(void) {
    ESP_LOGI(TAG, "ML inference engine initialized");
    return ESP_OK;
}

static void softmax(float* input, int size) {
    float max_val = input[0];
    for (int i = 1; i < size; i++) {
        if (input[i] > max_val) max_val = input[i];
    }
    
    float sum = 0.0f;
    for (int i = 0; i < size; i++) {
        input[i] = expf(input[i] - max_val);
        sum += input[i];
    }
    
    for (int i = 0; i < size; i++) {
        input[i] /= sum;
    }
}

esp_err_t ml_inference_run(const ml_input_t* ml_input, model_type_t model_type, ml_result_t* result) {
    if (!ml_input || !result) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Get the model
    ml_model_t* model = model_manager_get_model(model_type);
    if (!model) {
        ESP_LOGE(TAG, "Model type %d not loaded", model_type);
        return ESP_ERR_INVALID_STATE;
    }
    
    tflite::MicroInterpreter* interpreter = (tflite::MicroInterpreter*)model->interpreter;
    if (!interpreter) {
        ESP_LOGE(TAG, "Invalid interpreter for model type %d", model_type);
        return ESP_ERR_INVALID_STATE;
    }
    
    // Start timing
    uint64_t start_time = esp_timer_get_time();
    
    // Prepare input tensor
    TfLiteTensor* input_tensor = interpreter->input(0);
    if (!input_tensor) {
        ESP_LOGE(TAG, "Failed to get input tensor");
        return ESP_FAIL;
    }
    
    // Copy input data to tensor
    float* input_data = input_tensor->data.f;
    uint32_t input_elements = input_tensor->bytes / sizeof(float);
    
    // Ensure we don't exceed tensor size
    uint32_t copy_count = (ml_input->feature_count < input_elements) ? 
                          ml_input->feature_count : input_elements;
    
    memcpy(input_data, ml_input->features, copy_count * sizeof(float));
    
    // Zero-pad remaining elements if needed
    if (copy_count < input_elements) {
        memset(&input_data[copy_count], 0, (input_elements - copy_count) * sizeof(float));
    }
    
    // Run inference
    TfLiteStatus invoke_status = interpreter->Invoke();
    if (invoke_status != kTfLiteOk) {
        ESP_LOGE(TAG, "Failed to invoke interpreter");
        return ESP_FAIL;
    }
    
    // Get output tensor
    TfLiteTensor* output_tensor = interpreter->output(0);
    if (!output_tensor) {
        ESP_LOGE(TAG, "Failed to get output tensor");
        return ESP_FAIL;
    }
    
    // Process output
    float* output_data = output_tensor->data.f;
    uint32_t output_elements = output_tensor->bytes / sizeof(float);
    
    // Apply softmax to get probabilities
    softmax(output_data, output_elements);
    
    // Find highest probability class
    float max_confidence = 0.0f;
    uint32_t predicted_class = 0;
    
    for (uint32_t i = 0; i < output_elements && i < MAX_GESTURES; i++) {
        if (output_data[i] > max_confidence) {
            max_confidence = output_data[i];
            predicted_class = i;
        }
        result->class_probabilities[i] = output_data[i];
    }
    
    // Calculate inference time
    uint64_t end_time = esp_timer_get_time();
    
    // Fill result structure
    result->gesture_id = predicted_class;
    strncpy(result->gesture_name, gesture_names[predicted_class], 
            MAX_GESTURE_NAME_LEN - 1);
    result->gesture_name[MAX_GESTURE_NAME_LEN - 1] = '\0';
    result->confidence = max_confidence;
    result->inference_time_us = (uint32_t)(end_time - start_time);
    result->is_valid = (max_confidence >= CONFIDENCE_THRESHOLD);
    
    ESP_LOGD(TAG, "Inference: %s (%.2f confidence, %u μs)", 
             result->gesture_name, result->confidence, result->inference_time_us);
    
    return ESP_OK;
}

esp_err_t ml_inference_hybrid(const ml_input_t* static_input, 
                              const ml_input_t* dynamic_input, 
                              ml_result_t* result) {
    if (!static_input || !dynamic_input || !result) {
        return ESP_ERR_INVALID_ARG;
    }
    
    ml_result_t static_result, dynamic_result;
    
    // Run both models
    esp_err_t ret1 = ml_inference_run(static_input, MODEL_TYPE_STATIC_CNN, &static_result);
    esp_err_t ret2 = ml_inference_run(dynamic_input, MODEL_TYPE_DYNAMIC_LSTM, &dynamic_result);
    
    if (ret1 != ESP_OK && ret2 != ESP_OK) {
        return ESP_FAIL;
    }
    
    // Combine results using confidence weighting
    if (ret1 == ESP_OK && ret2 == ESP_OK) {
        // Both models successful - use confidence-weighted combination
        if (static_result.confidence > dynamic_result.confidence) {
            *result = static_result;
            result->confidence = (static_result.confidence + dynamic_result.confidence) / 2.0f;
        } else {
            *result = dynamic_result;
            result->confidence = (static_result.confidence + dynamic_result.confidence) / 2.0f;
        }
    } else if (ret1 == ESP_OK) {
        // Only static model successful
        *result = static_result;
    } else {
        // Only dynamic model successful
        *result = dynamic_result;
    }
    
    return ESP_OK;
}

const char* ml_inference_get_gesture_name(uint32_t gesture_id) {
    if (gesture_id >= MAX_GESTURES) {
        return gesture_names[0]; // "UNKNOWN"
    }
    return gesture_names[gesture_id];
}
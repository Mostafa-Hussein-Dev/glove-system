#ifndef ML_ML_TYPES_H
#define ML_ML_TYPES_H

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "config/system_config.h"

// Maximum number of gestures we can recognize

#define MAX_GESTURE_NAME_LEN 32

// Model types
typedef enum {
    MODEL_TYPE_STATIC_CNN,      // For static hand poses
    MODEL_TYPE_DYNAMIC_LSTM,    // For dynamic gestures
    MODEL_TYPE_HYBRID           // Combined approach
} model_type_t;

// Input data for ML inference
typedef struct {
    float features[64];         // Feature vector (max 64 features)
    uint32_t feature_count;     // Actual number of features
    uint32_t timestamp;         // Timestamp for temporal models
    bool is_sequence_start;     // For LSTM models
    bool is_sequence_end;       // For LSTM models
} ml_input_t;

// ML inference result
typedef struct {
    uint32_t gesture_id;                        // Predicted gesture ID
    char gesture_name[MAX_GESTURE_NAME_LEN];    // Gesture name
    float confidence;                           // Confidence score (0.0-1.0)
    float class_probabilities[MAX_GESTURES];    // Individual class probabilities
    uint32_t inference_time_us;                 // Inference time in microseconds
    bool is_valid;                              // Whether result is valid
} ml_result_t;

// Model information
typedef struct {
    model_type_t type;
    const char* name;
    const uint8_t* model_data;
    size_t model_size;
    uint32_t input_size;
    uint32_t output_size;
    bool is_loaded;
    void* interpreter;          // TFLite interpreter
    void* input_tensor;         // Input tensor pointer
    void* output_tensor;        // Output tensor pointer
} ml_model_t;

#endif /* ML_ML_TYPES_H */
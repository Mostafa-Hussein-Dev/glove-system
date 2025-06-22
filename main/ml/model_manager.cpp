#include "esp_system.h"
#include "model_manager.h"
#include "esp_log.h"
#include "esp_spiffs.h"
#include "esp_heap_caps.h"
#include <string.h>
#include <stdio.h>

// TensorFlow Lite includes
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/schema/schema_generated.h"

static const char* TAG = "MODEL_MANAGER";

// Global models storage
static ml_model_t models[3];  // Static, Dynamic, Hybrid
static bool manager_initialized = false;

// TensorFlow Lite global objects
static tflite::ErrorReporter* error_reporter = nullptr;
static tflite::MicroMutableOpResolver<10> resolver;  // Specify max ops

// Memory arena for TensorFlow Lite (adjust size based on your models)
constexpr int kTensorArenaSize = 100 * 1024;  // 100KB
static uint8_t tensor_arena[kTensorArenaSize];

esp_err_t model_manager_init(void) {
    if (manager_initialized) {
        return ESP_OK;
    }
    
    // Initialize models array
    memset(models, 0, sizeof(models));
    
    // Add required operations to resolver
    resolver.AddFullyConnected();
    resolver.AddSoftmax();
    resolver.AddQuantize();
    resolver.AddDequantize();
    resolver.AddReshape();
    resolver.AddAdd();
    resolver.AddMul();
    resolver.AddLogistic();

    // Initialize SPIFFS if not already done
    esp_vfs_spiffs_conf_t conf = {
        .base_path = "/spiffs",
        .partition_label = NULL,
        .max_files = 5,
        .format_if_mount_failed = true
    };
    
    esp_err_t ret = esp_vfs_spiffs_register(&conf);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "Failed to initialize SPIFFS (%s)", esp_err_to_name(ret));
        return ret;
    }
    
    manager_initialized = true;
    ESP_LOGI(TAG, "Model manager initialized");
    return ESP_OK;
}

static esp_err_t load_model_from_file(const char* filepath, uint8_t** model_data, size_t* model_size) {
    FILE* file = fopen(filepath, "rb");
    if (!file) {
        ESP_LOGE(TAG, "Failed to open model file: %s", filepath);
        return ESP_FAIL;
    }
    
    // Get file size
    fseek(file, 0, SEEK_END);
    *model_size = ftell(file);
    fseek(file, 0, SEEK_SET);
    
    // Allocate memory for model
    *model_data = (uint8_t*)heap_caps_malloc(*model_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!*model_data) {
        ESP_LOGE(TAG, "Failed to allocate memory for model (%zu bytes)", *model_size);
        fclose(file);
        return ESP_ERR_NO_MEM;
    }
    
    // Read model data
    size_t read_size = fread(*model_data, 1, *model_size, file);
    fclose(file);
    
    if (read_size != *model_size) {
        ESP_LOGE(TAG, "Failed to read complete model file");
        free(*model_data);
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "Loaded model from %s (%zu bytes)", filepath, *model_size);
    return ESP_OK;
}

esp_err_t model_manager_load_model(const char* model_name, model_type_t model_type) {
    if (!manager_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (model_type >= 3) {
        return ESP_ERR_INVALID_ARG;
    }
    
    ml_model_t* model = &models[model_type];
    
    // Build file path
    char filepath[64];
    snprintf(filepath, sizeof(filepath), "/spiffs/%s", model_name);
    
    // Load model data from file
    uint8_t* model_data;
    size_t model_size;
    esp_err_t ret = load_model_from_file(filepath, &model_data, &model_size);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Initialize TensorFlow Lite model
    const tflite::Model* tflite_model = tflite::GetModel(model_data);
    if (tflite_model->version() != TFLITE_SCHEMA_VERSION) {
        ESP_LOGE(TAG, "Model schema version mismatch!");
        free(model_data);
        return ESP_FAIL;
    }
    
    // Create interpreter
    static tflite::MicroInterpreter* interpreter = nullptr;
    interpreter = new tflite::MicroInterpreter(
        tflite_model, resolver, tensor_arena, kTensorArenaSize);
    
    if (!interpreter) {
        ESP_LOGE(TAG, "Failed to create TensorFlow Lite interpreter");
        free(model_data);
        return ESP_FAIL;
    }
    
    // Allocate tensors
    TfLiteStatus allocate_status = interpreter->AllocateTensors();
    if (allocate_status != kTfLiteOk) {
        ESP_LOGE(TAG, "Failed to allocate tensors");
        delete interpreter;
        free(model_data);
        return ESP_FAIL;
    }
    
    // Fill model structure
    model->type = model_type;
    model->name = model_name;
    model->model_data = model_data;
    model->model_size = model_size;
    model->is_loaded = true;
    model->interpreter = (void*)interpreter;
    model->input_tensor = (void*)interpreter->input(0);
    model->output_tensor = (void*)interpreter->output(0);
    
    // Get tensor dimensions
    TfLiteTensor* input = interpreter->input(0);
    TfLiteTensor* output = interpreter->output(0);
    model->input_size = input->bytes;
    model->output_size = output->bytes;
    
    ESP_LOGI(TAG, "Model loaded successfully: %s (type: %d)", model_name, model_type);
    ESP_LOGI(TAG, "Input size: %d bytes, Output size: %d bytes", 
             model->input_size, model->output_size);
    
    return ESP_OK;
}

ml_model_t* model_manager_get_model(model_type_t type) {
    if (!manager_initialized || type >= 3) {
        return NULL;
    }
    
    return models[type].is_loaded ? &models[type] : NULL;
}

bool model_manager_is_ready(model_type_t type) {
    ml_model_t* model = model_manager_get_model(type);
    return model != NULL && model->is_loaded;
}

esp_err_t model_manager_optimize_memory(void) {
    ESP_LOGI(TAG, "Optimizing memory usage...");
    
    // Print memory usage before optimization
    size_t free_heap_before = esp_get_free_heap_size();
    size_t free_spiram_before = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
    
    ESP_LOGI(TAG, "Memory before optimization:");
    ESP_LOGI(TAG, "  Free heap: %zu bytes", free_heap_before);
    ESP_LOGI(TAG, "  Free SPIRAM: %zu bytes", free_spiram_before);

    // Print memory usage after optimization
    size_t free_heap_after = esp_get_free_heap_size();
    size_t free_spiram_after = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
    
    ESP_LOGI(TAG, "Memory after optimization:");
    ESP_LOGI(TAG, "  Free heap: %zu bytes (+%zd)", 
             free_heap_after, free_heap_after - free_heap_before);
    ESP_LOGI(TAG, "  Free SPIRAM: %zu bytes (+%zd)", 
             free_spiram_after, free_spiram_after - free_spiram_before);
    
    return ESP_OK;
}

esp_err_t model_manager_cleanup(void) {
    for (int i = 0; i < 3; i++) {
        if (models[i].is_loaded) {
            if (models[i].interpreter) {
                delete (tflite::MicroInterpreter*)models[i].interpreter;
            }
            if (models[i].model_data) {
                free((void*)models[i].model_data);
            }
            memset(&models[i], 0, sizeof(ml_model_t));
        }
    }
    
    manager_initialized = false;
    ESP_LOGI(TAG, "Model manager cleanup complete");
    return ESP_OK;
}
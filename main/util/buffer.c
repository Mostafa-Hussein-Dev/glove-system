#include "util/buffer.h"
#include <stdlib.h>
#include <string.h>
#include "esp_log.h"

static const char *TAG = "BUFFER";

esp_err_t buffer_init(sensor_data_buffer_t* buffer, size_t capacity) {
    if (buffer == NULL || capacity == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    
    buffer->buffer = (sensor_data_t*)malloc(capacity * sizeof(sensor_data_t));
    if (buffer->buffer == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for buffer");
        return ESP_ERR_NO_MEM;
    }
    
    buffer->capacity = capacity;
    buffer->size = 0;
    buffer->head = 0;
    buffer->tail = 0;
    
    return ESP_OK;
}

esp_err_t buffer_free(sensor_data_buffer_t* buffer) {
    if (buffer == NULL || buffer->buffer == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Free camera frame buffers if they exist
    for (size_t i = 0; i < buffer->capacity; i++) {
        if (buffer->buffer[i].camera_data_valid && 
            buffer->buffer[i].camera_data.buffer != NULL) {  // Changed from frame_buffer to buffer
            free(buffer->buffer[i].camera_data.buffer);      // Changed from frame_buffer to buffer
            buffer->buffer[i].camera_data.buffer = NULL;     // Changed from frame_buffer to buffer
        }
    }
    
    free(buffer->buffer);
    buffer->buffer = NULL;
    buffer->capacity = 0;
    buffer->size = 0;
    buffer->head = 0;
    buffer->tail = 0;

    return ESP_OK;
}

esp_err_t buffer_push(sensor_data_buffer_t* buffer, const sensor_data_t* data) {
    if (buffer == NULL || data == NULL || buffer->buffer == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (buffer_is_full(buffer)) {
        // Buffer is full, overwrite oldest data
        // First free any camera frame buffer in the oldest slot to prevent memory leak
        if (buffer->buffer[buffer->tail].camera_data_valid && 
            buffer->buffer[buffer->tail].camera_data.buffer != NULL) {  // Changed from frame_buffer to buffer
            free(buffer->buffer[buffer->tail].camera_data.buffer);      // Changed from frame_buffer to buffer
            buffer->buffer[buffer->tail].camera_data.buffer = NULL;     // Changed from frame_buffer to buffer
        }
        
        // Move tail forward
        buffer->tail = (buffer->tail + 1) % buffer->capacity;
        buffer->size--; // Reduce size as we're about to overwrite
    }
    
    // Copy data to the head position
    memcpy(&buffer->buffer[buffer->head], data, sizeof(sensor_data_t));
    
    // Special handling for camera frame buffer - need a deep copy
    if (data->camera_data_valid && data->camera_data.buffer != NULL && data->camera_data.buffer_size > 0) {  // Changed from frame_buffer to buffer
        buffer->buffer[buffer->head].camera_data.buffer = (uint8_t*)malloc(data->camera_data.buffer_size);  // Changed from frame_buffer to buffer
        if (buffer->buffer[buffer->head].camera_data.buffer == NULL) {  // Changed from frame_buffer to buffer
            ESP_LOGE(TAG, "Failed to allocate memory for camera frame");
            buffer->buffer[buffer->head].camera_data_valid = false;
        } else {
            memcpy(buffer->buffer[buffer->head].camera_data.buffer,  // Changed from frame_buffer to buffer
                   data->camera_data.buffer,                         // Changed from frame_buffer to buffer
                   data->camera_data.buffer_size);
        }
    }
    
    // Move head forward
    buffer->head = (buffer->head + 1) % buffer->capacity;
    buffer->size++;
    
    return ESP_OK;
}

esp_err_t buffer_pop(sensor_data_buffer_t* buffer, sensor_data_t* data) {
    if (buffer == NULL || data == NULL || buffer->buffer == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (buffer_is_empty(buffer)) {
        return ESP_ERR_NOT_FOUND;
    }
    
    // Copy data from tail position
    memcpy(data, &buffer->buffer[buffer->tail], sizeof(sensor_data_t));
    
    // Special handling for camera frame buffer - transfer ownership
    if (buffer->buffer[buffer->tail].camera_data_valid) {
        data->camera_data.buffer = buffer->buffer[buffer->tail].camera_data.buffer;  // Changed from frame_buffer to buffer
        buffer->buffer[buffer->tail].camera_data.buffer = NULL;                     // Changed from frame_buffer to buffer
    }
    
    // Move tail forward
    buffer->tail = (buffer->tail + 1) % buffer->capacity;
    buffer->size--;
    
    return ESP_OK;
}

esp_err_t buffer_get(const sensor_data_buffer_t* buffer, size_t index, sensor_data_t* data) {
    if (buffer == NULL || data == NULL || buffer->buffer == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (buffer_is_empty(buffer) || index >= buffer->size) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Calculate the actual index in the circular buffer
    // index 0 = oldest data (at tail), index (size-1) = newest data
    size_t actual_index = (buffer->tail + index) % buffer->capacity;
    
    // Copy data from the specified position
    memcpy(data, &buffer->buffer[actual_index], sizeof(sensor_data_t));
    
    // Special handling for camera frame buffer - make a copy of the pointer
    // Note: This creates a shared pointer - be careful with memory management
    if (buffer->buffer[actual_index].camera_data_valid) {
        data->camera_data.buffer = buffer->buffer[actual_index].camera_data.buffer;
    }
    
    return ESP_OK;
}

bool buffer_is_empty(const sensor_data_buffer_t* buffer) {
    if (buffer == NULL) {
        return true;
    }
    return buffer->size == 0;
}

bool buffer_is_full(const sensor_data_buffer_t* buffer) {
    if (buffer == NULL) {
        return false;
    }
    return buffer->size == buffer->capacity;
}

size_t buffer_get_size(const sensor_data_buffer_t* buffer) {
    if (buffer == NULL) {
        return 0;
    }
    return buffer->size;
}


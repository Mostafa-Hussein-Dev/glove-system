
#include "util/buffer.h"
#include <stdlib.h>
#include <string.h>
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static const char *TAG = "BUFFER_FIXED";

// Global statistics for monitoring
static struct {
    size_t total_allocations;
    size_t total_deallocations;
    size_t current_camera_buffers;
    size_t max_camera_buffers;
    size_t overflow_count;
    size_t bytes_allocated;
    size_t bytes_freed;
    size_t peak_memory_usage;
} buffer_stats = {0};

// Global mutex for thread-safe statistics
static SemaphoreHandle_t stats_mutex = NULL;

/**
 * @brief Camera buffer reference counting structure
 */
typedef struct camera_buffer_ref {
    uint8_t* buffer;
    size_t size;
    uint32_t ref_count;
    uint32_t magic;  // For corruption detection
    struct camera_buffer_ref* next; // For tracking allocated buffers
} camera_buffer_ref_t;

#define CAMERA_BUFFER_MAGIC 0xCAFEBABE
#define MAX_CAMERA_BUFFERS 10  // Limit total camera buffers

// Linked list of all allocated camera buffers for tracking
static camera_buffer_ref_t* allocated_buffers = NULL;
static SemaphoreHandle_t buffer_list_mutex = NULL;

/**
 * @brief Initialize buffer statistics and mutexes
 */
static void buffer_stats_init(void) {
    if (stats_mutex == NULL) {
        stats_mutex = xSemaphoreCreateMutex();
    }
    if (buffer_list_mutex == NULL) {
        buffer_list_mutex = xSemaphoreCreateMutex();
    }
}

/**
 * @brief Update buffer statistics safely
 */
static void buffer_stats_update(int alloc_delta, size_t bytes_delta, bool is_allocation) {
    if (stats_mutex == NULL) buffer_stats_init();
    
    if (xSemaphoreTake(stats_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        if (is_allocation) {
            buffer_stats.total_allocations++;
            buffer_stats.current_camera_buffers += alloc_delta;
            buffer_stats.bytes_allocated += bytes_delta;
            
            if (buffer_stats.current_camera_buffers > buffer_stats.max_camera_buffers) {
                buffer_stats.max_camera_buffers = buffer_stats.current_camera_buffers;
            }
            
            size_t current_usage = buffer_stats.bytes_allocated - buffer_stats.bytes_freed;
            if (current_usage > buffer_stats.peak_memory_usage) {
                buffer_stats.peak_memory_usage = current_usage;
            }
        } else {
            buffer_stats.total_deallocations++;
            buffer_stats.current_camera_buffers -= alloc_delta;
            buffer_stats.bytes_freed += bytes_delta;
        }
        xSemaphoreGive(stats_mutex);
    }
}

/**
 * @brief Create camera buffer with reference counting
 */
static camera_buffer_ref_t* camera_buffer_ref_create(const uint8_t* data, size_t size) {
    if (data == NULL || size == 0) {
        ESP_LOGE(TAG, "Invalid camera buffer parameters");
        return NULL;
    }
    
    // Check memory limits
    if (buffer_stats.current_camera_buffers >= MAX_CAMERA_BUFFERS) {
        ESP_LOGW(TAG, "Camera buffer limit reached (%d), refusing allocation", MAX_CAMERA_BUFFERS);
        return NULL;
    }
    
    // Check available heap
    size_t free_heap = heap_caps_get_free_size(MALLOC_CAP_DEFAULT);
    size_t required = sizeof(camera_buffer_ref_t) + size;
    
    if (free_heap < required + 10240) { // Keep 10KB safety margin
        ESP_LOGW(TAG, "Insufficient heap for camera buffer (need %zu, have %zu)", required, free_heap);
        return NULL;
    }
    
    // Allocate reference structure + buffer in one block for efficiency
    camera_buffer_ref_t* ref = (camera_buffer_ref_t*)malloc(required);
    if (ref == NULL) {
        ESP_LOGE(TAG, "Failed to allocate camera buffer reference (%zu bytes)", required);
        return NULL;
    }
    
    // Set up reference structure
    ref->buffer = (uint8_t*)(ref + 1);  // Buffer follows the ref structure
    ref->size = size;
    ref->ref_count = 1;
    ref->magic = CAMERA_BUFFER_MAGIC;
    ref->next = NULL;
    
    // Copy data
    memcpy(ref->buffer, data, size);
    
    // Add to tracking list
    if (buffer_list_mutex && xSemaphoreTake(buffer_list_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        ref->next = allocated_buffers;
        allocated_buffers = ref;
        xSemaphoreGive(buffer_list_mutex);
    }
    
    // Update statistics
    buffer_stats_update(1, required, true);
    
    ESP_LOGD(TAG, "Created camera buffer ref: %p, size: %zu, total buffers: %zu", 
             ref, size, buffer_stats.current_camera_buffers);
    
    return ref;
}

/**
 * @brief Increment reference count for camera buffer
 */
static camera_buffer_ref_t* camera_buffer_ref_retain(camera_buffer_ref_t* ref) {
    if (ref == NULL || ref->magic != CAMERA_BUFFER_MAGIC) {
        ESP_LOGE(TAG, "Invalid camera buffer reference");
        return NULL;
    }
    
    ref->ref_count++;
    ESP_LOGD(TAG, "Retained camera buffer ref: %p, ref_count: %u", ref, ref->ref_count);
    return ref;
}

/**
 * @brief Decrement reference count and free if zero
 */
static void camera_buffer_ref_release(camera_buffer_ref_t* ref) {
    if (ref == NULL || ref->magic != CAMERA_BUFFER_MAGIC) {
        ESP_LOGE(TAG, "Invalid camera buffer reference for release");
        return;
    }
    
    ref->ref_count--;
    ESP_LOGD(TAG, "Released camera buffer ref: %p, ref_count: %u", ref, ref->ref_count);
    
    if (ref->ref_count == 0) {
        // Remove from tracking list
        if (buffer_list_mutex && xSemaphoreTake(buffer_list_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            camera_buffer_ref_t** current = &allocated_buffers;
            while (*current) {
                if (*current == ref) {
                    *current = ref->next;
                    break;
                }
                current = &(*current)->next;
            }
            xSemaphoreGive(buffer_list_mutex);
        }
        
        // Update statistics before freeing
        size_t freed_bytes = sizeof(camera_buffer_ref_t) + ref->size;
        buffer_stats_update(1, freed_bytes, false);
        
        // Clear magic to detect use-after-free
        ref->magic = 0xDEADBEEF;
        
        ESP_LOGD(TAG, "Freeing camera buffer ref: %p, size: %zu", ref, ref->size);
        free(ref);
    }
}

/**
 * @brief Initialize a circular buffer for sensor data
 */
esp_err_t buffer_init(sensor_data_buffer_t* buffer, size_t capacity) {
    if (buffer == NULL || capacity == 0) {
        ESP_LOGE(TAG, "Invalid buffer parameters");
        return ESP_ERR_INVALID_ARG;
    }
    
    // Initialize global components if needed
    buffer_stats_init();
    
    // Allocate buffer array
    size_t buffer_size = capacity * sizeof(sensor_data_t);
    buffer->buffer = (sensor_data_t*)calloc(capacity, sizeof(sensor_data_t));
    if (buffer->buffer == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for buffer (%zu bytes)", buffer_size);
        return ESP_ERR_NO_MEM;
    }
    
    // Initialize buffer structure
    buffer->capacity = capacity;
    buffer->size = 0;
    buffer->head = 0;
    buffer->tail = 0;
    
    ESP_LOGI(TAG, "Buffer initialized: capacity=%zu, size=%zu bytes", capacity, buffer_size);
    return ESP_OK;
}

/**
 * @brief Free and cleanup circular buffer
 */
esp_err_t buffer_free(sensor_data_buffer_t* buffer) {
    if (buffer == NULL || buffer->buffer == NULL) {
        ESP_LOGE(TAG, "Invalid buffer for cleanup");
        return ESP_ERR_INVALID_ARG;
    }
    
    ESP_LOGI(TAG, "Freeing buffer with %zu items", buffer->size);
    
    // Free all camera frame buffers in the buffer
    for (size_t i = 0; i < buffer->capacity; i++) {
        if (buffer->buffer[i].camera_data_valid && 
            buffer->buffer[i].camera_data.buffer != NULL) {
            
            // Release camera buffer reference
            camera_buffer_ref_t* ref = (camera_buffer_ref_t*)
                ((uint8_t*)buffer->buffer[i].camera_data.buffer - sizeof(camera_buffer_ref_t));
            
            if (ref->magic == CAMERA_BUFFER_MAGIC) {
                camera_buffer_ref_release(ref);
            } else {
                ESP_LOGW(TAG, "Found invalid camera buffer during cleanup at index %zu", i);
            }
            
            buffer->buffer[i].camera_data.buffer = NULL;
            buffer->buffer[i].camera_data_valid = false;
        }
    }
    
    // Free main buffer
    free(buffer->buffer);
    
    // Reset buffer structure
    buffer->buffer = NULL;
    buffer->capacity = 0;
    buffer->size = 0;
    buffer->head = 0;
    buffer->tail = 0;

    ESP_LOGI(TAG, "Buffer cleanup completed");
    return ESP_OK;
}

/**
 * @brief Push data into the circular buffer with proper memory management
 */
esp_err_t buffer_push(sensor_data_buffer_t* buffer, const sensor_data_t* data) {
    if (buffer == NULL || data == NULL || buffer->buffer == NULL) {
        ESP_LOGE(TAG, "Invalid parameters for buffer_push");
        return ESP_ERR_INVALID_ARG;
    }
    
    // Handle buffer overflow
    if (buffer_is_full(buffer)) {
        buffer_stats.overflow_count++;
        ESP_LOGW(TAG, "Buffer overflow detected (count: %zu), removing oldest data", 
                buffer_stats.overflow_count);
        
        // Free camera buffer in the slot we're about to overwrite
        if (buffer->buffer[buffer->tail].camera_data_valid && 
            buffer->buffer[buffer->tail].camera_data.buffer != NULL) {
            
            camera_buffer_ref_t* ref = (camera_buffer_ref_t*)
                ((uint8_t*)buffer->buffer[buffer->tail].camera_data.buffer - sizeof(camera_buffer_ref_t));
            
            if (ref->magic == CAMERA_BUFFER_MAGIC) {
                camera_buffer_ref_release(ref);
            } else {
                ESP_LOGE(TAG, "Corrupt camera buffer reference during overflow");
            }
            
            buffer->buffer[buffer->tail].camera_data.buffer = NULL;
            buffer->buffer[buffer->tail].camera_data_valid = false;
        }
        
        // Move tail forward (overwrite oldest)
        buffer->tail = (buffer->tail + 1) % buffer->capacity;
        buffer->size--;
    }
    
    // Copy data to the head position
    memcpy(&buffer->buffer[buffer->head], data, sizeof(sensor_data_t));
    
    // Special handling for camera frame buffer - create reference-counted copy
    if (data->camera_data_valid && data->camera_data.buffer != NULL && data->camera_data.buffer_size > 0) {
        camera_buffer_ref_t* ref = camera_buffer_ref_create(data->camera_data.buffer, data->camera_data.buffer_size);
        
        if (ref != NULL) {
            // Point to the buffer inside the reference structure
            buffer->buffer[buffer->head].camera_data.buffer = ref->buffer;
            ESP_LOGD(TAG, "Camera buffer copied with ref counting: %p", ref);
        } else {
            ESP_LOGW(TAG, "Failed to create camera buffer reference, marking as invalid");
            buffer->buffer[buffer->head].camera_data_valid = false;
            buffer->buffer[buffer->head].camera_data.buffer = NULL;
        }
    }
    
    // Move head forward
    buffer->head = (buffer->head + 1) % buffer->capacity;
    buffer->size++;
    
    ESP_LOGD(TAG, "Buffer push completed: size=%zu/%zu, head=%zu, tail=%zu", 
             buffer->size, buffer->capacity, buffer->head, buffer->tail);
    
    return ESP_OK;
}

/**
 * @brief Pop data from the circular buffer with ownership transfer
 */
esp_err_t buffer_pop(sensor_data_buffer_t* buffer, sensor_data_t* data) {
    if (buffer == NULL || data == NULL || buffer->buffer == NULL) {
        ESP_LOGE(TAG, "Invalid parameters for buffer_pop");
        return ESP_ERR_INVALID_ARG;
    }
    
    if (buffer_is_empty(buffer)) {
        return ESP_ERR_NOT_FOUND;
    }
    
    // Copy data from tail position
    memcpy(data, &buffer->buffer[buffer->tail], sizeof(sensor_data_t));
    
    // Transfer camera buffer ownership (no need to copy, just transfer reference)
    if (buffer->buffer[buffer->tail].camera_data_valid && 
        buffer->buffer[buffer->tail].camera_data.buffer != NULL) {
        
        // Transfer ownership - caller now responsible for releasing
        data->camera_data.buffer = buffer->buffer[buffer->tail].camera_data.buffer;
        
        // Clear from buffer (ownership transferred)
        buffer->buffer[buffer->tail].camera_data.buffer = NULL;
        buffer->buffer[buffer->tail].camera_data_valid = false;
    }
    
    // Move tail forward
    buffer->tail = (buffer->tail + 1) % buffer->capacity;
    buffer->size--;
    
    ESP_LOGD(TAG, "Buffer pop completed: size=%zu/%zu", buffer->size, buffer->capacity);
    return ESP_OK;
}

/**
 * @brief Get data at a specific index without transferring ownership
 */
esp_err_t buffer_get(const sensor_data_buffer_t* buffer, size_t index, sensor_data_t* data) {
    if (buffer == NULL || data == NULL || buffer->buffer == NULL) {
        ESP_LOGE(TAG, "Invalid parameters for buffer_get");
        return ESP_ERR_INVALID_ARG;
    }
    
    if (buffer_is_empty(buffer) || index >= buffer->size) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Calculate the actual index in the circular buffer
    size_t actual_index = (buffer->tail + index) % buffer->capacity;
    
    // Copy data from the specified position
    memcpy(data, &buffer->buffer[actual_index], sizeof(sensor_data_t));
    
    // For camera buffer, create a new reference (increment ref count)
    if (buffer->buffer[actual_index].camera_data_valid && 
        buffer->buffer[actual_index].camera_data.buffer != NULL) {
        
        camera_buffer_ref_t* ref = (camera_buffer_ref_t*)
            ((uint8_t*)buffer->buffer[actual_index].camera_data.buffer - sizeof(camera_buffer_ref_t));
        
        if (ref->magic == CAMERA_BUFFER_MAGIC) {
            camera_buffer_ref_retain(ref);
            data->camera_data.buffer = ref->buffer;
        } else {
            ESP_LOGW(TAG, "Invalid camera buffer reference in buffer_get");
            data->camera_data_valid = false;
            data->camera_data.buffer = NULL;
        }
    }
    
    return ESP_OK;
}

/**
 * @brief Clear all data from buffer
 */
esp_err_t buffer_clear(sensor_data_buffer_t* buffer) {
    if (buffer == NULL || buffer->buffer == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    ESP_LOGI(TAG, "Clearing buffer with %zu items", buffer->size);
    
    // Release all camera buffers
    for (size_t i = 0; i < buffer->capacity; i++) {
        if (buffer->buffer[i].camera_data_valid && 
            buffer->buffer[i].camera_data.buffer != NULL) {
            
            camera_buffer_ref_t* ref = (camera_buffer_ref_t*)
                ((uint8_t*)buffer->buffer[i].camera_data.buffer - sizeof(camera_buffer_ref_t));
            
            if (ref->magic == CAMERA_BUFFER_MAGIC) {
                camera_buffer_ref_release(ref);
            }
            
            buffer->buffer[i].camera_data.buffer = NULL;
            buffer->buffer[i].camera_data_valid = false;
        }
    }
    
    // Reset buffer pointers
    buffer->size = 0;
    buffer->head = 0;
    buffer->tail = 0;
    
    return ESP_OK;
}

/**
 * @brief Check if buffer is empty
 */
bool buffer_is_empty(const sensor_data_buffer_t* buffer) {
    if (buffer == NULL) {
        return true;
    }
    return buffer->size == 0;
}

/**
 * @brief Check if buffer is full
 */
bool buffer_is_full(const sensor_data_buffer_t* buffer) {
    if (buffer == NULL) {
        return false;
    }
    return buffer->size == buffer->capacity;
}

/**
 * @brief Get current size of buffer
 */
size_t buffer_get_size(const sensor_data_buffer_t* buffer) {
    if (buffer == NULL) {
        return 0;
    }
    return buffer->size;
}

/**
 * @brief Get buffer statistics for monitoring
 */
void buffer_get_stats(buffer_stats_t* stats) {
    if (stats == NULL) return;
    
    if (stats_mutex && xSemaphoreTake(stats_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        stats->total_allocations = buffer_stats.total_allocations;
        stats->total_deallocations = buffer_stats.total_deallocations;
        stats->current_camera_buffers = buffer_stats.current_camera_buffers;
        stats->max_camera_buffers = buffer_stats.max_camera_buffers;
        stats->overflow_count = buffer_stats.overflow_count;
        stats->bytes_allocated = buffer_stats.bytes_allocated;
        stats->bytes_freed = buffer_stats.bytes_freed;
        stats->peak_memory_usage = buffer_stats.peak_memory_usage;
        stats->memory_leaked = (buffer_stats.bytes_allocated > buffer_stats.bytes_freed) ? 
                              (buffer_stats.bytes_allocated - buffer_stats.bytes_freed) : 0;
        xSemaphoreGive(stats_mutex);
    }
}

/**
 * @brief Print buffer statistics for debugging
 */
void buffer_print_stats(void) {
    buffer_stats_t stats;
    buffer_get_stats(&stats);
    
    ESP_LOGI(TAG, "=== BUFFER STATISTICS ===");
    ESP_LOGI(TAG, "Allocations: %zu", stats.total_allocations);
    ESP_LOGI(TAG, "Deallocations: %zu", stats.total_deallocations);
    ESP_LOGI(TAG, "Current camera buffers: %zu", stats.current_camera_buffers);
    ESP_LOGI(TAG, "Max camera buffers: %zu", stats.max_camera_buffers);
    ESP_LOGI(TAG, "Buffer overflows: %zu", stats.overflow_count);
    ESP_LOGI(TAG, "Bytes allocated: %zu", stats.bytes_allocated);
    ESP_LOGI(TAG, "Bytes freed: %zu", stats.bytes_freed);
    ESP_LOGI(TAG, "Peak memory usage: %zu", stats.peak_memory_usage);
    ESP_LOGI(TAG, "Memory leaked: %zu", stats.memory_leaked);
    ESP_LOGI(TAG, "=========================");
}

/**
 * @brief Emergency cleanup - force free all camera buffers
 */
void buffer_emergency_cleanup(void) {
    ESP_LOGW(TAG, "Emergency buffer cleanup initiated");
    
    if (buffer_list_mutex && xSemaphoreTake(buffer_list_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        camera_buffer_ref_t* current = allocated_buffers;
        size_t freed_count = 0;
        
        while (current) {
            camera_buffer_ref_t* next = current->next;
            
            ESP_LOGW(TAG, "Force freeing camera buffer: %p (ref_count: %u)", 
                     current, current->ref_count);
            
            current->magic = 0xDEADBEEF;
            free(current);
            freed_count++;
            
            current = next;
        }
        
        allocated_buffers = NULL;
        buffer_stats.current_camera_buffers = 0;
        
        ESP_LOGW(TAG, "Emergency cleanup completed: freed %zu buffers", freed_count);
        xSemaphoreGive(buffer_list_mutex);
    }
}
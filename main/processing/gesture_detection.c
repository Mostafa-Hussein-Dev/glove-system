#include "processing/gesture_detection.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "util/buffer.h"
#include "util/debug.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "nvs.h"

static const char *TAG = "GESTURE_DETECT";

// Gesture detection state
static bool gesture_detection_initialized = false;

// Number of gestures in our vocabulary
#define MAX_GESTURES 50
#define CURRENT_GESTURES 10  // Currently implemented gestures

// NVS namespace for gesture templates
#define GESTURE_NVS_NAMESPACE "gestures"

// Gesture template structure
typedef struct {
    char name[32];           // Gesture name
    float template_features[FEATURE_BUFFER_SIZE];  // Template feature vector
    uint16_t feature_count;  // Number of features used
    bool is_dynamic;         // Static vs dynamic gesture
} gesture_template_t;

// Array of gesture templates
static gesture_template_t gesture_templates[MAX_GESTURES];
static uint8_t gesture_count = 0;

// Last detected gesture for debouncing
static char last_detected_gesture[32] = {0};
static uint32_t last_detection_time = 0;
static const uint32_t GESTURE_DEBOUNCE_TIME_MS = 500;

// Helper functions
static esp_err_t save_gesture_templates(void);
static esp_err_t load_gesture_templates(void);
static void init_default_gestures(void);
static float calculate_similarity(const float* features1, const float* features2, uint16_t count);

esp_err_t gesture_detection_init(void) {
    // Initialize templates array
    memset(gesture_templates, 0, sizeof(gesture_templates));
    gesture_count = 0;
    
    // Try to load templates from NVS
    esp_err_t ret = load_gesture_templates();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to load gesture templates from NVS, initializing defaults");
        init_default_gestures();
        
        // Save default templates to NVS
        save_gesture_templates();
    }
    
    gesture_detection_initialized = true;
    ESP_LOGI(TAG, "Gesture detection initialized with %d gestures", gesture_count);
    
    return ESP_OK;
}

esp_err_t gesture_detection_deinit(void) {
    gesture_detection_initialized = false;
    ESP_LOGI(TAG, "Gesture detection deinitialized");
    
    return ESP_OK;
}

esp_err_t gesture_detection_process(feature_vector_t *feature_vector, processing_result_t *result) {
    if (!gesture_detection_initialized || feature_vector == NULL || result == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Initialize result
    memset(result, 0, sizeof(processing_result_t));
    
    // Get current time for timestamps and debouncing
    uint32_t current_time = esp_timer_get_time() / 1000;
    
    // Simple threshold-based detection for specific gestures
    // This is a basic approach using heuristics rather than ML
    
    float best_match_score = 0.0f;
    int best_match_index = -1;
    
    // Compare input features to each template using Euclidean distance
    for (int i = 0; i < gesture_count; i++) {
        // Skip if the feature counts don't match enough
        if (gesture_templates[i].feature_count < 10 || feature_vector->feature_count < 10) {
            continue;
        }
        
        // Calculate similarity (higher is better)
        float similarity = calculate_similarity(
            feature_vector->features, 
            gesture_templates[i].template_features,
            (gesture_templates[i].feature_count < feature_vector->feature_count) ? 
                gesture_templates[i].feature_count : feature_vector->feature_count
        );
        
        // Keep track of best match
        if (similarity > best_match_score) {
            best_match_score = similarity;
            best_match_index = i;
        }
    }
    
    // If we found a good match and it passes our confidence threshold
    if (best_match_index >= 0 && best_match_score >= CONFIDENCE_THRESHOLD) {
        // Check for debouncing (avoid rapid repeated detections of the same gesture)
        if (strcmp(last_detected_gesture, gesture_templates[best_match_index].name) == 0) {
            // Same gesture as last time, check time elapsed
            if (current_time - last_detection_time < GESTURE_DEBOUNCE_TIME_MS) {
                // Not enough time elapsed, ignore this detection
                return ESP_OK;
            }
        }
        
        // Fill in the result
        result->gesture_id = best_match_index;
        strncpy(result->gesture_name, gesture_templates[best_match_index].name, sizeof(result->gesture_name) - 1);
        result->confidence = best_match_score;
        result->is_dynamic = gesture_templates[best_match_index].is_dynamic;
        result->duration_ms = 0;  // We're not tracking duration in this simplified version
        result->timestamp = current_time;
        
        // Save for debouncing
        strncpy(last_detected_gesture, result->gesture_name, sizeof(last_detected_gesture) - 1);
        last_detection_time = current_time;
        
        ESP_LOGI(TAG, "Gesture detected: %s (confidence: %.2f)", result->gesture_name, result->confidence);
        return ESP_OK;
    }
    
    // No gesture detected with sufficient confidence
    return ESP_OK;
}

esp_err_t gesture_detection_add_template(const char *name, feature_vector_t *features, bool is_dynamic) {
    if (!gesture_detection_initialized || name == NULL || features == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Find if this gesture already exists
    int index = -1;
    for (int i = 0; i < gesture_count; i++) {
        if (strcmp(gesture_templates[i].name, name) == 0) {
            index = i;
            break;
        }
    }
    
    // If not found and we have space, add a new one
    if (index < 0) {
        if (gesture_count >= MAX_GESTURES) {
            ESP_LOGW(TAG, "Cannot add more gestures, reached limit of %d", MAX_GESTURES);
            return ESP_ERR_NO_MEM;
        }
        
        index = gesture_count;
        gesture_count++;
    }
    
    // Update template
    strncpy(gesture_templates[index].name, name, sizeof(gesture_templates[index].name) - 1);
    
    uint16_t count = features->feature_count;
    if (count > FEATURE_BUFFER_SIZE) {
        count = FEATURE_BUFFER_SIZE;
    }
    
    memcpy(gesture_templates[index].template_features, features->features, count * sizeof(float));
    gesture_templates[index].feature_count = count;
    gesture_templates[index].is_dynamic = is_dynamic;
    
    // Save the updated templates to NVS
    save_gesture_templates();
    
    ESP_LOGI(TAG, "Template added/updated for gesture: %s", name);
    return ESP_OK;
}

// Helper function to calculate similarity between two feature vectors
static float calculate_similarity(const float* features1, const float* features2, uint16_t count) {
    if (features1 == NULL || features2 == NULL || count == 0) {
        return 0.0f;
    }
    
    // Calculate Euclidean distance
    float sum_sq_diff = 0.0f;
    for (int i = 0; i < count; i++) {
        float diff = features1[i] - features2[i];
        sum_sq_diff += diff * diff;
    }
    
    float distance = sqrtf(sum_sq_diff);
    
    // Convert distance to similarity score (0-1 range)
    // Smaller distance means higher similarity
    float similarity = 1.0f / (1.0f + distance/10.0f);  // Normalized, adjust divisor as needed
    
    return similarity;
}

// Initialize default gesture templates
static void init_default_gestures(void) {
    gesture_count = 0;
    
    // ASL Letter A
    strncpy(gesture_templates[gesture_count].name, "A", sizeof(gesture_templates[0].name) - 1);
    // Set template values for a fist with thumb alongside
    for (int i = 0; i < 10; i++) {
        gesture_templates[gesture_count].template_features[i] = 75.0f;  // All fingers curled
    }
    gesture_templates[gesture_count].template_features[0] = 30.0f;  // Thumb less curled
    gesture_templates[gesture_count].template_features[1] = 40.0f;
    gesture_templates[gesture_count].feature_count = 32;
    gesture_templates[gesture_count].is_dynamic = false;
    gesture_count++;
    
    // ASL Letter B
    strncpy(gesture_templates[gesture_count].name, "B", sizeof(gesture_templates[0].name) - 1);
    // Set template values for a flat hand with fingers together
    for (int i = 0; i < 10; i++) {
        gesture_templates[gesture_count].template_features[i] = 5.0f;  // All fingers extended
    }
    gesture_templates[gesture_count].feature_count = 32;
    gesture_templates[gesture_count].is_dynamic = false;
    gesture_count++;
    
    // ASL Letter C
    strncpy(gesture_templates[gesture_count].name, "C", sizeof(gesture_templates[0].name) - 1);
    // Set template values for curved hand
    for (int i = 0; i < 10; i++) {
        gesture_templates[gesture_count].template_features[i] = 35.0f;  // All fingers partially curled
    }
    gesture_templates[gesture_count].feature_count = 32;
    gesture_templates[gesture_count].is_dynamic = false;
    gesture_count++;
    
    // Add more default gestures as needed...
    // ASL Letter O
    strncpy(gesture_templates[gesture_count].name, "O", sizeof(gesture_templates[0].name) - 1);
    // Set template values for fingertips touching thumb in O shape
    for (int i = 0; i < 10; i++) {
        gesture_templates[gesture_count].template_features[i] = 50.0f;  // All fingers curved to meet thumb
    }
    gesture_templates[gesture_count].feature_count = 32;
    gesture_templates[gesture_count].is_dynamic = false;
    gesture_count++;
    
    // ASL Letter Y
    strncpy(gesture_templates[gesture_count].name, "Y", sizeof(gesture_templates[0].name) - 1);
    // Thumb and pinky extended, other fingers closed
    for (int i = 0; i < 10; i++) {
        gesture_templates[gesture_count].template_features[i] = 70.0f;  // Most fingers curled
    }
    // Thumb and pinky extended
    gesture_templates[gesture_count].template_features[0] = 10.0f;  // Thumb extended
    gesture_templates[gesture_count].template_features[1] = 15.0f;
    gesture_templates[gesture_count].template_features[8] = 10.0f;  // Pinky extended
    gesture_templates[gesture_count].template_features[9] = 15.0f;
    gesture_templates[gesture_count].feature_count = 32;
    gesture_templates[gesture_count].is_dynamic = false;
    gesture_count++;
    
    // Continue with more gestures as needed...
    
    ESP_LOGI(TAG, "Initialized %d default gesture templates", gesture_count);
}

// Save gesture templates to NVS
static esp_err_t save_gesture_templates(void) {
    nvs_handle_t nvs_handle;
    esp_err_t ret;
    
    // Open NVS
    ret = nvs_open(GESTURE_NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error opening NVS for gestures: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Save gesture count
    ret = nvs_set_u8(nvs_handle, "count", gesture_count);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error saving gesture count: %s", esp_err_to_name(ret));
        nvs_close(nvs_handle);
        return ret;
    }
    
    // Save each gesture template
    for (int i = 0; i < gesture_count; i++) {
        char key[16];
        snprintf(key, sizeof(key), "gesture_%d", i);
        
        ret = nvs_set_blob(nvs_handle, key, &gesture_templates[i], sizeof(gesture_template_t));
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Error saving gesture template %d: %s", i, esp_err_to_name(ret));
            nvs_close(nvs_handle);
            return ret;
        }
    }
    
    // Commit changes
    ret = nvs_commit(nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error committing NVS: %s", esp_err_to_name(ret));
    }
    
    nvs_close(nvs_handle);
    ESP_LOGI(TAG, "Saved %d gesture templates to NVS", gesture_count);
    
    return ret;
}

// Load gesture templates from NVS
static esp_err_t load_gesture_templates(void) {
    nvs_handle_t nvs_handle;
    esp_err_t ret;
    
    // Open NVS
    ret = nvs_open(GESTURE_NVS_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error opening NVS for gestures: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Get gesture count
    ret = nvs_get_u8(nvs_handle, "count", &gesture_count);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error reading gesture count: %s", esp_err_to_name(ret));
        nvs_close(nvs_handle);
        return ret;
    }
    
    // Check if count is valid
    if (gesture_count > MAX_GESTURES) {
        ESP_LOGE(TAG, "Invalid gesture count: %d", gesture_count);
        nvs_close(nvs_handle);
        return ESP_ERR_INVALID_STATE;
    }
    
    // Load each gesture template
    for (int i = 0; i < gesture_count; i++) {
        char key[16];
        snprintf(key, sizeof(key), "gesture_%d", i);
        
        size_t required_size = sizeof(gesture_template_t);
        ret = nvs_get_blob(nvs_handle, key, &gesture_templates[i], &required_size);
        
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Error loading gesture template %d: %s", i, esp_err_to_name(ret));
            nvs_close(nvs_handle);
            return ret;
        }
        
        if (required_size != sizeof(gesture_template_t)) {
            ESP_LOGE(TAG, "Size mismatch for gesture template %d", i);
            nvs_close(nvs_handle);
            return ESP_ERR_INVALID_SIZE;
        }
    }
    
    nvs_close(nvs_handle);
    ESP_LOGI(TAG, "Loaded %d gesture templates from NVS", gesture_count);
    
    return ESP_OK;
}
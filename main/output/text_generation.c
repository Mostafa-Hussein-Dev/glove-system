#include "output/text_generation.h"
#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "util/debug.h"

static const char *TAG = "TEXT_GEN";

// Text generation state
static bool text_generation_initialized = false;

// Buffer for the current sentence being constructed
static char current_sentence[128] = {0};

esp_err_t text_generation_init(void) {
    // Clear the current sentence buffer
    memset(current_sentence, 0, sizeof(current_sentence));
    
    text_generation_initialized = true;
    ESP_LOGI(TAG, "Text generation initialized");
    
    return ESP_OK;
}

esp_err_t text_generation_deinit(void) {
    text_generation_initialized = false;
    ESP_LOGI(TAG, "Text generation deinitialized");
    
    return ESP_OK;
}

esp_err_t text_generation_generate_text(processing_result_t *result, char *output_text, size_t max_length) {
    if (!text_generation_initialized || result == NULL || output_text == NULL || max_length == 0) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Clear output buffer
    memset(output_text, 0, max_length);
    
    // For this simplified version, we'll just output the gesture name directly
    // In a real implementation, you would have more sophisticated language processing
    
    // Special cases for certain gestures
    if (strcmp(result->gesture_name, "SPACE") == 0) {
        // Add a space to the current sentence
        size_t current_length = strlen(current_sentence);
        if (current_length < sizeof(current_sentence) - 1) {
            current_sentence[current_length] = ' ';
            current_sentence[current_length + 1] = '\0';
        }
        
        // Copy the current sentence to the output
        snprintf(output_text, max_length, "%s", current_sentence);
    }
    else if (strcmp(result->gesture_name, "BACKSPACE") == 0) {
        // Remove the last character from the current sentence
        size_t current_length = strlen(current_sentence);
        if (current_length > 0) {
            current_sentence[current_length - 1] = '\0';
        }
        
        // Copy the current sentence to the output
        snprintf(output_text, max_length, "%s", current_sentence);
    }
    else if (strcmp(result->gesture_name, "CLEAR") == 0) {
        // Clear the current sentence
        memset(current_sentence, 0, sizeof(current_sentence));
        
        // Set output text
        snprintf(output_text, max_length, "Text cleared");
    }
    else {
        // For alphabet gestures (A-Z), add the letter to the current sentence
        if (strlen(result->gesture_name) == 1 && 
            result->gesture_name[0] >= 'A' && result->gesture_name[0] <= 'Z') {
            // Add letter to the current sentence
            size_t current_length = strlen(current_sentence);
            if (current_length < sizeof(current_sentence) - 1) {
                current_sentence[current_length] = result->gesture_name[0];
                current_sentence[current_length + 1] = '\0';
            }
            
            // Copy the current sentence to the output
            snprintf(output_text, max_length, "%s", current_sentence);
        }
        // For word gestures, add the word followed by a space
        else {
            // Add the word to the current sentence
            size_t current_length = strlen(current_sentence);
            size_t word_length = strlen(result->gesture_name);
            
            // Check if we need to add a space first
            if (current_length > 0 && current_sentence[current_length - 1] != ' ') {
                if (current_length < sizeof(current_sentence) - 1) {
                    current_sentence[current_length] = ' ';
                    current_length++;
                }
            }
            
            // Add the word if there's space
            if (current_length + word_length < sizeof(current_sentence) - 1) {
                strncpy(current_sentence + current_length, result->gesture_name, 
                        sizeof(current_sentence) - current_length - 1);
            }
            
            // Copy the current sentence to the output
            snprintf(output_text, max_length, "%s", current_sentence);
        }
    }
    
    return ESP_OK;
}

esp_err_t text_generation_get_current_text(char *output_text, size_t max_length) {
    if (!text_generation_initialized || output_text == NULL || max_length == 0) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Copy the current sentence to the output
    snprintf(output_text, max_length, "%s", current_sentence);
    
    return ESP_OK;
}

esp_err_t text_generation_clear_text(void) {
    if (!text_generation_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Clear the current sentence
    memset(current_sentence, 0, sizeof(current_sentence));
    
    ESP_LOGI(TAG, "Text cleared");
    return ESP_OK;
}
#include "file_upload.h"
#include "esp_log.h"
#include "esp_spiffs.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

static const char* TAG = "FILE_UPLOAD";

// Upload state
static struct {
    FILE* current_file;
    char filename[64];
    size_t expected_size;
    size_t received_size;
    bool upload_active;
} upload_state = {0};

esp_err_t file_upload_init(void) {
    memset(&upload_state, 0, sizeof(upload_state));
    ESP_LOGI(TAG, "File upload handler initialized");
    return ESP_OK;
}

esp_err_t file_upload_process_command(const char* line) {
    if (!line) return ESP_ERR_INVALID_ARG;
    
    // Check for upload command: UPLOAD_FILE:filename:size
    if (strncmp(line, "UPLOAD_FILE:", 12) == 0) {
        if (upload_state.upload_active) {
            ESP_LOGW(TAG, "Upload already in progress");
            printf("ERROR:UPLOAD_IN_PROGRESS\n");
            return ESP_FAIL;
        }
        
        // Parse command
        char* cmd_copy = strdup(line + 12);  // Skip "UPLOAD_FILE:"
        char* filename = strtok(cmd_copy, ":");
        char* size_str = strtok(NULL, ":");
        
        if (!filename || !size_str) {
            ESP_LOGE(TAG, "Invalid upload command format");
            printf("ERROR:INVALID_FORMAT\n");
            free(cmd_copy);
            return ESP_FAIL;
        }
        
        size_t file_size = atoi(size_str);
        
        // Open file for writing
        FILE* file = fopen(filename, "wb");
        if (!file) {
            ESP_LOGE(TAG, "Failed to open file for writing: %s", filename);
            printf("ERROR:FILE_CREATE_FAILED\n");
            free(cmd_copy);
            return ESP_FAIL;
        }
        
        // Initialize upload state
        upload_state.current_file = file;
        strncpy(upload_state.filename, filename, sizeof(upload_state.filename) - 1);
        upload_state.expected_size = file_size;
        upload_state.received_size = 0;
        upload_state.upload_active = true;
        
        ESP_LOGI(TAG, "Starting upload: %s (%zu bytes)", filename, file_size);
        printf("UPLOAD_START:OK\n");
        
        free(cmd_copy);
        return ESP_OK;
    }
    
    return ESP_ERR_NOT_FOUND;  // Command not handled
}

// This function should be called from your main serial processing loop
esp_err_t file_upload_process_data(const uint8_t* data, size_t len) {
    if (!upload_state.upload_active || !upload_state.current_file) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Write data to file
    size_t written = fwrite(data, 1, len, upload_state.current_file);
    if (written != len) {
        ESP_LOGE(TAG, "Failed to write data to file");
        fclose(upload_state.current_file);
        upload_state.upload_active = false;
        printf("ERROR:WRITE_FAILED\n");
        return ESP_FAIL;
    }
    
    upload_state.received_size += written;
    printf("CHUNK_OK\n");
    
    // Check if upload is complete
    if (upload_state.received_size >= upload_state.expected_size) {
        fclose(upload_state.current_file);
        upload_state.upload_active = false;
        
        ESP_LOGI(TAG, "Upload complete: %s (%zu bytes)", 
                upload_state.filename, upload_state.received_size);
        printf("UPLOAD_COMPLETE:OK\n");
    }
    
    return ESP_OK;
}
#include "tasks/output_task.h"
#include <string.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "drivers/display.h"
#include "drivers/audio.h"
#include "drivers/haptic.h"
#include "output/text_generation.h"
#include "output/output_manager.h"
#include "app_main.h"
#include "config/system_config.h"
#include "util/debug.h"
#include "freertos/queue.h"

static const char *TAG = "OUTPUT_TASK";

// Task handle
static TaskHandle_t output_task_handle = NULL;

// Output task function
static void output_task(void *arg);

esp_err_t output_task_init(void) {
    // Create the output task
    BaseType_t xReturned = xTaskCreatePinnedToCore(
        output_task,
        "output_task",
        OUTPUT_TASK_STACK_SIZE,
        NULL,
        OUTPUT_TASK_PRIORITY,
        &output_task_handle,
        OUTPUT_TASK_CORE
    );
    
    if (xReturned != pdPASS) {
        ESP_LOGE(TAG, "Failed to create output task");
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "Output task initialized on core %d", OUTPUT_TASK_CORE);
    return ESP_OK;
}

static void output_task(void *arg) {
    ESP_LOGI(TAG, "Output task started");
    
    // Set output task as ready
    xEventGroupSetBits(g_system_event_group, SYSTEM_EVENT_OUTPUT_READY);
    
    // Wait for system initialization to complete
    xEventGroupWaitBits(g_system_event_group, 
                        SYSTEM_EVENT_INIT_COMPLETE, 
                        pdFALSE, pdTRUE, portMAX_DELAY);
    
    // Processing result and output command
    processing_result_t result;
    output_command_t command;
    
    // Show the system is ready on the display
    display_clear();
    display_draw_text("Ready", 0, 20, DISPLAY_FONT_SMALL, DISPLAY_ALIGN_CENTER);
    display_draw_text("Waiting for gestures...", 0, 36, DISPLAY_FONT_SMALL, DISPLAY_ALIGN_CENTER);
    display_update();
    
    // Play a short beep to indicate readiness
    audio_play_beep(1000, 100);
    
    while (1) {
        // Check for output commands first (priority)
        if (xQueueReceive(g_output_command_queue, &command, 0) == pdTRUE) {
            // Process the command
            output_manager_handle_command(&command);
        }
        
        // Add this code wherever processing results are received from queue
        UBaseType_t result_queue_items = uxQueueMessagesWaiting(g_processing_result_queue);
        if (result_queue_items > (PROCESSING_QUEUE_SIZE * 0.9)) {
            ESP_LOGW(TAG, "Processing result queue nearly full: %u/%d items", 
                    (unsigned int)result_queue_items, PROCESSING_QUEUE_SIZE);
            
            // Process multiple items quickly to clear backlog
            processing_result_t results[3];
            for (int i = 0; i < 3 && uxQueueMessagesWaiting(g_processing_result_queue) > 0; i++) {
                if (xQueueReceive(g_processing_result_queue, &results[i], 0) == pdTRUE) {
                    // Process result immediately
                    ESP_LOGI(TAG, "Batch processing result %d: %s", i, results[i].gesture_name);
                }
            }
        }

        // Check for processing results
        if (xQueueReceive(g_processing_result_queue, &result, 0) == pdTRUE) {
            // Generate text from the recognition result
            char text[64];
            text_generation_generate_text(&result, text, sizeof(text));
            
            // Create output commands based on the current output mode
            switch (g_system_config.output_mode) {
                case OUTPUT_MODE_TEXT_ONLY:
                    // Display the text
                    command.type = OUTPUT_CMD_DISPLAY_TEXT;
                    strncpy(command.data.display.text, text, sizeof(command.data.display.text) - 1);
                    command.data.display.size = DISPLAY_FONT_SMALL;
                    command.data.display.line = 1;
                    command.data.display.clear_first = true;
                    
                    // Process the command
                    output_manager_handle_command(&command);
                    break;
                    
                case OUTPUT_MODE_AUDIO_ONLY:
                    // Speak the text
                    command.type = OUTPUT_CMD_SPEAK_TEXT;
                    strncpy(command.data.speak.text, text, sizeof(command.data.speak.text) - 1);
                    command.data.speak.priority = 0;
                    
                    // Process the command
                    output_manager_handle_command(&command);
                    break;
                    
                case OUTPUT_MODE_TEXT_AND_AUDIO:
                    // Display the text
                    command.type = OUTPUT_CMD_DISPLAY_TEXT;
                    strncpy(command.data.display.text, text, sizeof(command.data.display.text) - 1);
                    command.data.display.size = DISPLAY_FONT_SMALL;
                    command.data.display.line = 1;
                    command.data.display.clear_first = true;
                    
                    // Process the command
                    output_manager_handle_command(&command);
                    
                    // Speak the text
                    command.type = OUTPUT_CMD_SPEAK_TEXT;
                    strncpy(command.data.speak.text, text, sizeof(command.data.speak.text) - 1);
                    command.data.speak.priority = 0;
                    
                    // Process the command
                    output_manager_handle_command(&command);
                    break;
                    
                case OUTPUT_MODE_MINIMAL:
                    // Just provide haptic feedback for confirmation
                    command.type = OUTPUT_CMD_HAPTIC_FEEDBACK;
                    command.data.haptic.pattern = 0;  // Simple pattern
                    command.data.haptic.intensity = g_system_config.haptic_intensity;
                    command.data.haptic.duration_ms = 100;
                    
                    // Process the command
                    output_manager_handle_command(&command);
                    break;
            }
        }
        
        // Short delay to prevent CPU hogging
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void output_task_deinit(void) {
    // Cleanup resources when task is deleted
    if (output_task_handle != NULL) {
        vTaskDelete(output_task_handle);
        output_task_handle = NULL;
    }
    
    ESP_LOGI(TAG, "Output task deinitialized");
}
#include "tasks/communication_task.h"
#include <string.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "communication/ble_service.h"
#include "app_main.h"
#include "core/power_management.h"
#include "config/system_config.h"
#include "util/debug.h"
#include "util/buffer.h"
#include "esp_heap_caps.h"

static const char *TAG = "COMM_TASK";

// Task handle
static TaskHandle_t communication_task_handle = NULL;
static QueueHandle_t comm_queue = NULL;
static bool task_running = false;
static bool camera_enabled = false;
static uint32_t last_camera_attempt = 0;

// External queue references
extern QueueHandle_t g_processing_result_queue;
extern QueueHandle_t g_system_command_queue;

#define COMMUNICATION_QUEUE_SIZE        10
#define CAMERA_RETRY_DELAY_MS          5000
#define CAMERA_CONNECTION_TIMEOUT_MS   30000

typedef enum {
    PROCESS_CAMERA_FRAME = 1,
    PROCESS_SENSOR_DATA,
    PROCESS_GESTURE_ANALYSIS
} processing_command_type_t;

// Processing command structure
typedef struct {
    processing_command_type_t type;
    union {
        ble_camera_frame_t frame;
        // Add other data types as needed
    } data;
} processing_command_t;

typedef enum {
    COMM_CMD_SEND_GESTURE = 1,
    COMM_CMD_SEND_TEXT,
    COMM_CMD_SEND_STATUS,
    COMM_CMD_SEND_DEBUG,
    COMM_CMD_START_CAMERA,
    COMM_CMD_STOP_CAMERA,
    COMM_CMD_CAPTURE_FRAME,
    COMM_CMD_RECONNECT_CAMERA
} communication_command_t;

// Command data structure
typedef struct {
    communication_command_t cmd;
    union {
        struct {
            uint8_t gesture_id;
            char gesture_name[32];
            float confidence;
        } gesture;
        struct {
            char text[256];
        } text;
        struct {
            uint8_t battery_level;
            uint8_t state;
            uint8_t error;
        } status;
        struct {
            char message[128];
        } debug;
    } data;
} communication_cmd_t;

// Last status update time
#define STATUS_UPDATE_INTERVAL_MS 5000  // Update status every 5 seconds

// Forward declarations
static void communication_task(void *param);
static esp_err_t handle_communication_command(const communication_cmd_t *cmd);
static void camera_management_task(void);
static void process_camera_frames(void);
static void handle_ble_command(const uint8_t *data, size_t length);
static esp_err_t send_camera_frame_for_processing(const ble_camera_frame_t *frame);

esp_err_t communication_task_init(void) {
    if (task_running) {
        ESP_LOGW(TAG, "Communication task already running");
        return ESP_OK;
    }
    
    ESP_LOGI(TAG, "Initializing communication task...");
    
    // Create command queue
    comm_queue = xQueueCreate(COMMUNICATION_QUEUE_SIZE, sizeof(communication_cmd_t));
    if (!comm_queue) {
        ESP_LOGE(TAG, "Failed to create communication queue");
        return ESP_FAIL;
    }
    
    // Initialize BLE service
    esp_err_t ret = ble_service_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize BLE service: %s", esp_err_to_name(ret));
        vQueueDelete(comm_queue);
        return ret;
    }
    
    // Register BLE command callback
    ble_service_register_command_callback(handle_ble_command);
    
    // Initialize BLE camera
    ret = ble_camera_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize BLE camera: %s", esp_err_to_name(ret));
        ble_service_deinit();
        vQueueDelete(comm_queue);
        return ret;
    }
    
    // Create communication task
    BaseType_t task_ret = xTaskCreatePinnedToCore(
        communication_task,
        "communication_task",
        COMMUNICATION_TASK_STACK_SIZE,
        NULL,
        COMMUNICATION_TASK_PRIORITY,
        &communication_task_handle,
        COMMUNICATION_TASK_CORE
    );
    
    if (task_ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create communication task");
        ble_camera_deinit();
        ble_service_deinit();
        vQueueDelete(comm_queue);
        return ESP_FAIL;
    }
    
    task_running = true;
    ESP_LOGI(TAG, "Communication task initialized successfully");
    return ESP_OK;
}

esp_err_t communication_task_deinit(void) {
    if (!task_running) {
        return ESP_OK;
    }
    
    ESP_LOGI(TAG, "Deinitializing communication task...");
    
    // Stop task
    task_running = false;
    if (communication_task_handle) {
        vTaskDelete(communication_task_handle);
        communication_task_handle = NULL;
    }
    
    // Cleanup resources
    if (comm_queue) {
        vQueueDelete(comm_queue);
        comm_queue = NULL;
    }
    
    // Deinitialize BLE components
    ble_camera_deinit();
    ble_service_deinit();
    
    ESP_LOGI(TAG, "Communication task deinitialized");
    return ESP_OK;
}

esp_err_t communication_task_start(void) {
    if (!task_running) {
        ESP_LOGE(TAG, "Communication task not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    ESP_LOGI(TAG, "Starting communication services...");
    
    // Enable BLE service
    esp_err_t ret = ble_service_enable();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable BLE service: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Enable camera
    camera_enabled = true;
    
    ESP_LOGI(TAG, "Communication services started");
    return ESP_OK;
}

esp_err_t communication_task_stop(void) {
    ESP_LOGI(TAG, "Stopping communication services...");
    
    // Disable camera
    camera_enabled = false;
    if (ble_camera_is_connected()) {
        ble_camera_disconnect();
    }
    
    // Disable BLE service
    ble_service_disable();
    
    ESP_LOGI(TAG, "Communication services stopped");
    return ESP_OK;
}

esp_err_t communication_send_gesture(uint8_t gesture_id, const char *gesture_name, float confidence) {
    if (!task_running || !comm_queue) {
        return ESP_ERR_INVALID_STATE;
    }
    
    communication_cmd_t cmd = {
        .cmd = COMM_CMD_SEND_GESTURE,
        .data.gesture = {
            .gesture_id = gesture_id,
            .confidence = confidence
        }
    };
    
    if (gesture_name) {
        strncpy(cmd.data.gesture.gesture_name, gesture_name, sizeof(cmd.data.gesture.gesture_name) - 1);
        cmd.data.gesture.gesture_name[sizeof(cmd.data.gesture.gesture_name) - 1] = '\0';
    }
    
    if (xQueueSend(comm_queue, &cmd, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGW(TAG, "Failed to queue gesture command");
        return ESP_FAIL;
    }
    
    return ESP_OK;
}

esp_err_t communication_send_text(const char *text) {
    if (!task_running || !comm_queue || !text) {
        return ESP_ERR_INVALID_ARG;
    }
    
    communication_cmd_t cmd = {
        .cmd = COMM_CMD_SEND_TEXT
    };
    
    strncpy(cmd.data.text.text, text, sizeof(cmd.data.text.text) - 1);
    cmd.data.text.text[sizeof(cmd.data.text.text) - 1] = '\0';
    
    if (xQueueSend(comm_queue, &cmd, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGW(TAG, "Failed to queue text command");
        return ESP_FAIL;
    }
    
    return ESP_OK;
}

esp_err_t communication_send_status(uint8_t battery_level, uint8_t state, uint8_t error) {
    if (!task_running || !comm_queue) {
        return ESP_ERR_INVALID_STATE;
    }
    
    communication_cmd_t cmd = {
        .cmd = COMM_CMD_SEND_STATUS,
        .data.status = {
            .battery_level = battery_level,
            .state = state,
            .error = error
        }
    };
    
    if (xQueueSend(comm_queue, &cmd, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGW(TAG, "Failed to queue status command");
        return ESP_FAIL;
    }
    
    return ESP_OK;
}

esp_err_t communication_send_debug(const char *message) {
    if (!task_running || !comm_queue || !message) {
        return ESP_ERR_INVALID_ARG;
    }
    
    communication_cmd_t cmd = {
        .cmd = COMM_CMD_SEND_DEBUG
    };
    
    strncpy(cmd.data.debug.message, message, sizeof(cmd.data.debug.message) - 1);
    cmd.data.debug.message[sizeof(cmd.data.debug.message) - 1] = '\0';
    
    if (xQueueSend(comm_queue, &cmd, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGW(TAG, "Failed to queue debug command");
        return ESP_FAIL;
    }
    
    return ESP_OK;
}

esp_err_t communication_capture_camera_frame(void) {
    if (!task_running || !comm_queue) {
        return ESP_ERR_INVALID_STATE;
    }
    
    communication_cmd_t cmd = {
        .cmd = COMM_CMD_CAPTURE_FRAME
    };
    
    if (xQueueSend(comm_queue, &cmd, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGW(TAG, "Failed to queue capture frame command");
        return ESP_FAIL;
    }
    
    return ESP_OK;
}

// Main communication task
static void communication_task(void *param) {
    communication_cmd_t cmd;
    TickType_t last_camera_check = 0;
    
    ESP_LOGI(TAG, "Communication task started");
    
    while (task_running) {
        // Process commands from queue
        if (xQueueReceive(comm_queue, &cmd, pdMS_TO_TICKS(100)) == pdTRUE) {
            handle_communication_command(&cmd);
        }
        
        // Camera management - check every 1 second
        TickType_t current_time = xTaskGetTickCount();
        if ((current_time - last_camera_check) > pdMS_TO_TICKS(1000)) {
            camera_management_task();
            last_camera_check = current_time;
        }
        
        // Process camera frames if available
        if (camera_enabled && ble_camera_is_connected()) {
            process_camera_frames();
        }
        
        // Small delay to prevent task from consuming too much CPU
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    ESP_LOGI(TAG, "Communication task ended");
    vTaskDelete(NULL);
}

static esp_err_t handle_communication_command(const communication_cmd_t *cmd) {
    if (!cmd) {
        return ESP_ERR_INVALID_ARG;
    }
    
    esp_err_t ret = ESP_OK;
    
    switch (cmd->cmd) {
        case COMM_CMD_SEND_GESTURE:
            ret = ble_service_send_gesture(cmd->data.gesture.gesture_id,
                                         cmd->data.gesture.gesture_name,
                                         cmd->data.gesture.confidence);
            break;
            
        case COMM_CMD_SEND_TEXT:
            ret = ble_service_send_text(cmd->data.text.text);
            break;
            
        case COMM_CMD_SEND_STATUS:
            ret = ble_service_send_status(cmd->data.status.battery_level,
                                        cmd->data.status.state,
                                        cmd->data.status.error);
            break;
            
        case COMM_CMD_SEND_DEBUG:
            ret = ble_service_send_debug(cmd->data.debug.message);
            break;
            
        case COMM_CMD_START_CAMERA:
            if (ble_camera_is_connected()) {
                ret = ble_camera_start_streaming();
            } else {
                ESP_LOGW(TAG, "Camera not connected, cannot start streaming");
                ret = ESP_ERR_INVALID_STATE;
            }
            break;
            
        case COMM_CMD_STOP_CAMERA:
            if (ble_camera_is_connected()) {
                ret = ble_camera_stop_streaming();
            }
            break;
            
        case COMM_CMD_CAPTURE_FRAME:
            if (ble_camera_is_connected()) {
                ble_camera_frame_t frame;
                ret = ble_camera_capture_frame(&frame);
                if (ret == ESP_OK) {
                    send_camera_frame_for_processing(&frame);
                }
            } else {
                ESP_LOGW(TAG, "Camera not connected, cannot capture frame");
                ret = ESP_ERR_INVALID_STATE;
            }
            break;
            
        case COMM_CMD_RECONNECT_CAMERA:
            if (ble_camera_is_connected()) {
                ble_camera_disconnect();
                vTaskDelay(pdMS_TO_TICKS(1000));
            }
            ret = ble_camera_connect("ESP32CAM-SLG");
            break;
            
        default:
            ESP_LOGW(TAG, "Unknown communication command: %d", cmd->cmd);
            ret = ESP_ERR_INVALID_ARG;
            break;
    }
    
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Command %d failed: %s", cmd->cmd, esp_err_to_name(ret));
    }
    
    return ret;
}

static void camera_management_task(void) {
    if (!camera_enabled) {
        return;
    }
    
    uint32_t current_time = esp_timer_get_time() / 1000;
    
    // Check if camera is connected
    if (!ble_camera_is_connected()) {
        // Try to connect if enough time has passed since last attempt
        if ((current_time - last_camera_attempt) > CAMERA_RETRY_DELAY_MS) {
            ESP_LOGI(TAG, "Attempting to connect to ESP32-CAM...");
            
            esp_err_t ret = ble_camera_connect("ESP32CAM-SLG");
            if (ret != ESP_OK) {
                ESP_LOGW(TAG, "Camera connection attempt failed: %s", esp_err_to_name(ret));
            }
            
            last_camera_attempt = current_time;
        }
    } else {
        // Camera is connected, check status
        ble_camera_stats_t stats;
        if (ble_camera_get_stats(&stats) == ESP_OK) {
            // Log stats periodically (every 30 seconds)
            static uint32_t last_stats_log = 0;
            if ((current_time - last_stats_log) > 30000) {
                ESP_LOGI(TAG, "Camera stats: frames_rx=%lu, frames_dropped=%lu, fps=%.1f",
                         stats.frames_received, stats.frames_dropped, stats.frame_rate);
                last_stats_log = current_time;
            }
        }
    }
}

static void process_camera_frames(void) {
    // Check if frames are available
    if (!ble_camera_frame_available()) {
        return;
    }
    
    // Get frame
    ble_camera_frame_t frame;
    esp_err_t ret = ble_camera_capture_frame(&frame);
    if (ret == ESP_OK && frame.valid) {
        // Send frame for gesture processing
        send_camera_frame_for_processing(&frame);
        
        // Release frame resources
        ble_camera_release_frame();
    }
}

static esp_err_t send_camera_frame_for_processing(const ble_camera_frame_t *frame) {
    // Create processing command with frame data
    processing_command_t cmd = {
        .type = PROCESS_CAMERA_FRAME,
        .data.frame = *frame  // Copy frame data
    };
    
    // Send to processing task queue
    if (g_processing_result_queue) {
        return xQueueSend(g_processing_result_queue, &cmd, pdMS_TO_TICKS(100));
    }
    
    return ESP_ERR_INVALID_STATE;
}

static void handle_ble_command(const uint8_t *data, size_t length) {
    if (!data || length == 0) {
        return;
    }
    
    uint8_t cmd_id = data[0];
    ESP_LOGI(TAG, "Received BLE command: 0x%02x", cmd_id);
    
    switch (cmd_id) {
        case 0x01: // Start camera streaming
            {
                communication_cmd_t cmd = { .cmd = COMM_CMD_START_CAMERA };
                xQueueSend(comm_queue, &cmd, pdMS_TO_TICKS(100));
            }
            break;
            
        case 0x02: // Stop camera streaming
            {
                communication_cmd_t cmd = { .cmd = COMM_CMD_STOP_CAMERA };
                xQueueSend(comm_queue, &cmd, pdMS_TO_TICKS(100));
            }
            break;
            
        case 0x03: // Capture single frame
            {
                communication_cmd_t cmd = { .cmd = COMM_CMD_CAPTURE_FRAME };
                xQueueSend(comm_queue, &cmd, pdMS_TO_TICKS(100));
            }
            break;
            
        case 0x04: // Reconnect camera
            {
                communication_cmd_t cmd = { .cmd = COMM_CMD_RECONNECT_CAMERA };
                xQueueSend(comm_queue, &cmd, pdMS_TO_TICKS(100));
            }
            break;
            
        case 0x10: // Send debug info
            {
                communication_cmd_t cmd = { .cmd = COMM_CMD_SEND_DEBUG };
                strcpy(cmd.data.debug.message, "Debug info requested via BLE");
                xQueueSend(comm_queue, &cmd, pdMS_TO_TICKS(100));
            }
            break;
            
        default:
            ESP_LOGW(TAG, "Unknown BLE command: 0x%02x", cmd_id);
            break;
    }
}

void* communication_task_get_handle(void) {
    extern TaskHandle_t communication_task_handle;  // Declare external reference
    return (void*)communication_task_handle;
}
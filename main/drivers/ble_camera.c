#include "drivers/ble_camera.h"
#include "esp_log.h"
#include "esp_bt.h"
#include "esp_pm.h"
#include "esp_gap_ble_api.h"
#include "esp_gatt_defs.h"
#include "esp_gattc_api.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include <string.h>

static const char* TAG = "BLE_CAMERA";

// BLE Camera Service UUIDs (must match ESP32-CAM server)
#define BLE_CAMERA_SERVICE_UUID         0x1820
#define BLE_CAMERA_CHAR_IMAGE_UUID      0x2A30
#define BLE_CAMERA_CHAR_CONTROL_UUID    0x2A31
#define BLE_CAMERA_CHAR_STATUS_UUID     0x2A32

#define ESP_APP_ID  0x55
#define MAX_FRAME_SIZE 32768
#define CHUNK_TIMEOUT_MS 5000

// Image chunk header structure
typedef struct {
    uint8_t marker[2];      // 0xFF, 0xFE
    uint32_t total_length;  // Total image length
    uint8_t frame_size;     // Frame size info
    uint8_t quality;        // JPEG quality
} image_header_t;

// Chunk reassembly state
typedef struct {
    uint8_t *buffer;
    uint32_t total_size;
    uint32_t received_size;
    uint32_t last_chunk_time;
    bool receiving;
    uint16_t sequence;
} chunk_state_t;

// Camera state
static bool camera_initialized = false;
static ble_camera_status_t camera_status = BLE_CAMERA_STATUS_DISCONNECTED;
static ble_camera_frame_t current_frame = {0};
static ble_camera_stats_t camera_stats = {0};

// BLE connection handles
static esp_gatt_if_t gattc_if = ESP_GATT_IF_NONE;
static uint16_t conn_id = 0;
static char target_device_name[32] = "ESP32CAM-SLG";
static uint16_t char_control_handle = 0;
static uint16_t char_image_handle = 0;
static uint16_t char_status_handle = 0;

static esp_bd_addr_t remote_bda;
static bool scan_in_progress = false;
static uint16_t service_start_handle = 0;
static uint16_t service_end_handle = 0;

// Frame buffer and synchronization
static QueueHandle_t frame_queue = NULL;
static SemaphoreHandle_t frame_mutex = NULL;
static uint8_t frame_buffer[MAX_FRAME_SIZE];
static chunk_state_t chunk_state = {0};

// BLE Camera commands
typedef enum {
    BLE_CAM_CMD_START_STREAM = 0x01,
    BLE_CAM_CMD_STOP_STREAM = 0x02,
    BLE_CAM_CMD_CAPTURE = 0x03,
    BLE_CAM_CMD_SET_RESOLUTION = 0x04,
    BLE_CAM_CMD_SET_QUALITY = 0x05
} ble_camera_command_t;

// Static function declarations
static void gattc_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);
static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
static esp_err_t send_camera_command(uint8_t cmd, uint8_t *data, size_t len);
static void process_image_data(uint8_t *data, size_t len);
static void reset_chunk_state(void);
static esp_err_t process_image_header(uint8_t *data, size_t len);
static esp_err_t process_image_chunk(uint8_t *data, size_t len);
static void complete_frame_assembly(void);

esp_err_t ble_camera_init(void) {
    if (camera_initialized) {
        ESP_LOGW(TAG, "BLE camera already initialized");
        return ESP_OK;
    }
    
    ESP_LOGI(TAG, "Initializing BLE camera client...");
    
    // Create synchronization objects
    frame_mutex = xSemaphoreCreateMutex();
    if (!frame_mutex) {
        ESP_LOGE(TAG, "Failed to create frame mutex");
        return ESP_FAIL;
    }

    // CREATE THE MISSING QUEUE! - This was the critical bug
    frame_queue = xQueueCreate(5, sizeof(ble_camera_frame_t));
    if (!frame_queue) {
        ESP_LOGE(TAG, "Failed to create frame queue");
        vSemaphoreDelete(frame_mutex);
        return ESP_FAIL;
    }
    
    // Initialize chunk state
    chunk_state.buffer = NULL;
    reset_chunk_state();

    esp_bt_controller_status_t controller_status = esp_bt_controller_get_status();
    if (controller_status != ESP_BT_CONTROLLER_STATUS_ENABLED) {
        ESP_LOGE(TAG, "BT controller not enabled: %d", controller_status);
        goto cleanup;
    }

    esp_bluedroid_status_t bluedroid_status = esp_bluedroid_get_status();
    if (bluedroid_status != ESP_BLUEDROID_STATUS_ENABLED) {
        ESP_LOGE(TAG, "Bluedroid not enabled: %d", bluedroid_status);
        goto cleanup;
    }

    ESP_LOGI(TAG, "BLE stack is ready, registering GATT client callbacks...");

    // Wait a bit for BLE stack to be fully ready
    vTaskDelay(pdMS_TO_TICKS(100));
        
    // Register BLE callbacks
    esp_err_t ret = esp_ble_gattc_register_callback(gattc_event_handler);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register GATTC callback: %s", esp_err_to_name(ret));
        goto cleanup;
    }
    
    ret = esp_ble_gap_register_callback(gap_event_handler);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register GAP callback: %s", esp_err_to_name(ret));
        goto cleanup;
    }
    
    // Register GATT client application
    ret = esp_ble_gattc_app_register(ESP_APP_ID);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register GATT app: %s", esp_err_to_name(ret));
        goto cleanup;
    }
    
    camera_initialized = true;
    ESP_LOGI(TAG, "BLE camera client initialized successfully");
    return ESP_OK;

cleanup:
    if (frame_queue) {
        vQueueDelete(frame_queue);
        frame_queue = NULL;
    }
    if (frame_mutex) {
        vSemaphoreDelete(frame_mutex);
        frame_mutex = NULL;
    }
    return ESP_FAIL;
}

esp_err_t ble_camera_deinit(void) {
    if (!camera_initialized) {
        return ESP_OK;
    }
    
    ESP_LOGI(TAG, "Deinitializing BLE camera...");
    
    // Disconnect if connected
    if (camera_status != BLE_CAMERA_STATUS_DISCONNECTED) {
        ble_camera_disconnect();
    }
    
    // Cleanup resources
    if (frame_queue) {
        vQueueDelete(frame_queue);
        frame_queue = NULL;
    }
    
    if (frame_mutex) {
        vSemaphoreDelete(frame_mutex);
        frame_mutex = NULL;
    }
    
    if (chunk_state.buffer) {
        free(chunk_state.buffer);
        chunk_state.buffer = NULL;
    }
    
    camera_initialized = false;
    ESP_LOGI(TAG, "BLE camera deinitialized");
    return ESP_OK;
}

esp_err_t ble_camera_connect(const char* device_name) {
    if (!camera_initialized) {
        ESP_LOGE(TAG, "BLE camera not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (camera_status == BLE_CAMERA_STATUS_CONNECTED || 
        camera_status == BLE_CAMERA_STATUS_CONNECTING) {
        ESP_LOGW(TAG, "Already connected or connecting");
        return ESP_OK;
    }
    
    // Store target device name
    if (device_name) {
        strncpy(target_device_name, device_name, sizeof(target_device_name) - 1);
        target_device_name[sizeof(target_device_name) - 1] = '\0';
    }
    
    // Force clean state
    if (camera_status != BLE_CAMERA_STATUS_DISCONNECTED && conn_id != 0) {
        ESP_LOGI(TAG, "Forcing clean disconnect...");
        ble_camera_disconnect();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    // Reset connection state completely
    camera_status = BLE_CAMERA_STATUS_DISCONNECTED;
    conn_id = 0;
    service_start_handle = 0;
    service_end_handle = 0;
    char_control_handle = 0;
    char_image_handle = 0;
    char_status_handle = 0;

    ESP_LOGI(TAG, "Connecting to ESP32-CAM: %s", target_device_name);
    
    // ESP32-CAM MAC address (hardcoded for reliability)
    esp_bd_addr_t camera_mac = {0xe0, 0x5a, 0x1b, 0xad, 0x3a, 0x3e};
    
    // Set connecting status BEFORE connection attempt
    camera_status = BLE_CAMERA_STATUS_CONNECTING;
    
    // Connect directly
    esp_err_t ret = esp_ble_gattc_open(gattc_if, camera_mac, BLE_ADDR_TYPE_PUBLIC, true);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Direct connection failed: %s", esp_err_to_name(ret));
        camera_status = BLE_CAMERA_STATUS_ERROR;
        return ret;
    }
    
    ESP_LOGI(TAG, "Direct connection initiated to ESP32-CAM");
    return ESP_OK;
}

esp_err_t ble_camera_disconnect(void) {
    ESP_LOGI(TAG, "Disconnecting from camera...");
    
    // Stop any ongoing streaming
    if (camera_status == BLE_CAMERA_STATUS_STREAMING) {
        ble_camera_stop_streaming();
    }
    
    // Close GATT connection if we have a valid conn_id
    if (gattc_if != ESP_GATT_IF_NONE && conn_id != 0) {
        esp_err_t ret = esp_ble_gattc_close(gattc_if, conn_id);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to close GATT connection: %s", esp_err_to_name(ret));
        }
        ESP_LOGI(TAG, "Disconnect initiated");
        
        // Wait a bit for the close event, then force reset if needed
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    
    // Force reset connection state
    camera_status = BLE_CAMERA_STATUS_DISCONNECTED;
    conn_id = 0;
    service_start_handle = 0;
    service_end_handle = 0;
    char_control_handle = 0;
    char_image_handle = 0;
    char_status_handle = 0;
    
    // Reset chunk state
    reset_chunk_state();
    
    return ESP_OK;
}

esp_err_t ble_camera_start_streaming(void) {
    if (camera_status != BLE_CAMERA_STATUS_CONNECTED) {
        ESP_LOGW(TAG, "Camera not connected, cannot start streaming");
        return ESP_ERR_INVALID_STATE;
    }
    
    ESP_LOGI(TAG, "Starting camera streaming...");
    
    esp_err_t ret = send_camera_command(BLE_CAM_CMD_START_STREAM, NULL, 0);
    if (ret == ESP_OK) {
        camera_status = BLE_CAMERA_STATUS_STREAMING;
        ESP_LOGI(TAG, "Camera streaming started");
    }
    
    return ret;
}

esp_err_t ble_camera_stop_streaming(void) {
    if (camera_status != BLE_CAMERA_STATUS_STREAMING) {
        return ESP_OK; // Already stopped
    }
    
    ESP_LOGI(TAG, "Stopping camera streaming...");
    
    esp_err_t ret = send_camera_command(BLE_CAM_CMD_STOP_STREAM, NULL, 0);
    if (ret == ESP_OK) {
        camera_status = BLE_CAMERA_STATUS_CONNECTED;
        ESP_LOGI(TAG, "Camera streaming stopped");
    }
    
    return ret;
}

esp_err_t ble_camera_capture_frame(ble_camera_frame_t *frame) {
    if (!camera_initialized || !frame) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (camera_status != BLE_CAMERA_STATUS_CONNECTED && 
        camera_status != BLE_CAMERA_STATUS_STREAMING) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Send capture command if not streaming
    if (camera_status == BLE_CAMERA_STATUS_CONNECTED) {
        esp_err_t ret = send_camera_command(BLE_CAM_CMD_CAPTURE, NULL, 0);
        if (ret != ESP_OK) {
            return ret;
        }
    }
    
    // Wait for frame with timeout
    ble_camera_frame_t received_frame;
    if (xQueueReceive(frame_queue, &received_frame, pdMS_TO_TICKS(5000)) == pdTRUE) {
        if (xSemaphoreTake(frame_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            memcpy(frame, &received_frame, sizeof(ble_camera_frame_t));
            xSemaphoreGive(frame_mutex);
            camera_stats.frames_received++;
            camera_stats.last_frame_time = esp_timer_get_time() / 1000;
            return ESP_OK;
        }
    }
    
    camera_stats.frames_dropped++;
    return ESP_FAIL;
}

esp_err_t ble_camera_release_frame(void) {
    // For BLE camera, frames are automatically managed
    return ESP_OK;
}

esp_err_t ble_camera_get_status(ble_camera_status_t *status) {
    if (!status) {
        return ESP_ERR_INVALID_ARG;
    }
    
    *status = camera_status;
    return ESP_OK;
}

esp_err_t ble_camera_get_stats(ble_camera_stats_t *stats) {
    if (!stats) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (xSemaphoreTake(frame_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        memcpy(stats, &camera_stats, sizeof(ble_camera_stats_t));
        
        // Calculate frame rate
        uint32_t current_time = esp_timer_get_time() / 1000;
        if (camera_stats.last_frame_time > 0 && current_time > camera_stats.last_frame_time) {
            uint32_t time_diff = current_time - camera_stats.last_frame_time;
            stats->frame_rate = 1000.0f / time_diff;
        }
        
        stats->signal_strength = -65; // Simulated RSSI
        
        xSemaphoreGive(frame_mutex);
    }
    
    return ESP_OK;
}

esp_err_t ble_camera_set_resolution(uint8_t width, uint8_t height) {
    ESP_LOGI(TAG, "Setting camera resolution to %dx%d", width, height);
    
    uint8_t data[2] = {width, height};
    return send_camera_command(BLE_CAM_CMD_SET_RESOLUTION, data, 2);
}

esp_err_t ble_camera_set_quality(uint8_t quality) {
    ESP_LOGI(TAG, "Setting camera quality to %d", quality);
    
    return send_camera_command(BLE_CAM_CMD_SET_QUALITY, &quality, 1);
}

bool ble_camera_is_connected(void) {
    return (camera_status == BLE_CAMERA_STATUS_CONNECTED || 
            camera_status == BLE_CAMERA_STATUS_STREAMING);
}

bool ble_camera_frame_available(void) {
    if (!ble_camera_is_connected() || !frame_queue) {
        return false;
    }
    
    return (uxQueueMessagesWaiting(frame_queue) > 0);
}

void debug_camera_state(void) {
    ESP_LOGI(TAG, "=== Camera Debug State ===");
    ESP_LOGI(TAG, "camera_status: %d", camera_status);
    ESP_LOGI(TAG, "gattc_if: %d", gattc_if);
    ESP_LOGI(TAG, "conn_id: %d", conn_id);
    ESP_LOGI(TAG, "char_control_handle: %d", char_control_handle);
    ESP_LOGI(TAG, "char_image_handle: %d", char_image_handle);
    ESP_LOGI(TAG, "char_status_handle: %d", char_status_handle);
    ESP_LOGI(TAG, "service_start_handle: %d", service_start_handle);
    ESP_LOGI(TAG, "service_end_handle: %d", service_end_handle);
    ESP_LOGI(TAG, "chunk_state.receiving: %d", chunk_state.receiving);
    ESP_LOGI(TAG, "chunk_state.received_size: %lu", chunk_state.received_size);
    ESP_LOGI(TAG, "========================");
}

static void gattc_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if_param, esp_ble_gattc_cb_param_t *param) {
    ESP_LOGI(TAG, "GATTC event: %d", event);
    
    switch (event) {
        case ESP_GATTC_REG_EVT:
            if (gattc_if == ESP_GATT_IF_NONE) {
                ESP_LOGI(TAG, "GATT client registered, app_id: %d", param->reg.app_id);
                gattc_if = gattc_if_param;
            }
            break;
            
        case ESP_GATTC_OPEN_EVT:
            ESP_LOGI(TAG, "GATT connection result: status=%d, conn_id=%d", 
                     param->open.status, param->open.conn_id);
            
            if (param->open.status == ESP_GATT_OK) {
                conn_id = param->open.conn_id;
                camera_status = BLE_CAMERA_STATUS_CONNECTED;
                ESP_LOGI(TAG, "Connected to ESP32-CAM, starting service discovery...");
                
                // Start service discovery
                esp_ble_gattc_search_service(gattc_if, conn_id, NULL);
            } else {
                ESP_LOGE(TAG, "Failed to connect to ESP32-CAM: %d", param->open.status);
                camera_status = BLE_CAMERA_STATUS_ERROR;
                conn_id = 0;
            }
            break;
            
        case ESP_GATTC_SEARCH_RES_EVT:
            ESP_LOGI(TAG, "Service found: UUID=0x%04x, start_handle=%d, end_handle=%d",
                     param->search_res.srvc_id.uuid.uuid.uuid16,
                     param->search_res.start_handle,
                     param->search_res.end_handle);
            
            // Check if this is our camera service
            if (param->search_res.srvc_id.uuid.len == ESP_UUID_LEN_16 &&
                param->search_res.srvc_id.uuid.uuid.uuid16 == BLE_CAMERA_SERVICE_UUID) {
                ESP_LOGI(TAG, "Found ESP32-CAM service!");
                service_start_handle = param->search_res.start_handle;
                service_end_handle = param->search_res.end_handle;
            }
            break;
            
        case ESP_GATTC_SEARCH_CMPL_EVT:
        ESP_LOGI(TAG, "Service discovery complete, status: %d", param->search_cmpl.status);
        
        if (param->search_cmpl.status == ESP_GATT_OK && service_start_handle != 0) {
            ESP_LOGI(TAG, "ESP32-CAM service found, manually discovering characteristics...");
            
            // Manually assign characteristic handles based on ESP32-CAM server implementation
            // These handles are typically sequential after the service handle
            char_image_handle = service_start_handle + 2;   // Image characteristic
            char_control_handle = service_start_handle + 4; // Control characteristic  
            char_status_handle = service_start_handle + 6;  // Status characteristic
            
            ESP_LOGI(TAG, "Assigned handles - Image: %d, Control: %d, Status: %d",
                    char_image_handle, char_control_handle, char_status_handle);
            
            // Enable notifications for image characteristic
            if (char_image_handle != 0) {
                esp_ble_gattc_register_for_notify(gattc_if, remote_bda, char_image_handle);
            }
            
            // Enable notifications for status characteristic
            if (char_status_handle != 0) {
                esp_ble_gattc_register_for_notify(gattc_if, remote_bda, char_status_handle);
            }
            
            ESP_LOGI(TAG, "ESP32-CAM ready for communication!");
            camera_status = BLE_CAMERA_STATUS_CONNECTED;
        } else {
            ESP_LOGE(TAG, "Service discovery failed or camera service not found");
            camera_status = BLE_CAMERA_STATUS_ERROR;
        }
        break;
            
        case ESP_GATTC_REG_FOR_NOTIFY_EVT:
            ESP_LOGI(TAG, "Notification registration result: status=%d, handle=%d",
                     param->reg_for_notify.status, param->reg_for_notify.handle);
            break;
            
        case ESP_GATTC_NOTIFY_EVT:
            // ESP_LOGI(TAG, "Notification received: handle=%d, len=%d", 
            //          param->notify.handle, param->notify.value_len);
            
            // Process received data based on characteristic
            if (param->notify.handle == char_image_handle) {
                process_image_data(param->notify.value, param->notify.value_len);
            } else if (param->notify.handle == char_status_handle) {
                ESP_LOGI(TAG, "Status update received: %d bytes", param->notify.value_len);
            }
            break;
            
        case ESP_GATTC_CLOSE_EVT:
            ESP_LOGI(TAG, "GATT connection closed, reason: %d", param->close.reason);
            camera_status = BLE_CAMERA_STATUS_DISCONNECTED;
            conn_id = 0;
            service_start_handle = 0;
            service_end_handle = 0;
            char_control_handle = 0;
            char_image_handle = 0;
            char_status_handle = 0;
            reset_chunk_state();
            break;
            
        default:
            ESP_LOGI(TAG, "Unhandled GATTC event: %d", event);
            break;
    }
}

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    switch (event) {
        case ESP_GAP_BLE_SCAN_RESULT_EVT:
            // Scanning logic if needed in future
            break;
            
        case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:
            ESP_LOGI(TAG, "Scan stopped");
            scan_in_progress = false;
            break;
            
        default:
            break;
    }
}

static esp_err_t send_camera_command(uint8_t cmd, uint8_t *data, size_t len) {
    // Check connection state
    if (conn_id == 0) {
        ESP_LOGE(TAG, "No active connection (conn_id = 0)");
        camera_status = BLE_CAMERA_STATUS_DISCONNECTED;
        return ESP_ERR_INVALID_STATE;
    }

    if (gattc_if == ESP_GATT_IF_NONE) {
        ESP_LOGE(TAG, "GATT interface not established");
        return ESP_ERR_INVALID_STATE;
    }

    if (char_control_handle == 0) {
        ESP_LOGE(TAG, "Control characteristic not found");
        return ESP_ERR_INVALID_STATE;
    }
    
    // Prepare command packet: [CMD][DATA_LEN][DATA...]
    uint8_t command_packet[64];
    command_packet[0] = cmd;
    command_packet[1] = (uint8_t)len;
    
    size_t packet_len = 2;
    if (data && len > 0) {
        if (len > 62) len = 62; // Limit data size
        memcpy(command_packet + 2, data, len);
        packet_len += len;
    }
    
    // Send command via GATT write
    esp_err_t ret = esp_ble_gattc_write_char(gattc_if, conn_id, char_control_handle,
                                            packet_len, command_packet,
                                            ESP_GATT_WRITE_TYPE_NO_RSP,
                                            ESP_GATT_AUTH_REQ_NONE);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send command 0x%02x: %s", cmd, esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "Command 0x%02x sent successfully (%d bytes)", cmd, packet_len);
    return ESP_OK;
}

static void process_image_data(uint8_t *data, size_t len) {
    if (!data || len == 0) {
        return;
    }
    
    uint32_t current_time = esp_timer_get_time() / 1000;
    
    // Check for timeout on existing frame
    if (chunk_state.receiving && 
        (current_time - chunk_state.last_chunk_time) > CHUNK_TIMEOUT_MS) {
        ESP_LOGW(TAG, "Frame timeout, resetting chunk state");
        reset_chunk_state();
    }
    
    // Check if this is a header packet (starts with 0xFF 0xFE)
    if (len >= 8 && data[0] == 0xFF && data[1] == 0xFE) {
        ESP_LOGI(TAG, "Received image header");
        process_image_header(data, len);
    } else if (chunk_state.receiving) {
        // This is a data chunk
        process_image_chunk(data, len);
    } else {
        ESP_LOGW(TAG, "Unexpected data received (no header)");
    }
    
    chunk_state.last_chunk_time = current_time;
}

static esp_err_t process_image_header(uint8_t *data, size_t len) {
    if (len < 8) {
        ESP_LOGE(TAG, "Invalid header size: %d", len);
        return ESP_ERR_INVALID_ARG;
    }
    
    // Reset chunk state for new frame
    reset_chunk_state();
    
    // Parse header
    image_header_t header;
    header.marker[0] = data[0];
    header.marker[1] = data[1];
    header.total_length = (data[2] << 24) | (data[3] << 16) | (data[4] << 8) | data[5];
    header.frame_size = data[6];
    header.quality = data[7];
    
    ESP_LOGI(TAG, "Image header: size=%lu, frame_size=%d, quality=%d", 
             header.total_length, header.frame_size, header.quality);
    
    // Validate size
    if (header.total_length > MAX_FRAME_SIZE) {
        ESP_LOGE(TAG, "Frame too large: %lu bytes", header.total_length);
        return ESP_ERR_NO_MEM;
    }
    
    if (header.total_length == 0) {
        ESP_LOGE(TAG, "Invalid frame size: 0");
        return ESP_ERR_INVALID_ARG;
    }
    
    // Allocate buffer for frame
    chunk_state.buffer = malloc(header.total_length);
    if (!chunk_state.buffer) {
        ESP_LOGE(TAG, "Failed to allocate frame buffer: %lu bytes", header.total_length);
        return ESP_ERR_NO_MEM;
    }
    
    // Initialize chunk state
    chunk_state.total_size = header.total_length;
    chunk_state.received_size = 0;
    chunk_state.receiving = true;
    chunk_state.sequence = 0;
    
    ESP_LOGI(TAG, "Frame buffer allocated, waiting for %lu bytes", header.total_length);
    return ESP_OK;
}

static esp_err_t process_image_chunk(uint8_t *data, size_t len) {
    if (!chunk_state.receiving || !chunk_state.buffer) {
        ESP_LOGW(TAG, "Received chunk but not expecting data");
        return ESP_ERR_INVALID_STATE;
    }
    
    // Check if chunk fits in buffer
    if (chunk_state.received_size + len > chunk_state.total_size) {
        ESP_LOGW(TAG, "Chunk overflow: received=%lu, chunk=%d, total=%lu", 
                 chunk_state.received_size, len, chunk_state.total_size);
        len = chunk_state.total_size - chunk_state.received_size;
    }
    
    // Copy chunk data
    memcpy(chunk_state.buffer + chunk_state.received_size, data, len);
    chunk_state.received_size += len;
    chunk_state.sequence++;
    
    ESP_LOGD(TAG, "Chunk %d: %d bytes (total: %lu/%lu)", 
             chunk_state.sequence, len, chunk_state.received_size, chunk_state.total_size);
    
    // Check if frame is complete
    if (chunk_state.received_size >= chunk_state.total_size) {
        ESP_LOGI(TAG, "Frame complete: %lu bytes in %d chunks", 
                 chunk_state.received_size, chunk_state.sequence);
        complete_frame_assembly();
    }
    
    return ESP_OK;
}

static void complete_frame_assembly(void) {
    if (!chunk_state.buffer || chunk_state.received_size == 0) {
        ESP_LOGE(TAG, "Invalid frame state for completion");
        return;
    }
    
    if (xSemaphoreTake(frame_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        // Copy assembled frame to frame buffer
        size_t copy_size = (chunk_state.received_size > sizeof(frame_buffer)) ? 
                          sizeof(frame_buffer) : chunk_state.received_size;
        
        memcpy(frame_buffer, chunk_state.buffer, copy_size);
        
        // Update current frame info
        current_frame.buffer = frame_buffer;
        current_frame.buffer_size = copy_size;
        current_frame.width = 320;  // Default QVGA (could be parsed from header)
        current_frame.height = 240;
        current_frame.format = 1;   // JPEG
        current_frame.timestamp = esp_timer_get_time() / 1000;
        current_frame.sequence++;
        current_frame.valid = true;
        
        // Update statistics
        camera_stats.frames_received++;
        camera_stats.bytes_received += copy_size;
        camera_stats.last_frame_time = current_frame.timestamp;
        
        // Send frame to queue for applications
        if (frame_queue) {
            ble_camera_frame_t frame_copy = current_frame;
            if (xQueueSend(frame_queue, &frame_copy, 0) != pdTRUE) {
                ESP_LOGW(TAG, "Frame queue full, dropping frame");
                camera_stats.frames_dropped++;
            }
        }
        
        xSemaphoreGive(frame_mutex);
        
        ESP_LOGI(TAG, "Frame assembled and queued: %d bytes", copy_size);
    }
    
    // Reset chunk state for next frame
    reset_chunk_state();
}

static void reset_chunk_state(void) {
    if (chunk_state.buffer) {
        free(chunk_state.buffer);
        chunk_state.buffer = NULL;
    }
    
    chunk_state.total_size = 0;
    chunk_state.received_size = 0;
    chunk_state.receiving = false;
    chunk_state.sequence = 0;
    chunk_state.last_chunk_time = 0;
}
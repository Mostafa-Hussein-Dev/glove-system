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

// Camera state
static bool camera_initialized = false;
static ble_camera_status_t camera_status = BLE_CAMERA_STATUS_DISCONNECTED;
static ble_camera_frame_t current_frame = {0};
static ble_camera_stats_t camera_stats = {0};

// BLE connection handles
static esp_gatt_if_t gattc_if = ESP_GATT_IF_NONE;
static uint16_t conn_id = 0;
static char target_device_name[32] = {0};
static uint16_t char_control_handle = 0;
static uint16_t char_image_handle = 0;
static uint16_t char_status_handle = 0;

static esp_bd_addr_t remote_bda;
static bool scan_in_progress = false;
static uint16_t service_start_handle = 0;
static uint16_t service_end_handle = 0;

// Frame buffer and synchronization
static QueueHandle_t frame_queue;
static SemaphoreHandle_t frame_mutex = NULL;
static uint8_t frame_buffer[32768]; // 32KB max frame size

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

    // CREATE THE MISSING QUEUE!
    frame_queue = xQueueCreate(5, sizeof(ble_camera_frame_t));
    if (!frame_queue) {
        ESP_LOGE(TAG, "Failed to create frame queue");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Frame queue created successfully");

    esp_err_t ret;
    
    ret = esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
    if (ret) {
        ESP_LOGI(TAG, "BT classic memory release: %s", esp_err_to_name(ret));
    }
    
    // Initialize Bluetooth controller with matching config
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    

    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(TAG, "BT controller init failed: %s", esp_err_to_name(ret));
        goto cleanup;
    }
    
    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(TAG, "BT controller enable failed: %s", esp_err_to_name(ret));
        esp_bt_controller_deinit();
        goto cleanup;
    }
    
    // Initialize Bluedroid stack
    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(TAG, "Bluedroid init failed: %s", esp_err_to_name(ret));
        esp_bt_controller_disable();
        esp_bt_controller_deinit();
        goto cleanup;
    }
    
    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(TAG, "Bluedroid enable failed: %s", esp_err_to_name(ret));
        esp_bluedroid_deinit();
        esp_bt_controller_disable();
        esp_bt_controller_deinit();
        goto cleanup;
    }

    
    ret = esp_ble_gap_register_callback(gap_event_handler);
    if (ret) {
        ESP_LOGE(TAG, "GAP callback register failed: %s", esp_err_to_name(ret));
        goto cleanup_bt;
    }
    
    ret = esp_ble_gattc_register_callback(gattc_event_handler);
    if (ret) {
        ESP_LOGE(TAG, "GATTC callback register failed: %s", esp_err_to_name(ret));
        goto cleanup_bt;
    }
    
    // CRITICAL: Register with specific app_id (match server)
    ret = esp_ble_gattc_app_register(ESP_APP_ID);
    if (ret) {
        ESP_LOGE(TAG, "GATTC app register failed: %s", esp_err_to_name(ret));
        goto cleanup_bt;
    }
    
    // Set larger MTU for image transfer
    ret = esp_ble_gatt_set_local_mtu(512);
    if (ret) {
        ESP_LOGW(TAG, "MTU set failed: %s", esp_err_to_name(ret));
    }

    
    ESP_LOGI(TAG, "BLE camera client initialized successfully");
    camera_initialized = true;
    return ESP_OK;

    cleanup_bt:
        esp_bluedroid_disable();
        esp_bluedroid_deinit();
        esp_bt_controller_disable();
        esp_bt_controller_deinit();

    cleanup:
        if (frame_mutex) {
            vSemaphoreDelete(frame_mutex);
            frame_mutex = NULL;
        }
        if (frame_queue) {
            vQueueDelete(frame_queue);
            frame_queue = NULL;
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
    
    // Clean up BLE
    if (esp_bluedroid_get_status() == ESP_BLUEDROID_STATUS_ENABLED) {
        esp_bluedroid_disable();
        esp_bluedroid_deinit();
    }
    
    if (esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_ENABLED) {
        esp_bt_controller_disable();
        esp_bt_controller_deinit();
    }
    
    // Clean up synchronization objects
    if (frame_mutex) {
        vSemaphoreDelete(frame_mutex);
        frame_mutex = NULL;
    }

    if (frame_queue) {
        vQueueDelete(frame_queue);
        frame_queue = NULL;
    }
    
    camera_initialized = false;
    ESP_LOGI(TAG, "BLE camera deinitialized");
    
    return ESP_OK;
}

esp_err_t ble_camera_connect(const char* device_name) {
    if (!camera_initialized || gattc_if == ESP_GATT_IF_NONE) {
        ESP_LOGE(TAG, "BLE camera not properly initialized");
        return ESP_ERR_INVALID_STATE;
    }

    // Check current BLE controller state first
    esp_bt_controller_status_t controller_status = esp_bt_controller_get_status();
    if (controller_status != ESP_BT_CONTROLLER_STATUS_ENABLED) {
        ESP_LOGE(TAG, "BT controller not enabled: %d", controller_status);
        return ESP_ERR_INVALID_STATE;
    }

    // CRITICAL: Check if we have an active connection attempt in progress
    if (camera_status == BLE_CAMERA_STATUS_CONNECTING) {
        ESP_LOGW(TAG, "Connection already in progress, waiting...");
        return ESP_OK;
    }

    // If status shows connected but conn_id is 0, force disconnect
    if (camera_status == BLE_CAMERA_STATUS_CONNECTED && conn_id == 0) {
        ESP_LOGW(TAG, "Invalid connection state (connected but conn_id=0), resetting...");
        camera_status = BLE_CAMERA_STATUS_DISCONNECTED;
    }

    // Only disconnect if we have a valid connection
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

    ESP_LOGI(TAG, "Connecting directly to ESP32-CAM...");
    
    // ESP32-CAM MAC address
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
    
    // Close GATT connection if we have a valid conn_id
    if (gattc_if != ESP_GATT_IF_NONE && conn_id != 0) {
        esp_err_t ret = esp_ble_gattc_close(gattc_if, conn_id);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to close GATT connection: %s", esp_err_to_name(ret));
        }
        ESP_LOGI(TAG, "Disconnect initiated");
        
        // Wait a bit for the close event, then force reset if needed
        vTaskDelay(pdMS_TO_TICKS(500));
        
        // Force reset connection state
        camera_status = BLE_CAMERA_STATUS_DISCONNECTED;
        conn_id = 0;
        service_start_handle = 0;
        service_end_handle = 0;
        char_control_handle = 0;
        char_image_handle = 0;
        char_status_handle = 0;
    } else {
        // Already disconnected
        camera_status = BLE_CAMERA_STATUS_DISCONNECTED;
        conn_id = 0;
    }
    
    return ESP_OK;
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
    // No explicit release needed as frames are copied to local buffer
    return ESP_OK;
}

esp_err_t ble_camera_start_streaming(void) {
    if (camera_status != BLE_CAMERA_STATUS_CONNECTED) {
        ESP_LOGW(TAG, "Camera not connected, cannot start streaming");
        return ESP_ERR_INVALID_STATE;
    }
    
    ESP_LOGI(TAG, "Starting camera streaming...");
    
    esp_err_t ret = send_camera_command(0x01, NULL, 0); // START_STREAM command
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
    
    esp_err_t ret = send_camera_command(0x02, NULL, 0); // STOP_STREAM command
    if (ret == ESP_OK) {
        camera_status = BLE_CAMERA_STATUS_CONNECTED;
        ESP_LOGI(TAG, "Camera streaming stopped");
    }
    
    return ret;
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
            stats->frame_rate = 1000.0f / time_diff; // FPS
        }
        
        // Simulate signal strength
        stats->signal_strength = -65; // dBm
        
        xSemaphoreGive(frame_mutex);
    }
    
    return ESP_OK;
}

esp_err_t ble_camera_set_resolution(uint8_t width, uint8_t height) {
    ESP_LOGI(TAG, "Setting camera resolution to %dx%d", width, height);
    
    uint8_t data[2] = {width, height};
    return send_camera_command(0x04, data, 2); // SET_RESOLUTION command
}

esp_err_t ble_camera_set_quality(uint8_t quality) {
    ESP_LOGI(TAG, "Setting camera quality to %d", quality);
    
    return send_camera_command(0x05, &quality, 1); // SET_QUALITY command
}

bool ble_camera_is_connected(void) {
    return (camera_status == BLE_CAMERA_STATUS_CONNECTED || 
            camera_status == BLE_CAMERA_STATUS_STREAMING);
}

bool ble_camera_frame_available(void) {
    if (!ble_camera_is_connected()) {
        return false;
    }
    
    bool available = false;
    if (xSemaphoreTake(frame_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        available = current_frame.valid;
        xSemaphoreGive(frame_mutex);
    }
    
    return available;
}

static void gattc_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if_param, esp_ble_gattc_cb_param_t *param) {
    ESP_LOGI(TAG, "GATTC event: %d", event);  // Add debug logging

    
    switch (event) {
        case ESP_GATTC_REG_EVT:
            if (gattc_if == ESP_GATT_IF_NONE) {
                ESP_LOGI(TAG, "GATT client registered, app_id: %d", param->reg.app_id);
                gattc_if = gattc_if_param;
            } else {
                ESP_LOGW(TAG, "GATT client already registered, ignoring duplicate");
            }
            break;
            
        case ESP_GATTC_OPEN_EVT:
            ESP_LOGI(TAG, "GATT connection result: status=%d, conn_id=%d", 
                     param->open.status, param->open.conn_id);
            
            if (param->open.status == ESP_GATT_OK) {

                /*
                if (param->open.conn_id == 0) {
                    ESP_LOGE(TAG, "CRITICAL: Invalid conn_id=0 from BLE stack");
                    camera_status = BLE_CAMERA_STATUS_ERROR;
                    
                    // Close this invalid connection
                    esp_ble_gattc_close(gattc_if, param->open.conn_id);
                    break;
                }
                */
                
                conn_id = param->open.conn_id;
                camera_status = BLE_CAMERA_STATUS_CONNECTED;
                
                ESP_LOGI(TAG, "✓ Connected with valid conn_id: %d", conn_id);
                
                // Request larger MTU for image data transfer
                esp_ble_gattc_send_mtu_req(gattc_if, conn_id);
                
                // Start service discovery
                esp_ble_gattc_search_service(gattc_if, conn_id, NULL);
                
            } else {
                ESP_LOGE(TAG, "Connection failed: status %d", param->open.status);
                camera_status = BLE_CAMERA_STATUS_ERROR;
                conn_id = 0;
            }
            break;
            
        case ESP_GATTC_CLOSE_EVT:
            ESP_LOGI(TAG, "Disconnected from camera");
            camera_status = BLE_CAMERA_STATUS_DISCONNECTED;
            conn_id = 0;
            break;
            
        case ESP_GATTC_CFG_MTU_EVT:
            ESP_LOGI(TAG, "MTU exchange complete: %d bytes", param->cfg_mtu.mtu);
            break;   

        case ESP_GATTC_SEARCH_RES_EVT:
            // Check if this is our camera service
            if (param->search_res.srvc_id.uuid.len == ESP_UUID_LEN_16 &&
                param->search_res.srvc_id.uuid.uuid.uuid16 == BLE_CAMERA_SERVICE_UUID) {
                ESP_LOGI(TAG, "Camera service found");
                service_start_handle = param->search_res.start_handle;
                service_end_handle = param->search_res.end_handle;
            }
            break;
            
        case ESP_GATTC_SEARCH_CMPL_EVT:
            ESP_LOGI(TAG, "Service discovery complete");
            if (service_start_handle > 0) {
                // Use fixed handle offsets (standard GATT practice)
                char_image_handle = service_start_handle + 2;
                char_control_handle = service_start_handle + 4; 
                char_status_handle = service_start_handle + 6;

                ESP_LOGI(TAG, "Connection ready!");
                ESP_LOGI(TAG, "Using handles - Image: %d, Control: %d, Status: %d", 
                        char_image_handle, char_control_handle, char_status_handle);
                
                uint16_t notify_en = 0x0001;
                esp_ble_gattc_write_char_descr(gattc_if, conn_id, 
                                            char_image_handle + 1,  // CCCD handle is usually +1
                                            sizeof(notify_en), 
                                            (uint8_t*)&notify_en,
                                            ESP_GATT_WRITE_TYPE_RSP,
                                            ESP_GATT_AUTH_REQ_NONE);
                
                ESP_LOGI(TAG, "GATT connection fully established - notifications enabled!");
            }
            break;

        case ESP_GATTC_WRITE_DESCR_EVT:
            if (param->write.status == ESP_GATT_OK) {
                ESP_LOGI(TAG, "Notifications enabled successfully");
            } else {
                ESP_LOGE(TAG, "Failed to enable notifications: %d", param->write.status);
            }
            break;
        
        /*    
        case ESP_GATTC_READ_CHAR_EVT:
            if (param->get_all_char.status == ESP_GATT_OK) {
                esp_gattc_char_elem_t *char_elem = param->get_all_char.char_elem_result;
                for (int i = 0; i < param->get_all_char.count; i++) {
                    if (char_elem[i].uuid.len == ESP_UUID_LEN_16) {
                        uint16_t char_uuid = char_elem[i].uuid.uuid.uuid16;
                        switch (char_uuid) {
                            case BLE_CAMERA_CHAR_CONTROL_UUID:
                                char_control_handle = char_elem[i].char_handle;
                                ESP_LOGI(TAG, "Control char handle: %d", char_control_handle);
                                break;
                            case BLE_CAMERA_CHAR_IMAGE_UUID:
                                char_image_handle = char_elem[i].char_handle;
                                ESP_LOGI(TAG, "Image char handle: %d", char_image_handle);
                                break;
                            case BLE_CAMERA_CHAR_STATUS_UUID:
                                char_status_handle = char_elem[i].char_handle;
                                ESP_LOGI(TAG, "Status char handle: %d", char_status_handle);
                                break;
                        }
                    }
                }
                ESP_LOGI(TAG, "All characteristics discovered - connection ready!");
            }
            break;
        */
            
        case ESP_GATTC_NOTIFY_EVT:
            if (param->notify.handle == char_image_handle) {
                ESP_LOGI(TAG, "Received image data: %d bytes", param->notify.value_len);
                process_image_data(param->notify.value, param->notify.value_len);
                
                // Update stats to show frames are being received
                camera_stats.frames_received++;
                camera_stats.bytes_received += param->notify.value_len;
            } else {
                ESP_LOGW(TAG, "Notification from unknown handle: %d", param->notify.handle);
            }
            break;

        default:
            break;
    }
}

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    ESP_LOGI(TAG, "GAP event: %d", event);  // Add debug logging
    
    switch (event) {

        /*
        case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT:
            ESP_LOGI(TAG, "Scan parameters set, status: %d", param->scan_param_cmpl.status);
            break;
            
        case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
            if (param->scan_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(TAG, "Scan start failed, status: %d", param->scan_start_cmpl.status);
                scan_in_progress = false;
            } else {
                ESP_LOGI(TAG, "Scan started successfully");
                scan_in_progress = true;
            }
            break;
            
        case ESP_GAP_BLE_SCAN_RESULT_EVT: {
            esp_ble_gap_cb_param_t *scan_result = param;
            
            if (scan_result->scan_rst.search_evt == ESP_GAP_SEARCH_INQ_RES_EVT) {
                // Log sMAC address for identification
                ESP_LOGI(TAG, "Device found - MAC: %02x:%02x:%02x:%02x:%02x:%02x, RSSI: %d", 
                        scan_result->scan_rst.bda[0], scan_result->scan_rst.bda[1], 
                        scan_result->scan_rst.bda[2], scan_result->scan_rst.bda[3],
                        scan_result->scan_rst.bda[4], scan_result->scan_rst.bda[5],
                        scan_result->scan_rst.rssi);
                
                uint8_t *adv_name = NULL;
                uint8_t adv_name_len = 0;
                
                adv_name = esp_ble_resolve_adv_data(scan_result->scan_rst.ble_adv,
                                                ESP_BLE_AD_TYPE_NAME_CMPL, &adv_name_len);
                
                if (adv_name != NULL && adv_name_len > 0) {
                    char device_name[64] = {0};
                    memcpy(device_name, adv_name, adv_name_len < 63 ? adv_name_len : 63);
                    ESP_LOGI(TAG, "Device name: %s", device_name);
                    
                    if (strcmp(device_name, target_device_name) == 0) {
                        ESP_LOGI(TAG, "Target camera found! Connecting...");
                        esp_ble_gap_stop_scanning();
                        memcpy(remote_bda, scan_result->scan_rst.bda, sizeof(esp_bd_addr_t));
                        esp_ble_gattc_open(gattc_if, remote_bda, scan_result->scan_rst.ble_addr_type, true);
                    }
                } else {
                    ESP_LOGI(TAG, "No device name in advertisement");
                }
            }
            break;
        }
        
        case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:
            ESP_LOGI(TAG, "Scan stopped, status: %d", param->scan_stop_cmpl.status);
            scan_in_progress = false;
            break;
        */
            
        default:
            ESP_LOGI(TAG, "Unhandled GAP event: %d", event);
            break;
    }
}

static esp_err_t send_camera_command(uint8_t cmd, uint8_t *data, size_t len) {
    // CRITICAL: Check conn_id first (conn_id = 0 means no connection)
    if (conn_id == 0) {
        ESP_LOGE(TAG, "No active connection (conn_id = 0)");
        camera_status = BLE_CAMERA_STATUS_DISCONNECTED; // Fix state mismatch
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
    
    // Send command via GATT write to control characteristic
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
    
    ESP_LOGI(TAG, "Processing image data: %d bytes", len);
    
    if (xSemaphoreTake(frame_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        // Copy data to frame buffer
        size_t copy_len = (len > sizeof(frame_buffer)) ? sizeof(frame_buffer) : len;
        memcpy(frame_buffer, data, copy_len);
        
        // Update current frame info
        current_frame.buffer = frame_buffer;
        current_frame.buffer_size = copy_len;
        current_frame.width = 320;  // Default QVGA
        current_frame.height = 240;
        current_frame.format = 1;   // JPEG
        current_frame.timestamp = esp_timer_get_time() / 1000;
        current_frame.sequence++;
        current_frame.valid = true;
        
        // Update statistics
        camera_stats.frames_received++;
        camera_stats.bytes_received += copy_len;
        camera_stats.last_frame_time = current_frame.timestamp;
        
        xSemaphoreGive(frame_mutex);
    }
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
    ESP_LOGI(TAG, "========================");
}
// ================================================================
// ESP32-CAM FIRMWARE - BLE Camera Server
// This code runs on the separate ESP32-CAM module
// ================================================================

#include "esp_camera.h"
#include "esp_log.h"
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_main.h"
#include "esp_mac.h"
#include "esp_gatt_common_api.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "nvs_flash.h"
#include <string.h>

static const char* TAG = "ESP32_CAM_SERVER";

// Camera pins for ESP32-CAM module
#define CAMERA_MODEL_AI_THINKER 1
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

// BLE Service UUIDs (must match client)
#define BLE_CAMERA_SERVICE_UUID         0x1820
#define BLE_CAMERA_CHAR_IMAGE_UUID      0x2A30
#define BLE_CAMERA_CHAR_CONTROL_UUID    0x2A31
#define BLE_CAMERA_CHAR_STATUS_UUID     0x2A32

#define DEVICE_NAME "ESP32CAM-SLG"  // Sign Language Glove Camera
#define MTU_SIZE 512

// Camera commands
typedef enum {
    BLE_CAM_CMD_START_STREAM = 0x01,
    BLE_CAM_CMD_STOP_STREAM = 0x02,
    BLE_CAM_CMD_CAPTURE = 0x03,
    BLE_CAM_CMD_SET_RESOLUTION = 0x04,
    BLE_CAM_CMD_SET_QUALITY = 0x05
} ble_camera_command_t;

// Camera state
static bool camera_initialized = false;
static bool streaming_active = false;
static bool client_connected = false;
static framesize_t current_framesize = FRAMESIZE_QVGA;
static int jpeg_quality = 12;

// BLE handles
static esp_gatt_if_t gatts_if = ESP_GATT_IF_NONE;
static uint16_t conn_id = 0xFFFF;
static uint16_t service_handle = 0;
static uint16_t char_image_handle = 0;
static uint16_t char_control_handle = 0;
static uint16_t char_status_handle = 0;

// Task handles
static TaskHandle_t camera_task_handle = NULL;
static QueueHandle_t command_queue;

// Command structure
typedef struct {
    uint8_t cmd;
    uint8_t data[16];
    size_t data_len;
} camera_command_t;

// Forward declarations
static void camera_task(void *arg);
static void init_camera(void);
esp_err_t send_image_in_chunks(uint8_t *data, size_t total_len);
static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
static esp_err_t send_status_update(uint8_t status);

void app_main() {
    esp_err_t ret;
    
    // Initialize NVS
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    ESP_LOGI(TAG, "ESP32-CAM BLE Server starting...");
    
    // Initialize camera
    init_camera();
    
    // Create command queue
    command_queue = xQueueCreate(10, sizeof(camera_command_t));
    
    // Initialize BLE
    ret = esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
    if (ret) {
        ESP_LOGI(TAG, "Bluetooth controller release failed: %s", esp_err_to_name(ret));
    }
    
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    ESP_ERROR_CHECK(ret);
    
    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    ESP_ERROR_CHECK(ret);
    
    ret = esp_bluedroid_init();
    ESP_ERROR_CHECK(ret);
    
    ret = esp_bluedroid_enable();
    ESP_ERROR_CHECK(ret);
    
    // Register callbacks
    ret = esp_ble_gap_register_callback(gap_event_handler);
    ESP_ERROR_CHECK(ret);
    
    ret = esp_ble_gatts_register_callback(gatts_event_handler);
    ESP_ERROR_CHECK(ret);
    
    ret = esp_ble_gatts_app_register(0);
    ESP_ERROR_CHECK(ret);
    
    // Set device name
    ret = esp_ble_gap_set_device_name(DEVICE_NAME);
    ESP_ERROR_CHECK(ret);
    
    // Create camera task
    xTaskCreatePinnedToCore(camera_task, "camera_task", 8192, NULL, 5, &camera_task_handle, 1);
    
    ESP_LOGI(TAG, "ESP32-CAM BLE Server ready");
}

static void init_camera(void) {
    camera_config_t config = {
        .pin_pwdn = PWDN_GPIO_NUM,
        .pin_reset = RESET_GPIO_NUM,
        .pin_xclk = XCLK_GPIO_NUM,
        .pin_sccb_sda = SIOD_GPIO_NUM,
        .pin_sccb_scl = SIOC_GPIO_NUM,
        .pin_d7 = Y9_GPIO_NUM,
        .pin_d6 = Y8_GPIO_NUM,
        .pin_d5 = Y7_GPIO_NUM,
        .pin_d4 = Y6_GPIO_NUM,
        .pin_d3 = Y5_GPIO_NUM,
        .pin_d2 = Y4_GPIO_NUM,
        .pin_d1 = Y3_GPIO_NUM,
        .pin_d0 = Y2_GPIO_NUM,
        .pin_vsync = VSYNC_GPIO_NUM,
        .pin_href = HREF_GPIO_NUM,
        .pin_pclk = PCLK_GPIO_NUM,
        .xclk_freq_hz = 20000000,
        .ledc_timer = LEDC_TIMER_0,
        .ledc_channel = LEDC_CHANNEL_0,
        .pixel_format = PIXFORMAT_JPEG,
        .frame_size = FRAMESIZE_QVGA,  // 320x240
        .jpeg_quality = 12,
        .fb_count = 2,
        .fb_location = CAMERA_FB_IN_PSRAM,
        .grab_mode = CAMERA_GRAB_WHEN_EMPTY
    };
    
    // Initialize camera
    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera init failed with error 0x%x", err);
        return;
    }
    
    // Set initial camera settings for gesture recognition
    sensor_t *s = esp_camera_sensor_get();
    if (s) {
        s->set_framesize(s, current_framesize);
        s->set_quality(s, jpeg_quality);
        s->set_brightness(s, 0);     // -2 to 2
        s->set_contrast(s, 0);       // -2 to 2
        s->set_saturation(s, 0);     // -2 to 2
        s->set_special_effect(s, 0); // 0 to 6 (0 - No Effect)
        s->set_whitebal(s, 1);       // 0 = disable , 1 = enable
        s->set_awb_gain(s, 1);       // 0 = disable , 1 = enable
        s->set_wb_mode(s, 0);        // 0 to 4 - if awb_gain enabled
        s->set_exposure_ctrl(s, 1);  // 0 = disable , 1 = enable
        s->set_aec2(s, 0);           // 0 = disable , 1 = enable
        s->set_ae_level(s, 0);       // -2 to 2
        s->set_aec_value(s, 300);    // 0 to 1200
        s->set_gain_ctrl(s, 1);      // 0 = disable , 1 = enable
        s->set_agc_gain(s, 0);       // 0 to 30
        s->set_gainceiling(s, (gainceiling_t)0);  // 0 to 6
        s->set_bpc(s, 0);            // 0 = disable , 1 = enable
        s->set_wpc(s, 1);            // 0 = disable , 1 = enable
        s->set_raw_gma(s, 1);        // 0 = disable , 1 = enable
        s->set_lenc(s, 1);           // 0 = disable , 1 = enable
        s->set_hmirror(s, 0);        // 0 = disable , 1 = enable
        s->set_vflip(s, 0);          // 0 = disable , 1 = enable
        s->set_dcw(s, 1);            // 0 = disable , 1 = enable
        s->set_colorbar(s, 0);       // 0 = disable , 1 = enable
    }
    
    camera_initialized = true;
    ESP_LOGI(TAG, "Camera initialized successfully");
}

static void camera_task(void *arg) {
    camera_command_t cmd;
    camera_fb_t *fb = NULL;
    
    ESP_LOGI(TAG, "Camera task started");
    
    while (1) {
        // Check for commands
        if (xQueueReceive(command_queue, &cmd, pdMS_TO_TICKS(100)) == pdTRUE) {
            switch (cmd.cmd) {
                case BLE_CAM_CMD_START_STREAM:
                    ESP_LOGI(TAG, "Starting video stream");
                    streaming_active = true;
                    send_status_update(1); // Streaming status
                    break;
                    
                case BLE_CAM_CMD_STOP_STREAM:
                    ESP_LOGI(TAG, "Stopping video stream");
                    streaming_active = false;
                    send_status_update(0); // Idle status
                    break;
                    
                case BLE_CAM_CMD_CAPTURE:
                    ESP_LOGI(TAG, "Single frame capture requested");
                    // Will capture below
                    break;
                    
                case BLE_CAM_CMD_SET_RESOLUTION:
                    if (cmd.data_len >= 2) {
                        uint8_t width_code = cmd.data[0];
                        framesize_t new_size = FRAMESIZE_QVGA;
                        
                        switch (width_code) {
                            case 160: new_size = FRAMESIZE_QQVGA; break;
                            case 320: new_size = FRAMESIZE_QVGA; break;
                            case 640: new_size = FRAMESIZE_VGA; break;
                        }
                        
                        sensor_t *s = esp_camera_sensor_get();
                        if (s) {
                            s->set_framesize(s, new_size);
                            current_framesize = new_size;
                            ESP_LOGI(TAG, "Resolution changed to %d", width_code);
                        }
                    }
                    break;
                    
                case BLE_CAM_CMD_SET_QUALITY:
                    if (cmd.data_len >= 1) {
                        jpeg_quality = cmd.data[0];
                        sensor_t *s = esp_camera_sensor_get();
                        if (s) {
                            s->set_quality(s, jpeg_quality);
                            ESP_LOGI(TAG, "JPEG quality set to %d", jpeg_quality);
                        }
                    }
                    break;
            }
        }
        
        // Capture and send frame if streaming or single capture requested
        if (client_connected && (streaming_active || cmd.cmd == BLE_CAM_CMD_CAPTURE)) {
            fb = esp_camera_fb_get();
            if (fb) {
                // Send frame in chunks due to BLE MTU limitations
                esp_err_t ret = send_image_in_chunks(fb->buf, fb->len);
                if (ret != ESP_OK) {
                    ESP_LOGW(TAG, "Failed to send image: %s", esp_err_to_name(ret));
                }
                esp_camera_fb_return(fb);
            } else {
                ESP_LOGW(TAG, "Failed to capture frame");
            }
            
            // Add delay for streaming to control frame rate
            if (streaming_active) {
                vTaskDelay(pdMS_TO_TICKS(200)); // 5 FPS
            }
        } else {
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}

esp_err_t send_image_in_chunks(uint8_t *data, size_t total_len) {
    if (!client_connected || gatts_if == ESP_GATT_IF_NONE) {
        return ESP_ERR_INVALID_STATE;
    }
    
    const size_t chunk_size = MTU_SIZE - 10; // Leave room for headers
    size_t offset = 0;
    uint16_t sequence = 0;
    
    // Send header packet first
    uint8_t header[8];
    header[0] = 0xFF; // Header marker
    header[1] = 0xFE; // Header marker
    header[2] = (total_len >> 24) & 0xFF; // Total length (big endian)
    header[3] = (total_len >> 16) & 0xFF;
    header[4] = (total_len >> 8) & 0xFF;
    header[5] = total_len & 0xFF;
    header[6] = (uint8_t)current_framesize; // Frame size info
    header[7] = jpeg_quality; // Quality info
    
    esp_err_t ret = esp_ble_gatts_send_indicate(gatts_if, conn_id, char_image_handle,
                                               sizeof(header), header, false);
    if (ret != ESP_OK) {
        return ret;
    }
    
    vTaskDelay(pdMS_TO_TICKS(10)); // Small delay between header and data
    
    // Send data chunks
    while (offset < total_len) {
        size_t chunk_len = (total_len - offset > chunk_size) ? chunk_size : (total_len - offset);
        
        // Prepare chunk with sequence number
        uint8_t chunk_data[MTU_SIZE];
        chunk_data[0] = (sequence >> 8) & 0xFF;
        chunk_data[1] = sequence & 0xFF;
        memcpy(chunk_data + 2, data + offset, chunk_len);
        
        ret = esp_ble_gatts_send_indicate(gatts_if, conn_id, char_image_handle,
                                         chunk_len + 2, chunk_data, false);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to send chunk %d: %s", sequence, esp_err_to_name(ret));
            return ret;
        }
        
        offset += chunk_len;
        sequence++;
        
        // Small delay to prevent overwhelming the BLE stack
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    
    ESP_LOGI(TAG, "Sent image: %d bytes in %d chunks", total_len, sequence);
    return ESP_OK;
}

static esp_err_t send_status_update(uint8_t status) {
    if (!client_connected || gatts_if == ESP_GATT_IF_NONE) {
        return ESP_ERR_INVALID_STATE;
    }
    
    uint8_t status_data[4];
    status_data[0] = status; // 0=idle, 1=streaming, 2=error
    status_data[1] = (uint8_t)current_framesize;
    status_data[2] = jpeg_quality;
    status_data[3] = camera_initialized ? 1 : 0;
    
    return esp_ble_gatts_send_indicate(gatts_if, conn_id, char_status_handle,
                                      sizeof(status_data), status_data, false);
}

// Advertising parameters
static esp_ble_adv_params_t adv_params = {
    .adv_int_min = 0x20,
    .adv_int_max = 0x40,
    .adv_type = ADV_TYPE_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if_param, esp_ble_gatts_cb_param_t *param) {
    switch (event) {
        case ESP_GATTS_REG_EVT:
            ESP_LOGI(TAG, "GATT server registered, app_id: %d", param->reg.app_id);
            gatts_if = gatts_if_param;
            
            // Create service
            esp_gatt_srvc_id_t service_uuid = {
                .is_primary = true,
                .id = {
                    .inst_id = 0,
                    .uuid = {
                        .len = ESP_UUID_LEN_16,
                        .uuid = { .uuid16 = BLE_CAMERA_SERVICE_UUID }
                    }
                }
            };
            esp_ble_gatts_create_service(gatts_if, &service_uuid, 8);
            break;
            
        case ESP_GATTS_CREATE_EVT:
            ESP_LOGI(TAG, "Service created, service_handle: %d", param->create.service_handle);
            service_handle = param->create.service_handle;
            
            // Start service
            esp_ble_gatts_start_service(service_handle);
            
            // Add characteristics
            esp_bt_uuid_t char_image_uuid = {
                .len = ESP_UUID_LEN_16,
                .uuid.uuid16 = BLE_CAMERA_CHAR_IMAGE_UUID
            };
            esp_ble_gatts_add_char(service_handle, &char_image_uuid,
                                  ESP_GATT_PERM_READ,
                                  ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY,
                                  NULL, NULL);
            
            esp_bt_uuid_t char_control_uuid = {
                .len = ESP_UUID_LEN_16,
                .uuid.uuid16 = BLE_CAMERA_CHAR_CONTROL_UUID
            };
            esp_ble_gatts_add_char(service_handle, &char_control_uuid,
                                  ESP_GATT_PERM_WRITE,
                                  ESP_GATT_CHAR_PROP_BIT_WRITE,
                                  NULL, NULL);
            
            esp_bt_uuid_t char_status_uuid = {
                .len = ESP_UUID_LEN_16,
                .uuid.uuid16 = BLE_CAMERA_CHAR_STATUS_UUID
            };
            esp_ble_gatts_add_char(service_handle, &char_status_uuid,
                                  ESP_GATT_PERM_READ,
                                  ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY,
                                  NULL, NULL);
            break;
            
        case ESP_GATTS_ADD_CHAR_EVT:
            ESP_LOGI(TAG, "Characteristic added, char_handle: %d, uuid: 0x%04x",
                     param->add_char.attr_handle, param->add_char.char_uuid.uuid.uuid16);
            
            // Store characteristic handles
            switch (param->add_char.char_uuid.uuid.uuid16) {
                case BLE_CAMERA_CHAR_IMAGE_UUID:
                    char_image_handle = param->add_char.attr_handle;
                    break;
                case BLE_CAMERA_CHAR_CONTROL_UUID:
                    char_control_handle = param->add_char.attr_handle;
                    break;
                case BLE_CAMERA_CHAR_STATUS_UUID:
                    char_status_handle = param->add_char.attr_handle;
                    break;
            }
            break;
            
        case ESP_GATTS_START_EVT:
            ESP_LOGI(TAG, "Service started");
            
            // Configure advertising
            esp_ble_adv_data_t adv_data = {
                .set_scan_rsp = false,
                .include_name = true,
                .include_txpower = false,
                .min_interval = 0x0006,
                .max_interval = 0x0010,
                .appearance = 0x00,
                .manufacturer_len = 0,
                .p_manufacturer_data = NULL,
                .service_data_len = 0,
                .p_service_data = NULL,
                .service_uuid_len = sizeof(uint16_t),
                .p_service_uuid = (uint8_t*)&service_uuid.id.uuid.uuid.uuid16,
                .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
            };
            esp_ble_gap_config_adv_data(&adv_data);
            break;
            
        case ESP_GATTS_CONNECT_EVT:
            ESP_LOGI(TAG, "Client connected, conn_id: %d", param->connect.conn_id);
            conn_id = param->connect.conn_id;
            client_connected = true;
            
            // Update connection parameters for better throughput
            esp_ble_conn_update_params_t conn_params = {0};
            memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
            conn_params.latency = 0;
            conn_params.max_int = 0x20;    // 40ms
            conn_params.min_int = 0x10;    // 20ms  
            conn_params.timeout = 400;     // 4s
            esp_ble_gap_update_conn_params(&conn_params);
            
            send_status_update(0); // Send initial status
            break;
            
        case ESP_GATTS_DISCONNECT_EVT:
            ESP_LOGI(TAG, "Client disconnected, conn_id: %d", param->disconnect.conn_id);
            client_connected = false;
            streaming_active = false;
            conn_id = 0xFFFF;
            
            // Restart advertising
            esp_ble_gap_start_advertising(&adv_params);
            break;
            
        case ESP_GATTS_WRITE_EVT:
            if (param->write.handle == char_control_handle) {
                // Process control commands
                camera_command_t cmd;
                cmd.cmd = param->write.value[0];
                cmd.data_len = param->write.len - 1;
                if (cmd.data_len > 0) {
                    memcpy(cmd.data, param->write.value + 1, 
                           cmd.data_len > 16 ? 16 : cmd.data_len);
                }
                
                xQueueSend(command_queue, &cmd, 0);
                ESP_LOGI(TAG, "Received command: 0x%02x", cmd.cmd);
            }
            break;
            
        default:
            break;
    }
}

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    switch (event) {
        case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
            ESP_LOGI(TAG, "Advertising data set, starting advertising");
            esp_ble_gap_start_advertising(&adv_params);
            break;
            
        case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
            if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(TAG, "Advertising start failed");
            } else {
                ESP_LOGI(TAG, "Advertising started successfully");
            }
            break;
            
        default:
            break;
    }
}

#include "communication/ble_service.h"
#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "config/system_config.h"
#include "util/debug.h"
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_main.h"
#include "esp_bt_defs.h"
#include "esp_gatt_common_api.h"

static const char *TAG = "BLE_SERVICE";

// Define UUIDs
#define GATTS_SERVICE_UUID_SIGN_LANGUAGE   0x1800
#define GATTS_CHAR_UUID_GESTURE            0x2A1D
#define GATTS_CHAR_UUID_TEXT               0x2A1E
#define GATTS_CHAR_UUID_STATUS             0x2A1F
#define GATTS_CHAR_UUID_DEBUG              0x2A20
#define GATTS_CHAR_UUID_COMMAND            0x2A21

#define GATTS_NUM_HANDLE                   20
#define PROFILE_NUM                        1
#define PROFILE_APP_IDX                    0

// Application ID
#define GATTS_APP_ID                       0x66

// MTU size for BLE communication
#define BLE_MTU_SIZE                       500

// Characteristic properties
#define CHAR_PROP_READ                     (ESP_GATT_CHAR_PROP_BIT_READ)
#define CHAR_PROP_WRITE                    (ESP_GATT_CHAR_PROP_BIT_WRITE)
#define CHAR_PROP_NOTIFY                   (ESP_GATT_CHAR_PROP_BIT_NOTIFY)

// Descriptor UUID for Client Characteristic Configuration
#define ESP_GATT_UUID_CHAR_CLIENT_CONFIG   0x2902

// BLE advertising parameters
static esp_ble_adv_params_t adv_params = {
    .adv_int_min        = 0x20,
    .adv_int_max        = 0x40,
    .adv_type           = ADV_TYPE_IND,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    .channel_map        = ADV_CHNL_ALL,
    .adv_filter_policy  = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

// Profile structure
struct gatts_profile_inst {
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_handle;
    esp_gatt_srvc_id_t service_id;
    
    // Characteristic handles
    uint16_t gesture_char_handle;
    uint16_t gesture_descr_handle;
    uint16_t text_char_handle;
    uint16_t text_descr_handle;
    uint16_t status_char_handle;
    uint16_t status_descr_handle;
    uint16_t debug_char_handle;
    uint16_t debug_descr_handle;
    uint16_t command_char_handle;
};

// Global variables
static struct gatts_profile_inst gl_profile = {
    .gatts_cb = NULL,
    .gatts_if = ESP_GATT_IF_NONE,
    .app_id = GATTS_APP_ID,
    .conn_id = 0xFFFF,
    .service_handle = 0,
};

// Connection status
static bool is_connected = false;
static bool is_initialized = false;

// Command callback
static ble_command_callback_t command_callback = NULL;

// Notification enable flags
static bool gesture_notify_enable = false;
static bool text_notify_enable = false;
static bool status_notify_enable = false;
static bool debug_notify_enable = false;

// Forward declarations
static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

// Advertising data
static uint8_t adv_service_uuid128[16] = {
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 
    0x00, 0x10, 0x00, 0x00, 0x20, 0x18, 0x00, 0x00,
};

static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = true,
    .min_interval = 0x0006, //slave connection min interval, Time = min_interval * 1.25 msec
    .max_interval = 0x0010, //slave connection max interval, Time = max_interval * 1.25 msec
    .appearance = 0x00,
    .manufacturer_len = 0,
    .p_manufacturer_data =  NULL,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(adv_service_uuid128),
    .p_service_uuid = adv_service_uuid128,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

// Scan response data
static esp_ble_adv_data_t scan_rsp_data = {
    .set_scan_rsp = true,
    .include_name = true,
    .include_txpower = true,
    .appearance = 0x00,
    .manufacturer_len = 0,
    .p_manufacturer_data = NULL,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = 0,
    .p_service_uuid = NULL,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

esp_err_t ble_service_init(void) {
    if (is_initialized) {
        ESP_LOGW(TAG, "BLE service already initialized");
        return ESP_OK;
    }
    
    ESP_LOGI(TAG, "Initializing BLE service...");
    
    // === ADD THIS BLE STACK INITIALIZATION ===
    ESP_LOGI(TAG, "Initializing BLE stack...");
    
    // Release classic BT memory
    esp_err_t ret = esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "BT controller mem release failed: %s", esp_err_to_name(ret));
    }
    
    // Initialize BT controller
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "BT controller init failed: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "BT controller initialized");
    
    // Enable BT controller
    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "BT controller enable failed: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "BT controller enabled");
    
    // Initialize Bluedroid
    ret = esp_bluedroid_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Bluedroid init failed: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "Bluedroid initialized");
    
    // Enable Bluedroid
    ret = esp_bluedroid_enable();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Bluedroid enable failed: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "Bluedroid enabled - BLE stack ready!");
    // === END BLE STACK INITIALIZATION ===
    
    ret = esp_ble_gap_register_callback(gap_event_handler);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register GAP callback: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "GAP callback registered successfully");
    
    // Register GATTS callback
    ret = esp_ble_gatts_register_callback(gatts_profile_event_handler);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register GATTS callback: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "GATTS callback registered successfully");
    
    // Register application
    ret = esp_ble_gatts_app_register(GATTS_APP_ID);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register GATTS app: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "GATTS app registered successfully");
    
    // Set device name
    ret = esp_ble_gap_set_device_name("SignLanguageGlove");
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set device name: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "Device name set successfully");
    
    is_initialized = true;
    ESP_LOGI(TAG, "BLE service initialized successfully - but NO BLE STACK INIT!");
    return ESP_OK;
}


esp_err_t ble_service_deinit(void) {
    if (!is_initialized) {
        return ESP_OK;
    }
    
    ESP_LOGI(TAG, "Deinitializing BLE service...");
    
    // Stop advertising if active
    esp_ble_gap_stop_advertising();
    
    // Disconnect if connected
    if (is_connected && gl_profile.conn_id != 0xFFFF) {
        esp_ble_gatts_close(gl_profile.gatts_if, gl_profile.conn_id);
    }
    
    is_initialized = false;
    is_connected = false;
    command_callback = NULL;
    
    ESP_LOGI(TAG, "BLE service deinitialized");
    return ESP_OK;
}

esp_err_t ble_service_enable(void) {
    if (!is_initialized) {
        ESP_LOGE(TAG, "BLE service not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    ESP_LOGI(TAG, "Enabling BLE service...");
    
    // Start advertising
    esp_err_t ret = esp_ble_gap_start_advertising(&adv_params);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start advertising: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "BLE service enabled - advertising started");
    return ESP_OK;
}

esp_err_t ble_service_disable(void) {
    ESP_LOGI(TAG, "Disabling BLE service...");
    
    // Stop advertising
    esp_err_t ret = esp_ble_gap_stop_advertising();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to stop advertising: %s", esp_err_to_name(ret));
    }
    
    ESP_LOGI(TAG, "BLE service disabled");
    return ESP_OK;
}

esp_err_t ble_service_is_connected(bool *connected) {
    if (!connected) {
        return ESP_ERR_INVALID_ARG;
    }
    
    *connected = is_connected;
    return ESP_OK;
}

esp_err_t ble_service_send_gesture(uint8_t gesture_id, const char *gesture_name, float confidence) {
    if (!is_connected || !gesture_notify_enable) {
        return ESP_OK;  // Not connected or notifications not enabled
    }
    
    if (!gesture_name) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Prepare data packet
    uint8_t buffer[64];
    size_t len = 0;
    
    buffer[len++] = gesture_id;
    
    // Copy gesture name
    size_t name_len = strlen(gesture_name);
    if (name_len > 32) name_len = 32;  // Limit to 32 characters
    
    buffer[len++] = (uint8_t)name_len;
    memcpy(buffer + len, gesture_name, name_len);
    len += name_len;
    
    // Copy confidence (as float, 4 bytes)
    memcpy(buffer + len, &confidence, sizeof(float));
    len += sizeof(float);
    
    // Send notification
    esp_err_t ret = esp_ble_gatts_send_indicate(gl_profile.gatts_if, gl_profile.conn_id, 
                                               gl_profile.gesture_char_handle, 
                                               len, buffer, false);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to send gesture notification: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "Gesture notification sent: ID=%d, name=%s, conf=%.2f", 
             gesture_id, gesture_name, confidence);
    return ESP_OK;
}

esp_err_t ble_service_send_text(const char *text) {
    if (!is_connected || !text_notify_enable) {
        return ESP_OK;  // Not connected or notifications not enabled
    }
    
    if (!text) {
        return ESP_ERR_INVALID_ARG;
    }
    
    size_t len = strlen(text);
    if (len > BLE_MTU_SIZE - 3) {
        len = BLE_MTU_SIZE - 3;  // Limit to MTU size minus ATT headers
    }
    
    // Send notification
    esp_err_t ret = esp_ble_gatts_send_indicate(gl_profile.gatts_if, gl_profile.conn_id, 
                                               gl_profile.text_char_handle, 
                                               len, (uint8_t *)text, false);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to send text notification: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "Text notification sent: %.*s", (int)len, text);
    return ESP_OK;
}

esp_err_t ble_service_send_status(uint8_t battery_level, uint8_t state, uint8_t error) {
    if (!is_connected || !status_notify_enable) {
        return ESP_OK;  // Not connected or notifications not enabled
    }
    
    // Prepare data packet
    uint8_t buffer[3];
    buffer[0] = battery_level;
    buffer[1] = state;
    buffer[2] = error;
    
    // Send notification
    esp_err_t ret = esp_ble_gatts_send_indicate(gl_profile.gatts_if, gl_profile.conn_id, 
                                               gl_profile.status_char_handle, 
                                               sizeof(buffer), buffer, false);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to send status notification: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "Status notification sent: battery=%d%%, state=%d, error=%d", 
             battery_level, state, error);
    return ESP_OK;
}

esp_err_t ble_service_send_debug(const char *data) {
    if (!is_connected || !debug_notify_enable) {
        return ESP_OK;  // Not connected or notifications not enabled
    }
    
    if (!data) {
        return ESP_ERR_INVALID_ARG;
    }
    
    size_t len = strlen(data);
    if (len > BLE_MTU_SIZE - 3) {
        len = BLE_MTU_SIZE - 3;  // Limit to MTU size minus ATT headers
    }
    
    // Send notification
    esp_err_t ret = esp_ble_gatts_send_indicate(gl_profile.gatts_if, gl_profile.conn_id, 
                                               gl_profile.debug_char_handle, 
                                               len, (uint8_t *)data, false);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to send debug notification: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGD(TAG, "Debug notification sent: %.*s", (int)len, data);
    return ESP_OK;
}

esp_err_t ble_service_process_command(const uint8_t *data, size_t length) {
    if (!data || length == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Process command based on first byte (command ID)
    uint8_t cmd_id = data[0];
    
    ESP_LOGI(TAG, "Received BLE command: 0x%02x, length: %d", cmd_id, length);
    
    // Call registered callback if any
    if (command_callback != NULL) {
        command_callback(data, length);
    }
    
    return ESP_OK;
}

esp_err_t ble_service_register_command_callback(ble_command_callback_t callback) {
    command_callback = callback;
    ESP_LOGI(TAG, "Command callback registered");
    return ESP_OK;
}

// GAP Event Handler
static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    switch (event) {
        case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
            ESP_LOGI(TAG, "Advertising data set complete");
            esp_ble_gap_start_advertising(&adv_params);
            break;
            
        case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
            ESP_LOGI(TAG, "Scan response data set complete");
            break;
            
        case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
            if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(TAG, "Advertising start failed: %d", param->adv_start_cmpl.status);
            } else {
                ESP_LOGI(TAG, "Advertising started successfully");
            }
            break;
            
        case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
            if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(TAG, "Advertising stop failed: %d", param->adv_stop_cmpl.status);
            } else {
                ESP_LOGI(TAG, "Advertising stopped");
            }
            break;
            
        case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
            ESP_LOGI(TAG, "Connection parameters updated: min_int=%d, max_int=%d, latency=%d, timeout=%d",
                     param->update_conn_params.min_int,
                     param->update_conn_params.max_int,
                     param->update_conn_params.latency,
                     param->update_conn_params.timeout);
            break;
            
        default:
            ESP_LOGD(TAG, "GAP event: %d", event);
            break;
    }
}

// GATTS Profile Event Handler - COMPLETE IMPLEMENTATION
static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    switch (event) {
        case ESP_GATTS_REG_EVT:
            ESP_LOGI(TAG, "GATT server registered, app_id: %d", param->reg.app_id);
            gl_profile.gatts_if = gatts_if;
            
            // Set advertising data
            esp_ble_gap_config_adv_data(&adv_data);
            esp_ble_gap_config_adv_data(&scan_rsp_data);
            
            // Create service
            esp_gatt_srvc_id_t service_id = {
                .is_primary = true,
                .id = {
                    .inst_id = 0,
                    .uuid = {
                        .len = ESP_UUID_LEN_16,
                        .uuid = { .uuid16 = GATTS_SERVICE_UUID_SIGN_LANGUAGE }
                    }
                }
            };
            
            gl_profile.service_id = service_id;
            esp_ble_gatts_create_service(gatts_if, &service_id, GATTS_NUM_HANDLE);
            break;
            
        case ESP_GATTS_CREATE_EVT:
            ESP_LOGI(TAG, "Service created, service_handle: %d", param->create.service_handle);
            gl_profile.service_handle = param->create.service_handle;
            
            // Start service
            esp_ble_gatts_start_service(gl_profile.service_handle);
            
            // Add gesture characteristic
            esp_bt_uuid_t gesture_char_uuid = {
                .len = ESP_UUID_LEN_16,
                .uuid.uuid16 = GATTS_CHAR_UUID_GESTURE
            };
            esp_ble_gatts_add_char(gl_profile.service_handle, &gesture_char_uuid,
                                   ESP_GATT_PERM_READ,
                                   ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY,
                                   NULL, NULL);
            break;
            
        case ESP_GATTS_ADD_CHAR_EVT:
            ESP_LOGI(TAG, "Characteristic added: UUID=0x%04x, handle=%d",
                     param->add_char.char_uuid.uuid.uuid16,
                     param->add_char.attr_handle);
            
            // Store characteristic handles
            switch (param->add_char.char_uuid.uuid.uuid16) {
                case GATTS_CHAR_UUID_GESTURE:
                    gl_profile.gesture_char_handle = param->add_char.attr_handle;
                    
                    // Add text characteristic
                    esp_bt_uuid_t text_char_uuid = {
                        .len = ESP_UUID_LEN_16,
                        .uuid.uuid16 = GATTS_CHAR_UUID_TEXT
                    };
                    esp_ble_gatts_add_char(gl_profile.service_handle, &text_char_uuid,
                                           ESP_GATT_PERM_READ,
                                           ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY,
                                           NULL, NULL);
                    break;
                    
                case GATTS_CHAR_UUID_TEXT:
                    gl_profile.text_char_handle = param->add_char.attr_handle;
                    
                    // Add status characteristic
                    esp_bt_uuid_t status_char_uuid = {
                        .len = ESP_UUID_LEN_16,
                        .uuid.uuid16 = GATTS_CHAR_UUID_STATUS
                    };
                    esp_ble_gatts_add_char(gl_profile.service_handle, &status_char_uuid,
                                           ESP_GATT_PERM_READ,
                                           ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY,
                                           NULL, NULL);
                    break;
                    
                case GATTS_CHAR_UUID_STATUS:
                    gl_profile.status_char_handle = param->add_char.attr_handle;
                    
                    // Add debug characteristic
                    esp_bt_uuid_t debug_char_uuid = {
                        .len = ESP_UUID_LEN_16,
                        .uuid.uuid16 = GATTS_CHAR_UUID_DEBUG
                    };
                    esp_ble_gatts_add_char(gl_profile.service_handle, &debug_char_uuid,
                                           ESP_GATT_PERM_READ,
                                           ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY,
                                           NULL, NULL);
                    break;
                    
                case GATTS_CHAR_UUID_DEBUG:
                    gl_profile.debug_char_handle = param->add_char.attr_handle;
                    
                    // Add command characteristic
                    esp_bt_uuid_t command_char_uuid = {
                        .len = ESP_UUID_LEN_16,
                        .uuid.uuid16 = GATTS_CHAR_UUID_COMMAND
                    };
                    esp_ble_gatts_add_char(gl_profile.service_handle, &command_char_uuid,
                                           ESP_GATT_PERM_WRITE,
                                           ESP_GATT_CHAR_PROP_BIT_WRITE,
                                           NULL, NULL);
                    break;
                    
                case GATTS_CHAR_UUID_COMMAND:
                    gl_profile.command_char_handle = param->add_char.attr_handle;
                    ESP_LOGI(TAG, "All characteristics added successfully");
                    break;
                    
                default:
                    break;
            }
            
            // Add descriptor for notification characteristics
            if (param->add_char.char_uuid.uuid.uuid16 != GATTS_CHAR_UUID_COMMAND) {
                esp_bt_uuid_t descr_uuid = {
                    .len = ESP_UUID_LEN_16,
                    .uuid.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG
                };
                esp_ble_gatts_add_char_descr(gl_profile.service_handle, &descr_uuid,
                                             ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                                             NULL, NULL);
            }
            break;
            
        case ESP_GATTS_ADD_CHAR_DESCR_EVT:
            ESP_LOGI(TAG, "Descriptor added: handle=%d", param->add_char_descr.attr_handle);
            
            // Store descriptor handles based on the characteristic they belong to
            // This is a simplified approach - in production you might want more sophisticated tracking
            static uint8_t descr_count = 0;
            descr_count++;
            
            switch (descr_count) {
                case 1:
                    gl_profile.gesture_descr_handle = param->add_char_descr.attr_handle;
                    break;
                case 2:
                    gl_profile.text_descr_handle = param->add_char_descr.attr_handle;
                    break;
                case 3:
                    gl_profile.status_descr_handle = param->add_char_descr.attr_handle;
                    break;
                case 4:
                    gl_profile.debug_descr_handle = param->add_char_descr.attr_handle;
                    ESP_LOGI(TAG, "All descriptors added - service ready!");
                    break;
                default:
                    break;
            }
            break;
            
        case ESP_GATTS_START_EVT:
            ESP_LOGI(TAG, "Service started, status: %d, service_handle: %d", 
                     param->start.status, param->start.service_handle);
            break;
            
        case ESP_GATTS_CONNECT_EVT:
            ESP_LOGI(TAG, "Client connected, conn_id: %d", param->connect.conn_id);
            is_connected = true;
            gl_profile.conn_id = param->connect.conn_id;
            
            // Update connection parameters for better performance
            esp_ble_conn_update_params_t conn_params = {0};
            memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
            conn_params.latency = 0;
            conn_params.max_int = 0x20;    // 40ms
            conn_params.min_int = 0x10;    // 20ms
            conn_params.timeout = 400;     // 4s
            
            esp_ble_gap_update_conn_params(&conn_params);
            
            // Stop advertising since we have a connection
            esp_ble_gap_stop_advertising();
            break;
            
        case ESP_GATTS_DISCONNECT_EVT:
            ESP_LOGI(TAG, "Client disconnected, conn_id: %d, reason: %d", 
                     param->disconnect.conn_id, param->disconnect.reason);
            is_connected = false;
            gl_profile.conn_id = 0xFFFF;
            
            // Reset notification flags
            gesture_notify_enable = false;
            text_notify_enable = false;
            status_notify_enable = false;
            debug_notify_enable = false;
            
            // Restart advertising
            esp_ble_gap_start_advertising(&adv_params);
            break;
            
        case ESP_GATTS_WRITE_EVT:
            ESP_LOGI(TAG, "Write event: handle=%d, len=%d", 
                     param->write.handle, param->write.len);
            
            // Handle descriptor writes (notification enable/disable)
            if (param->write.len == 2 && !param->write.is_prep) {
                uint16_t descr_value = param->write.value[1] << 8 | param->write.value[0];
                bool notify_enable = (descr_value == 0x0001);
                
                // Determine which characteristic descriptor was written
                if (param->write.handle == gl_profile.gesture_descr_handle) {
                    gesture_notify_enable = notify_enable;
                    ESP_LOGI(TAG, "Gesture notifications %s", notify_enable ? "enabled" : "disabled");
                } else if (param->write.handle == gl_profile.text_descr_handle) {
                    text_notify_enable = notify_enable;
                    ESP_LOGI(TAG, "Text notifications %s", notify_enable ? "enabled" : "disabled");
                } else if (param->write.handle == gl_profile.status_descr_handle) {
                    status_notify_enable = notify_enable;
                    ESP_LOGI(TAG, "Status notifications %s", notify_enable ? "enabled" : "disabled");
                } else if (param->write.handle == gl_profile.debug_descr_handle) {
                    debug_notify_enable = notify_enable;
                    ESP_LOGI(TAG, "Debug notifications %s", notify_enable ? "enabled" : "disabled");
                }
            }
            // Handle command characteristic writes
            else if (param->write.handle == gl_profile.command_char_handle) {
                ESP_LOGI(TAG, "Command received: %d bytes", param->write.len);
                ble_service_process_command(param->write.value, param->write.len);
            }
            
            // Send write response if needed
            if (param->write.need_rsp) {
                esp_ble_gatts_send_response(gatts_if, param->write.conn_id, 
                                           param->write.trans_id, ESP_GATT_OK, NULL);
            }
            break;
            
        case ESP_GATTS_READ_EVT:
            ESP_LOGI(TAG, "Read event: handle=%d", param->read.handle);
            
            // Prepare read response with dummy data
            esp_gatt_rsp_t rsp;
            memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
            rsp.attr_value.handle = param->read.handle;
            
            // Return appropriate data based on characteristic
            if (param->read.handle == gl_profile.gesture_char_handle) {
                strcpy((char*)rsp.attr_value.value, "GESTURE");
                rsp.attr_value.len = 7;
            } else if (param->read.handle == gl_profile.text_char_handle) {
                strcpy((char*)rsp.attr_value.value, "TEXT");
                rsp.attr_value.len = 4;
            } else if (param->read.handle == gl_profile.status_char_handle) {
                rsp.attr_value.value[0] = 85;  // 85% battery
                rsp.attr_value.value[1] = 1;   // Active state
                rsp.attr_value.value[2] = 0;   // No error
                rsp.attr_value.len = 3;
            } else if (param->read.handle == gl_profile.debug_char_handle) {
                strcpy((char*)rsp.attr_value.value, "DEBUG");
                rsp.attr_value.len = 5;
            } else {
                // Default response
                strcpy((char*)rsp.attr_value.value, "OK");
                rsp.attr_value.len = 2;
            }
            
            esp_ble_gatts_send_response(gatts_if, param->read.conn_id, 
                                       param->read.trans_id, ESP_GATT_OK, &rsp);
            break;
            
        case ESP_GATTS_EXEC_WRITE_EVT:
            ESP_LOGI(TAG, "Execute write event");
            esp_ble_gatts_send_response(gatts_if, param->exec_write.conn_id, 
                                       param->exec_write.trans_id, ESP_GATT_OK, NULL);
            break;
            
        case ESP_GATTS_MTU_EVT:
            ESP_LOGI(TAG, "MTU changed to: %d", param->mtu.mtu);
            break;
            
        case ESP_GATTS_CONF_EVT:
            ESP_LOGD(TAG, "Confirmation received for handle: %d", param->conf.handle);
            break;
            
        default:
            ESP_LOGD(TAG, "GATTS event: %d", event);
            break;
    }
}
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

#define GATTS_NUM_HANDLE                   10
#define PROFILE_NUM                        1
#define PROFILE_APP_IDX                    0

// BLE service parameters
#define DEVICE_NAME                        BLE_DEVICE_NAME
#define SERVICE_UUID                       GATTS_SERVICE_UUID_SIGN_LANGUAGE
#define SERVICE_INSTANCE_ID                0

// MTU size for BLE communication
#define BLE_MTU_SIZE                       500

// Characteristic properties
#define CHAR_PROP_READ                     (ESP_GATT_CHAR_PROP_BIT_READ)
#define CHAR_PROP_WRITE                    (ESP_GATT_CHAR_PROP_BIT_WRITE)
#define CHAR_PROP_NOTIFY                   (ESP_GATT_CHAR_PROP_BIT_NOTIFY)

// BLE advertising parameters
static esp_ble_adv_params_t adv_params = {
    .adv_int_min        = 0x20,
    .adv_int_max        = 0x40,
    .adv_type           = ADV_TYPE_IND,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    .channel_map        = ADV_CHNL_ALL,
    .adv_filter_policy  = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

// Service and characteristic handles
static uint16_t service_handle;
static uint16_t gesture_char_handle;
static uint16_t text_char_handle;
static uint16_t status_char_handle;
static uint16_t debug_char_handle;
static uint16_t command_char_handle;

// Connection status
static bool is_connected = false;

// Registration status
static bool is_registered = false;

// Interface ID
static uint8_t gatts_if = 0xFF;

// Connection ID
static uint16_t conn_id = 0xFFFF;

// Command callback
static ble_command_callback_t command_callback = NULL;

// Notification enable flags
static bool gesture_notify_enable = false;
static bool text_notify_enable = false;
static bool status_notify_enable = false;
static bool debug_notify_enable = false;

// Forward declarations for internal functions
static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

// Service definition
static struct gatts_profile_inst {
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_handle;
    esp_gatt_srvc_id_t service_id;
} profile_tab[PROFILE_NUM] = {
    [PROFILE_APP_IDX] = {
        .gatts_cb = gatts_profile_event_handler,
        .gatts_if = ESP_GATT_IF_NONE,
    },
};

esp_err_t ble_service_init(void) {
    esp_err_t ret;
    
    // Initialize NVS - should be done in app_main already
    
    ESP_LOGI(TAG, "Initializing BLE service...");
    
    // Release BT controller memory if needed
    ret = esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
    if (ret) {
        ESP_LOGW(TAG, "Failed to release BT controller memory: %s", esp_err_to_name(ret));
        // This is not a critical error, can continue
    }
    
    // Initialize BT controller with BLE mode
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(TAG, "Failed to initialize BT controller: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(TAG, "Failed to enable BT controller: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(TAG, "Failed to initialize Bluedroid: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(TAG, "Failed to enable Bluedroid: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Register callbacks
    ret = esp_ble_gatts_register_callback(gatts_event_handler);
    if (ret) {
        ESP_LOGE(TAG, "Failed to register GATTS callback: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = esp_ble_gap_register_callback(gap_event_handler);
    if (ret) {
        ESP_LOGE(TAG, "Failed to register GAP callback: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Register app
    ret = esp_ble_gatts_app_register(PROFILE_APP_IDX);
    if (ret) {
        ESP_LOGE(TAG, "Failed to register GATTS app: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Set device name
    ret = esp_ble_gap_set_device_name(DEVICE_NAME);
    if (ret) {
        ESP_LOGE(TAG, "Failed to set device name: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Configure MTU size
    ret = esp_ble_gatt_set_local_mtu(BLE_MTU_SIZE);
    if (ret) {
        ESP_LOGE(TAG, "Failed to set local MTU: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "BLE service initialized");
    return ESP_OK;
}

esp_err_t ble_service_deinit(void) {
    esp_err_t ret;
    
    // Stop advertising if needed
    if (is_registered) {
        esp_ble_gap_stop_advertising();
    }
    
    // Disable Bluedroid
    ret = esp_bluedroid_disable();
    if (ret) {
        ESP_LOGE(TAG, "Failed to disable Bluedroid: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = esp_bluedroid_deinit();
    if (ret) {
        ESP_LOGE(TAG, "Failed to deinitialize Bluedroid: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Disable BT controller
    ret = esp_bt_controller_disable();
    if (ret) {
        ESP_LOGE(TAG, "Failed to disable BT controller: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = esp_bt_controller_deinit();
    if (ret) {
        ESP_LOGE(TAG, "Failed to deinitialize BT controller: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Reset state
    is_connected = false;
    is_registered = false;
    gatts_if = 0xFF;
    conn_id = 0xFFFF;
    
    gesture_notify_enable = false;
    text_notify_enable = false;
    status_notify_enable = false;
    debug_notify_enable = false;
    
    ESP_LOGI(TAG, "BLE service deinitialized");
    return ESP_OK;
}

esp_err_t ble_service_enable(void) {
    if (!is_registered) {
        ESP_LOGE(TAG, "BLE service not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    // Start advertising
    esp_err_t ret = esp_ble_gap_start_advertising(&adv_params);
    if (ret) {
        ESP_LOGE(TAG, "Failed to start advertising: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "BLE advertising started");
    return ESP_OK;
}

esp_err_t ble_service_disable(void) {
    if (!is_registered) {
        return ESP_OK;  // Already disabled
    }
    
    // Stop advertising
    esp_err_t ret = esp_ble_gap_stop_advertising();
    if (ret) {
        ESP_LOGE(TAG, "Failed to stop advertising: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "BLE advertising stopped");
    return ESP_OK;
}

esp_err_t ble_service_is_connected(bool *connected) {
    if (connected == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    *connected = is_connected;
    return ESP_OK;
}

esp_err_t ble_service_send_gesture(uint8_t gesture_id, const char *gesture_name, float confidence) {
    if (!is_connected || !gesture_notify_enable) {
        return ESP_OK;  // Not connected or notifications not enabled
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
    esp_err_t ret = esp_ble_gatts_send_indicate(gatts_if, conn_id, gesture_char_handle, 
                                               len, buffer, false);
    if (ret) {
        ESP_LOGW(TAG, "Failed to send gesture notification: %s", esp_err_to_name(ret));
        return ret;
    }
    
    return ESP_OK;
}

esp_err_t ble_service_send_text(const char *text) {
    if (!is_connected || !text_notify_enable) {
        return ESP_OK;  // Not connected or notifications not enabled
    }
    
    size_t len = strlen(text);
    if (len > BLE_MTU_SIZE - 3) {
        len = BLE_MTU_SIZE - 3;  // Limit to MTU size minus ATT headers
    }
    
    // Send notification
    esp_err_t ret = esp_ble_gatts_send_indicate(gatts_if, conn_id, text_char_handle, 
                                               len, (uint8_t *)text, false);
    if (ret) {
        ESP_LOGW(TAG, "Failed to send text notification: %s", esp_err_to_name(ret));
        return ret;
    }
    
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
    esp_err_t ret = esp_ble_gatts_send_indicate(gatts_if, conn_id, status_char_handle, 
                                               sizeof(buffer), buffer, false);
    if (ret) {
        ESP_LOGW(TAG, "Failed to send status notification: %s", esp_err_to_name(ret));
        return ret;
    }
    
    return ESP_OK;
}

esp_err_t ble_service_send_debug(const char *data) {
    if (!is_connected || !debug_notify_enable) {
        return ESP_OK;  // Not connected or notifications not enabled
    }
    
    size_t len = strlen(data);
    if (len > BLE_MTU_SIZE - 3) {
        len = BLE_MTU_SIZE - 3;  // Limit to MTU size minus ATT headers
    }
    
    // Send notification
    esp_err_t ret = esp_ble_gatts_send_indicate(gatts_if, conn_id, debug_char_handle, 
                                               len, (uint8_t *)data, false);
    if (ret) {
        ESP_LOGW(TAG, "Failed to send debug notification: %s", esp_err_to_name(ret));
        return ret;
    }
    
    return ESP_OK;
}

esp_err_t ble_service_process_command(const uint8_t *data, size_t length) {
    if (data == NULL || length == 0) {
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
    return ESP_OK;
}

// Private function implementations

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    switch (event) {
        case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
            ESP_LOGI(TAG, "BLE advertising data set");
            esp_ble_gap_start_advertising(&adv_params);
            break;
            
        case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
            if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(TAG, "BLE advertising start failed: %d", param->adv_start_cmpl.status);
            } else {
                ESP_LOGI(TAG, "BLE advertising started");
            }
            break;
            
        case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
            if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(TAG, "BLE advertising stop failed: %d", param->adv_stop_cmpl.status);
            } else {
                ESP_LOGI(TAG, "BLE advertising stopped");
            }
            break;
            
        case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
            ESP_LOGI(TAG, "BLE connection parameters updated");
            break;
            
        default:
            break;
    }
}

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t _gatts_if, esp_ble_gatts_cb_param_t *param) {
    if (event == ESP_GATTS_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            profile_tab[param->reg.app_id].gatts_if = _gatts_if;
        } else {
            ESP_LOGE(TAG, "GATTS registration failed: %d", param->reg.status);
            return;
        }
    }
    
    // Find the appropriate callback based on gatts_if
    do {
        int idx;
        for (idx = 0; idx < PROFILE_NUM; idx++) {
            if (_gatts_if == ESP_GATT_IF_NONE || _gatts_if == profile_tab[idx].gatts_if) {
                if (profile_tab[idx].gatts_cb) {
                    profile_tab[idx].gatts_cb(event, _gatts_if, param);
                }
            }
        }
    } while (0);
}

static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t _gatts_if, esp_ble_gatts_cb_param_t *param) {
    switch (event) {
        case ESP_GATTS_REG_EVT:
            ESP_LOGI(TAG, "GATTS registered, status: %d, app_id: %d", param->reg.status, param->reg.app_id);
            
            // Save gatts_if for future use
            gatts_if = _gatts_if;
            
            // Create service
            esp_gatt_srvc_id_t service_id;
            service_id.id.inst_id = SERVICE_INSTANCE_ID;
            service_id.id.uuid.len = ESP_UUID_LEN_16;
            service_id.id.uuid.uuid.uuid16 = SERVICE_UUID;
            service_id.is_primary = true;
            
            esp_ble_gatts_create_service(_gatts_if, &service_id, GATTS_NUM_HANDLE);
            
            // Set the service flag
            is_registered = true;
            break;
            
        case ESP_GATTS_CREATE_EVT:
            ESP_LOGI(TAG, "CREATE_SERVICE_EVT, status %d, service_handle %d", param->create.status, param->create.service_handle);
            service_handle = param->create.service_handle;
            
            // Add gesture characteristic
            esp_bt_uuid_t gesture_uuid;
            gesture_uuid.len = ESP_UUID_LEN_16;
            gesture_uuid.uuid.uuid16 = GATTS_CHAR_UUID_GESTURE;
            
            esp_bt_uuid_t text_uuid;
            text_uuid.len = ESP_UUID_LEN_16;
            text_uuid.uuid.uuid16 = GATTS_CHAR_UUID_TEXT;
            
            esp_bt_uuid_t status_uuid;
            status_uuid.len = ESP_UUID_LEN_16;
            status_uuid.uuid.uuid16 = GATTS_CHAR_UUID_STATUS;
            
            esp_bt_uuid_t debug_uuid;
            debug_uuid.len = ESP_UUID_LEN_16;
            debug_uuid.uuid.uuid16 = GATTS_CHAR_UUID_DEBUG;
            
            esp_bt_uuid_t command_uuid;
            command_uuid.len = ESP_UUID_LEN_16;
            command_uuid.uuid.uuid16 = GATTS_CHAR_UUID_COMMAND;
            
            // Add characteristics
            esp_ble_gatts_add_char(service_handle, &gesture_uuid, ESP_GATT_PERM_READ, 
                                 CHAR_PROP_READ | CHAR_PROP_NOTIFY,
                                 NULL, NULL);
            
            esp_ble_gatts_add_char(service_handle, &text_uuid, ESP_GATT_PERM_READ, 
                                 CHAR_PROP_READ | CHAR_PROP_NOTIFY,
                                 NULL, NULL);
            
            esp_ble_gatts_add_char(service_handle, &status_uuid, ESP_GATT_PERM_READ, 
                                 CHAR_PROP_READ | CHAR_PROP_NOTIFY,
                                 NULL, NULL);
            
            esp_ble_gatts_add_char(service_handle, &debug_uuid, ESP_GATT_PERM_READ, 
                                 CHAR_PROP_READ | CHAR_PROP_NOTIFY,
                                 NULL, NULL);
            
            esp_ble_gatts_add_char(service_handle, &command_uuid, ESP_GATT_PERM_WRITE, 
                                 CHAR_PROP_WRITE,
                                 NULL, NULL);
            
            // Start service
            esp_ble_gatts_start_service(service_handle);
            
            // Configure advertising data
            esp_ble_adv_data_t adv_data = {
                .set_scan_rsp = false,
                .include_name = true,
                .include_txpower = false,
                .min_interval = 0x0006, // 7.5ms
                .max_interval = 0x0010, // 20ms
                .appearance = 0x00,
                .manufacturer_len = 0,
                .p_manufacturer_data = NULL,
                .service_data_len = 0,
                .p_service_data = NULL,
                .service_uuid_len = 0,
                .p_service_uuid = NULL,
                .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
            };
            
            esp_ble_gap_config_adv_data(&adv_data);
            break;
            
        case ESP_GATTS_ADD_CHAR_EVT:
            ESP_LOGI(TAG, "ADD_CHAR_EVT, char UUID: %04x, status: %d", param->add_char.char_uuid.uuid.uuid16, param->add_char.status);
            
            // Store characteristic handles based on UUID
            switch (param->add_char.char_uuid.uuid.uuid16) {
                case GATTS_CHAR_UUID_GESTURE:
                    gesture_char_handle = param->add_char.attr_handle;
                    break;
                case GATTS_CHAR_UUID_TEXT:
                    text_char_handle = param->add_char.attr_handle;
                    break;
                case GATTS_CHAR_UUID_STATUS:
                    status_char_handle = param->add_char.attr_handle;
                    break;
                case GATTS_CHAR_UUID_DEBUG:
                    debug_char_handle = param->add_char.attr_handle;
                    break;
                case GATTS_CHAR_UUID_COMMAND:
                    command_char_handle = param->add_char.attr_handle;
                    break;
                default:
                    break;
            }
            break;
            
        case ESP_GATTS_START_EVT:
            ESP_LOGI(TAG, "SERVICE_START_EVT, status: %d, service_handle: %d", param->start.status, param->start.service_handle);
            break;
            
        case ESP_GATTS_CONNECT_EVT:
            ESP_LOGI(TAG, "BLE client connected, conn_id: %d", param->connect.conn_id);
            is_connected = true;
            conn_id = param->connect.conn_id;
            
            // Update connection parameters
            esp_ble_conn_update_params_t conn_params = {0};
            memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
            conn_params.latency = 0;
            conn_params.max_int = 0x20;    // Max connection interval: 40ms
            conn_params.min_int = 0x10;    // Min connection interval: 20ms
            conn_params.timeout = 400;     // Supervision timeout: 4s
            
            esp_ble_gap_update_conn_params(&conn_params);
            break;
            
        case ESP_GATTS_DISCONNECT_EVT:
            ESP_LOGI(TAG, "BLE client disconnected, reason: %d", param->disconnect.reason);
            is_connected = false;
            conn_id = 0xFFFF;
            
            // Reset notification flags
            gesture_notify_enable = false;
            text_notify_enable = false;
            status_notify_enable = false;
            debug_notify_enable = false;
            
            // Restart advertising
            esp_ble_gap_start_advertising(&adv_params);
            break;
            
        case ESP_GATTS_WRITE_EVT:
            ESP_LOGI(TAG, "WRITE_EVT, handle: %d, value len: %d", param->write.handle, param->write.len);
            
            // Check if this is a client configuration descriptor write
            if (param->write.len == 2 && !param->write.is_prep) {
                uint16_t descr_value = param->write.value[1] << 8 | param->write.value[0];
                
                // Determine which characteristic is being configured for notifications
                if (param->write.handle == gesture_char_handle + 1) { // +1 for descriptor
                    gesture_notify_enable = (descr_value == 0x0001);
                    ESP_LOGI(TAG, "Gesture notifications %s", gesture_notify_enable ? "enabled" : "disabled");
                } else if (param->write.handle == text_char_handle + 1) {
                    text_notify_enable = (descr_value == 0x0001);
                    ESP_LOGI(TAG, "Text notifications %s", text_notify_enable ? "enabled" : "disabled");
                } else if (param->write.handle == status_char_handle + 1) {
                    status_notify_enable = (descr_value == 0x0001);
                    ESP_LOGI(TAG, "Status notifications %s", status_notify_enable ? "enabled" : "disabled");
                } else if (param->write.handle == debug_char_handle + 1) {
                    debug_notify_enable = (descr_value == 0x0001);
                    ESP_LOGI(TAG, "Debug notifications %s", debug_notify_enable ? "enabled" : "disabled");
                }
            }
            // Check if this is a command write
            else if (param->write.handle == command_char_handle && !param->write.is_prep) {
                // Process command
                ble_service_process_command(param->write.value, param->write.len);
            }
            break;
            
        case ESP_GATTS_READ_EVT:
            ESP_LOGI(TAG, "READ_EVT, handle: %d", param->read.handle);
            break;
            
        default:
            break;
    }
}
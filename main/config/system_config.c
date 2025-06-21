// Add this to main/config/system_config.c (create this file):

#include "config/system_config.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"

static const char *TAG = "SYSTEM_CONFIG";
#define CONFIG_NVS_NAMESPACE "system_config"
#define CONFIG_NVS_KEY "config"  

esp_err_t system_config_init(void) {
    return ESP_OK;
}

esp_err_t system_config_save(void) {
    nvs_handle_t nvs_handle;
    esp_err_t ret = nvs_open(CONFIG_NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error opening NVS handle: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = nvs_set_blob(nvs_handle, CONFIG_NVS_KEY, &g_system_config, sizeof(system_config_t));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error writing to NVS: %s", esp_err_to_name(ret));
    } else {
        ret = nvs_commit(nvs_handle);
    }
    
    nvs_close(nvs_handle);
    return ret;
}

esp_err_t system_config_load(void) {
    nvs_handle_t nvs_handle;
    esp_err_t ret = nvs_open(CONFIG_NVS_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (ret != ESP_OK) {
        return ret;
    }
    
    size_t required_size = sizeof(system_config_t);
    ret = nvs_get_blob(nvs_handle, CONFIG_NVS_KEY, &g_system_config, &required_size);
    nvs_close(nvs_handle);
    return ret;
}

esp_err_t system_config_reset_to_default(void) {
    // Reset to defaults (implement based on your default values)
    g_system_config.system_state = SYSTEM_STATE_INIT;
    g_system_config.last_error = SYSTEM_ERROR_NONE;
    g_system_config.output_mode = OUTPUT_MODE_TEXT_AND_AUDIO;
    // ... set other defaults
    return system_config_save();
}
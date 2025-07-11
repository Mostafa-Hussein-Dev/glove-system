#include "core/power_management.h"
#include <string.h>
#include "esp_log.h"
#include "esp_sleep.h"
#include "esp_pm.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "config/pin_definitions.h"
#include "util/debug.h"
#include "drivers/display.h"
#include "communication/ble_service.h"
#include "drivers/ble_camera.h"
#include "config/system_config.h"

static const char *TAG = "POWER_MGMT";

#define PERIPHERAL_SENSORS   0
#define PERIPHERAL_DISPLAY   1
#define PERIPHERAL_AUDIO     2
#define PERIPHERAL_BLE       3
#define PERIPHERAL_BLE_CAMERA    4

// ADC handles
static adc_oneshot_unit_handle_t adc_handle;
static adc_cali_handle_t adc_cali_handle = NULL;

// Power management state
static struct {
    power_mode_t current_mode;
    uint32_t last_activity_time_ms;
    bool peripherals_enabled[5];  // Status of each peripheral
    battery_status_t battery;
    uint32_t inactivity_timeout_ms;
    uint32_t deep_sleep_timeout_ms;
    bool is_sleeping;
} power_state = {
    .current_mode = POWER_MODE_BALANCED,
    .last_activity_time_ms = 0,
    .peripherals_enabled = {true, true, true, true, true},
    .battery = {0},
    .inactivity_timeout_ms = INACTIVITY_TIMEOUT_SEC * 1000,
    .deep_sleep_timeout_ms = DEEP_SLEEP_TIMEOUT_SEC * 1000,
    .is_sleeping = false
};

// Battery voltage to percentage mapping (approximate for 3.7V LiPo)
static const struct {
    uint16_t voltage_mv;
    uint8_t percentage;
} battery_levels[] = {
    {4200, 100},
    {4100, 90},
    {4000, 80},
    {3900, 70},
    {3800, 60},
    {3700, 50},
    {3600, 40},
    {3500, 30},
    {3400, 20},
    {3300, 10},
    {3200, 5},
    {3100, 0}
};
#define BATTERY_LEVELS_COUNT (sizeof(battery_levels) / sizeof(battery_levels[0]))

esp_err_t power_management_init(void) {
    esp_err_t ret;
    
    // FIXED: Initialize ADC for battery monitoring
    adc_oneshot_unit_init_cfg_t adc_init_config = {
        .unit_id = BATTERY_ADC_UNIT,  // Now correctly ADC_UNIT_1
    };
    ret = adc_oneshot_new_unit(&adc_init_config, &adc_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize ADC unit: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Configure ADC channel
    adc_oneshot_chan_cfg_t channel_config = {
        .atten = BATTERY_ADC_ATTENUATION,
        .bitwidth = ADC_BITWIDTH_12,
    };
    ret = adc_oneshot_config_channel(adc_handle, BATTERY_ADC_CHANNEL, &channel_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure ADC channel: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Initialize calibration
    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = BATTERY_ADC_UNIT,
        .atten = BATTERY_ADC_ATTENUATION,
        .bitwidth = ADC_BITWIDTH_12,
    };
    ret = adc_cali_create_scheme_curve_fitting(&cali_config, &adc_cali_handle);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "ADC calibration failed: %s", esp_err_to_name(ret));
        // Continue without calibration
    }
    
    // Configure GPIO for power control
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << SENSOR_POWER_CTRL_PIN),
        .pull_down_en = 0,
        .pull_up_en = 0
    };
    gpio_config(&io_conf);
    
    // NEW: Configure USB detection pin if defined
    #ifdef USB_DETECT_PIN
    gpio_config_t usb_detect_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << USB_DETECT_PIN),
        .pull_down_en = 1,
        .pull_up_en = 0
    };
    gpio_config(&usb_detect_conf);
    #endif
    
    // Enable all peripherals initially
    gpio_set_level(SENSOR_POWER_CTRL_PIN, 1);
    
    // Get initial battery status
    ret = power_management_get_battery_status(&power_state.battery);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get initial battery status");
        return ret;
    }
    
    // Set initial power mode based on battery status
    if (power_state.battery.is_critical) {
        power_management_set_mode(POWER_MODE_MAX_POWER_SAVE);
    } else if (power_state.battery.is_low) {
        power_management_set_mode(POWER_MODE_POWER_SAVE);
    } else {
        power_management_set_mode(POWER_MODE_BALANCED);
    }
    









    power_management_set_mode(POWER_MODE_BALANCED);










    
    // Set current time as last activity time
    power_state.last_activity_time_ms = esp_timer_get_time() / 1000;
    
    ESP_LOGI(TAG, "Power management initialized successfully");
    ESP_LOGI(TAG, "Battery: %.2fV (%d%%), %s", 
             power_state.battery.voltage_mv / 1000.0f,
             power_state.battery.percentage,
             power_state.battery.is_charging ? "Charging" : "Not charging");
    
    return ESP_OK;
}

esp_err_t power_management_set_mode(power_mode_t mode) {
    esp_err_t ret = ESP_OK;
    
    if (mode == power_state.current_mode) {
        // No change needed
        return ESP_OK;
    }
    
    // Update power mode
    power_state.current_mode = mode;
    
    // Apply power mode settings
    switch (mode) {
        case POWER_MODE_PERFORMANCE:
            // Maximum CPU frequency
            power_management_set_cpu_frequency(240);
            
            // Enable all peripherals
            for (int i = 0; i < 5; i++) {
                power_management_set_peripheral_power(i, true);
            }
            
            // Disable automatic light sleep
            esp_pm_configure(NULL);
            
            // Set long timeouts
            power_state.inactivity_timeout_ms = INACTIVITY_TIMEOUT_SEC * 2 * 1000;
            power_state.deep_sleep_timeout_ms = DEEP_SLEEP_TIMEOUT_SEC * 2 * 1000;
            
            ESP_LOGI(TAG, "Power mode set to PERFORMANCE");
            break;
            
        case POWER_MODE_BALANCED:
            // Medium CPU frequency
            power_management_set_cpu_frequency(160);
            
            // Enable all important peripherals
            power_management_set_peripheral_power(PERIPHERAL_SENSORS, true);
            power_management_set_peripheral_power(PERIPHERAL_DISPLAY, true);
            power_management_set_peripheral_power(PERIPHERAL_AUDIO, true);
            power_management_set_peripheral_power(PERIPHERAL_BLE, true);
            power_management_set_peripheral_power(PERIPHERAL_BLE_CAMERA, false);
            
            {
                // Configure automatic light sleep
                esp_pm_config_t balanced_pm_config = {
                    .max_freq_mhz = 160,
                    .min_freq_mhz = 80,
                    .light_sleep_enable = true
                };
                esp_pm_configure(&balanced_pm_config);
            }
            
            // Set standard timeouts
            power_state.inactivity_timeout_ms = INACTIVITY_TIMEOUT_SEC * 1000;
            power_state.deep_sleep_timeout_ms = DEEP_SLEEP_TIMEOUT_SEC * 1000;
            
            ESP_LOGI(TAG, "Power mode set to BALANCED");
            break;
            
        case POWER_MODE_POWER_SAVE:
            // Lower CPU frequency
            power_management_set_cpu_frequency(80);
            
            // Disable non-essential peripherals
            power_management_set_peripheral_power(PERIPHERAL_SENSORS, true);
            power_management_set_peripheral_power(PERIPHERAL_DISPLAY, true);
            power_management_set_peripheral_power(PERIPHERAL_AUDIO, false);
            power_management_set_peripheral_power(PERIPHERAL_BLE, true);
            power_management_set_peripheral_power(PERIPHERAL_BLE_CAMERA, false);
            
            {
                // Configure aggressive automatic light sleep
                esp_pm_config_t power_save_pm_config = {
                    .max_freq_mhz = 80,
                    .min_freq_mhz = 40,
                    .light_sleep_enable = true
                };
                esp_pm_configure(&power_save_pm_config);
            }
            
            // Set shorter timeouts
            power_state.inactivity_timeout_ms = (INACTIVITY_TIMEOUT_SEC / 2) * 1000;
            power_state.deep_sleep_timeout_ms = (DEEP_SLEEP_TIMEOUT_SEC / 2) * 1000;
            
            ESP_LOGI(TAG, "Power mode set to POWER_SAVE");
            break;
            
        case POWER_MODE_MAX_POWER_SAVE:
            // Minimum CPU frequency
            power_management_set_cpu_frequency(40);
            
            // Disable most peripherals
            power_management_set_peripheral_power(PERIPHERAL_SENSORS, true);
            power_management_set_peripheral_power(PERIPHERAL_DISPLAY, false);
            power_management_set_peripheral_power(PERIPHERAL_AUDIO, false);
            power_management_set_peripheral_power(PERIPHERAL_BLE, false);
            power_management_set_peripheral_power(PERIPHERAL_BLE_CAMERA, false);
            
            {
                // Configure very aggressive automatic light sleep
                esp_pm_config_t max_save_pm_config = {
                    .max_freq_mhz = 40,
                    .min_freq_mhz = 40,
                    .light_sleep_enable = true
                };
                esp_pm_configure(&max_save_pm_config);
            }
            
            // Set very short timeouts
            power_state.inactivity_timeout_ms = (INACTIVITY_TIMEOUT_SEC / 4) * 1000;
            power_state.deep_sleep_timeout_ms = (DEEP_SLEEP_TIMEOUT_SEC / 4) * 1000;
            
            ESP_LOGI(TAG, "Power mode set to MAX_POWER_SAVE");
            break;
    }
    
    return ret;
}

power_mode_t power_management_get_mode(void) {
    return power_state.current_mode;
}

esp_err_t power_management_get_battery_status(battery_status_t* status) {
    if (status == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Read battery voltage from ADC
    int adc_reading = 0;
    esp_err_t ret = adc_oneshot_read(adc_handle, BATTERY_ADC_CHANNEL, &adc_reading);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read ADC: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Convert ADC reading to voltage
    int voltage_mv = 0;
    if (adc_cali_handle) {
        ret = adc_cali_raw_to_voltage(adc_cali_handle, adc_reading, &voltage_mv);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to convert ADC reading to voltage: %s", esp_err_to_name(ret));
            // Fall back to simple calculation
            voltage_mv = adc_reading * 3300 / 4095;
        }
    } else {
        // No calibration available, use approximation
        voltage_mv = adc_reading * 3300 / 4095;
    }
    
    // FIXED: Apply voltage divider conversion (10kΩ + 10kΩ divider = 2x)
    voltage_mv *= 2;
    
    // Convert voltage to percentage
    uint8_t percentage = 0;
    for (int i = 0; i < BATTERY_LEVELS_COUNT - 1; i++) {
        if (voltage_mv >= battery_levels[i+1].voltage_mv) {
            // Linear interpolation between two points
            uint16_t v_diff = battery_levels[i].voltage_mv - battery_levels[i+1].voltage_mv;
            uint8_t p_diff = battery_levels[i].percentage - battery_levels[i+1].percentage;
            uint16_t v_offset = voltage_mv - battery_levels[i+1].voltage_mv;
            percentage = battery_levels[i+1].percentage + (p_diff * v_offset) / v_diff;
            break;
        }
    }
    
    // Detect charging status
    bool is_charging = false;
    #ifdef USB_DETECT_PIN
    // Check USB power presence
    is_charging = gpio_get_level(USB_DETECT_PIN);
    #else
    // Alternative: detect charging by voltage trend (simplified)
    static uint16_t prev_voltage = 0;
    if (prev_voltage > 0 && voltage_mv > prev_voltage + 50) {
        is_charging = true; // Voltage rising significantly
    }
    prev_voltage = voltage_mv;
    #endif
    
    // Fill status structure
    status->voltage_mv = voltage_mv;
    status->percentage = percentage;
    status->is_charging = is_charging;
    status->is_low = (voltage_mv < 3400);      // Below 20%
    status->is_critical = (voltage_mv < 3300); // Below 10%
    
    // Update internal state
    memcpy(&power_state.battery, status, sizeof(battery_status_t));
    
    return ESP_OK;
}

esp_err_t power_management_light_sleep(uint32_t sleep_duration_ms) {
    ESP_LOGI(TAG, "Entering light sleep for %d ms", sleep_duration_ms);
    
    // Set wakeup sources
    if (sleep_duration_ms > 0) {
        esp_sleep_enable_timer_wakeup(sleep_duration_ms * 1000);
    }
    
    // Enable wakeup from touch pad or IMU interrupt if connected
    esp_sleep_enable_ext0_wakeup(IMU_INT_PIN, 1);
    
    // Record that we're sleeping
    power_state.is_sleeping = true;
    
    // Enter light sleep
    esp_light_sleep_start();
    
    // Code continues here after wakeup
    power_state.is_sleeping = false;
    ESP_LOGI(TAG, "Woke up from light sleep");
    
    // Reset inactivity timer
    power_management_reset_inactivity_timer();
    
    return ESP_OK;
}

esp_err_t power_management_deep_sleep(uint32_t sleep_duration_ms) {
    ESP_LOGI(TAG, "Entering deep sleep for %d ms", sleep_duration_ms);
    
    // Set wakeup sources
    if (sleep_duration_ms > 0) {
        esp_sleep_enable_timer_wakeup(sleep_duration_ms * 1000);
    }
    
    // Enable wakeup from touch pad or IMU interrupt if connected
    esp_sleep_enable_ext0_wakeup(IMU_INT_PIN, 1);
    
    // Enter deep sleep (will reset the chip)
    esp_deep_sleep_start();
    
    // This code is never reached as deep sleep resets the chip
    return ESP_OK;
}

esp_err_t power_management_wake_up(void) {
    // This is called after waking up from light sleep
    // or at system startup after deep sleep
    
    // Reset inactivity timer
    power_management_reset_inactivity_timer();
    
    // If we were in MAX_POWER_SAVE mode, go back to BALANCED
    if (power_state.current_mode == POWER_MODE_MAX_POWER_SAVE) {
        power_management_set_mode(POWER_MODE_BALANCED);
    }
    
    return ESP_OK;
}

esp_err_t power_management_set_cpu_frequency(uint32_t frequency_mhz) {
    ESP_LOGI(TAG, "Setting CPU frequency to %d MHz", frequency_mhz);
    
    // Validate frequency
    if (frequency_mhz != 240 && frequency_mhz != 160 && 
        frequency_mhz != 80 && frequency_mhz != 40) {
        ESP_LOGE(TAG, "Invalid CPU frequency: %d MHz", frequency_mhz);
        return ESP_ERR_INVALID_ARG;
    }
    
    // Set CPU frequency
    esp_pm_config_t pm_config = {
        .max_freq_mhz = frequency_mhz,
        .min_freq_mhz = frequency_mhz,
        .light_sleep_enable = false
    };
    return esp_pm_configure(&pm_config);
}

esp_err_t power_management_set_peripheral_power(uint8_t peripheral, bool enable) {
    if (peripheral >= 5) {  // Updated to 5 peripherals
        return ESP_ERR_INVALID_ARG;
    }
    
    power_state.peripherals_enabled[peripheral] = enable;
    
    switch (peripheral) {
        case PERIPHERAL_SENSORS:
            // Control sensor power via GPIO
            gpio_set_level(SENSOR_POWER_CTRL_PIN, enable ? 1 : 0);
            ESP_LOGI(TAG, "Sensors power %s", enable ? "ON" : "OFF");
            break;
            
        case PERIPHERAL_DISPLAY:
            // Display power control
            if (enable) {
                display_init();
            } else {
                display_clear();
            }
            ESP_LOGI(TAG, "Display power %s", enable ? "ON" : "OFF");
            break;
            
        case PERIPHERAL_AUDIO:
            // Audio power control - using the SD pin for MAX98357A
            gpio_set_level(I2S_SD_PIN, enable ? 1 : 0);
            ESP_LOGI(TAG, "Audio power %s", enable ? "ON" : "OFF");
            break;
            
        case PERIPHERAL_BLE:
            // BLE power control
            if (enable) {
                ble_service_enable();
            } else {
                ble_service_disable();
            }
            ESP_LOGI(TAG, "BLE power %s", enable ? "ON" : "OFF");
            break;
            
        case PERIPHERAL_BLE_CAMERA:  // NEW: BLE Camera control
            if (enable) {
                ble_camera_connect(DEVICE_NAME);
            } else {
                ble_camera_disconnect();
            }
            ESP_LOGI(TAG, "BLE Camera power %s", enable ? "ON" : "OFF");
            break;
    }
    
    return ESP_OK;
}

esp_err_t power_management_process_inactivity(uint32_t current_time_ms) {
    // Skip if we're already sleeping
    if (power_state.is_sleeping) {
        return ESP_OK;
    }
    
    // Calculate inactivity time
    uint32_t inactivity_time_ms = current_time_ms - power_state.last_activity_time_ms;
    
    // Check for deep sleep timeout
    if (inactivity_time_ms > power_state.deep_sleep_timeout_ms) {
        ESP_LOGI(TAG, "Inactivity timeout reached for deep sleep: %d ms", inactivity_time_ms);
        
        // Enter deep sleep
        return power_management_deep_sleep(0); // 0 means indefinite sleep until external wakeup
    }
    
    // Check for light sleep timeout
    if (inactivity_time_ms > power_state.inactivity_timeout_ms) {
        ESP_LOGI(TAG, "Inactivity timeout reached for light sleep: %d ms", inactivity_time_ms);
        
        // Enter light sleep
        return power_management_light_sleep(power_state.deep_sleep_timeout_ms - inactivity_time_ms);
    }
    
    return ESP_OK;
}

esp_err_t power_management_reset_inactivity_timer(void) {
    power_state.last_activity_time_ms = esp_timer_get_time() / 1000;
    return ESP_OK;
}

esp_err_t power_management_optimize_for_camera(bool camera_active) {
    if (camera_active) {
        // Increase CPU frequency for BLE data processing
        esp_pm_config_t config = {
            .max_freq_mhz = 160,
            .min_freq_mhz = 80,   // Higher minimum for BLE stability
            .light_sleep_enable = false  // Disable sleep during camera use
        };
        esp_pm_configure(&config);
        
        // Disable other power-hungry peripherals temporarily
        power_management_set_peripheral_power(PERIPHERAL_AUDIO, false);
        
        ESP_LOGI(TAG, "Power optimized for camera mode");
    } else {
        // Return to normal power management
        power_management_set_mode(power_state.current_mode);
        ESP_LOGI(TAG, "Power returned to normal mode");
    }
    
    return ESP_OK;
}
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

#define PERIPHERAL_DISPLAY      1 
#define PERIPHERAL_AUDIO        2

// Sleep thresholds (simplified)
#define LIGHT_SLEEP_TIMEOUT_MS    30000   // 30 seconds
#define DEEP_SLEEP_TIMEOUT_MS     300000  // 5 minutes

// ADC handles
static adc_oneshot_unit_handle_t adc_handle = NULL;
static adc_cali_handle_t adc_cali_handle = NULL;

// Power state
static struct {
    power_mode_t current_mode;
    uint32_t last_activity_time_ms;
    battery_status_t battery;
    bool peripherals_enabled[5];
} power_state = {
    .current_mode = POWER_MODE_BALANCED,
    .last_activity_time_ms = 0,
    .battery = {0}
};

// 3.7V LiPo battery voltage-to-percentage mapping (simplified)
static const struct {
    uint16_t voltage_mv;
    uint8_t percentage;
} battery_curve[] = {
    {4200, 100}, {4100, 90}, {4000, 80}, {3900, 70}, {3800, 60},
    {3700, 50}, {3600, 40}, {3500, 30}, {3400, 20}, {3300, 10}, {3200, 0}
};

#define BATTERY_CURVE_SIZE (sizeof(battery_curve) / sizeof(battery_curve[0]))


esp_err_t power_management_init(void) {
    esp_err_t ret;
    
    ESP_LOGI(TAG, "Initializing simplified power management...");
    
    // Initialize ADC for battery monitoring (ADC2 Channel 7 = GPIO20)
    adc_oneshot_unit_init_cfg_t adc_config = {
        .unit_id = BATTERY_ADC_UNIT  // ADC_UNIT_2
    };
    ret = adc_oneshot_new_unit(&adc_config, &adc_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init ADC: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Configure battery monitoring channel
    adc_oneshot_chan_cfg_t channel_config = {
        .atten = BATTERY_ADC_ATTENUATION,  // ADC_ATTEN_DB_12 for 0-3.3V range
        .bitwidth = ADC_BITWIDTH_12
    };
    ret = adc_oneshot_config_channel(adc_handle, BATTERY_ADC_CHANNEL, &channel_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to config ADC channel: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Initialize ADC calibration for accurate voltage reading
    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = BATTERY_ADC_UNIT,
        .atten = BATTERY_ADC_ATTENUATION,
        .bitwidth = ADC_BITWIDTH_12
    };
    ret = adc_cali_create_scheme_curve_fitting(&cali_config, &adc_cali_handle);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "ADC calibration failed, using approximation");
        adc_cali_handle = NULL;
    }
    
    // Configure power control for audio (SD pin for MAX98357A)
    gpio_config_t audio_gpio = {
        .pin_bit_mask = (1ULL << I2S_SD_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .intr_type = GPIO_INTR_DISABLE,
        .pull_down_en = 0,
        .pull_up_en = 0
    };
    gpio_config(&audio_gpio);
    gpio_set_level(I2S_SD_PIN, 1); // Enable audio initially
    
    // Configure USB detection pin
    gpio_config_t usb_gpio = {
        .pin_bit_mask = (1ULL << USB_DETECT_PIN),
        .mode = GPIO_MODE_INPUT,
        .intr_type = GPIO_INTR_DISABLE,
        .pull_down_en = 1,  // Pull down when no USB
        .pull_up_en = 0
    };
    gpio_config(&usb_gpio);
    
    // Set initial activity time
    power_state.last_activity_time_ms = esp_timer_get_time() / 1000;
    
    // Get initial battery status
    power_management_get_battery_status(&power_state.battery);
    
    // Set initial power mode
    power_management_set_mode(POWER_MODE_BALANCED);
    
    ESP_LOGI(TAG, "Power management initialized - Battery: %.2fV (%d%%)", 
             power_state.battery.voltage_mv / 1000.0f, power_state.battery.percentage);
    
    return ESP_OK;
}

esp_err_t power_management_set_mode(power_mode_t mode) {
    esp_err_t ret = ESP_OK;
    
    ESP_LOGI(TAG, "Setting power mode: %d", mode);
    
    switch (mode) {
        case POWER_MODE_PERFORMANCE:
            // Maximum performance: 240MHz, all peripherals ON
            {
                esp_pm_config_t perf_config = {
                    .max_freq_mhz = 240,
                    .min_freq_mhz = 240,
                    .light_sleep_enable = false
                };
                ret = esp_pm_configure(&perf_config);
            }
            // Enable all peripherals
            power_management_set_peripheral_power(PERIPHERAL_DISPLAY, true);
            power_management_set_peripheral_power(PERIPHERAL_AUDIO, true);
            ESP_LOGI(TAG, "PERFORMANCE mode: 240MHz, all peripherals ON");
            break;
            
        case POWER_MODE_BALANCED:
            // Balanced: 160MHz, smart power management
            {
                esp_pm_config_t bal_config = {
                    .max_freq_mhz = 160,
                    .min_freq_mhz = 40,
                    .light_sleep_enable = true  // Allow automatic light sleep
                };
                ret = esp_pm_configure(&bal_config);
            }
            // Enable essential peripherals
            power_management_set_peripheral_power(PERIPHERAL_DISPLAY, true);
            power_management_set_peripheral_power(PERIPHERAL_AUDIO, true);
            ESP_LOGI(TAG, "BALANCED mode: 160MHz, auto light sleep enabled");
            break;
            
        case POWER_MODE_POWER_SAVE:
            // Power save: 80MHz, minimal peripherals
            {
                esp_pm_config_t save_config = {
                    .max_freq_mhz = 80,
                    .min_freq_mhz = 40,
                    .light_sleep_enable = true
                };
                ret = esp_pm_configure(&save_config);
            }
            // Disable non-essential peripherals
            power_management_set_peripheral_power(PERIPHERAL_DISPLAY, false);  // Turn off display
            power_management_set_peripheral_power(PERIPHERAL_AUDIO, false);    // Turn off audio
            ESP_LOGI(TAG, "POWER_SAVE mode: 80MHz, display and audio OFF");
            break;
            
        default:
            ESP_LOGE(TAG, "Invalid power mode: %d", mode);
            return ESP_ERR_INVALID_ARG;
    }
    
    if (ret == ESP_OK) {
        power_state.current_mode = mode;
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
    
    // Read raw ADC value
    int adc_raw = 0;
    esp_err_t ret = adc_oneshot_read(adc_handle, BATTERY_ADC_CHANNEL, &adc_raw);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ADC read failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Convert to voltage
    int voltage_mv = 0;
    if (adc_cali_handle) {
        // Use calibration for accurate reading
        ret = adc_cali_raw_to_voltage(adc_cali_handle, adc_raw, &voltage_mv);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Calibration failed, using approximation");
            voltage_mv = (adc_raw * 3300) / 4095;  // 12-bit ADC approximation
        }
    } else {
        voltage_mv = (adc_raw * 3300) / 4095;
    }
    
    // Apply voltage divider correction (assuming 2:1 divider: 10kΩ + 10kΩ)
    voltage_mv *= 2;
    
    // Convert voltage to percentage using lookup table
    uint8_t percentage = 0;
    for (int i = 0; i < BATTERY_CURVE_SIZE - 1; i++) {
        if (voltage_mv >= battery_curve[i + 1].voltage_mv) {
            // Linear interpolation between two points
            uint16_t v_diff = battery_curve[i].voltage_mv - battery_curve[i + 1].voltage_mv;
            uint8_t p_diff = battery_curve[i].percentage - battery_curve[i + 1].percentage;
            uint16_t v_offset = voltage_mv - battery_curve[i + 1].voltage_mv;
            percentage = battery_curve[i + 1].percentage + (p_diff * v_offset) / v_diff;
            break;
        }
    }
    
    // Check USB charging status
    bool is_charging = gpio_get_level(USB_DETECT_PIN);
    
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

sleep_action_t power_management_check_sleep(uint32_t inactive_time_ms, bool low_battery) {
    // Simple sleep decision logic
    if (low_battery) {
        // If battery is low, sleep sooner
        if (inactive_time_ms > (LIGHT_SLEEP_TIMEOUT_MS / 2)) {
            return SLEEP_ACTION_LIGHT;
        }
        if (inactive_time_ms > (DEEP_SLEEP_TIMEOUT_MS / 2)) {
            return SLEEP_ACTION_DEEP;
        }
    } else {
        // Normal sleep thresholds
        if (inactive_time_ms > LIGHT_SLEEP_TIMEOUT_MS) {
            return SLEEP_ACTION_LIGHT;
        }
        if (inactive_time_ms > DEEP_SLEEP_TIMEOUT_MS) {
            return SLEEP_ACTION_DEEP;
        }
    }
    
    return SLEEP_ACTION_NONE;
}

esp_err_t power_management_light_sleep(uint32_t wakeup_time_ms) {
    ESP_LOGI(TAG, "Entering light sleep for %d ms", wakeup_time_ms);
    
    // Configure wakeup sources
    if (wakeup_time_ms > 0) {
        esp_sleep_enable_timer_wakeup(wakeup_time_ms * 1000);  // Convert to microseconds
    }
    
    // Enable wakeup from IMU interrupt (motion detection)
    esp_sleep_enable_ext0_wakeup(IMU_INT_PIN, 1);
    
    // Enter light sleep (system continues after wake)
    esp_light_sleep_start();
    
    ESP_LOGI(TAG, "Woke up from light sleep");
    
    // Reset activity timer
    power_management_reset_activity();
    
    return ESP_OK;
}

esp_err_t power_management_deep_sleep(uint32_t wakeup_time_ms) {
    ESP_LOGI(TAG, "Entering deep sleep for %d ms", wakeup_time_ms);
    
    // Configure wakeup sources for deep sleep
    if (wakeup_time_ms > 0) {
        esp_sleep_enable_timer_wakeup(wakeup_time_ms * 1000);
    }
    
    // Enable wakeup from external interrupt
    esp_sleep_enable_ext0_wakeup(IMU_INT_PIN, 1);
    
    // Enter deep sleep (this will restart the system on wake)
    esp_deep_sleep_start();
    
    // This code is never reached as deep sleep resets the chip
    return ESP_OK;
}

void power_management_reset_activity(void) {
    power_state.last_activity_time_ms = esp_timer_get_time() / 1000;
}

uint32_t power_management_get_inactive_time(void) {
    uint32_t current_time_ms = esp_timer_get_time() / 1000;
    return current_time_ms - power_state.last_activity_time_ms;
}

esp_err_t power_management_set_peripheral_power(uint8_t peripheral, bool enable) {
    if (peripheral >= 2) {  // Only 2 peripherals now
        return ESP_ERR_INVALID_ARG;
    }
    
    switch (peripheral) {
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
            // Audio power control using SD pin for MAX98357A
            gpio_set_level(I2S_SD_PIN, enable ? 1 : 0);
            ESP_LOGI(TAG, "Audio power %s", enable ? "ON" : "OFF");
            break;
            
        default:
            return ESP_ERR_INVALID_ARG;
    }
    
    return ESP_OK;
}
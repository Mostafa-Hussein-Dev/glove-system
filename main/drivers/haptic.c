#include "drivers/haptic.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "config/pin_definitions.h"
#include "util/debug.h"

static const char *TAG = "HAPTIC";

// LEDC configuration for haptic PWM
#define HAPTIC_LEDC_TIMER          LEDC_TIMER_0
#define HAPTIC_LEDC_MODE           LEDC_LOW_SPEED_MODE
#define HAPTIC_LEDC_CHANNEL        LEDC_CHANNEL_0
#define HAPTIC_LEDC_DUTY_RES       LEDC_TIMER_8_BIT
#define HAPTIC_LEDC_FREQ           5000  // 5KHz PWM frequency
#define HAPTIC_MAX_DUTY            ((1 << HAPTIC_LEDC_DUTY_RES) - 1)

// Timer handle for pattern generation
static TimerHandle_t haptic_timer = NULL;

// State variables
static bool haptic_initialized = false;
static bool haptic_active = false;
static uint8_t haptic_intensity = 100;  // 0-100%
static const haptic_pattern_t *current_pattern = NULL;
static int current_step = 0;
static uint8_t current_pattern_length = 0;

// Forward declarations
static void haptic_timer_callback(TimerHandle_t xTimer);
static void haptic_set_motor_duty(uint8_t duty);

esp_err_t haptic_init(void) {
    esp_err_t ret;
    
    if (haptic_initialized) {
        return ESP_OK;  // Already initialized
    }
    
    // Configure LEDC timer
    ledc_timer_config_t ledc_timer = {
        .speed_mode = HAPTIC_LEDC_MODE,
        .timer_num = HAPTIC_LEDC_TIMER,
        .duty_resolution = HAPTIC_LEDC_DUTY_RES,
        .freq_hz = HAPTIC_LEDC_FREQ,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ret = ledc_timer_config(&ledc_timer);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure LEDC timer: %d", ret);
        return ret;
    }
    
    // Configure LEDC channel
    ledc_channel_config_t ledc_channel = {
        .speed_mode = HAPTIC_LEDC_MODE,
        .channel = HAPTIC_LEDC_CHANNEL,
        .timer_sel = HAPTIC_LEDC_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = HAPTIC_PIN,
        .duty = 0,  // Initial duty cycle 0 (motor off)
        .hpoint = 0
    };
    ret = ledc_channel_config(&ledc_channel);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure LEDC channel: %d", ret);
        return ret;
    }
    
    // Create a timer for pattern generation
    haptic_timer = xTimerCreate(
        "haptic_timer",
        pdMS_TO_TICKS(100),  // Default period, will be changed when used
        pdFALSE,  // One-shot timer
        0,
        haptic_timer_callback
    );
    
    if (haptic_timer == NULL) {
        ESP_LOGE(TAG, "Failed to create haptic timer");
        return ESP_ERR_NO_MEM;
    }
    
    haptic_initialized = true;
    haptic_active = false;
    
    ESP_LOGI(TAG, "Haptic driver initialized");
    
    return ESP_OK;
}

esp_err_t haptic_deinit(void) {
    if (!haptic_initialized) {
        return ESP_OK;  // Already deinitialized
    }
    
    // Stop any active haptic feedback
    haptic_stop();
    
    // Delete the timer
    if (haptic_timer != NULL) {
        xTimerDelete(haptic_timer, portMAX_DELAY);
        haptic_timer = NULL;
    }
    
    // Turn off LEDC
    ledc_stop(HAPTIC_LEDC_MODE, HAPTIC_LEDC_CHANNEL, 0);
    
    haptic_initialized = false;
    ESP_LOGI(TAG, "Haptic driver deinitialized");
    
    return ESP_OK;
}

esp_err_t haptic_set_intensity(uint8_t intensity) {
    if (!haptic_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Clamp intensity to 0-100
    if (intensity > 100) {
        intensity = 100;
    }
    
    haptic_intensity = intensity;
    ESP_LOGI(TAG, "Haptic intensity set to %d%%", intensity);
    
    return ESP_OK;
}

esp_err_t haptic_vibrate(uint16_t duration_ms) {
    if (!haptic_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Stop any active pattern
    if (haptic_active) {
        haptic_stop();
    }
    
    // Convert intensity to duty cycle (0-255)
    uint8_t duty = (haptic_intensity * HAPTIC_MAX_DUTY) / 100;
    
    // Set motor duty cycle
    haptic_set_motor_duty(duty);
    haptic_active = true;
    
    // Create one-shot timer to stop the motor
    xTimerChangePeriod(haptic_timer, pdMS_TO_TICKS(duration_ms), portMAX_DELAY);
    xTimerStart(haptic_timer, portMAX_DELAY);
    
    return ESP_OK;
}

esp_err_t haptic_play_pattern(const haptic_pattern_t *pattern, uint8_t pattern_length) {
    if (!haptic_initialized || pattern == NULL || pattern_length == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Stop any active pattern
    if (haptic_active) {
        haptic_stop();
    }
    
    // Store pattern information
    current_pattern = pattern;
    current_pattern_length = pattern_length;
    current_step = 0;
    
    // Start the pattern
    haptic_active = true;
    
    // Start with the first step
    uint8_t duty = (pattern[0].intensity * haptic_intensity * HAPTIC_MAX_DUTY) / (100 * 100);
    haptic_set_motor_duty(duty);
    
    // Set timer for the first step duration
    xTimerChangePeriod(haptic_timer, pdMS_TO_TICKS(pattern[0].duration_ms), portMAX_DELAY);
    xTimerStart(haptic_timer, portMAX_DELAY);
    
    return ESP_OK;
}

esp_err_t haptic_stop(void) {
    if (!haptic_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (!haptic_active) {
        return ESP_OK;  // Already stopped
    }
    
    // Stop the timer
    if (haptic_timer != NULL) {
        xTimerStop(haptic_timer, portMAX_DELAY);
    }
    
    // Turn off the motor
    haptic_set_motor_duty(0);
    
    // Reset pattern state
    current_pattern = NULL;
    current_pattern_length = 0;
    current_step = 0;
    
    haptic_active = false;
    
    return ESP_OK;
}

esp_err_t haptic_is_active(bool *active) {
    if (!haptic_initialized || active == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    *active = haptic_active;
    return ESP_OK;
}

// Timer callback function for haptic pattern generation
static void haptic_timer_callback(TimerHandle_t xTimer) {
    if (current_pattern == NULL) {
        // Simple vibrate mode
        haptic_set_motor_duty(0);  // Turn off motor
        haptic_active = false;
        return;
    }
    
    // Move to the next step in the pattern
    current_step++;
    
    if (current_step >= current_pattern_length) {
        // Pattern complete
        haptic_set_motor_duty(0);  // Turn off motor
        haptic_active = false;
        return;
    }
    
    // Set motor duty for the current step
    uint8_t duty = (current_pattern[current_step].intensity * haptic_intensity * HAPTIC_MAX_DUTY) / (100 * 100);
    haptic_set_motor_duty(duty);
    
    // Schedule the next step
    xTimerChangePeriod(haptic_timer, pdMS_TO_TICKS(current_pattern[current_step].duration_ms), portMAX_DELAY);
    xTimerStart(haptic_timer, portMAX_DELAY);
}

// Set motor duty cycle
static void haptic_set_motor_duty(uint8_t duty) {
    ledc_set_duty(HAPTIC_LEDC_MODE, HAPTIC_LEDC_CHANNEL, duty);
    ledc_update_duty(HAPTIC_LEDC_MODE, HAPTIC_LEDC_CHANNEL);
}
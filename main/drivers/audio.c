#include "drivers/audio.h"
#include <string.h>
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/i2s_std.h"
#include "driver/i2s_common.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "config/pin_definitions.h"
#include "util/debug.h"
#include "math.h"

static const char *TAG = "AUDIO";
static i2s_chan_handle_t tx_chan = NULL;

// I2S configuration
#define I2S_NUM I2S_NUM_0
#define I2S_SAMPLE_RATE 16000
#define I2S_BITS_PER_SAMPLE 16
#define I2S_DMA_BUFFER_SIZE 512
#define I2S_DMA_BUFFER_COUNT 8

// Audio task parameters
#define AUDIO_TASK_STACK_SIZE 2048
#define AUDIO_TASK_PRIORITY 10

// Audio buffer for playback
#define AUDIO_BUFFER_SIZE 1024
static int16_t audio_buffer[AUDIO_BUFFER_SIZE];

// Audio state
static bool audio_initialized = false;
static bool audio_playback_active = false;
static uint8_t audio_volume = 80;  // 0-100

// Queue for audio commands
static QueueHandle_t audio_command_queue = NULL;

// Task handle for audio task
static TaskHandle_t audio_task_handle = NULL;

// Simple audio command structure
typedef struct {
    audio_command_t command;
    char text[128];  // Text for TTS
    uint16_t tone_freq;  // Frequency for tone
    uint16_t duration_ms;  // Duration for tone
} audio_command_data_t;

// Forward declarations
static void audio_task(void *pvParameters);
static void audio_play_tone(uint16_t frequency, uint16_t duration_ms);
static void audio_speak_text(const char *text);

esp_err_t audio_init(void) {
    esp_err_t ret;
    
    if (audio_initialized) {
        return ESP_OK;  // Already initialized
    }
    
    // Configure I2S channel
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM, I2S_ROLE_MASTER);
    ret = i2s_new_channel(&chan_cfg, &tx_chan, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create I2S channel: %d", ret);
        return ret;
    }

    ESP_LOGI(TAG, "Configuring I2S pins: BCK=%d, WS=%d, DOUT=%d",
         I2S_BCK_PIN, I2S_WS_PIN, I2S_DATA_OUT_PIN);
    
    // Configure I2S standard
    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(I2S_SAMPLE_RATE),
        .slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = I2S_BCK_PIN,
            .ws = I2S_WS_PIN,
            .dout = I2S_DATA_OUT_PIN,
            .din = I2S_GPIO_UNUSED,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
        },
    };
    
    ret = i2s_channel_init_std_mode(tx_chan, &std_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize I2S standard mode: %d", ret);
        i2s_del_channel(tx_chan);
        return ret;
    }
    
    // Enable I2S channel
    ret = i2s_channel_enable(tx_chan);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable I2S channel: %d", ret);
        i2s_del_channel(tx_chan);
        return ret;
    }
    
    // Configure MAX98357A shutdown pin
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << I2S_SD_PIN),
        .pull_down_en = 0,
        .pull_up_en = 0
    };
    gpio_config(&io_conf);
    
    // Enable MAX98357A
    gpio_set_level(I2S_SD_PIN, 1);
    
    // Create audio command queue
    audio_command_queue = xQueueCreate(10, sizeof(audio_command_data_t));
    if (audio_command_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create audio command queue");
        i2s_channel_disable(tx_chan);
        i2s_del_channel(tx_chan);
        return ESP_ERR_NO_MEM;
    }
    
    // Create audio task
    BaseType_t xReturned = xTaskCreate(
        audio_task,
        "audio_task",
        AUDIO_TASK_STACK_SIZE,
        NULL,
        AUDIO_TASK_PRIORITY,
        &audio_task_handle
    );
    
    if (xReturned != pdPASS) {
        ESP_LOGE(TAG, "Failed to create audio task");
        vQueueDelete(audio_command_queue);
        i2s_channel_disable(tx_chan);
        i2s_del_channel(tx_chan);
        return ESP_ERR_NO_MEM;
    }
    
    audio_initialized = true;
    ESP_LOGI(TAG, "Audio system initialized");
    
    // Play startup sound
    audio_play_beep(1000, 100);
    
    return ESP_OK;
}

esp_err_t audio_deinit(void) {
    if (!audio_initialized) {
        return ESP_OK;  // Already deinitialized
    }
    
    // Stop any ongoing playback
    audio_stop();
    
    // Disable MAX98357A
    gpio_set_level(I2S_SD_PIN, 0);
    
    // Delete audio task
    if (audio_task_handle != NULL) {
        vTaskDelete(audio_task_handle);
        audio_task_handle = NULL;
    }
    
    // Delete command queue
    if (audio_command_queue != NULL) {
        vQueueDelete(audio_command_queue);
        audio_command_queue = NULL;
    }
    
    // Disable and delete I2S channel
    if (tx_chan != NULL) {
        i2s_channel_disable(tx_chan);
        i2s_del_channel(tx_chan);
        tx_chan = NULL;
    }
    
    audio_initialized = false;
    ESP_LOGI(TAG, "Audio system deinitialized");
    
    return ESP_OK;
}

esp_err_t audio_play_beep(uint16_t frequency, uint16_t duration_ms) {
    if (!audio_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    audio_command_data_t cmd = {
        .command = AUDIO_CMD_PLAY_TONE,
        .tone_freq = frequency,
        .duration_ms = duration_ms
    };
    
    if (xQueueSend(audio_command_queue, &cmd, pdMS_TO_TICKS(100)) != pdPASS) {
        ESP_LOGW(TAG, "Failed to queue audio command");
        return ESP_FAIL;
    }
    
    return ESP_OK;
}

esp_err_t audio_speak(const char *text) {
    if (!audio_initialized || text == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    
    audio_command_data_t cmd = {
        .command = AUDIO_CMD_SPEAK_TEXT
    };
    
    // Copy text (with truncation if needed)
    strncpy(cmd.text, text, sizeof(cmd.text) - 1);
    cmd.text[sizeof(cmd.text) - 1] = '\0';
    
    if (xQueueSend(audio_command_queue, &cmd, pdMS_TO_TICKS(100)) != pdPASS) {
        ESP_LOGW(TAG, "Failed to queue audio command");
        return ESP_FAIL;
    }
    
    return ESP_OK;
}

esp_err_t audio_stop(void) {
    if (!audio_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    audio_command_data_t cmd = {
        .command = AUDIO_CMD_STOP
    };
    
    if (xQueueSend(audio_command_queue, &cmd, pdMS_TO_TICKS(100)) != pdPASS) {
        ESP_LOGW(TAG, "Failed to queue audio command");
        return ESP_FAIL;
    }
    
    // Wait for playback to stop (short delay)
    vTaskDelay(pdMS_TO_TICKS(50));
    
    return ESP_OK;
}

esp_err_t audio_set_volume(uint8_t volume) {
    if (!audio_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Clamp volume to 0-100
    if (volume > 100) {
        volume = 100;
    }
    
    audio_volume = volume;
    ESP_LOGI(TAG, "Audio volume set to %d%%", volume);
    
    return ESP_OK;
}

uint8_t audio_get_volume(void) {
    return audio_volume;
}

bool audio_is_active(void) {
    return audio_playback_active;
}

// Audio task function
static void audio_task(void *pvParameters) {
    audio_command_data_t cmd;
    
    while (1) {
        // Wait for a command
        if (xQueueReceive(audio_command_queue, &cmd, portMAX_DELAY) == pdPASS) {
            switch (cmd.command) {
                case AUDIO_CMD_PLAY_TONE:
                    audio_playback_active = true;
                    audio_play_tone(cmd.tone_freq, cmd.duration_ms);
                    audio_playback_active = false;
                    break;
                    
                case AUDIO_CMD_SPEAK_TEXT:
                    audio_playback_active = true;
                    audio_speak_text(cmd.text);
                    audio_playback_active = false;
                    break;
                    
                case AUDIO_CMD_STOP:
                    // Reset I2S for immediate stop
                    i2s_channel_disable(tx_chan);
                    i2s_channel_enable(tx_chan); 
                    audio_playback_active = false;
                    break;
                    
                default:
                    ESP_LOGW(TAG, "Unknown audio command: %d", cmd.command);
                    break;
            }
        }
    }
}

// Generate and play a simple tone
static void audio_play_tone(uint16_t frequency, uint16_t duration_ms) {
    size_t i2s_bytes_written = 0;
    
    // Calculate parameters
    uint32_t sample_count = I2S_SAMPLE_RATE * duration_ms / 1000;
    float volume_scale = (float)audio_volume / 100.0f;
    float volume_factor = 32767.0f * volume_scale;  // Scale to int16_t range
    
    // Calculate wave parameters
    float angular_frequency = 2.0f * M_PI * frequency / I2S_SAMPLE_RATE;
    
    // Generate sine wave and send to I2S
    for (uint32_t i = 0; i < sample_count; i += AUDIO_BUFFER_SIZE / 2) {  // Divide by 2 because we need to generate both L and R channels
        uint32_t buffer_samples = (i + AUDIO_BUFFER_SIZE / 2 < sample_count) ? 
                                  AUDIO_BUFFER_SIZE / 2 : (sample_count - i);
        
        // Generate samples for both channels
        for (uint32_t j = 0; j < buffer_samples; j++) {
            float sample_value = sinf((i + j) * angular_frequency);
            int16_t sample = (int16_t)(sample_value * volume_factor);
            
            // Fill left and right channels with the same data
            audio_buffer[j*2] = sample;      // Left channel
            audio_buffer[j*2+1] = sample;    // Right channel
        }
        
        // Send to I2S (blocking)
        i2s_channel_write(tx_chan, audio_buffer, buffer_samples * 4, &i2s_bytes_written, portMAX_DELAY);
    }
    
    // Ensure buffer is flushed
    i2s_channel_disable(tx_chan);
    i2s_channel_enable(tx_chan); 
}

// Very simple text-to-speech (just beeps for now - would need a real TTS engine)
static void audio_speak_text(const char *text) {
    // For a real implementation, we'd integrate a TTS engine here
    // For now, just beep with different tones based on text length
    ESP_LOGI(TAG, "TTS (simulated): %s", text);
    
    // Play a sequence of beeps to simulate speech
    for (int i = 0; i < strlen(text) && i < 10; i++) {
        uint16_t freq = 500 + (text[i] % 1000);  // Generate frequency based on character
        audio_play_tone(freq, 100);
        vTaskDelay(pdMS_TO_TICKS(50));  // Short pause between beeps
    }
}
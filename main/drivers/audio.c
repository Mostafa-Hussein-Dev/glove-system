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
#include "esp_spiffs.h"
#include <sys/stat.h>
#include <sys/types.h>
#include <errno.h>

static const char *TAG = "AUDIO";
static i2s_chan_handle_t tx_chan = NULL;

#define AUDIO_FILE_BUFFER_SIZE 1024
#define AUDIO_FILES_PATH "/spiffs/audio/"

// I2S configuration
#define I2S_NUM I2S_NUM_0
#define I2S_SAMPLE_RATE 44100
#define I2S_BITS_PER_SAMPLE 16
#define I2S_DMA_BUFFER_SIZE 512
#define I2S_DMA_BUFFER_COUNT 8

// Audio task parameters
#define AUDIO_TASK_STACK_SIZE 4096
#define AUDIO_TASK_PRIORITY 10

// Audio buffer for playback
#define AUDIO_BUFFER_SIZE 1024
static int16_t audio_buffer[AUDIO_BUFFER_SIZE];

// Audio state
static bool audio_initialized = false;
static bool audio_playback_active = false;
static uint8_t audio_volume = 10;  // 0-100

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
esp_err_t audio_play_tone(uint16_t frequency, uint16_t duration_ms);

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
        .slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
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
    
    // Create audio command slot_cfg 
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

esp_err_t audio_play_tone(uint16_t frequency, uint16_t duration_ms) {
   if (!audio_initialized) {
       ESP_LOGE(TAG, "Audio system not initialized");
       return ESP_ERR_INVALID_STATE;
   }
   
   if (tx_chan == NULL) {
       ESP_LOGE(TAG, "I2S channel not available");
       return ESP_ERR_INVALID_STATE;
   }
   
   if (frequency == 0 || duration_ms == 0) {
       ESP_LOGW(TAG, "Invalid parameters: freq=%d, duration=%d", frequency, duration_ms);
       return ESP_ERR_INVALID_ARG;
   }
   
   size_t i2s_bytes_written = 0;
   esp_err_t ret = ESP_OK;
   
   // Calculate parameters
   uint32_t sample_count = I2S_SAMPLE_RATE * duration_ms / 1000;
   float volume_scale = (float)audio_volume / 100.0f;
   float volume_factor = 16383.0f * volume_scale; //Was 32767.0f
   
   // Calculate wave parameters
   float angular_frequency = 2.0f * M_PI * frequency / I2S_SAMPLE_RATE;
   
   ESP_LOGI(TAG, "Playing tone: %dHz for %dms (%lu samples)", frequency, duration_ms, sample_count);
   
   // Clear buffer first to prevent garbage data
   memset(audio_buffer, 0, sizeof(audio_buffer));

   // Generate sine wave and send to I2S
   for (uint32_t i = 0; i < sample_count; i += AUDIO_BUFFER_SIZE / 2) {
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

       // ADDED: Pad remaining buffer with zeros to prevent artifacts
       for (uint32_t j = buffer_samples; j < AUDIO_BUFFER_SIZE / 2; j++) {
           audio_buffer[j*2] = 0;
           audio_buffer[j*2+1] = 0;
       }
       
       // Send to I2S with timeout and error checking
       ret = i2s_channel_write(tx_chan, audio_buffer, buffer_samples * 4, 
                              &i2s_bytes_written, pdMS_TO_TICKS(1000));
       
       if (ret != ESP_OK) {
           ESP_LOGE(TAG, "I2S write failed: %s", esp_err_to_name(ret));
           break;
       }
       
       if (i2s_bytes_written != buffer_samples * 4) {
           ESP_LOGW(TAG, "Incomplete I2S write: expected %lu, wrote %zu", 
                    buffer_samples * 4, i2s_bytes_written);
       }
   }
   
   memset(audio_buffer, 0, sizeof(audio_buffer));
   i2s_channel_write(tx_chan, audio_buffer, 1024, &i2s_bytes_written, pdMS_TO_TICKS(100));
   
   if (ret == ESP_OK) {
       ESP_LOGI(TAG, "Tone playback completed successfully");
   }
   
   return ret;
}

esp_err_t audio_speak(const char *text) {
    if (!audio_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Try to play gesture audio file first
    if (audio_gesture_exists(text)) {
        audio_play_beep(1000, 200);
        return audio_play_gesture(text);
    }
    
    // Fallback to existing beep system
    audio_command_data_t cmd = {
        .command = AUDIO_CMD_SPEAK_TEXT
    };
    
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
    vTaskDelay(pdMS_TO_TICKS(200));
    
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

esp_err_t audio_play_gesture(const char* gesture_name) {
    if (!audio_initialized || !gesture_name) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Build file path
    char filepath[64];
    snprintf(filepath, sizeof(filepath), "%s%s.raw", AUDIO_FILES_PATH, gesture_name);
    
    // Open file
    FILE* audio_file = fopen(filepath, "rb");
    if (!audio_file) {
        ESP_LOGW(TAG, "Audio file not found: %s", filepath);
        // Fallback to beep
        return audio_play_beep(800, 200);
    }
    
    ESP_LOGI(TAG, "Playing audio: %s", gesture_name);

    // ✅ FIXED: Proper buffer declarations
    static int16_t file_buffer[AUDIO_FILE_BUFFER_SIZE];
    static int16_t stereo_buffer[AUDIO_FILE_BUFFER_SIZE * 2]; // For stereo conversion
    size_t bytes_read;
    size_t bytes_written;
    esp_err_t ret = ESP_OK;
    
    // ✅ FIXED: Proper silence padding at start
    memset(stereo_buffer, 0, 400 * sizeof(int16_t)); // 200 stereo samples = 400 int16_t
    i2s_channel_write(tx_chan, stereo_buffer, 800, &bytes_written, pdMS_TO_TICKS(100)); // 400 samples * 2 bytes

    // ✅ REMOVED: No channel disable/enable - this was causing clicking!
    // i2s_channel_disable(tx_chan);
    // vTaskDelay(pdMS_TO_TICKS(10));
    // i2s_channel_enable(tx_chan);

    // ✅ FIXED: Proper file reading and stereo conversion
    while ((bytes_read = fread(file_buffer, 1, sizeof(file_buffer), audio_file)) > 0) {
        // Convert mono samples to stereo
        size_t samples_read = bytes_read / sizeof(int16_t);
        
        for (size_t i = 0; i < samples_read; i++) {
            // ✅ FIXED: Convert mono to stereo and reduce volume
            int16_t sample = file_buffer[i] / 2; // Reduce volume to prevent clipping
            stereo_buffer[i * 2] = sample;      // Left channel
            stereo_buffer[i * 2 + 1] = sample;  // Right channel
        }
        
        // ✅ FIXED: Write correct amount of stereo data
        size_t stereo_bytes = samples_read * 2 * sizeof(int16_t);
        ret = i2s_channel_write(tx_chan, stereo_buffer, stereo_bytes, 
                               &bytes_written, pdMS_TO_TICKS(1000));
        
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "I2S write failed: %s", esp_err_to_name(ret));
            break;
        }
        
        if (bytes_written != stereo_bytes) {
            ESP_LOGW(TAG, "Incomplete I2S write: expected %zu, wrote %zu", 
                     stereo_bytes, bytes_written);
        }
    }

    // ✅ FIXED: Proper silence padding at end
    memset(stereo_buffer, 0, 400 * sizeof(int16_t));
    i2s_channel_write(tx_chan, stereo_buffer, 800, &bytes_written, pdMS_TO_TICKS(100));
    
    fclose(audio_file);
    
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Audio file playback completed successfully");
    }
    
    return ret;
}

bool audio_gesture_exists(const char* gesture_name) {
    if (!gesture_name) return false;
    
    char filepath[64];
    snprintf(filepath, sizeof(filepath), "%s%s.raw", AUDIO_FILES_PATH, gesture_name);
    
    FILE* file = fopen(filepath, "rb");
    if (file) {
        fclose(file);
        return true;
    }
    return false;
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
                    
                case AUDIO_CMD_STOP:
                    // Reset I2S for immediate stop
                    i2s_channel_disable(tx_chan);
                    i2s_channel_enable(tx_chan); 
                    audio_playback_active = false;
                    break;

                case AUDIO_CMD_SPEAK_TEXT:
                    ESP_LOGI(TAG, "Fallback beep for: %s", cmd.text);
                    audio_playback_active = true;
                    audio_play_tone(800, 200);  // Fallback beep
                    audio_playback_active = false;
                    break;
                    
                default:
                    ESP_LOGW(TAG, "Unknown audio command: %d", cmd.command);
                    break;
            }
        }
    }
}


#ifndef DRIVERS_AUDIO_H
#define DRIVERS_AUDIO_H

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Audio command types
 */
typedef enum {
    AUDIO_CMD_PLAY_TONE,
    AUDIO_CMD_SPEAK_TEXT,
    AUDIO_CMD_STOP
} audio_command_t;

/**
 * @brief Initialize audio subsystem
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t audio_init(void);

/**
 * @brief Deinitialize audio subsystem
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t audio_deinit(void);

/**
 * @brief Play a beep tone
 * 
 * @param frequency Tone frequency in Hz
 * @param duration_ms Duration in milliseconds
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t audio_play_beep(uint16_t frequency, uint16_t duration_ms);

/**
 * @brief Play a  tone
 * 
 * @param frequency Tone frequency in Hz
 * @param duration_ms Duration in milliseconds
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t audio_play_tone(uint16_t frequency, uint16_t duration_ms);

/**
 * @brief Speak text using TTS
 * 
 * @param text Text to speak
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t audio_speak(const char *text);

/**
 * @brief Stop audio playback
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t audio_stop(void);

/**
 * @brief Set audio volume
 * 
 * @param volume Volume level (0-100)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t audio_set_volume(uint8_t volume);

/**
 * @brief Get current audio volume
 * 
 * @return Current volume (0-100)
 */
uint8_t audio_get_volume(void);

/**
 * @brief Check if audio playback is active
 * 
 * @return true if audio is playing, false otherwise
 */
bool audio_is_active(void);

#endif /* DRIVERS_AUDIO_H */
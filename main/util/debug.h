#ifndef UTIL_DEBUG_H
#define UTIL_DEBUG_H

#include <stdint.h>
#include "esp_err.h"
#include "esp_log.h"

/**
 * @brief Debug log levels
 */
typedef enum {
    DEBUG_LEVEL_NONE = 0,   // No debug output
    DEBUG_LEVEL_ERROR,      // Only errors
    DEBUG_LEVEL_WARNING,    // Errors and warnings
    DEBUG_LEVEL_INFO,       // Errors, warnings, and info messages
    DEBUG_LEVEL_DEBUG,      // All messages
    DEBUG_LEVEL_VERBOSE     // Verbose debug information
} debug_level_t;

/**
 * @brief Debug mode flags
 */
typedef enum {
    DEBUG_MODE_NONE       = 0,
    DEBUG_MODE_UART       = (1 << 0),  // Output to UART
    DEBUG_MODE_DISPLAY    = (1 << 1),  // Output to OLED display
    DEBUG_MODE_BLUETOOTH  = (1 << 2),  // Output over BLE
} debug_mode_t;

/**
 * @brief Initialize debug subsystem
 * 
 * @param level Debug level
 * @param mode Debug mode flags (can be combined)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t debug_init(debug_level_t level, uint8_t mode);

/**
 * @brief Set debug level
 * 
 * @param level New debug level
 */
void debug_set_level(debug_level_t level);

/**
 * @brief Set debug mode
 * 
 * @param mode Debug mode flags (can be combined)
 */
void debug_set_mode(uint8_t mode);

/**
 * @brief Log a debug message
 * 
 * @param level Message level
 * @param tag Module tag
 * @param format Message format (printf style)
 * @param ... Message arguments
 */
void debug_log(debug_level_t level, const char* tag, const char* format, ...);

/**
 * @brief Log a debug message with buffer hex dump
 * 
 * @param level Message level
 * @param tag Module tag
 * @param prefix Message prefix
 * @param buffer Buffer to dump
 * @param length Buffer length
 */
void debug_buffer(debug_level_t level, const char* tag, const char* prefix, const void* buffer, size_t length);

/**
 * @brief Get current system time as string (for debugging)
 * 
 * @param buffer Buffer to store time string
 * @param size Buffer size
 */
void debug_get_time_str(char* buffer, size_t size);

/**
 * @brief Assert a condition and log error if failed
 * 
 * @param condition Condition to check
 * @param tag Module tag
 * @param message Error message
 * @param line Line number
 * @param file File name
 * @return ESP_OK if condition is true, ESP_FAIL otherwise
 */
esp_err_t debug_assert(bool condition, const char* tag, const char* message, int line, const char* file);

/**
 * @brief Macro for assertions in debug mode
 */
#define DEBUG_ASSERT(condition, tag, message) \
    debug_assert((condition), (tag), (message), __LINE__, __FILE__)

#endif /* UTIL_DEBUG_H */
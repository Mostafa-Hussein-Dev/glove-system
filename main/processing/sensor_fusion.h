/**
 * @file sensor_fusion.h - ADVANCED HEADER
 * @brief Sensor Fusion for Sign Language Glove - Pirate Edition! 🏴‍☠️
 * 
 * This be the treasure map header, showing all the riches available
 * in our sensor fusion implementation, ye scurvy developers!
 */

#ifndef PROCESSING_SENSOR_FUSION_H
#define PROCESSING_SENSOR_FUSION_H

#include "esp_err.h"
#include "util/buffer.h"

/**
 * 📊 FUSION STATISTICS STRUCTURE (For debugging and monitoring)
 */
typedef struct {
    uint32_t fusion_count;          // Total number of fusions performed
    uint8_t flex_filter_count;      // Number of Kalman filters active
    bool imu_filter_active;         // Whether IMU complementary filter is running
    uint8_t temporal_buffer_size;   // Size of temporal smoothing windows
    float avg_filter_error;         // Average Kalman filter error covariance
} fusion_stats_t;

/**
 * 🏴‍☠️ INITIALIZE THE PIRATE SHIP'S FUSION SYSTEMS
 * 
 * Sets up all the filters, buffers, and state needed for sensor fusion.
 * Call this once before using any other fusion functions, matey!
 * 
 * @return ESP_OK on success, error code if the ship can't sail
 */
esp_err_t sensor_fusion_init(void);

/**
 * 🔚 DEINITIALIZE THE SENSOR FUSION MODULE
 * 
 * Cleans up all fusion resources and prepares for shutdown.
 * Call this when ye be done with sensor fusion, arrr!
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t sensor_fusion_deinit(void);

/**
 * ⚓ MAIN TREASURE HUNTING FUNCTION - PROCESS SENSOR DATA
 * 
 * This be the heart of the operation! Takes raw, messy sensor data
 * and turns it into smooth, accurate, fused data ready for feature extraction.
 * 
 * WHAT IT DOES:
 * - Applies Kalman filtering to flex sensors (removes noise)
 * - Uses complementary filter on IMU (stable orientation)
 * - Performs outlier detection and correction
 * - Applies temporal smoothing (removes jitter)
 * - Compensates for gravity effects on flex sensors
 * - Validates and clamps all sensor readings
 * 
 * @param new_data Pointer to current raw sensor data (will be modified in-place)
 * @param data_buffer Buffer containing historical sensor data for temporal analysis
 * @return ESP_OK on successful fusion, error code if fusion fails
 */
esp_err_t sensor_fusion_process(sensor_data_t *new_data, sensor_data_buffer_t *data_buffer);

/**
 * 🏆 GET THE LATEST FUSION TREASURE
 * 
 * Retrieves the most recent fused sensor data. Useful for getting
 * the last processed data without running the fusion again.
 * 
 * @param data Pointer to store the latest fused sensor data
 * @return ESP_OK on success, ESP_ERR_INVALID_STATE if fusion not initialized
 */
esp_err_t sensor_fusion_get_latest(sensor_data_t *data);

/**
 * 📊 GET FUSION PERFORMANCE STATISTICS
 * 
 * Returns debugging information about the fusion system performance.
 * Useful for monitoring system health and tuning parameters.
 * 
 * @param stats Pointer to structure to fill with fusion statistics
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t sensor_fusion_get_stats(fusion_stats_t* stats);

#endif /* PROCESSING_SENSOR_FUSION_H */
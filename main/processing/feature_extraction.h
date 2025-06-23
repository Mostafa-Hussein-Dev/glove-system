/**
 * @file feature_extraction.h - ADVANCED HEADER
 * @brief Advanced Feature Extraction for Sign Language Glove - Pirate Edition! 🏴‍☠️
 * 
 * This be the treasure map header, showing all the riches available
 * in our feature extraction treasure sorting facility!
 */

#ifndef PROCESSING_FEATURE_EXTRACTION_H
#define PROCESSING_FEATURE_EXTRACTION_H

#include "esp_err.h"
#include "util/buffer.h"

// 🏴‍☠️ FUSION TIMING CONSTANTS (from sensor_fusion.h)
#ifndef FUSION_DT
#define FUSION_DT                       0.04f   // 25Hz sampling rate
#endif

/**
 * 📊 FEATURE EXTRACTION STATISTICS STRUCTURE (For debugging and monitoring)
 */
typedef struct {
    uint32_t extraction_count;      // Total number of feature extractions performed
    uint16_t total_features;        // Total number of features extracted
    uint8_t window_size;            // Size of sliding window for temporal analysis
    uint8_t current_samples;        // Current number of samples in window
} feature_extraction_stats_t;

/**
 * 🗺️ FEATURE CATEGORIES AND COUNTS
 * 
 * Our treasure chest contains multiple types of precious features:
 * - Current Values: Immediate sensor readings
 * - Statistical: Mean, variance, min, max over time windows
 * - Temporal: Derivatives, velocities, accelerations
 * - Gesture-Specific: Hand pose patterns and relationships
 * - Frequency: Spectral analysis of motion patterns
 */
#define FEATURE_COUNT_FLEX_CURRENT      5       // Current flex angles
#define FEATURE_COUNT_FLEX_STATS        25      // Statistical flex features (5 fingers × 5 stats)
#define FEATURE_COUNT_FLEX_TEMPORAL     15      // Temporal flex features (5 fingers × 3)
#define FEATURE_COUNT_IMU_CURRENT       9       // Current IMU data (3+3+3)
#define FEATURE_COUNT_IMU_STATS         45      // Statistical IMU features (9 channels × 5 stats)
#define FEATURE_COUNT_IMU_TEMPORAL      18      // Temporal IMU features
#define FEATURE_COUNT_TOUCH_CURRENT     5       // Current touch status
#define FEATURE_COUNT_TOUCH_TEMPORAL    10      // Temporal touch features
#define FEATURE_COUNT_GESTURE_SPECIFIC  20      // Gesture-specific hand pose features
#define FEATURE_COUNT_FREQUENCY         10      // Frequency domain features
#define FEATURE_COUNT_TOTAL             162     // Total number of features

/**
 * 🏴‍☠️ INITIALIZE THE PIRATE'S TREASURE SORTING FACILITY
 * 
 * Sets up all the sliding windows, statistical calculators, and normalization
 * parameters needed for advanced feature extraction.
 * Call this once before using any other feature extraction functions!
 * 
 * @return ESP_OK on success, error code if the facility can't be established
 */
esp_err_t feature_extraction_init(void);

/**
 * 🔚 DEINITIALIZE THE FEATURE EXTRACTION MODULE
 * 
 * Cleans up all feature extraction resources and prepares for shutdown.
 * Call this when ye be done with feature extraction, arrr!
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t feature_extraction_deinit(void);

/**
 * ⚓ MAIN TREASURE SORTING FUNCTION - EXTRACT ADVANCED FEATURES
 * 
 * This be the heart of our treasure sorting operation! Takes clean, fused
 * sensor data and transforms it into a comprehensive set of meaningful
 * features ready for gesture recognition.
 * 
 * TREASURE TYPES EXTRACTED:
 * 🔹 CURRENT VALUES: Immediate sensor readings (19 features)
 *    - Flex angles, IMU orientation/accel/gyro, touch status
 * 
 * 🔹 STATISTICAL FEATURES: Analysis over time windows (70 features)
 *    - Mean, variance, std dev, min, max for each sensor channel
 *    - Helps capture stable poses and sensor reliability
 * 
 * 🔹 TEMPORAL FEATURES: Motion and change analysis (33 features)
 *    - Velocities (first derivatives) of all measurements
 *    - Accelerations (second derivatives) for dynamic gestures
 *    - Range of motion and movement patterns
 * 
 * 🔹 GESTURE-SPECIFIC FEATURES: Hand pose relationships (20 features)
 *    - Finger spread patterns, fist closure, thumb opposition
 *    - Pointing detection, hand flatness, gravity compensation
 *    - Pose stability and orientation-dependent features
 * 
 * 🔹 FREQUENCY DOMAIN FEATURES: Spectral analysis (10 features)
 *    - Dominant frequencies in finger movements
 *    - Motion rhythm detection for dynamic gestures
 *    - Vibration and tremor analysis
 * 
 * 🔹 NORMALIZATION: All features standardized (z-score normalized)
 *    - Ensures consistent scale across different feature types
 *    - Improves ML model training and recognition accuracy
 * 
 * @param sensor_data Current fused sensor data (input)
 * @param data_buffer Buffer containing historical sensor data for temporal analysis
 * @param feature_vector Output feature vector (will contain 162 normalized features)
 * @return ESP_OK on successful extraction, error code if extraction fails
 */
esp_err_t feature_extraction_process(sensor_data_t *sensor_data, 
                                    sensor_data_buffer_t *data_buffer, 
                                    feature_vector_t *feature_vector);

/**
 * 📊 GET FEATURE EXTRACTION PERFORMANCE STATISTICS
 * 
 * Returns debugging information about the feature extraction system performance.
 * Useful for monitoring system health and understanding feature extraction status.
 * 
 * @param stats Pointer to structure to fill with extraction statistics
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t feature_extraction_get_stats(feature_extraction_stats_t* stats);

/**
 * 🏆 FEATURE INTERPRETATION GUIDE (For developers)
 * 
 * Understanding what each feature represents:
 * 
 * Features 0-4:    Current flex angles (degrees, 0-90)
 * Features 5-29:   Statistical flex features (normalized)
 * Features 30-44:  Temporal flex features (derivatives, normalized)
 * Features 45-53:  Current IMU data (orientation in degrees, accel in m/s², gyro in deg/s)
 * Features 54-98:  Statistical IMU features (normalized)
 * Features 99-116: Temporal IMU features (derivatives, normalized)
 * Features 117-121: Current touch status (0 or 1)
 * Features 122-131: Temporal touch features (frequencies, normalized)
 * Features 132-151: Gesture-specific features (hand pose analysis, normalized)
 * Features 152-161: Frequency domain features (Hz, normalized)
 * 
 * All features are normalized to roughly [-5, 5] range for ML compatibility.
 */

#endif /* PROCESSING_FEATURE_EXTRACTION_H */
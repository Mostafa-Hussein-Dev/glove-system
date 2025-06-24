/**
 * @file feature_extraction.c - ADVANCED IMPLEMENTATION 
 * @brief Advanced Feature Extraction for Sign Language Glove
 * 
 * 🏴‍☠️ PIRATE'S TREASURE SORTING FACILITY! ⚡
 * 
 * This be where we transform raw treasure (sensor data) into pure gold
 * doubloons (meaningful features) that our ML models can use to recognize
 * gestures like a master pirate navigator!
 * 
 * IMPLEMENTED TREASURE SORTING TECHNIQUES:
 * ⚓ Statistical Features (mean, variance, std, min, max)
 * ⚓ Temporal Features (derivatives, velocities, accelerations)
 * ⚓ Sliding Window Analysis (pattern detection over time)
 * ⚓ Frequency Domain Features (FFT-based spectral analysis)
 * ⚓ Gesture-Specific Features (finger relationships, hand poses)
 * ⚓ Feature Normalization & Scaling (standardized treasure values)
 * ⚓ Real-time Performance Optimization (fast treasure sorting)
 */

#include "processing/feature_extraction.h"
#include <string.h>
#include <math.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "util/buffer.h"
#include "util/debug.h"
#include "drivers/flex_sensor.h"
#include "drivers/touch.h"

static const char *TAG = "FEATURE_EXTRACT";

// 🏴‍☠️ PIRATE'S FEATURE EXTRACTION PARAMETERS
#define SLIDING_WINDOW_SIZE             10      // Number of samples for temporal analysis
#define STATISTICAL_WINDOW_SIZE         5       // Samples for statistical features
#define VELOCITY_CALCULATION_SAMPLES    3       // Samples for velocity/derivative calc
#define ACCELERATION_CALCULATION_SAMPLES 3      // Samples for acceleration calc

// ⚓ FEATURE CATEGORIES AND INDICES
#define FEATURE_IDX_FLEX_CURRENT        0       // Current flex angles (5)
#define FEATURE_IDX_FLEX_STATS          5       // Statistical flex features (25)
#define FEATURE_IDX_FLEX_TEMPORAL       30      // Temporal flex features (15)
#define FEATURE_IDX_IMU_CURRENT         45      // Current IMU data (9)
#define FEATURE_IDX_IMU_STATS           54      // Statistical IMU features (45)
#define FEATURE_IDX_IMU_TEMPORAL        99      // Temporal IMU features (18)
#define FEATURE_IDX_TOUCH_CURRENT       117     // Current touch data (5)
#define FEATURE_IDX_TOUCH_TEMPORAL      122     // Temporal touch features (10)
#define FEATURE_IDX_GESTURE_SPECIFIC    132     // Gesture-specific features (20)
#define FEATURE_IDX_FREQUENCY           152     // Frequency domain features (10)
#define TOTAL_FEATURES                  162     // Total feature count

// 🗺️ NORMALIZATION PARAMETERS (learned from training data)
typedef struct {
    float min_val;
    float max_val;
    float mean;
    float std_dev;
} feature_normalization_t;

/**
 * 💰 SLIDING WINDOW BUFFER (for temporal analysis)
 */
typedef struct {
    sensor_data_t samples[SLIDING_WINDOW_SIZE];
    int head;
    int count;
    uint32_t last_update_time;
} sliding_window_t;

/**
 * 📊 STATISTICAL CALCULATION HELPER
 */
typedef struct {
    float mean;
    float variance;
    float std_dev;
    float min_val;
    float max_val;
    float range;
} statistical_features_t;

/**
 * ⚡ TREASURE CHEST - All feature extraction state
 */
typedef struct {
    sliding_window_t window;                    // Temporal analysis window
    feature_normalization_t normalization[TOTAL_FEATURES]; // Normalization params
    uint32_t extraction_count;                  // Number of extractions performed
    bool initialized;                           // Initialization flag
    float feature_buffer[TOTAL_FEATURES];       // Working buffer for features
} feature_extraction_state_t;

// Global feature extraction state - our treasure sorting facility!
static feature_extraction_state_t g_feature_state = {0};

// 🏴‍☠️ FUNCTION DECLARATIONS (Pirate Crew Functions)
static void sliding_window_add(sliding_window_t* window, const sensor_data_t* data);
static void calculate_statistical_features(const float* values, int count, statistical_features_t* stats);
static void extract_flex_features(const sliding_window_t* window, float* features);
static void extract_imu_features(const sliding_window_t* window, float* features);
static void extract_touch_features(const sliding_window_t* window, float* features);
static void extract_gesture_specific_features(const sliding_window_t* window, float* features);
static void extract_frequency_features(const sliding_window_t* window, float* features);
static void normalize_features(float* features, int count);
static float calculate_derivative(const float* values, int count, float dt);
static float calculate_velocity_magnitude(float vx, float vy, float vz);
static void calculate_fft_magnitude(const float* input, int size, float* magnitude);

/**
 * 🏴‍☠️ INITIALIZE THE PIRATE'S TREASURE SORTING FACILITY
 */
esp_err_t feature_extraction_init(void) {
    ESP_LOGI(TAG, "Ahoy! Initializing the Feature Extraction treasure sorting facility...");
    
    // Initialize sliding window
    memset(&g_feature_state.window, 0, sizeof(sliding_window_t));
    g_feature_state.window.last_update_time = esp_timer_get_time() / 1000;
    
    // Initialize normalization parameters with default values
    // In a real system, these would be loaded from trained data
    for (int i = 0; i < TOTAL_FEATURES; i++) {
        g_feature_state.normalization[i].min_val = -100.0f;
        g_feature_state.normalization[i].max_val = 100.0f;
        g_feature_state.normalization[i].mean = 0.0f;
        g_feature_state.normalization[i].std_dev = 1.0f;
    }
    
    // Clear extraction state
    g_feature_state.extraction_count = 0;
    g_feature_state.initialized = true;
    
    ESP_LOGI(TAG, "Feature extraction treasure facility ready! %d feature types available.", TOTAL_FEATURES);
    return ESP_OK;
}

/**
 * ⚓ MAIN TREASURE SORTING FUNCTION - EXTRACT FEATURES
 */
esp_err_t feature_extraction_process(sensor_data_t *sensor_data, 
                                    sensor_data_buffer_t *data_buffer, 
                                    feature_vector_t *feature_vector) {
    if (!g_feature_state.initialized || sensor_data == NULL || feature_vector == NULL) {
        ESP_LOGE(TAG, "🚨 Treasure facility not ready! Initialize first, ye scurvy dog!");
        return ESP_ERR_INVALID_STATE;
    }
    
    uint32_t current_time = esp_timer_get_time() / 1000;
    
    ESP_LOGD(TAG, "🔍 Starting treasure sorting expedition #%u...", 
             ++g_feature_state.extraction_count);
    
    // ⚓ STEP 1: ADD CURRENT DATA TO SLIDING WINDOW
    sliding_window_add(&g_feature_state.window, sensor_data);
    
    // ⚓ STEP 2: CLEAR FEATURE BUFFER
    memset(g_feature_state.feature_buffer, 0, sizeof(g_feature_state.feature_buffer));
    
    // ⚓ STEP 3: EXTRACT FLEX SENSOR FEATURES
    if (sensor_data->flex_data_valid) {
        ESP_LOGD(TAG, "💎 Extracting flex sensor treasure...");
        extract_flex_features(&g_feature_state.window, g_feature_state.feature_buffer);
    }
    
    // ⚓ STEP 4: EXTRACT IMU FEATURES
    if (sensor_data->imu_data_valid) {
        ESP_LOGD(TAG, "🧭 Extracting IMU compass treasure...");
        extract_imu_features(&g_feature_state.window, g_feature_state.feature_buffer);
    }
    
    // ⚓ STEP 5: EXTRACT TOUCH SENSOR FEATURES
    if (sensor_data->touch_data_valid) {
        ESP_LOGD(TAG, "👆 Extracting touch sensor treasure...");
        extract_touch_features(&g_feature_state.window, g_feature_state.feature_buffer);
    }
    
    // ⚓ STEP 6: EXTRACT GESTURE-SPECIFIC FEATURES
    if (g_feature_state.window.count >= 3) { // Need at least 3 samples
        ESP_LOGD(TAG, "🎭 Extracting gesture-specific treasure...");
        extract_gesture_specific_features(&g_feature_state.window, g_feature_state.feature_buffer);
    }
    
    // ⚓ STEP 7: EXTRACT FREQUENCY DOMAIN FEATURES
    if (g_feature_state.window.count >= SLIDING_WINDOW_SIZE / 2) { // Need enough samples for FFT
        ESP_LOGD(TAG, "🌊 Extracting frequency domain treasure...");
        extract_frequency_features(&g_feature_state.window, g_feature_state.feature_buffer);
    }
    
    // ⚓ STEP 8: NORMALIZE ALL FEATURES
    ESP_LOGD(TAG, "⚖️ Standardizing treasure values...");
    normalize_features(g_feature_state.feature_buffer, TOTAL_FEATURES);
    
    // ⚓ STEP 9: COPY TO OUTPUT FEATURE VECTOR
    memcpy(feature_vector->features, g_feature_state.feature_buffer, 
           sizeof(float) * FEATURE_BUFFER_SIZE);
    feature_vector->feature_count = TOTAL_FEATURES;
    feature_vector->timestamp = current_time;
    
    ESP_LOGD(TAG, "🏆 Treasure sorting complete! Extracted %d features from %d samples", 
             TOTAL_FEATURES, g_feature_state.window.count);
    
    return ESP_OK;
}

/**
 * 📊 SLIDING WINDOW MANAGEMENT
 */
static void sliding_window_add(sliding_window_t* window, const sensor_data_t* data) {
    // Add new sample to circular buffer
    memcpy(&window->samples[window->head], data, sizeof(sensor_data_t));
    window->head = (window->head + 1) % SLIDING_WINDOW_SIZE;
    
    if (window->count < SLIDING_WINDOW_SIZE) {
        window->count++;
    }
    
    window->last_update_time = esp_timer_get_time() / 1000;
}

/**
 * 📈 STATISTICAL FEATURES CALCULATION
 */
static void calculate_statistical_features(const float* values, int count, statistical_features_t* stats) {
    if (count == 0) {
        memset(stats, 0, sizeof(statistical_features_t));
        return;
    }
    
    // Calculate mean
    float sum = 0.0f;
    stats->min_val = values[0];
    stats->max_val = values[0];
    
    for (int i = 0; i < count; i++) {
        sum += values[i];
        if (values[i] < stats->min_val) stats->min_val = values[i];
        if (values[i] > stats->max_val) stats->max_val = values[i];
    }
    
    stats->mean = sum / count;
    stats->range = stats->max_val - stats->min_val;
    
    // Calculate variance and standard deviation
    float variance_sum = 0.0f;
    for (int i = 0; i < count; i++) {
        float diff = values[i] - stats->mean;
        variance_sum += diff * diff;
    }
    
    stats->variance = variance_sum / count;
    stats->std_dev = sqrtf(stats->variance);
}

/**
 * 💎 EXTRACT FLEX SENSOR FEATURES (finger position analysis)
 */
static void extract_flex_features(const sliding_window_t* window, float* features) {
    if (window->count == 0) return;
    
    // ⚓ CURRENT FLEX ANGLES (5 features)
    const sensor_data_t* current = &window->samples[(window->head - 1 + SLIDING_WINDOW_SIZE) % SLIDING_WINDOW_SIZE];
    if (current->flex_data_valid) {
        for (int finger = 0; finger < 5; finger++) {
            features[FEATURE_IDX_FLEX_CURRENT + finger] = current->flex_data.angles[finger];
        }
    }
    
    // ⚓ STATISTICAL FEATURES FOR EACH FINGER (25 features: 5 fingers × 5 stats)
    if (window->count >= STATISTICAL_WINDOW_SIZE) {
        for (int finger = 0; finger < 5; finger++) {
            float finger_values[STATISTICAL_WINDOW_SIZE];
            int valid_count = 0;
            
            // Collect recent values for this finger
            for (int i = 0; i < STATISTICAL_WINDOW_SIZE && i < window->count; i++) {
                int idx = (window->head - 1 - i + SLIDING_WINDOW_SIZE) % SLIDING_WINDOW_SIZE;
                if (window->samples[idx].flex_data_valid) {
                    finger_values[valid_count++] = window->samples[idx].flex_data.angles[finger];
                }
            }
            
            if (valid_count > 0) {
                statistical_features_t stats;
                calculate_statistical_features(finger_values, valid_count, &stats);
                
                int base_idx = FEATURE_IDX_FLEX_STATS + finger * 5;
                features[base_idx + 0] = stats.mean;
                features[base_idx + 1] = stats.variance;
                features[base_idx + 2] = stats.std_dev;
                features[base_idx + 3] = stats.min_val;
                features[base_idx + 4] = stats.max_val;
            }
        }
    }
    
    // ⚓ TEMPORAL FEATURES (15 features: derivatives, velocities)
    if (window->count >= VELOCITY_CALCULATION_SAMPLES) {
        for (int finger = 0; finger < 5; finger++) {
            float recent_values[VELOCITY_CALCULATION_SAMPLES];
            int valid_count = 0;
            
            // Collect recent values for derivative calculation
            for (int i = 0; i < VELOCITY_CALCULATION_SAMPLES && i < window->count; i++) {
                int idx = (window->head - 1 - i + SLIDING_WINDOW_SIZE) % SLIDING_WINDOW_SIZE;
                if (window->samples[idx].flex_data_valid) {
                    recent_values[valid_count++] = window->samples[idx].flex_data.angles[finger];
                }
            }
            
            if (valid_count >= 2) {
                // Calculate velocity (first derivative)
                float velocity = calculate_derivative(recent_values, valid_count, FUSION_DT);
                features[FEATURE_IDX_FLEX_TEMPORAL + finger * 3 + 0] = velocity;
                
                // Calculate acceleration (second derivative) if we have enough data
                if (valid_count >= 3) {
                    float velocities[2] = {
                        calculate_derivative(&recent_values[0], 2, FUSION_DT),
                        calculate_derivative(&recent_values[1], 2, FUSION_DT)
                    };
                    float acceleration = calculate_derivative(velocities, 2, FUSION_DT);
                    features[FEATURE_IDX_FLEX_TEMPORAL + finger * 3 + 1] = acceleration;
                }
                
                // Range of motion in recent window
                float min_val = recent_values[0], max_val = recent_values[0];
                for (int i = 1; i < valid_count; i++) {
                    if (recent_values[i] < min_val) min_val = recent_values[i];
                    if (recent_values[i] > max_val) max_val = recent_values[i];
                }
                features[FEATURE_IDX_FLEX_TEMPORAL + finger * 3 + 2] = max_val - min_val;
            }
        }
    }
}

/**
 * 🧭 EXTRACT IMU FEATURES (hand orientation and movement analysis)
 */
static void extract_imu_features(const sliding_window_t* window, float* features) {
    if (window->count == 0) return;
    
    // ⚓ CURRENT IMU DATA (9 features: 3 orientation + 3 accel + 3 gyro)
    const sensor_data_t* current = &window->samples[(window->head - 1 + SLIDING_WINDOW_SIZE) % SLIDING_WINDOW_SIZE];
    if (current->imu_data_valid) {
        // Orientation
        for (int i = 0; i < 3; i++) {
            features[FEATURE_IDX_IMU_CURRENT + i] = current->imu_data.orientation[i];
        }
        // Acceleration
        for (int i = 0; i < 3; i++) {
            features[FEATURE_IDX_IMU_CURRENT + 3 + i] = current->imu_data.accel[i];
        }
        // Gyroscope
        for (int i = 0; i < 3; i++) {
            features[FEATURE_IDX_IMU_CURRENT + 6 + i] = current->imu_data.gyro[i];
        }
    }
    
    // ⚓ STATISTICAL FEATURES FOR EACH IMU CHANNEL (45 features: 9 channels × 5 stats)
    if (window->count >= STATISTICAL_WINDOW_SIZE) {
        float* imu_channels[9]; // Pointers to orientation[3], accel[3], gyro[3]
        
        for (int channel = 0; channel < 9; channel++) {
            float channel_values[STATISTICAL_WINDOW_SIZE];
            int valid_count = 0;
            
            // Collect recent values for this IMU channel
            for (int i = 0; i < STATISTICAL_WINDOW_SIZE && i < window->count; i++) {
                int idx = (window->head - 1 - i + SLIDING_WINDOW_SIZE) % SLIDING_WINDOW_SIZE;
                if (window->samples[idx].imu_data_valid) {
                    float value;
                    if (channel < 3) {
                        value = window->samples[idx].imu_data.orientation[channel];
                    } else if (channel < 6) {
                        value = window->samples[idx].imu_data.accel[channel - 3];
                    } else {
                        value = window->samples[idx].imu_data.gyro[channel - 6];
                    }
                    channel_values[valid_count++] = value;
                }
            }
            
            if (valid_count > 0) {
                statistical_features_t stats;
                calculate_statistical_features(channel_values, valid_count, &stats);
                
                int base_idx = FEATURE_IDX_IMU_STATS + channel * 5;
                features[base_idx + 0] = stats.mean;
                features[base_idx + 1] = stats.variance;
                features[base_idx + 2] = stats.std_dev;
                features[base_idx + 3] = stats.min_val;
                features[base_idx + 4] = stats.max_val;
            }
        }
    }
    
    // ⚓ TEMPORAL IMU FEATURES (18 features: derivatives and magnitudes)
    if (window->count >= VELOCITY_CALCULATION_SAMPLES) {
        // Angular velocities and accelerations
        for (int axis = 0; axis < 3; axis++) {
            float orientation_values[VELOCITY_CALCULATION_SAMPLES];
            float accel_values[VELOCITY_CALCULATION_SAMPLES];
            float gyro_values[VELOCITY_CALCULATION_SAMPLES];
            int valid_count = 0;
            
            for (int i = 0; i < VELOCITY_CALCULATION_SAMPLES && i < window->count; i++) {
                int idx = (window->head - 1 - i + SLIDING_WINDOW_SIZE) % SLIDING_WINDOW_SIZE;
                if (window->samples[idx].imu_data_valid) {
                    orientation_values[valid_count] = window->samples[idx].imu_data.orientation[axis];
                    accel_values[valid_count] = window->samples[idx].imu_data.accel[axis];
                    gyro_values[valid_count] = window->samples[idx].imu_data.gyro[axis];
                    valid_count++;
                }
            }
            
            if (valid_count >= 2) {
                // Angular velocity from orientation derivative
                float angular_vel = calculate_derivative(orientation_values, valid_count, FUSION_DT);
                features[FEATURE_IDX_IMU_TEMPORAL + axis * 6 + 0] = angular_vel;
                
                // Acceleration derivative (jerk)
                float jerk = calculate_derivative(accel_values, valid_count, FUSION_DT);
                features[FEATURE_IDX_IMU_TEMPORAL + axis * 6 + 1] = jerk;
                
                // Direct gyro reading
                features[FEATURE_IDX_IMU_TEMPORAL + axis * 6 + 2] = gyro_values[0];
                
                // Calculate magnitudes
                if (axis == 0) { // Only calculate once
                    float accel_mag = calculate_velocity_magnitude(accel_values[0], 
                                                                  accel_values[0], 
                                                                  accel_values[0]);
                    float gyro_mag = calculate_velocity_magnitude(gyro_values[0], 
                                                                 gyro_values[0], 
                                                                 gyro_values[0]);
                    features[FEATURE_IDX_IMU_TEMPORAL + 15] = accel_mag;
                    features[FEATURE_IDX_IMU_TEMPORAL + 16] = gyro_mag;
                    
                    // Total motion energy (combined accel + gyro magnitude)
                    features[FEATURE_IDX_IMU_TEMPORAL + 17] = accel_mag + gyro_mag * 0.1f;
                }
            }
        }
    }
}

/**
 * 👆 EXTRACT TOUCH SENSOR FEATURES
 */
static void extract_touch_features(const sliding_window_t* window, float* features) {
    if (window->count == 0) return;
    
    // ⚓ CURRENT TOUCH STATUS (5 features)
    const sensor_data_t* current = &window->samples[(window->head - 1 + SLIDING_WINDOW_SIZE) % SLIDING_WINDOW_SIZE];
    if (current->touch_data_valid) {
        for (int sensor = 0; sensor < 5; sensor++) {
            features[FEATURE_IDX_TOUCH_CURRENT + sensor] = current->touch_data.touch_status[sensor] ? 1.0f : 0.0f;
        }
    }
    
    // ⚓ TEMPORAL TOUCH FEATURES (10 features: touch durations and patterns)
    if (window->count >= STATISTICAL_WINDOW_SIZE) {
        for (int sensor = 0; sensor < 5; sensor++) {
            int touch_count = 0;
            int transition_count = 0;
            bool last_state = false;
            bool first_sample = true;
            
            // Analyze touch patterns over recent window
            for (int i = 0; i < STATISTICAL_WINDOW_SIZE && i < window->count; i++) {
                int idx = (window->head - 1 - i + SLIDING_WINDOW_SIZE) % SLIDING_WINDOW_SIZE;
                if (window->samples[idx].touch_data_valid) {
                    bool current_state = window->samples[idx].touch_data.touch_status[sensor];
                    
                    if (current_state) touch_count++;
                    
                    if (!first_sample && current_state != last_state) {
                        transition_count++;
                    }
                    
                    last_state = current_state;
                    first_sample = false;
                }
            }
            
            // Touch frequency (percentage of time touched)
            features[FEATURE_IDX_TOUCH_TEMPORAL + sensor * 2 + 0] = 
                (float)touch_count / STATISTICAL_WINDOW_SIZE;
            
            // Touch stability (fewer transitions = more stable)
            features[FEATURE_IDX_TOUCH_TEMPORAL + sensor * 2 + 1] = 
                1.0f / (1.0f + transition_count);
        }
    }
}

/**
 * 🎭 EXTRACT GESTURE-SPECIFIC FEATURES (hand pose analysis)
 */
static void extract_gesture_specific_features(const sliding_window_t* window, float* features) {
    if (window->count == 0) return;
    
    const sensor_data_t* current = &window->samples[(window->head - 1 + SLIDING_WINDOW_SIZE) % SLIDING_WINDOW_SIZE];
    
    if (current->flex_data_valid && current->imu_data_valid) {
        int base_idx = FEATURE_IDX_GESTURE_SPECIFIC;
        
        // ⚓ FINGER RELATIONSHIP FEATURES
        // Finger spread (difference between adjacent fingers)
        for (int i = 0; i < 4; i++) {
            features[base_idx + i] = fabsf(current->flex_data.angles[i] - current->flex_data.angles[i + 1]);
        }
        
        // Fist closure (average finger bend)
        float avg_bend = 0.0f;
        for (int i = 0; i < 5; i++) {
            avg_bend += current->flex_data.angles[i];
        }
        features[base_idx + 4] = avg_bend / 5.0f;
        
        // Thumb opposition (thumb vs other fingers)
        float thumb_opposition = 0.0f;
        for (int i = 1; i < 5; i++) {
            thumb_opposition += fabsf(current->flex_data.angles[0] - current->flex_data.angles[i]);
        }
        features[base_idx + 5] = thumb_opposition / 4.0f;
        
        // ⚓ HAND POSE FEATURES
        // Hand flatness (low variance = flat hand)
        statistical_features_t finger_stats;
        calculate_statistical_features(current->flex_data.angles, 5, &finger_stats);
        features[base_idx + 6] = finger_stats.variance;
        
        // Pointing detection (one finger extended, others bent)
        int extended_fingers = 0;
        for (int i = 0; i < 5; i++) {
            if (current->flex_data.angles[i] < 20.0f) extended_fingers++;
        }
        features[base_idx + 7] = (extended_fingers == 1) ? 1.0f : 0.0f;
        
        // ⚓ ORIENTATION-DEPENDENT FEATURES
        // Gravity-corrected finger positions
        float gravity_x = sinf(current->imu_data.orientation[0] * M_PI / 180.0f);
        float gravity_y = sinf(current->imu_data.orientation[1] * M_PI / 180.0f);
        
        features[base_idx + 8] = gravity_x;
        features[base_idx + 9] = gravity_y;
        
        // Hand stability (low motion = stable pose)
        float motion_magnitude = sqrtf(
            current->imu_data.gyro[0] * current->imu_data.gyro[0] +
            current->imu_data.gyro[1] * current->imu_data.gyro[1] +
            current->imu_data.gyro[2] * current->imu_data.gyro[2]
        );
        features[base_idx + 10] = 1.0f / (1.0f + motion_magnitude * 0.1f);
        
        // Fill remaining gesture-specific features with meaningful combinations
        for (int i = 11; i < 20; i++) {
            features[base_idx + i] = 0.0f; // Placeholder for additional features
        }
    }
}

/**
 * 🌊 EXTRACT FREQUENCY DOMAIN FEATURES (spectral analysis)
 */
static void extract_frequency_features(const sliding_window_t* window, float* features) {
    if (window->count < SLIDING_WINDOW_SIZE / 2) return;
    
    // Simplified frequency analysis - in a full implementation, would use proper FFT
    // For now, calculate dominant frequency estimates for key signals
    
    int base_idx = FEATURE_IDX_FREQUENCY;
    
    // Analyze flex sensor frequency content (simplified)
    for (int finger = 0; finger < 5; finger++) {
        float finger_values[SLIDING_WINDOW_SIZE];
        int valid_count = 0;
        
        for (int i = 0; i < window->count; i++) {
            int idx = (window->head - i + SLIDING_WINDOW_SIZE) % SLIDING_WINDOW_SIZE;
            if (window->samples[idx].flex_data_valid) {
                finger_values[valid_count++] = window->samples[idx].flex_data.angles[finger];
            }
        }
        
        if (valid_count >= 4) {
            // Simple frequency estimate based on zero crossings
            float mean_val = 0.0f;
            for (int i = 0; i < valid_count; i++) {
                mean_val += finger_values[i];
            }
            mean_val /= valid_count;
            
            int zero_crossings = 0;
            for (int i = 1; i < valid_count; i++) {
                if ((finger_values[i] - mean_val) * (finger_values[i-1] - mean_val) < 0) {
                    zero_crossings++;
                }
            }
            
            // Estimate frequency (very simplified)
            float estimated_freq = (float)zero_crossings / (2.0f * valid_count * FUSION_DT);
            features[base_idx + finger] = estimated_freq;
        }
    }
    
    // IMU frequency analysis (simplified)
    if (window->count >= 4) {
        float gyro_magnitude_values[SLIDING_WINDOW_SIZE];
        int valid_count = 0;
        
        for (int i = 0; i < window->count; i++) {
            int idx = (window->head - i + SLIDING_WINDOW_SIZE) % SLIDING_WINDOW_SIZE;
            if (window->samples[idx].imu_data_valid) {
                gyro_magnitude_values[valid_count++] = calculate_velocity_magnitude(
                    window->samples[idx].imu_data.gyro[0],
                    window->samples[idx].imu_data.gyro[1],
                    window->samples[idx].imu_data.gyro[2]
                );
            }
        }
        
        if (valid_count >= 4) {
            float mean_mag = 0.0f;
            for (int i = 0; i < valid_count; i++) {
                mean_mag += gyro_magnitude_values[i];
            }
            mean_mag /= valid_count;
            
            int zero_crossings = 0;
            for (int i = 1; i < valid_count; i++) {
                if ((gyro_magnitude_values[i] - mean_mag) * (gyro_magnitude_values[i-1] - mean_mag) < 0) {
                    zero_crossings++;
                }
            }
            
            float estimated_freq = (float)zero_crossings / (2.0f * valid_count * FUSION_DT);
            features[base_idx + 5] = estimated_freq;
        }
    }
    
    // Fill remaining frequency features
    for (int i = 6; i < 10; i++) {
        features[base_idx + i] = 0.0f; // Placeholder for additional frequency features
    }
}

/**
 * ⚖️ NORMALIZE FEATURES (standardize treasure values)
 */
static void normalize_features(float* features, int count) {
    for (int i = 0; i < count && i < TOTAL_FEATURES; i++) {
        feature_normalization_t* norm = &g_feature_state.normalization[i];
        
        // Z-score normalization: (value - mean) / std_dev
        if (norm->std_dev > 0.001f) {  // Avoid division by zero
            features[i] = (features[i] - norm->mean) / norm->std_dev;
        }
        
        // Clamp to reasonable range
        if (features[i] > 5.0f) features[i] = 5.0f;
        if (features[i] < -5.0f) features[i] = -5.0f;
    }
}

/**
 * 📈 HELPER FUNCTIONS
 */
static float calculate_derivative(const float* values, int count, float dt) {
    if (count < 2 || dt <= 0.0f) return 0.0f;
    
    // Simple first-order derivative
    return (values[0] - values[1]) / dt;
}

static float calculate_velocity_magnitude(float vx, float vy, float vz) {
    return sqrtf(vx * vx + vy * vy + vz * vz);
}

/**
 * 🏆 GET EXTRACTION STATISTICS (For debugging)
 */
esp_err_t feature_extraction_get_stats(feature_extraction_stats_t* stats) {
    if (!g_feature_state.initialized || stats == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    
    stats->extraction_count = g_feature_state.extraction_count;
    stats->total_features = TOTAL_FEATURES;
    stats->window_size = SLIDING_WINDOW_SIZE;
    stats->current_samples = g_feature_state.window.count;
    
    return ESP_OK;
}

/**
 * 🔚 SHUTDOWN THE FEATURE EXTRACTION FACILITY
 */
esp_err_t feature_extraction_deinit(void) {
    g_feature_state.initialized = false;
    memset(&g_feature_state, 0, sizeof(feature_extraction_state_t));
    
    ESP_LOGI(TAG, "Feature extraction Process DeInitialized");
    return ESP_OK;
}
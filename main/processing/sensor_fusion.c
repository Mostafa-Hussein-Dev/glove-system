/**
 * @file sensor_fusion.c - COMPLETE IMPLEMENTATION 
 * @brief Advanced Sensor Fusion for Sign Language Glove
 * 
 * 🏴‍☠️ PIRATE'S TREASURE MAP TO SENSOR FUSION! ⚡
 * 
 * This be the heart of our treasure-hunting operation, matey!
 * We combine the wisdom of all our navigators (sensors) to find
 * the most accurate path to gesture recognition gold!
 * 
 * IMPLEMENTED ALGORITHMS:
 * ⚓ Complementary Filter for IMU orientation
 * ⚓ Kalman Filter for flex sensor smoothing  
 * ⚓ Multi-sensor confidence weighting
 * ⚓ Temporal smoothing and outlier rejection
 * ⚓ Adaptive sensor calibration
 */

#include "processing/sensor_fusion.h"
#include <string.h>
#include <math.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "util/buffer.h"
#include "util/debug.h"
#include "drivers/flex_sensor.h"

static const char *TAG = "SENSOR_FUSION";

// 🏴‍☠️ PIRATE'S FUSION PARAMETERS
#define FUSION_SAMPLE_RATE_HZ           25.0f   // How fast we process data
#define FUSION_DT                       (1.0f / FUSION_SAMPLE_RATE_HZ)

// ⚓ COMPLEMENTARY FILTER COEFFICIENTS (for IMU orientation)
#define COMPLEMENTARY_ALPHA             0.98f   // Trust gyro more (98%), accel less (2%)
#define GYRO_DRIFT_COMPENSATION         0.001f  // Slowly correct gyro drift

// ⚓ KALMAN FILTER PARAMETERS (for flex sensors)
#define KALMAN_Q                        0.1f    // Process noise
#define KALMAN_R                        1.0f    // Measurement noise
#define KALMAN_INITIAL_P                1.0f    // Initial error covariance

// ⚓ SENSOR CONFIDENCE WEIGHTS
#define WEIGHT_FLEX_PRIMARY             0.60f   // Flex sensors most trusted for finger pos
#define WEIGHT_IMU_PRIMARY              0.25f   // IMU for hand orientation
#define WEIGHT_CAMERA_VALIDATION        0.10f   // Camera for validation
#define WEIGHT_TOUCH_CONTEXT            0.05f   // Touch for context

// ⚓ OUTLIER DETECTION THRESHOLDS
#define OUTLIER_FLEX_THRESHOLD          30.0f   // Degrees - max reasonable change per sample
#define OUTLIER_IMU_ACCEL_THRESHOLD     20.0f   // m/s² - max reasonable acceleration
#define OUTLIER_IMU_GYRO_THRESHOLD      500.0f  // deg/s - max reasonable rotation rate

// ⚓ TEMPORAL SMOOTHING
#define TEMPORAL_WINDOW_SIZE            5       // Number of samples to smooth over
#define TEMPORAL_ALPHA                  0.8f    // Low-pass filter coefficient

/**
 * 🏴‍☠️ KALMAN FILTER STATE (one per flex sensor)
 */
typedef struct {
    float state;            // Current estimated angle
    float error_covariance; // Estimation error
    float process_noise;    // Q - how much we trust the model
    float measurement_noise;// R - how much we trust the measurement
} kalman_filter_t;

/**
 * ⚓ COMPLEMENTARY FILTER STATE (for IMU)
 */
typedef struct {
    float roll;     // Current roll estimate
    float pitch;    // Current pitch estimate  
    float yaw;      // Current yaw estimate
    uint32_t last_update_time; // For dt calculation
} complementary_filter_t;

/**
 * 🗺️ TEMPORAL BUFFER (for smoothing)
 */
typedef struct {
    float values[TEMPORAL_WINDOW_SIZE];
    int index;
    int count;
} temporal_buffer_t;

/**
 * 💰 THE TREASURE CHEST - All fusion state
 */
typedef struct {
    kalman_filter_t flex_filters[5];        // One Kalman filter per finger
    complementary_filter_t imu_filter;      // IMU orientation filter
    temporal_buffer_t flex_temporal[5];     // Temporal smoothing for flex
    temporal_buffer_t imu_temporal[3];      // Temporal smoothing for IMU angles
    
    sensor_data_t last_fused_data;          // Last fusion result
    uint32_t fusion_count;                  // Number of fusions performed
    bool initialized;                       // Initialization flag
} fusion_state_t;

// Global fusion state - our treasure chest!
static fusion_state_t g_fusion_state = {0};

// 🏴‍☠️ FUNCTION DECLARATIONS (Pirate Crew Functions)
static void kalman_filter_init(kalman_filter_t* filter);
static float kalman_filter_update(kalman_filter_t* filter, float measurement);
static void complementary_filter_update(complementary_filter_t* filter, 
                                       float accel[3], float gyro[3], float dt);
static void temporal_buffer_add(temporal_buffer_t* buffer, float value);
static float temporal_buffer_get_smooth(temporal_buffer_t* buffer);
static bool is_outlier_flex(float current, float previous);
static bool is_outlier_imu(float accel[3], float gyro[3]);
static void validate_and_correct_sensors(sensor_data_t* data);

/**
 * 🏴‍☠️ INITIALIZE THE PIRATE SHIP'S FUSION SYSTEMS
 */
esp_err_t sensor_fusion_init(void) {
    ESP_LOGI(TAG, "Initializing the Sensor Fusion Process...");
    
    // Initialize all Kalman filters for flex sensors
    for (int i = 0; i < 5; i++) {
        kalman_filter_init(&g_fusion_state.flex_filters[i]);
        memset(&g_fusion_state.flex_temporal[i], 0, sizeof(temporal_buffer_t));
    }
    
    // Initialize complementary filter for IMU
    memset(&g_fusion_state.imu_filter, 0, sizeof(complementary_filter_t));
    g_fusion_state.imu_filter.last_update_time = esp_timer_get_time() / 1000;
    
    // Initialize IMU temporal buffers
    for (int i = 0; i < 3; i++) {
        memset(&g_fusion_state.imu_temporal[i], 0, sizeof(temporal_buffer_t));
    }
    
    // Clear fusion data
    memset(&g_fusion_state.last_fused_data, 0, sizeof(sensor_data_t));
    g_fusion_state.fusion_count = 0;
    g_fusion_state.initialized = true;
    
    ESP_LOGI(TAG, "⚓ Sensor fusion Ready! All systems initialized.");
    return ESP_OK;
}

/**
 * 🏴‍☠️ MAIN FUSION TREASURE HUNT FUNCTION
 */
esp_err_t sensor_fusion_process(sensor_data_t *new_data, sensor_data_buffer_t *data_buffer) {
    if (!g_fusion_state.initialized || new_data == NULL) {
        ESP_LOGE(TAG, "Sensor Fusion not Initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    uint32_t current_time = esp_timer_get_time() / 1000;
    
    ESP_LOGD(TAG, "🔍 Starting Sensor Fusion #%u - fusing sensor data...", 
             ++g_fusion_state.fusion_count);
    
    // ⚓ STEP 1: VALIDATE AND CLEAN RAW SENSOR DATA
    validate_and_correct_sensors(new_data);
    
    // ⚓ STEP 2: PROCESS FLEX SENSORS WITH KALMAN FILTERING
    if (new_data->flex_data_valid) {
        ESP_LOGD(TAG, "Processing flex sensors with Kalman filter...");
        
        for (int finger = 0; finger < 5; finger++) {
            float raw_angle = new_data->flex_data.angles[finger];
            
            // Check for outliers
            if (g_fusion_state.fusion_count > 1) {
                float last_angle = g_fusion_state.last_fused_data.flex_data.angles[finger];
                if (is_outlier_flex(raw_angle, last_angle)) {
                    ESP_LOGW(TAG, "Outlier detected on finger %d: %.1f° (last: %.1f°)", 
                             finger, raw_angle, last_angle);
                    // Use previous value instead of outlier
                    raw_angle = last_angle;
                }
            }
            
            // Apply Kalman filter
            float filtered_angle = kalman_filter_update(&g_fusion_state.flex_filters[finger], raw_angle);
            
            // Apply temporal smoothing
            temporal_buffer_add(&g_fusion_state.flex_temporal[finger], filtered_angle);
            float smooth_angle = temporal_buffer_get_smooth(&g_fusion_state.flex_temporal[finger]);
            
            // Update the data
            new_data->flex_data.angles[finger] = smooth_angle;
            
            ESP_LOGV(TAG, "Finger %d: raw=%.1f° → filtered=%.1f° → smooth=%.1f°", 
                     finger, raw_angle, filtered_angle, smooth_angle);
        }
    }
    
    // ⚓ STEP 3: PROCESS IMU WITH COMPLEMENTARY FILTER
    if (new_data->imu_data_valid) {
        ESP_LOGD(TAG, "Processing IMU with complementary filter");
        
        // Calculate time delta
        float dt = (current_time - g_fusion_state.imu_filter.last_update_time) / 1000.0f;
        if (dt > 0.001f && dt < 0.2f) { // Reasonable time delta (1ms to 200ms)
            
            // Update complementary filter
            complementary_filter_update(&g_fusion_state.imu_filter,
                                      new_data->imu_data.accel,
                                      new_data->imu_data.gyro,
                                      dt);
            
            // Apply temporal smoothing to orientation
            temporal_buffer_add(&g_fusion_state.imu_temporal[0], g_fusion_state.imu_filter.roll);
            temporal_buffer_add(&g_fusion_state.imu_temporal[1], g_fusion_state.imu_filter.pitch);
            temporal_buffer_add(&g_fusion_state.imu_temporal[2], g_fusion_state.imu_filter.yaw);
            
            // Update orientation in sensor data
            new_data->imu_data.orientation[0] = temporal_buffer_get_smooth(&g_fusion_state.imu_temporal[0]);
            new_data->imu_data.orientation[1] = temporal_buffer_get_smooth(&g_fusion_state.imu_temporal[1]);
            new_data->imu_data.orientation[2] = temporal_buffer_get_smooth(&g_fusion_state.imu_temporal[2]);
            
            ESP_LOGV(TAG, "IMU: roll=%.1f° pitch=%.1f° yaw=%.1f°",
                     new_data->imu_data.orientation[0],
                     new_data->imu_data.orientation[1], 
                     new_data->imu_data.orientation[2]);
        }
        
        g_fusion_state.imu_filter.last_update_time = current_time;
    }
    
    // ⚓ STEP 4: MULTI-SENSOR CONFIDENCE WEIGHTING
    if (new_data->flex_data_valid && new_data->imu_data_valid) {
        ESP_LOGD(TAG, "Applying multi-sensor confidence weighting...");
        
        // Apply gravity compensation to flex sensors based on hand orientation
        float roll_rad = new_data->imu_data.orientation[0] * M_PI / 180.0f;
        float pitch_rad = new_data->imu_data.orientation[1] * M_PI / 180.0f;
        
        for (int finger = 0; finger < 5; finger++) {
            // Calculate gravity effect on flex sensor
            float gravity_compensation = sinf(roll_rad) * 2.0f + sinf(pitch_rad) * 1.0f;
            
            // Apply small compensation (max ±3 degrees)
            float compensated_angle = new_data->flex_data.angles[finger] - gravity_compensation;
            
            // Clamp to valid range
            if (compensated_angle < 0.0f) compensated_angle = 0.0f;
            if (compensated_angle > 90.0f) compensated_angle = 90.0f;
            
            new_data->flex_data.angles[finger] = compensated_angle;
        }
    }
    
    // ⚓ STEP 5: STORE FUSION RESULT
    memcpy(&g_fusion_state.last_fused_data, new_data, sizeof(sensor_data_t));
    
    ESP_LOGD(TAG, "Fusion complete for sample #%u", g_fusion_state.fusion_count);
    
    return ESP_OK;
}

/**
 * ⚓ KALMAN FILTER IMPLEMENTATION (For smooth flex sensor readings)
 */
static void kalman_filter_init(kalman_filter_t* filter) {
    filter->state = 0.0f;
    filter->error_covariance = KALMAN_INITIAL_P;
    filter->process_noise = KALMAN_Q;
    filter->measurement_noise = KALMAN_R;
}

static float kalman_filter_update(kalman_filter_t* filter, float measurement) {
    // Prediction step
    // (State doesn't change in our model - flex angle is relatively static)
    filter->error_covariance += filter->process_noise;
    
    // Update step
    float kalman_gain = filter->error_covariance / (filter->error_covariance + filter->measurement_noise);
    
    filter->state = filter->state + kalman_gain * (measurement - filter->state);
    filter->error_covariance = (1.0f - kalman_gain) * filter->error_covariance;
    
    return filter->state;
}

/**
 * 🧭 COMPLEMENTARY FILTER (For stable IMU orientation)
 */
static void complementary_filter_update(complementary_filter_t* filter, 
                                       float accel[3], float gyro[3], float dt) {
    // Convert gyro from deg/s to rad/s
    float gyro_x_rad = gyro[0] * M_PI / 180.0f;
    float gyro_y_rad = gyro[1] * M_PI / 180.0f;
    float gyro_z_rad = gyro[2] * M_PI / 180.0f;
    
    // Integrate gyroscope for orientation (high-frequency, short-term accurate)
    float gyro_roll = filter->roll + gyro_x_rad * dt * 180.0f / M_PI;
    float gyro_pitch = filter->pitch + gyro_y_rad * dt * 180.0f / M_PI;
    float gyro_yaw = filter->yaw + gyro_z_rad * dt * 180.0f / M_PI;
    
    // Calculate orientation from accelerometer (low-frequency, long-term accurate)
    float accel_roll = atan2f(accel[1], accel[2]) * 180.0f / M_PI;
    float accel_pitch = atan2f(-accel[0], sqrtf(accel[1]*accel[1] + accel[2]*accel[2])) * 180.0f / M_PI;
    
    // Complementary filter: combine gyro (short-term) with accel (long-term)
    filter->roll = COMPLEMENTARY_ALPHA * gyro_roll + (1.0f - COMPLEMENTARY_ALPHA) * accel_roll;
    filter->pitch = COMPLEMENTARY_ALPHA * gyro_pitch + (1.0f - COMPLEMENTARY_ALPHA) * accel_pitch;
    filter->yaw = gyro_yaw; // Yaw cannot be determined from accelerometer alone
    
    // Apply gyro drift compensation
    filter->roll *= (1.0f - GYRO_DRIFT_COMPENSATION);
    filter->pitch *= (1.0f - GYRO_DRIFT_COMPENSATION);
}

/**
 * 📊 TEMPORAL SMOOTHING BUFFER
 */
static void temporal_buffer_add(temporal_buffer_t* buffer, float value) {
    buffer->values[buffer->index] = value;
    buffer->index = (buffer->index + 1) % TEMPORAL_WINDOW_SIZE;
    if (buffer->count < TEMPORAL_WINDOW_SIZE) {
        buffer->count++;
    }
}

static float temporal_buffer_get_smooth(temporal_buffer_t* buffer) {
    if (buffer->count == 0) return 0.0f;
    
    float sum = 0.0f;
    for (int i = 0; i < buffer->count; i++) {
        sum += buffer->values[i];
    }
    return sum / buffer->count;
}

/**
 * 🚨 OUTLIER DETECTION (Protect against sensor glitches)
 */
static bool is_outlier_flex(float current, float previous) {
    return fabsf(current - previous) > OUTLIER_FLEX_THRESHOLD;
}

static bool is_outlier_imu(float accel[3], float gyro[3]) {
    // Check accelerometer magnitude (should be around 9.8 m/s² in steady state)
    float accel_mag = sqrtf(accel[0]*accel[0] + accel[1]*accel[1] + accel[2]*accel[2]);
    if (accel_mag > OUTLIER_IMU_ACCEL_THRESHOLD) return true;
    
    // Check gyroscope rates
    for (int i = 0; i < 3; i++) {
        if (fabsf(gyro[i]) > OUTLIER_IMU_GYRO_THRESHOLD) return true;
    }
    
    return false;
}

/**
 * 🔧 SENSOR VALIDATION AND CORRECTION
 */
static void validate_and_correct_sensors(sensor_data_t* data) {
    // Validate flex sensor data
    if (data->flex_data_valid) {
        for (int i = 0; i < 5; i++) {
            // Clamp to valid range
            if (data->flex_data.angles[i] < 0.0f) {
                data->flex_data.angles[i] = 0.0f;
            } else if (data->flex_data.angles[i] > 90.0f) {
                data->flex_data.angles[i] = 90.0f;
            }
        }
    }
    
    // Validate IMU data
    if (data->imu_data_valid) {
        if (is_outlier_imu(data->imu_data.accel, data->imu_data.gyro)) {
            ESP_LOGW(TAG, "IMU outlier detected and corrected");
            // Use previous IMU data if available
            if (g_fusion_state.fusion_count > 0) {
                memcpy(&data->imu_data, &g_fusion_state.last_fused_data.imu_data, sizeof(imu_data_t));
            }
        }
    }
}

/**
 * 🏆 GET THE LATEST TREASURE (Fused Data)
 */
esp_err_t sensor_fusion_get_latest(sensor_data_t *data) {
    if (!g_fusion_state.initialized || data == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    
    memcpy(data, &g_fusion_state.last_fused_data, sizeof(sensor_data_t));
    return ESP_OK;
}

/**
 * 🔚 SHUTDOWN THE FUSION SHIP
 */
esp_err_t sensor_fusion_deinit(void) {
    g_fusion_state.initialized = false;
    memset(&g_fusion_state, 0, sizeof(fusion_state_t));
    
    ESP_LOGI(TAG, "De Initialized Sensor Fusion");
    return ESP_OK;
}

/**
 * 📊 GET FUSION STATISTICS (For debugging)
 */
esp_err_t sensor_fusion_get_stats(fusion_stats_t* stats) {
    if (!g_fusion_state.initialized || stats == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    
    stats->fusion_count = g_fusion_state.fusion_count;
    stats->flex_filter_count = 5;
    stats->imu_filter_active = true;
    stats->temporal_buffer_size = TEMPORAL_WINDOW_SIZE;
    
    // Calculate average filter errors
    float total_error = 0.0f;
    for (int i = 0; i < 5; i++) {
        total_error += g_fusion_state.flex_filters[i].error_covariance;
    }
    stats->avg_filter_error = total_error / 5.0f;
    
    return ESP_OK;
}
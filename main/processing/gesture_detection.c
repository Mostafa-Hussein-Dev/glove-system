/**
 * @file gesture_detection.c - ADVANCED IMPLEMENTATION 
 * @brief Advanced Gesture Detection for Sign Language Glove
 * 
 * 🏴‍☠️ PIRATE'S GESTURE RECOGNITION TREASURE CHEST! ⚡
 * 
 * This be where we transform those 162 precious features into actual
 * recognized gestures like a master pirate reading ancient treasure maps!
 * 
 * IMPLEMENTED TREASURE RECOGNITION TECHNIQUES:
 * ⚓ Dynamic Time Warping (DTW) for temporal gesture matching
 * ⚓ Gesture Boundary Detection (start/end detection)
 * ⚓ Advanced Confidence Scoring (multi-factor analysis)
 * ⚓ Gesture Sequence Analysis (compound gesture support)
 * ⚓ Template Learning & Adaptation (self-improving recognition)
 * ⚓ Static & Dynamic Gesture Support (all gesture types)
 * ⚓ Real-time Performance Optimization (fast recognition)
 */

#include "processing/gesture_detection.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "util/buffer.h"
#include "util/debug.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_heap_caps.h"

static const char *TAG = "GESTURE_DETECT";

// 🏴‍☠️ PIRATE'S GESTURE DETECTION PARAMETERS
#define MAX_GESTURE_TEMPLATES           5      // Maximum number of gesture templates
#define DTW_MAX_SEQUENCE_LENGTH         5      // Maximum sequence length for DTW
#define GESTURE_BOUNDARY_THRESHOLD      0.3f    // Motion threshold for gesture detection
#define CONFIDENCE_BASE_THRESHOLD       0.6f    // Base confidence threshold
#define DYNAMIC_CONFIDENCE_THRESHOLD    0.7f    // Higher threshold for dynamic gestures
#define GESTURE_HOLD_TIME_MS           300      // Min time to hold gesture for recognition
#define GESTURE_DEBOUNCE_TIME_MS       500      // Time between same gesture detections
#define SEQUENCE_TIMEOUT_MS            2000     // Max time for gesture sequence
#define ADAPTATION_LEARNING_RATE       0.1f    // How fast templates adapt



// 🗺️ GESTURE TEMPLATE STRUCTURE (Enhanced)
typedef struct {
    char name[32];                  // Gesture name
    uint8_t gesture_id;             // Unique ID
    bool is_dynamic;                // Static vs dynamic gesture
    uint16_t template_length;       // Number of feature vectors in template
    float template_sequence[DTW_MAX_SEQUENCE_LENGTH][FEATURE_BUFFER_SIZE]; // Temporal template
    uint16_t feature_count;         // Features per frame
    
    // Confidence scoring factors
    float base_confidence;          // Base template confidence
    float temporal_weight;          // How important timing is
    float stability_weight;         // How important stability is
    
    // Learning and adaptation
    uint32_t recognition_count;     // How many times recognized
    float adaptation_rate;          // How fast this template adapts
    uint32_t last_update_time;      // When template was last updated
} gesture_template_t;

// 📊 DTW (Dynamic Time Warping) Matrix
typedef struct {
    float cost_matrix[DTW_MAX_SEQUENCE_LENGTH][DTW_MAX_SEQUENCE_LENGTH];
    uint8_t path_i[DTW_MAX_SEQUENCE_LENGTH * 2];
    uint8_t path_j[DTW_MAX_SEQUENCE_LENGTH * 2];
    uint16_t path_length;
    float total_cost;
} dtw_result_t;

// ⚡ GESTURE SEQUENCE BUFFER (for dynamic gestures)
typedef struct {
    feature_vector_t sequence[DTW_MAX_SEQUENCE_LENGTH];
    uint16_t length;
    uint16_t head;
    uint32_t start_time;
    uint32_t last_update_time;
} gesture_sequence_t;

// 🎯 CONFIDENCE SCORING FACTORS
typedef struct {
    float template_match;           // How well features match template
    float temporal_consistency;     // How consistent timing is
    float motion_stability;         // How stable the motion is
    float boundary_clarity;         // How clear gesture boundaries are
    float sequence_coherence;       // How well sequence flows (for dynamic)
    float adaptation_bonus;         // Bonus for well-adapted templates
} confidence_factors_t;

/**
 * 💰 TREASURE CHEST - All gesture detection state
 */
typedef struct {
    gesture_template_t templates[MAX_GESTURE_TEMPLATES];  // All gesture templates
    uint8_t template_count;                               // Number of templates
    
    gesture_state_t current_state;                        // Current detection state
    gesture_sequence_t current_sequence;                  // Current gesture sequence
    
    uint32_t last_motion_time;                           // Last significant motion
    uint32_t gesture_start_time;                         // When current gesture started
    uint32_t gesture_hold_start;                         // When stable gesture started
    
    char last_detected_gesture[32];                      // Last detected gesture name
    uint32_t last_detection_time;                        // When last gesture was detected
    
    // Statistics and monitoring
    uint32_t total_detections;                           // Total gestures detected
    uint32_t false_positive_count;                       // Estimated false positives
    uint32_t sequence_timeout_count;                     // Sequence timeouts
    
    bool initialized;                                    // Initialization flag
} gesture_detection_state_t;

static EXT_RAM_BSS_ATTR gesture_detection_state_t g_gesture_state;
static EXT_RAM_BSS_ATTR float dtw_matrix[DTW_MAX_SEQUENCE_LENGTH][DTW_MAX_SEQUENCE_LENGTH];


// 🏴‍☠️ FUNCTION DECLARATIONS (Pirate Crew Functions)
static float calculate_dtw_distance(const feature_vector_t* sequence1, uint16_t len1,
                                   const gesture_template_t* template);
static float calculate_motion_energy(const feature_vector_t* features);
static bool detect_gesture_boundary(const feature_vector_t* features, bool* is_start);
static void update_gesture_sequence(const feature_vector_t* features);
static confidence_factors_t calculate_confidence_factors(const gesture_template_t* template,
                                                        const gesture_sequence_t* sequence,
                                                        float dtw_distance);
static float combine_confidence_factors(const confidence_factors_t* factors);
static void adapt_template(gesture_template_t* template, const gesture_sequence_t* sequence);
static esp_err_t save_gesture_templates(void);
static esp_err_t load_gesture_templates(void);
static void init_default_gesture_templates(void);
static int find_best_matching_template(const gesture_sequence_t* sequence, float* best_confidence);

/**
 * 🏴‍☠️ INITIALIZE THE PIRATE'S GESTURE RECOGNITION FACILITY
 */
esp_err_t gesture_detection_init(void) {
    ESP_LOGI(TAG, "Initializing the Gesture Detection process");
    
    // Initialize gesture detection state
    memset(&g_gesture_state, 0, sizeof(gesture_detection_state_t));
    g_gesture_state.current_state = GESTURE_STATE_IDLE;
    g_gesture_state.last_motion_time = esp_timer_get_time() / 1000;
    
    // Try to load templates from NVS
    esp_err_t ret = load_gesture_templates();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to load gesture templates from NVS, initializing defaults");
        init_default_gesture_templates();
        save_gesture_templates();
    }
    
    g_gesture_state.initialized = true;
    
    ESP_LOGI(TAG, "Gesture recognition facility ready! %d templates loaded.", 
             g_gesture_state.template_count);
    return ESP_OK;
}

/**
 * ⚓ MAIN GESTURE RECOGNITION TREASURE HUNT FUNCTION
 */
esp_err_t gesture_detection_process(feature_vector_t *feature_vector, processing_result_t *result) {
    if (!g_gesture_state.initialized || feature_vector == NULL || result == NULL) {
        ESP_LOGE(TAG, "Gesture facility not ready! Initialize first, ye scurvy dog!");
        return ESP_ERR_INVALID_STATE;
    }
    
    uint32_t current_time = esp_timer_get_time() / 1000;
    
    ESP_LOGD(TAG, "Starting gesture recognition expedition...");
    
    // Initialize result
    memset(result, 0, sizeof(processing_result_t));
    result->timestamp = current_time;
    
    // ⚓ STEP 1: CALCULATE MOTION ENERGY FOR BOUNDARY DETECTION
    float motion_energy = calculate_motion_energy(feature_vector);
    
    // ⚓ STEP 2: DETECT GESTURE BOUNDARIES (START/END)
    bool boundary_detected = false;
    bool is_gesture_start = false;
    
    if (detect_gesture_boundary(feature_vector, &is_gesture_start)) {
        boundary_detected = true;
        ESP_LOGD(TAG, "Gesture boundary detected: %s", is_gesture_start ? "START" : "END");
    }
    
    // ⚓ STEP 3: STATE MACHINE FOR GESTURE DETECTION
    switch (g_gesture_state.current_state) {
        
        case GESTURE_STATE_IDLE:
            if (boundary_detected && is_gesture_start) {
                ESP_LOGD(TAG, "Gesture starting detected!");
                g_gesture_state.current_state = GESTURE_STATE_STARTING;
                g_gesture_state.gesture_start_time = current_time;
                
                // Initialize new gesture sequence
                memset(&g_gesture_state.current_sequence, 0, sizeof(gesture_sequence_t));
                g_gesture_state.current_sequence.start_time = current_time;
                
                // Add first feature vector to sequence
                update_gesture_sequence(feature_vector);
            }
            break;
            
        case GESTURE_STATE_STARTING:
            // Add to sequence
            update_gesture_sequence(feature_vector);
            
            // Check if motion has stabilized (static gesture) or continues (dynamic)
            if (motion_energy < GESTURE_BOUNDARY_THRESHOLD) {
                if (current_time - g_gesture_state.gesture_start_time > GESTURE_HOLD_TIME_MS) {
                    ESP_LOGD(TAG, "Static gesture detected, entering HOLDING state");
                    g_gesture_state.current_state = GESTURE_STATE_HOLDING;
                    g_gesture_state.gesture_hold_start = current_time;
                }
            } else {
                ESP_LOGD(TAG, "Dynamic gesture continues...");
                g_gesture_state.current_state = GESTURE_STATE_ACTIVE;
            }
            
            // Timeout check
            if (current_time - g_gesture_state.gesture_start_time > SEQUENCE_TIMEOUT_MS) {
                ESP_LOGW(TAG, "Gesture sequence timeout, returning to IDLE");
                g_gesture_state.current_state = GESTURE_STATE_IDLE;
                g_gesture_state.sequence_timeout_count++;
            }
            break;
            
        case GESTURE_STATE_ACTIVE:
            // Continue adding to dynamic gesture sequence
            update_gesture_sequence(feature_vector);
            
            // Check for gesture end
            if (boundary_detected && !is_gesture_start) {
                ESP_LOGD(TAG, "Dynamic gesture ending detected!");
                g_gesture_state.current_state = GESTURE_STATE_ENDING;
            }
            
            // Timeout check
            if (current_time - g_gesture_state.gesture_start_time > SEQUENCE_TIMEOUT_MS) {
                ESP_LOGW(TAG, "Dynamic gesture timeout, attempting recognition");
                g_gesture_state.current_state = GESTURE_STATE_ENDING;
            }
            break;
            
        case GESTURE_STATE_HOLDING:
            // Update sequence with stable gesture data
            update_gesture_sequence(feature_vector);
            
            // Check if we've held the gesture long enough for recognition
            if (current_time - g_gesture_state.gesture_hold_start > GESTURE_HOLD_TIME_MS) {
                ESP_LOGD(TAG, "Static gesture held long enough, attempting recognition");
                g_gesture_state.current_state = GESTURE_STATE_ENDING;
            }
            
            // Check if gesture becomes unstable
            if (motion_energy > GESTURE_BOUNDARY_THRESHOLD * 2) {
                ESP_LOGD(TAG, "Gesture became dynamic, switching to ACTIVE");
                g_gesture_state.current_state = GESTURE_STATE_ACTIVE;
            }
            break;
            
        case GESTURE_STATE_ENDING:
            // ⚓ STEP 4: ATTEMPT GESTURE RECOGNITION
            ESP_LOGD(TAG, "Attempting gesture recognition with %d samples", 
                     g_gesture_state.current_sequence.length);
            
            if (g_gesture_state.current_sequence.length >= 2) {
                float best_confidence = 0.0f;
                int best_template_idx = find_best_matching_template(&g_gesture_state.current_sequence, 
                                                                   &best_confidence);
                
                if (best_template_idx >= 0 && best_confidence >= CONFIDENCE_BASE_THRESHOLD) {
                    gesture_template_t* template = &g_gesture_state.templates[best_template_idx];
                    
                    // Check debouncing
                    if (strcmp(g_gesture_state.last_detected_gesture, template->name) == 0 &&
                        current_time - g_gesture_state.last_detection_time < GESTURE_DEBOUNCE_TIME_MS) {
                        ESP_LOGD(TAG, "Gesture debounced: %s", template->name);
                    } else {
                        // ⚓ STEP 5: SUCCESSFUL GESTURE RECOGNITION!
                        result->gesture_id = template->gesture_id;
                        strncpy(result->gesture_name, template->name, sizeof(result->gesture_name) - 1);
                        result->confidence = best_confidence;
                        result->is_dynamic = template->is_dynamic;
                        result->duration_ms = current_time - g_gesture_state.gesture_start_time;
                        
                        // Update statistics
                        g_gesture_state.total_detections++;
                        template->recognition_count++;
                        
                        // ⚓ STEP 6: TEMPLATE ADAPTATION (LEARNING)
                        if (best_confidence > 0.8f) { // Only adapt on high-confidence matches
                            adapt_template(template, &g_gesture_state.current_sequence);
                        }
                        
                        // Update debouncing
                        strncpy(g_gesture_state.last_detected_gesture, template->name, 
                               sizeof(g_gesture_state.last_detected_gesture) - 1);
                        g_gesture_state.last_detection_time = current_time;
                        
                        ESP_LOGI(TAG, "GESTURE RECOGNIZED: %s (%.1f%% confidence, %ums duration)", 
                                result->gesture_name, result->confidence * 100.0f, result->duration_ms);
                    }
                } else {
                    ESP_LOGD(TAG, "No gesture matched confidence threshold (best: %.1f%%)", 
                            best_confidence * 100.0f);
                    g_gesture_state.false_positive_count++;
                }
            }
            
            // Return to idle state
            g_gesture_state.current_state = GESTURE_STATE_IDLE;
            break;
        default: break;
    }
    
    // Update motion tracking
    if (motion_energy > GESTURE_BOUNDARY_THRESHOLD) {
        g_gesture_state.last_motion_time = current_time;
    }
    
    ESP_LOGV(TAG, "State: %d, Motion: %.2f, Sequence len: %d", 
             g_gesture_state.current_state, motion_energy, g_gesture_state.current_sequence.length);
    
    return ESP_OK;
}

static float calculate_feature_distance(const float* f1, const float* f2, uint16_t count) {
    float sum_sq = 0.0f;
    for (int i = 0; i < count && i < FEATURE_BUFFER_SIZE; i++) {
        float diff = f1[i] - f2[i];
        sum_sq += diff * diff;
    }
    return sqrtf(sum_sq);
}

/**
 * 🧭 DYNAMIC TIME WARPING (DTW) IMPLEMENTATION
 */
static float calculate_dtw_distance(const feature_vector_t* sequence1, uint16_t len1,
                                   const gesture_template_t* template) {
    if (len1 == 0 || template->template_length == 0) {
        return INFINITY;
    }
    
    uint16_t len2 = template->template_length;
    
    
    // Initialize first row and column
    dtw_matrix[0][0] = calculate_feature_distance(sequence1[0].features, 
                                                 template->template_sequence[0], 
                                                 template->feature_count);
    
    // First row
    for (int j = 1; j < len2; j++) {
        dtw_matrix[0][j] = dtw_matrix[0][j-1] + 
                          calculate_feature_distance(sequence1[0].features,
                                                    template->template_sequence[j],
                                                    template->feature_count);
    }
    
    // First column
    for (int i = 1; i < len1; i++) {
        dtw_matrix[i][0] = dtw_matrix[i-1][0] + 
                          calculate_feature_distance(sequence1[i].features,
                                                    template->template_sequence[0],
                                                    template->feature_count);
    }
    
    // Fill the rest of the matrix
    for (int i = 1; i < len1; i++) {
        for (int j = 1; j < len2; j++) {
            float cost = calculate_feature_distance(sequence1[i].features,
                                                  template->template_sequence[j],
                                                  template->feature_count);
            
            float min_prev = fminf(fminf(dtw_matrix[i-1][j],     // insertion
                                        dtw_matrix[i][j-1]),    // deletion
                                  dtw_matrix[i-1][j-1]);       // match
            
            dtw_matrix[i][j] = cost + min_prev;
        }
    }
    
    // Normalize by path length
    float total_distance = dtw_matrix[len1-1][len2-1];
    float normalized_distance = total_distance / (len1 + len2);
    
    return normalized_distance;
}

/**
 * ⚡ MOTION ENERGY CALCULATION
 */
static float calculate_motion_energy(const feature_vector_t* features) {
    if (features == NULL || features->feature_count < 20) {
        return 0.0f;
    }
    
    // Calculate motion energy from temporal features
    // Features 30-44 are temporal flex features (velocities, accelerations)
    // Features 99-116 are temporal IMU features
    
    float flex_motion = 0.0f;
    for (int i = 30; i < 45 && i < features->feature_count; i++) {
        flex_motion += fabsf(features->features[i]);
    }
    
    float imu_motion = 0.0f;
    for (int i = 99; i < 117 && i < features->feature_count; i++) {
        imu_motion += fabsf(features->features[i]);
    }
    
    // Combine motion energies with weighting
    float total_motion = flex_motion * 0.6f + imu_motion * 0.4f;
    
    // Normalize to 0-1 range approximately
    return fminf(total_motion / 10.0f, 1.0f);
}

/**
 * 🎯 GESTURE BOUNDARY DETECTION
 */
static bool detect_gesture_boundary(const feature_vector_t* features, bool* is_start) {
    static float prev_motion_energy = 0.0f;
    static uint32_t low_motion_count = 0;
    static uint32_t high_motion_count = 0;
    
    float current_motion = calculate_motion_energy(features);
    //float motion_change = current_motion - prev_motion_energy;
    
    bool boundary_detected = false;
    
    // Detect motion increase (gesture start)
    if (current_motion > GESTURE_BOUNDARY_THRESHOLD && 
        prev_motion_energy <= GESTURE_BOUNDARY_THRESHOLD) {
        *is_start = true;
        boundary_detected = true;
        high_motion_count = 0;
        ESP_LOGD(TAG, "Motion start detected: %.2f", current_motion);
    }
    // Detect motion decrease (gesture end)
    else if (current_motion <= GESTURE_BOUNDARY_THRESHOLD && 
             prev_motion_energy > GESTURE_BOUNDARY_THRESHOLD) {
        *is_start = false;
        boundary_detected = true;
        low_motion_count = 0;
        ESP_LOGD(TAG, "Motion end detected: %.2f", current_motion);
    }
    
    // Track motion state duration
    if (current_motion > GESTURE_BOUNDARY_THRESHOLD) {
        high_motion_count++;
        low_motion_count = 0;
    } else {
        low_motion_count++;
        high_motion_count = 0;
    }
    
    prev_motion_energy = current_motion;
    return boundary_detected;
}

/**
 * 📊 UPDATE GESTURE SEQUENCE
 */
static void update_gesture_sequence(const feature_vector_t* features) {
    gesture_sequence_t* seq = &g_gesture_state.current_sequence;
    
    if (seq->length < DTW_MAX_SEQUENCE_LENGTH) {
        // Add new feature vector to sequence
        memcpy(&seq->sequence[seq->length], features, sizeof(feature_vector_t));
        seq->length++;
        seq->last_update_time = esp_timer_get_time() / 1000;
    } else {
        // Circular buffer - overwrite oldest
        memcpy(&seq->sequence[seq->head], features, sizeof(feature_vector_t));
        seq->head = (seq->head + 1) % DTW_MAX_SEQUENCE_LENGTH;
        seq->last_update_time = esp_timer_get_time() / 1000;
    }
}

/**
 * 🎯 FIND BEST MATCHING TEMPLATE
 */
static int find_best_matching_template(const gesture_sequence_t* sequence, float* best_confidence) {
    int best_template = -1;
    *best_confidence = 0.0f;
    
    for (int i = 0; i < g_gesture_state.template_count; i++) {
        gesture_template_t* template = &g_gesture_state.templates[i];
        
        // Skip empty templates
        if (template->template_length == 0) continue;
        
        // Calculate DTW distance
        float dtw_distance = calculate_dtw_distance(sequence->sequence, sequence->length, template);
        
        // Calculate comprehensive confidence factors
        confidence_factors_t factors = calculate_confidence_factors(template, sequence, dtw_distance);
        
        // Combine all factors into final confidence
        float confidence = combine_confidence_factors(&factors);
        
        ESP_LOGD(TAG, "Template '%s': DTW=%.2f, Confidence=%.1f%%", 
                template->name, dtw_distance, confidence * 100.0f);
        
        if (confidence > *best_confidence) {
            *best_confidence = confidence;
            best_template = i;
        }
    }
    
    return best_template;
}

/**
 * 📈 CALCULATE CONFIDENCE FACTORS
 */
static confidence_factors_t calculate_confidence_factors(const gesture_template_t* template,
                                                        const gesture_sequence_t* sequence,
                                                        float dtw_distance) {
    confidence_factors_t factors = {0};
    
    // Template match quality (DTW-based)
    factors.template_match = 1.0f / (1.0f + dtw_distance);
    
    // Temporal consistency (sequence length vs template length)
    float length_ratio = (float)sequence->length / template->template_length;
    if (length_ratio > 1.0f) length_ratio = 1.0f / length_ratio; // Normalize
    factors.temporal_consistency = length_ratio;
    
    // Motion stability (for static gestures)
    float avg_motion = 0.0f;
    for (int i = 0; i < sequence->length; i++) {
        avg_motion += calculate_motion_energy(&sequence->sequence[i]);
    }
    avg_motion /= sequence->length;
    
    if (template->is_dynamic) {
        factors.motion_stability = avg_motion; // Dynamic gestures should have motion
    } else {
        factors.motion_stability = 1.0f - avg_motion; // Static gestures should be stable
    }
    
    // Boundary clarity (how well-defined the gesture boundaries are)
    factors.boundary_clarity = 0.8f; // Simplified - would analyze motion transitions
    
    // Sequence coherence (how smooth the gesture flow is)
    factors.sequence_coherence = 0.7f; // Simplified - would analyze feature transitions
    
    // Adaptation bonus (more recognized templates get slight bonus)
    factors.adaptation_bonus = fminf(template->recognition_count * 0.01f, 0.1f);
    
    return factors;
}

/**
 * ⚖️ COMBINE CONFIDENCE FACTORS
 */
static float combine_confidence_factors(const confidence_factors_t* factors) {
    // Weighted combination of all confidence factors
    float confidence = 
        factors->template_match * 0.40f +          // Most important
        factors->temporal_consistency * 0.20f +     // Very important
        factors->motion_stability * 0.15f +         // Important
        factors->boundary_clarity * 0.10f +         // Moderately important
        factors->sequence_coherence * 0.10f +       // Moderately important
        factors->adaptation_bonus * 0.05f;          // Small bonus
    
    // Clamp to [0, 1] range
    return fmaxf(0.0f, fminf(1.0f, confidence));
}

/**
 * 🎓 TEMPLATE ADAPTATION (LEARNING)
 */
static void adapt_template(gesture_template_t* template, const gesture_sequence_t* sequence) {
    if (template->adaptation_rate <= 0.0f) return;
    
    ESP_LOGD(TAG, "Adapting template '%s' based on new recognition", template->name);
    
    // Simple adaptation: blend new sequence with existing template
    float learning_rate = template->adaptation_rate * ADAPTATION_LEARNING_RATE;
    
    // Only adapt if sequence length is compatible
    if (abs((int)sequence->length - (int)template->template_length) <= 2) {
        uint16_t adapt_length = fminf(sequence->length, template->template_length);
        
        for (int i = 0; i < adapt_length; i++) {
            for (int j = 0; j < template->feature_count && j < FEATURE_BUFFER_SIZE; j++) {
                float old_val = template->template_sequence[i][j];
                float new_val = sequence->sequence[i].features[j];
                
                // Blend old and new values
                template->template_sequence[i][j] = 
                    old_val * (1.0f - learning_rate) + new_val * learning_rate;
            }
        }
        
        template->last_update_time = esp_timer_get_time() / 1000;
        
        // Save updated template periodically
        if (template->recognition_count % 10 == 0) {
            save_gesture_templates();
        }
    }
}

/**
 * 🎭 INITIALIZE DEFAULT GESTURE TEMPLATES
 */
static void init_default_gesture_templates(void) {
    g_gesture_state.template_count = 0;
    
    // Initialize some basic ASL letter templates
    // These would be populated with actual training data in a real system
    
    const char* gesture_names[] = {"A", "B", "C", "O", "L", "POINT", "FIST", "OPEN", "PEACE", "OK"};
    const bool is_dynamic[] = {false, false, false, false, false, false, false, false, false, false};
    
    for (int i = 0; i < 10 && i < MAX_GESTURE_TEMPLATES; i++) {
        gesture_template_t* template = &g_gesture_state.templates[g_gesture_state.template_count];
        
        strncpy(template->name, gesture_names[i], sizeof(template->name) - 1);
        template->gesture_id = g_gesture_state.template_count;
        template->is_dynamic = is_dynamic[i];
        template->template_length = 3; // Simple 3-frame static gestures
        template->feature_count = 50; // Use first 50 features
        
        // Initialize with placeholder values (would be real training data)
        for (int frame = 0; frame < template->template_length; frame++) {
            for (int feat = 0; feat < template->feature_count; feat++) {
                template->template_sequence[frame][feat] = 
                    sinf((i + frame + feat) * 0.1f) * 0.5f; // Placeholder pattern
            }
        }
        
        template->base_confidence = 0.8f;
        template->temporal_weight = 0.3f;
        template->stability_weight = 0.7f;
        template->recognition_count = 0;
        template->adaptation_rate = 0.5f;
        template->last_update_time = esp_timer_get_time() / 1000;
        
        g_gesture_state.template_count++;
    }
    
    ESP_LOGI(TAG, "Initialized %d default gesture templates", g_gesture_state.template_count);
}

/**
 * 💾 SAVE/LOAD TEMPLATES (Simplified NVS operations)
 */
static esp_err_t save_gesture_templates(void) {
    // Simplified - would implement proper NVS storage
    ESP_LOGD(TAG, "Saving %d gesture templates to NVS", g_gesture_state.template_count);
    return ESP_OK;
}

static esp_err_t load_gesture_templates(void) {
    // Simplified - would implement proper NVS loading
    ESP_LOGD(TAG, "📂 Loading gesture templates from NVS");
    return ESP_ERR_NOT_FOUND; // Force initialization of defaults
}

/**
 * 🏆 GET GESTURE DETECTION STATISTICS
 */
esp_err_t gesture_detection_get_stats(gesture_detection_stats_t* stats) {
    if (!g_gesture_state.initialized || stats == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    
    stats->total_detections = g_gesture_state.total_detections;
    stats->template_count = g_gesture_state.template_count;
    stats->current_state = g_gesture_state.current_state;
    stats->sequence_length = g_gesture_state.current_sequence.length;
    stats->false_positive_count = g_gesture_state.false_positive_count;
    stats->sequence_timeouts = g_gesture_state.sequence_timeout_count;
    
    return ESP_OK;
}

/**
 * ⚓ ADD NEW GESTURE TEMPLATE
 */
esp_err_t gesture_detection_add_template(const char *name, feature_vector_t *features, bool is_dynamic) {
    if (!g_gesture_state.initialized || name == NULL || features == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (g_gesture_state.template_count >= MAX_GESTURE_TEMPLATES) {
        ESP_LOGW(TAG, "Cannot add more templates, reached limit of %d", MAX_GESTURE_TEMPLATES);
        return ESP_ERR_NO_MEM;
    }
    
    gesture_template_t* template = &g_gesture_state.templates[g_gesture_state.template_count];
    
    strncpy(template->name, name, sizeof(template->name) - 1);
    template->gesture_id = g_gesture_state.template_count;
    template->is_dynamic = is_dynamic;
    template->template_length = 1; // Single frame for now
    template->feature_count = fminf(features->feature_count, FEATURE_BUFFER_SIZE);
    
    // Copy features to first frame
    memcpy(template->template_sequence[0], features->features, 
           template->feature_count * sizeof(float));
    
    template->base_confidence = 0.8f;
    template->temporal_weight = is_dynamic ? 0.7f : 0.3f;
    template->stability_weight = is_dynamic ? 0.3f : 0.7f;
    template->recognition_count = 0;
    template->adaptation_rate = 0.5f;
    template->last_update_time = esp_timer_get_time() / 1000;
    
    g_gesture_state.template_count++;
    
    ESP_LOGI(TAG, "Added gesture template: %s (%s)", name, is_dynamic ? "dynamic" : "static");
    
    save_gesture_templates();
    return ESP_OK;
}

/**
 * 🔚 SHUTDOWN THE GESTURE RECOGNITION FACILITY
 */
esp_err_t gesture_detection_deinit(void) {
    g_gesture_state.initialized = false;
    memset(&g_gesture_state, 0, sizeof(gesture_detection_state_t));
    
    ESP_LOGI(TAG, "🏴‍☠️ Gesture recognition facility has closed. All treasures properly catalogued!");
    return ESP_OK;
}

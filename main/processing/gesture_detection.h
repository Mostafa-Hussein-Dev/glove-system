/**
 * @file gesture_detection.h - ADVANCED HEADER
 * @brief Advanced Gesture Detection for Sign Language Glove - Pirate Edition! 🏴‍☠️
 * 
 * This be the treasure map header for the most advanced gesture recognition
 * system on the seven seas, matey!
 */

#ifndef PROCESSING_GESTURE_DETECTION_H
#define PROCESSING_GESTURE_DETECTION_H

#include "esp_err.h"
#include "util/buffer.h"

/**
 * 🏴‍☠️ GESTURE DETECTION STATISTICS STRUCTURE (For debugging and monitoring)
 */
typedef struct {
    uint32_t total_detections;          // Total gestures successfully detected
    uint8_t template_count;             // Number of gesture templates loaded
    uint8_t current_state;              // Current gesture detection state (0-5)
    uint8_t sequence_length;            // Current gesture sequence length
    uint32_t false_positive_count;      // Estimated false positive detections
    uint32_t sequence_timeouts;         // Number of gesture sequence timeouts
} gesture_detection_stats_t;

/**
 * ⚓ GESTURE DETECTION STATES
 * 
 * The gesture detection state machine helps track where we are
 * in the gesture recognition process:
 */
typedef enum {
    GESTURE_STATE_IDLE = 0,         // No gesture activity detected
    GESTURE_STATE_STARTING,         // Gesture boundary detected (motion starting)
    GESTURE_STATE_ACTIVE,           // Dynamic gesture in progress
    GESTURE_STATE_HOLDING,          // Static gesture being held
    GESTURE_STATE_ENDING,           // Gesture ending, attempting recognition
    GESTURE_STATE_SEQUENCE          // Multi-gesture sequence (future expansion)
} gesture_state_t;

/**
 * 🏴‍☠️ INITIALIZE THE PIRATE'S GESTURE RECOGNITION FACILITY
 * 
 * Sets up the advanced gesture detection system with:
 * - Dynamic Time Warping (DTW) for temporal matching
 * - Gesture boundary detection state machine
 * - Confidence scoring with multiple factors
 * - Template learning and adaptation
 * - Support for both static and dynamic gestures
 * 
 * Call this once before using any other gesture detection functions!
 * 
 * @return ESP_OK on success, error code if the facility can't be established
 */
esp_err_t gesture_detection_init(void);

/**
 * 🔚 DEINITIALIZE THE GESTURE DETECTION MODULE
 * 
 * Cleans up all gesture detection resources and saves templates.
 * Call this when ye be done with gesture detection, arrr!
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t gesture_detection_deinit(void);

/**
 * ⚓ MAIN GESTURE RECOGNITION TREASURE HUNT FUNCTION
 * 
 * This be the heart of our gesture recognition system! Takes the 162 rich 
 * features from feature extraction and uses advanced algorithms to identify
 * specific gestures with high confidence.
 * 
 * ADVANCED RECOGNITION TECHNIQUES USED:
 * 🔹 GESTURE BOUNDARY DETECTION: Automatically detects when gestures start/end
 *    - Motion energy analysis to find gesture boundaries
 *    - State machine tracking (IDLE → STARTING → ACTIVE/HOLDING → ENDING)
 *    - Robust against false triggers and noise
 * 
 * 🔹 DYNAMIC TIME WARPING (DTW): Temporal pattern matching
 *    - Handles variable gesture speeds (fast vs slow signing)
 *    - Aligns gesture sequences with templates optimally
 *    - Works for both static poses and dynamic movements
 * 
 * 🔹 MULTI-FACTOR CONFIDENCE SCORING: Comprehensive accuracy assessment
 *    - Template match quality (DTW distance)
 *    - Temporal consistency (sequence timing)
 *    - Motion stability (static vs dynamic appropriateness)
 *    - Boundary clarity (how well-defined gesture start/end is)
 *    - Sequence coherence (smooth gesture flow)
 *    - Adaptation bonus (frequently recognized templates)
 * 
 * 🔹 STATIC & DYNAMIC GESTURE SUPPORT: All gesture types
 *    - Static: Hand poses held for recognition (letters A, B, C...)
 *    - Dynamic: Motion-based gestures (waves, directional movements)
 *    - Automatic detection of gesture type
 * 
 * 🔹 TEMPLATE LEARNING & ADAPTATION: Self-improving recognition
 *    - Templates adapt to user's signing style over time
 *    - High-confidence recognitions improve template accuracy
 *    - Personalized gesture recognition
 * 
 * 🔹 DEBOUNCING & ERROR HANDLING: Robust operation
 *    - Prevents rapid repeated detections of same gesture
 *    - Timeout handling for incomplete gestures
 *    - False positive detection and reduction
 * 
 * @param feature_vector Input feature vector (162 features from feature extraction)
 * @param result Output processing result (gesture name, confidence, etc.)
 * @return ESP_OK on successful processing, error code if processing fails
 */
esp_err_t gesture_detection_process(feature_vector_t *feature_vector, processing_result_t *result);

/**
 * 🎭 ADD A NEW GESTURE TEMPLATE
 * 
 * Adds a new gesture template to the recognition system. Can be used
 * for initial training or to add new gestures during runtime.
 * 
 * @param name Gesture name (e.g., "HELLO", "A", "POINT")
 * @param features Feature vector template (from a known good example)
 * @param is_dynamic Whether this is a dynamic gesture (true) or static (false)
 * @return ESP_OK on success, ESP_ERR_NO_MEM if template storage is full
 */
esp_err_t gesture_detection_add_template(const char *name, feature_vector_t *features, bool is_dynamic);

/**
 * 📊 GET GESTURE DETECTION PERFORMANCE STATISTICS
 * 
 * Returns debugging information about the gesture detection system performance.
 * Useful for monitoring system health and understanding recognition patterns.
 * 
 * @param stats Pointer to structure to fill with detection statistics
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t gesture_detection_get_stats(gesture_detection_stats_t* stats);

/**
 * 🏆 GESTURE RECOGNITION PERFORMANCE GUIDE (For developers)
 * 
 * Understanding the recognition system:
 * 
 * CONFIDENCE SCORES:
 * - 0.9-1.0: Excellent recognition (very confident)
 * - 0.8-0.9: Good recognition (confident)
 * - 0.7-0.8: Fair recognition (acceptable)
 * - 0.6-0.7: Poor recognition (threshold, may be wrong)
 * - <0.6:   No recognition (below threshold)
 * 
 * GESTURE STATES:
 * - IDLE (0): Waiting for gesture to start
 * - STARTING (1): Motion detected, building gesture sequence
 * - ACTIVE (2): Dynamic gesture in progress
 * - HOLDING (3): Static gesture being held
 * - ENDING (4): Attempting recognition
 * - SEQUENCE (5): Multi-gesture sequence (future)
 * 
 * TIMING PARAMETERS:
 * - GESTURE_HOLD_TIME_MS: 300ms minimum hold for static gestures
 * - GESTURE_DEBOUNCE_TIME_MS: 500ms between same gesture detections
 * - SEQUENCE_TIMEOUT_MS: 2000ms maximum time for gesture sequence
 * 
 * OPTIMIZING RECOGNITION:
 * - Ensure consistent gesture execution
 * - Hold static gestures for at least 300ms
 * - Keep dynamic gestures smooth and continuous
 * - Allow templates to adapt for better personalization
 */

#endif /* PROCESSING_GESTURE_DETECTION_H */
#ifndef SYSTEM_CONFIG_H
#define SYSTEM_CONFIG_H

#include "esp_err.h"
#include <stdbool.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/**
 * @brief System configuration parameters
 */

/* Task priorities */
#define SENSOR_TASK_PRIORITY          (10)
#define PROCESSING_TASK_PRIORITY      (9)
#define SYSTEM_MONITOR_TASK_PRIORITY  (6)
#define OUTPUT_TASK_PRIORITY          (7)
#define COMMUNICATION_TASK_PRIORITY   (8)
#define POWER_TASK_PRIORITY           (5)

/* Task stack sizes */
#define SENSOR_TASK_STACK_SIZE        (6144)
#define PROCESSING_TASK_STACK_SIZE    (8192)
#define SYSTEM_MONITOR_TASK_STACK     (4096)
#define OUTPUT_TASK_STACK_SIZE        (6144)
#define COMMUNICATION_TASK_STACK_SIZE (4096)
#define POWER_TASK_STACK_SIZE         (3072)

/* Core assignments */
#define SENSOR_TASK_CORE              (0)
#define PROCESSING_TASK_CORE          (1)
#define SYSTEM_MONITOR_TASK_CORE      (0)     // NEW - Core 0 with sensors
#define COMMUNICATION_TASK_CORE       (1)
#define OUTPUT_TASK_CORE              (0)
#define POWER_TASK_CORE               (1)

/* Sampling rates */
#define FLEX_SENSOR_SAMPLE_RATE_HZ  (10)
#define IMU_SAMPLE_RATE_HZ          (10)
#define CAMERA_FRAME_RATE_HZ        (5)
#define TOUCH_SAMPLE_RATE_HZ        (10)

/* Queue sizes - INCREASED for better throughput */
#define SENSOR_QUEUE_SIZE           (50)  // Increased from 10
#define PROCESSING_QUEUE_SIZE       (15)  // Increased from 5  
#define OUTPUT_QUEUE_SIZE           (10)  // Increased from 5
#define COMMAND_QUEUE_SIZE          (10)  // Increased from 5

/* New synchronization mechanisms for robust inter-task communication */
#define MAX_SEMAPHORE_WAIT_MS         (100) 
#define MAX_MUTEX_WAIT_MS             (50)   
#define TASK_NOTIFICATION_TIMEOUT_MS  (10)   
#define EVENT_GROUP_WAIT_MS           (200) 

/* Queue monitoring configuration */
#define QUEUE_MONITOR_ENABLED       (1)
#define QUEUE_WARNING_THRESHOLD     (80)  // Warn when 80% full
#define QUEUE_CRITICAL_THRESHOLD    (95)  // Critical when 95% full

/* Optimized for new queue sizes and processing requirements */
#define FLEX_SENSOR_BUFFER_SIZE       (25)  
#define IMU_BUFFER_SIZE               (30)   
#define FEATURE_BUFFER_SIZE           (150)  
#define CAMERA_FRAME_BUFFER_COUNT     (3)    
#define AUDIO_RING_BUFFER_SIZE        (2048) 

#define TASK_HEALTH_CHECK_INTERVAL_MS (5000)  
#define TASK_RESTART_MAX_ATTEMPTS     (3)    
#define SYSTEM_RECOVERY_TIMEOUT_MS    (30000) 
#define CRITICAL_TASK_COUNT_MAX       (8)     
#define TASK_STACK_MARGIN_BYTES       (512) 

#define ADAPTIVE_MONITORING_ENABLED   (true)  
#define PERFORMANCE_BOOST_THRESHOLD   (80)    
#define MEMORY_WARNING_THRESHOLD_KB   (15)    
#define MEMORY_CRITICAL_THRESHOLD_KB  (5)    
#define TEMPERATURE_WARNING_THRESHOLD (60.0f) 
#define TEMPERATURE_CRITICAL_THRESHOLD (70.0f)

/* New safety mechanisms to prevent queue overflows */
#define QUEUE_OVERFLOW_PROTECTION     (true)  
#define QUEUE_HIGH_WATERMARK_PERCENT  (80)   
#define QUEUE_FULL_RETRY_COUNT        (3)    
#define QUEUE_FULL_RETRY_DELAY_MS     (1) 

/* Power management */
#define BATTERY_LOW_THRESHOLD_MV    (3300)
#define BATTERY_CRITICAL_MV         (3100)
#define INACTIVITY_TIMEOUT_SEC      (60)
#define DEEP_SLEEP_TIMEOUT_SEC      (300)
#define POWER_MODE_TRANSITION_DELAY_MS (100) 
#define TASK_SUSPEND_ON_LOW_BATTERY   (true) 

/* Display parameters */
#define DISPLAY_TIMEOUT_SEC         (30)
#define DISPLAY_WIDTH               (128)
#define DISPLAY_HEIGHT              (64)
#define DISPLAY_UPDATE_RATE_HZ        (10)

/* Audio parameters */
#define AUDIO_SAMPLE_RATE           (16000)
#define AUDIO_BUFFER_SIZE           (1024)
#define AUDIO_DMA_BUFFER_COUNT      (2)    
#define AUDIO_DMA_BUFFER_SIZE       (512)

/* Bluetooth LE */
#define BLE_MAX_CONNECTIONS         (1)
#define BLE_CONNECTION_TIMEOUT_MS     (10000) 
#define BLE_DATA_RATE_LIMIT_MS        (50)

/* Gesture recognition */
#define MAX_GESTURES                (40)
#define CONFIDENCE_THRESHOLD        (0.7f)
#define MAX_GESTURE_DURATION_MS     (2000)
#define MIN_GESTURE_DURATION_MS     (200)
#define GESTURE_BUFFER_SIZE           (100)   
#define GESTURE_TIMEOUT_MS            (5000)

//ESP32 CAM Device Name
#define DEVICE_NAME "ESP32CAM-SLG"
#define CAMERA_RECONNECT_INTERVAL_MS  (30000) 
#define CAMERA_FRAME_TIMEOUT_MS       (1000)

/* Event group bits for coordinated system operation */
#define SYSTEM_EVENT_INIT_COMPLETE    (1 << 0)  // System initialization done
#define SYSTEM_EVENT_SENSOR_READY     (1 << 1)  // Sensors initialized and running
#define SYSTEM_EVENT_PROCESSING_READY (1 << 2)  // Processing pipeline ready
#define SYSTEM_EVENT_OUTPUT_READY     (1 << 3)  // Output systems ready
#define SYSTEM_EVENT_BLE_READY        (1 << 4)  // BLE communication ready
#define SYSTEM_EVENT_POWER_READY      (1 << 5)  // Power management ready
#define SYSTEM_EVENT_MONITOR_READY    (1 << 6)  // NEW - System monitor ready
#define SYSTEM_EVENT_LOW_BATTERY      (1 << 7)  // Low battery condition
#define SYSTEM_EVENT_ERROR            (1 << 8)  // System error occurred
#define SYSTEM_EVENT_CALIBRATION      (1 << 9)  // Calibration in progress
#define SYSTEM_EVENT_RECOVERY_MODE    (1 << 10) // NEW - System recovery active

typedef enum {
    SYS_CMD_CHANGE_STATE,           // Change system state
    SYS_CMD_ENABLE_FEATURE,         // Enable a feature
    SYS_CMD_DISABLE_FEATURE,        // Disable a feature
    SYS_CMD_CALIBRATE,              // Start sensor calibration
    SYS_CMD_SAVE_CONFIG,            // Save configuration to NVS
    SYS_CMD_LOAD_CONFIG,            // Load configuration from NVS
    SYS_CMD_FACTORY_RESET,          // Factory reset
    SYS_CMD_SET_POWER_MODE,         // Set power mode
    SYS_CMD_RESTART,                // System restart
    SYS_CMD_SLEEP,                  // Enter sleep mode
    SYS_CMD_RESTART_TASK,           // Restart a specific task
    SYS_CMD_EMERGENCY_SHUTDOWN,     // Emergency system shutdown
    SYS_CMD_ENTER_RECOVERY,         // Enter recovery mode
    SYS_CMD_HEALTH_CHECK,           // Perform health check
    SYS_CMD_PERFORMANCE_BOOST,      // Boost system performance
    SYS_CMD_MAX
} system_command_type_t;

typedef struct {
    system_command_type_t type;     // Command type
    uint32_t parameter;             // Command parameter
    void* data;                     // Command data pointer
    uint32_t timestamp;             // Command timestamp
    TaskHandle_t source_task;       // Source task handle
} system_command_t;

/* System states */
typedef enum {
    SYSTEM_STATE_INIT,
    SYSTEM_STATE_IDLE,
    SYSTEM_STATE_ACTIVE,
    SYSTEM_STATE_STANDBY,
    SYSTEM_STATE_SLEEP,
    SYSTEM_STATE_CHARGING,
    SYSTEM_STATE_LOW_BATTERY,
    SYSTEM_STATE_ERROR,
    SYSTEM_STATE_CALIBRATION,
    SYSTEM_STATE_RECOVERY,         
    SYSTEM_STATE_MAX
} system_state_t;

/* ===== TASK SYNCHRONIZATION STRUCTURES ===== */
typedef enum {
    SYNC_EVENT_SENSOR_DATA,         // New sensor data available
    SYNC_EVENT_PROCESSING_DONE,     // Processing completed
    SYNC_EVENT_OUTPUT_REQUEST,      // Output requested
    SYNC_EVENT_POWER_CHANGE,        // Power mode changed
    SYNC_EVENT_ERROR_OCCURRED,      // Error occurred
    SYNC_EVENT_RECOVERY_NEEDED,     // Recovery action needed
    SYNC_EVENT_MAX
} system_sync_event_t;

/* ===== ERROR HANDLING CONFIGURATION ===== */
#define ERROR_RETRY_COUNT             (3)     // NEW - Default error retry count
#define ERROR_RETRY_DELAY_MS          (100)   // NEW - Delay between error retries
#define CRITICAL_ERROR_RESTART        (true)  // NEW - Restart system on critical errors
#define ERROR_LOG_BUFFER_SIZE         (10)    // NEW - Error log buffer size

/* ===== PERFORMANCE MONITORING ===== */
#define PERFORMANCE_MONITORING        (true)  // NEW - Enable performance monitoring
#define CPU_USAGE_CALCULATION_ENABLED (true)  // NEW - Enable CPU usage calculation
#define MEMORY_LEAK_DETECTION         (true)  // NEW - Enable memory leak detection
#define TASK_RUNTIME_MONITORING       (true)  // NEW - Monitor individual task runtimes

/* ===== DEBUGGING AND DEVELOPMENT ===== */
#define TASK_STACK_OVERFLOW_CHECK     (true)  // NEW - Enable stack overflow checking
#define TASK_PRIORITY_INHERITANCE     (true)  // NEW - Enable priority inheritance
#define DEADLOCK_DETECTION            (true)  // NEW - Enable deadlock detection
#define RESOURCE_USAGE_TRACKING       (true)  // NEW - Track resource usage

/* Error codes */
typedef enum {
    SYSTEM_ERROR_NONE,
    SYSTEM_ERROR_FLEX_SENSOR,
    SYSTEM_ERROR_IMU,
    SYSTEM_ERROR_CAMERA,
    SYSTEM_ERROR_DISPLAY,
    SYSTEM_ERROR_AUDIO,
    SYSTEM_ERROR_BLUETOOTH,
    SYSTEM_ERROR_MEMORY,
    SYSTEM_ERROR_BATTERY,
    SYSTEM_ERROR_UNKNOWN
} system_error_t;

/* Output modes */
typedef enum {
    OUTPUT_MODE_TEXT_ONLY,
    OUTPUT_MODE_AUDIO_ONLY,
    OUTPUT_MODE_TEXT_AND_AUDIO,
    OUTPUT_MODE_MINIMAL
} output_mode_t;

/**
 * @brief System configuration structure
 */
typedef struct {
    // Current system state
    system_state_t system_state;
    
    // Feature enable flags
    bool power_save_enabled;
    bool gesture_recognition_enabled;
    bool audio_feedback_enabled;
    bool haptic_feedback_enabled;
    bool ble_enabled;
    bool camera_enabled;
    
    // Performance settings
    uint8_t performance_mode;
    uint8_t sensor_sensitivity;
    uint8_t processing_quality;
    
    // User preferences
    uint8_t volume_level;
    uint8_t brightness_level;
    uint8_t haptic_intensity;
    
    // Calibration data
    bool sensors_calibrated;
    bool imu_calibrated;
    bool touch_calibrated;
    
    // System health
    bool system_healthy;
    uint32_t last_health_check;
    uint32_t error_count;
    uint32_t recovery_count;
    
    // Task monitoring
    bool task_monitoring_enabled;
    uint32_t critical_task_failures;
    uint32_t task_restarts;
    
} system_config_t;

typedef struct {
    TaskHandle_t task_handle;       // Task handle
    char task_name[16];             // Task name
    uint32_t stack_free;            // Free stack bytes
    uint32_t last_runtime;          // Last runtime measurement
    uint32_t total_runtime;         // Total runtime since start
    bool is_healthy;                // Task health status
    uint32_t error_count;           // Task error count
    uint32_t restart_count;         // Task restart count
} task_health_info_t;


/* Global configuration variable (declared in app_main.c) */
extern system_config_t g_system_config;

/* Function declarations */
esp_err_t system_config_init(void);
esp_err_t system_config_save(void);
esp_err_t system_config_load(void);

#endif /* SYSTEM_CONFIG_H */

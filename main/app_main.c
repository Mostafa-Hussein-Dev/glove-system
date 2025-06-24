#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "esp_system.h"
#include "esp_mac.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "nvs_flash.h"
#include "esp_spiffs.h"
#include "driver/i2c_master.h"
#include "esp_chip_info.h"   
#include "esp_flash.h"
#include "freertos/semphr.h"
#include "ml/model_manager.h"
#include "ml/ml_inference.h"
#include "ml/data_preprocessor.h"

// Include app_main header first
#include "app_main.h"

// Include all subsystems
#include "config/system_config.h"
#include "config/pin_definitions.h"
#include "core/power_management.h"
#include "core/system_monitor.h"
#include "drivers/flex_sensor.h"
#include "drivers/imu.h"
#include "drivers/ble_camera.h"
#include "drivers/touch.h"
#include "drivers/display.h"
#include "drivers/audio.h"
#include "drivers/haptic.h"
#include "processing/sensor_fusion.h"
#include "processing/feature_extraction.h"
#include "processing/gesture_detection.h"
#include "communication/ble_service.h"
#include "output/text_generation.h"
#include "output/output_manager.h"
#include "tasks/sensor_task.h"
#include "tasks/processing_task.h"
#include "tasks/output_task.h"
#include "tasks/communication_task.h"
#include "tasks/power_task.h"
#include "util/debug.h"
#include "util/buffer.h"

static const char *TAG = "APP_MAIN";

// You can change this to true to enable debug mode
#define DEBUG_MODE_ENABLED true
#define DEBUG_MODE_TIME_INTERVAL 5000

// Global I2C master bus handle (defined here, declared in util/i2c_utils.h)
i2c_master_bus_handle_t i2c_master_bus = NULL;

// Global I2C mutex
SemaphoreHandle_t g_i2c_mutex = NULL;

// Global system configuration
system_config_t g_system_config;

// Global queue handlers
QueueHandle_t g_sensor_data_queue;
QueueHandle_t g_processing_result_queue;
QueueHandle_t g_output_command_queue;
QueueHandle_t g_system_command_queue;

// Event group for system synchronization
EventGroupHandle_t g_system_event_group;

// Forward declarations for initialization functions
static esp_err_t init_nvs(void);
static esp_err_t init_spiffs(void);
static esp_err_t init_i2c(void);
static esp_err_t init_system_config(void);
static esp_err_t init_drivers(void);
static esp_err_t init_processing(void);
static esp_err_t init_communication(void);
static esp_err_t init_output(void);
static esp_err_t init_queues(void);
static esp_err_t init_tasks(void);
static esp_err_t register_critical_tasks(void);

// Debug mode functions
static void debug_mode_run(void);
static void debug_test_flex_sensors(void);
static void debug_test_imu(void);
static void debug_test_ble_camera(void);
static void debug_test_touch_sensors(void);
static void debug_test_display(void);
static void debug_test_audio(void);
static void debug_test_haptic(void);
static void debug_test_power_system(void);
static void debug_test_enhanced_monitor(void);
static void debug_display_system_info(void);

/**
 * @brief Initialize the application
 * 
 * This function initializes all subsystems, creates queues and tasks
 * and starts the system.
 * 
 * @return ESP_OK on success, or an error code on failure
 */
esp_err_t app_init(void) {
    esp_err_t ret;
    
    // Create system event group
    g_system_event_group = xEventGroupCreate();
    if (g_system_event_group == NULL) {
        ESP_LOGE(TAG, "Failed to create system event group");
        return ESP_FAIL;
    }
    
    // Initialize NVS
    ret = init_nvs();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize NVS: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Initialize SPIFFS
    ret = init_spiffs();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SPIFFS: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Initialize I2C bus (shared between multiple devices)
    ret = init_i2c();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize I2C: %s", esp_err_to_name(ret));
        return ret;
    }

    // Initialize drivers
    ret = init_drivers();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize drivers: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Initialize system configuration
    ret = init_system_config();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize system config: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Initialize debug subsystem
    ret = debug_init(DEBUG_LEVEL_INFO, DEBUG_MODE_UART | DEBUG_MODE_DISPLAY);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize debug subsystem: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Initialize interprocess queues
    ret = init_queues();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize queues: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Initialize processing modules
    ret = init_processing();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize processing: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Initialize communication
    ret = init_communication();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize communication: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Initialize output systems
    ret = init_output();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize output: %s", esp_err_to_name(ret));
        return ret;
    }

    // Initialize system tasks (only if not in debug mode)
    if (!DEBUG_MODE_ENABLED) {
        ret = init_tasks();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to initialize tasks: %s", esp_err_to_name(ret));
            return ret;
        }
    }
    
    
    // Set system initialization complete
    xEventGroupSetBits(g_system_event_group, SYSTEM_EVENT_INIT_COMPLETE);
    
    ESP_LOGI(TAG, "=== Sign Language Glove Ready ===");
   return ESP_OK;
}

static esp_err_t init_nvs(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // NVS partition was truncated and needs to be erased
        ESP_LOGW(TAG, "Erasing NVS partition...");
        ret = nvs_flash_erase();
        if (ret != ESP_OK) {
            return ret;
        }
        // Retry initialization
        ret = nvs_flash_init();
    }
    
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "NVS initialized successfully");
    }
    
    return ret;
}

static esp_err_t init_spiffs(void) {
    esp_vfs_spiffs_conf_t conf = {
        .base_path = "/spiffs",
        .partition_label = NULL,
        .max_files = 5,
        .format_if_mount_failed = true
    };
    
    esp_err_t ret = esp_vfs_spiffs_register(&conf);
    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount or format SPIFFS");
        } else if (ret == ESP_ERR_NOT_FOUND) {
            ESP_LOGE(TAG, "Failed to find SPIFFS partition");
        } else {
            ESP_LOGE(TAG, "Failed to initialize SPIFFS (%s)", esp_err_to_name(ret));
        }
        return ret;
    }
    
    size_t total = 0, used = 0;
    ret = esp_spiffs_info(NULL, &total, &used);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get SPIFFS partition information (%s)", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "SPIFFS partition size: total: %d, used: %d", total, used);
    }
    
    return ESP_OK;
}

static esp_err_t init_i2c(void) {
    esp_err_t ret;
    
    // Create I2C mutex FIRST
    g_i2c_mutex = xSemaphoreCreateMutex();
    if (g_i2c_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create I2C mutex");
        return ESP_ERR_NO_MEM;
    }

    // Configure I2C master bus (ONLY ONCE HERE)
    i2c_master_bus_config_t i2c_mst_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_MASTER_NUM,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,

    };
    
    ret = i2c_new_master_bus(&i2c_mst_config, &i2c_master_bus);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create I2C master bus: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "I2C master bus initialized successfully");
    vTaskDelay(pdMS_TO_TICKS(200)); 
    return ESP_OK;
}

static esp_err_t init_system_config(void) {
    // Initialize default system configuration
    g_system_config.system_state = SYSTEM_STATE_INIT;
    
    // Feature enable flags
    g_system_config.power_save_enabled = true;
    g_system_config.gesture_recognition_enabled = true;
    g_system_config.audio_feedback_enabled = true;
    g_system_config.haptic_feedback_enabled = true;
    g_system_config.ble_enabled = true;
    g_system_config.camera_enabled = true;
    
    // Performance settings
    g_system_config.performance_mode = 1;      // 0=Power Save, 1=Balanced, 2=Performance
    g_system_config.sensor_sensitivity = 80;   // 0-100%
    g_system_config.processing_quality = 85;   // 0-100%
    
    // User preferences
    g_system_config.volume_level = 80;         // 0-100%
    g_system_config.brightness_level = 100;    // 0-100%
    g_system_config.haptic_intensity = 80;     // 0-100%
    
    // Calibration status
    g_system_config.sensors_calibrated = false;
    g_system_config.imu_calibrated = false;
    g_system_config.touch_calibrated = false;
    
    // System health initialization
    g_system_config.system_healthy = true;
    g_system_config.last_health_check = 0;
    g_system_config.error_count = 0;
    g_system_config.recovery_count = 0;
    
    // Task monitoring initialization
    g_system_config.task_monitoring_enabled = true;
    g_system_config.critical_task_failures = 0;
    g_system_config.task_restarts = 0;
    
    // Load configuration from NVS if available
    ESP_LOGI(TAG, "Enhanced configuration defaults set:");
    ESP_LOGI(TAG, "  Performance mode: %d, Sensor sensitivity: %d%%", 
        g_system_config.performance_mode, g_system_config.sensor_sensitivity);
    ESP_LOGI(TAG, "  Features enabled: Gesture=%s, Audio=%s, Haptic=%s, BLE=%s", 
        g_system_config.gesture_recognition_enabled ? "Y" : "N",
        g_system_config.audio_feedback_enabled ? "Y" : "N",
        g_system_config.haptic_feedback_enabled ? "Y" : "N",
        g_system_config.ble_enabled ? "Y" : "N");
    ESP_LOGI(TAG, "  Task monitoring: %s", 
        g_system_config.task_monitoring_enabled ? "ENABLED" : "DISABLED");
    
    // Load configuration from NVS if available
    esp_err_t ret = system_config_load();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to load system configuration from NVS, using enhanced defaults");
        
        // If loading failed, save the enhanced default configuration
        ret = system_config_save();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to save enhanced default configuration: %s", esp_err_to_name(ret));
            return ret;
        } else {
            ESP_LOGI(TAG, "Enhanced default configuration saved to NVS");
        }
    } else {
        ESP_LOGI(TAG, "Enhanced system configuration loaded from NVS");
        
        // Validate loaded configuration
        if (g_system_config.performance_mode > 2) {
            ESP_LOGW(TAG, "Invalid performance mode %d, resetting to 1", g_system_config.performance_mode);
            g_system_config.performance_mode = 1;
        }
        
        if (g_system_config.sensor_sensitivity > 100) {
            ESP_LOGW(TAG, "Invalid sensor sensitivity %d, resetting to 80", g_system_config.sensor_sensitivity);
            g_system_config.sensor_sensitivity = 80;
        }
        
        // Ensure task monitoring is enabled for enhanced architecture
        if (!g_system_config.task_monitoring_enabled) {
            ESP_LOGI(TAG, "Enabling task monitoring for enhanced architecture");
            g_system_config.task_monitoring_enabled = true;
        }
    }
    
    // Log final configuration
    ESP_LOGI(TAG, "=== ENHANCED SYSTEM CONFIGURATION READY ===");
    ESP_LOGI(TAG, "State: %d, Health: %s, Monitoring: %s", 
        g_system_config.system_state,
        g_system_config.system_healthy ? "OK" : "UNHEALTHY",
        g_system_config.task_monitoring_enabled ? "ON" : "OFF");
    
    return ESP_OK;
}

static esp_err_t init_drivers(void) {
    esp_err_t ret;

    // Initialize display first (it will use the existing I2C bus)
    ret = display_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize display: %s", esp_err_to_name(ret));
        return ret;
    }
    
    
    // Initialize flex sensors
    ret = flex_sensor_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize flex sensors: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI("MAIN", "Testing flex sensor calibration...");
    for (int i = 0; i < FINGER_COUNT; i++) {
        flex_sensor_test_calibration_math(i);
    }
    
    
    // Initialize IMU (it will use the existing I2C bus)
    ret = imu_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize IMU: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Initialize touch sensors
    ret = touch_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize touch sensors: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Initialize audio
    ret = audio_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize audio: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Initialize haptic feedback
    ret = haptic_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize haptic feedback: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Initialize camera (if enabled)
    ret = ble_camera_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize BLE camera: %s", esp_err_to_name(ret));
        return ret;
    }

    //Connecting to camera
    ESP_LOGI(TAG, "Attempting to connect to ESP32-CAM...");
    ret = ble_camera_connect(DEVICE_NAME);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Initial camera connection failed, will retry automatically");
        // Don't return error - camera connection can be established later
    }
    
    // Initialize power management
    if (!DEBUG_MODE_ENABLED) {
        ret = power_management_init();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to initialize power management: %s", esp_err_to_name(ret));
            return ret;
        }
    }

    // Initialize system monitor
    ret = system_monitor_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize system monitor: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "All drivers initialized successfully");
    return ESP_OK;
    
}

static esp_err_t init_processing(void) {
    esp_err_t ret;

    // Initialize ML components
    ret = model_manager_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize model manager: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = ml_inference_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize ML inference: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = data_preprocessor_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize data preprocessor: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "Loading ML models...");
    esp_err_t model_ret1 = model_manager_load_model("static_cnn.tflite", MODEL_TYPE_STATIC_CNN);
    esp_err_t model_ret2 = model_manager_load_model("dynamic_lstm.tflite", MODEL_TYPE_DYNAMIC_LSTM);
    
    if (model_ret1 != ESP_OK && model_ret2 != ESP_OK) {
        ESP_LOGW(TAG, "No ML models loaded - using template-based detection only");
    } else {
        ESP_LOGI(TAG, "ML models loaded successfully");
    }
    
    ESP_LOGI(TAG, "Processing modules initialized successfully");
    return ESP_OK;
}

static esp_err_t init_communication(void) {
    esp_err_t ret;
    
    // Initialize BLE service if enabled
    if (g_system_config.ble_enabled) {
        ret = ble_service_init();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to initialize BLE service: %s", esp_err_to_name(ret));
            return ret;
        }
    }
    
    ESP_LOGI(TAG, "Communication modules initialized successfully");
    return ESP_OK;
}

static esp_err_t init_output(void) {
    esp_err_t ret;
    
    // Initialize text generation
    ret = text_generation_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize text generation: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Initialize output manager
    ret = output_manager_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize output manager: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "Output modules initialized successfully");
    return ESP_OK;
}

static esp_err_t init_queues(void) {
    ESP_LOGI(TAG, "=== INITIALIZING ENHANCED QUEUE SYSTEM ===");
    ESP_LOGI(TAG, "Queue Configuration (with overflow protection):");
    
    // Create sensor data queue with enhanced size
    g_sensor_data_queue = xQueueCreate(SENSOR_QUEUE_SIZE, sizeof(sensor_data_t));
    if (g_sensor_data_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create sensor data queue");
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "  Sensor queue: %d items (%d bytes)", 
        SENSOR_QUEUE_SIZE, SENSOR_QUEUE_SIZE * sizeof(sensor_data_t));
    
    // Create processing result queue with enhanced size
    g_processing_result_queue = xQueueCreate(PROCESSING_QUEUE_SIZE, sizeof(processing_result_t));
    if (g_processing_result_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create processing result queue");
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "  Processing queue: %d items (%d bytes)", 
        PROCESSING_QUEUE_SIZE, PROCESSING_QUEUE_SIZE * sizeof(processing_result_t));
    
    // Create output command queue with enhanced size
    g_output_command_queue = xQueueCreate(OUTPUT_QUEUE_SIZE, sizeof(output_command_t));
    if (g_output_command_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create output command queue");
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "  Output queue: %d items (%d bytes)", 
        OUTPUT_QUEUE_SIZE, OUTPUT_QUEUE_SIZE * sizeof(output_command_t));
    
    // Create system command queue with enhanced size
    g_system_command_queue = xQueueCreate(COMMAND_QUEUE_SIZE, sizeof(system_command_t));
    if (g_system_command_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create system command queue");
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "  System queue: %d items (%d bytes)", 
        COMMAND_QUEUE_SIZE, COMMAND_QUEUE_SIZE * sizeof(system_command_t));
    
    // Calculate total memory usage
    uint32_t total_queue_memory = 
        (SENSOR_QUEUE_SIZE * sizeof(sensor_data_t)) +
        (PROCESSING_QUEUE_SIZE * sizeof(processing_result_t)) +
        (OUTPUT_QUEUE_SIZE * sizeof(output_command_t)) +
        (COMMAND_QUEUE_SIZE * sizeof(system_command_t));
    
    ESP_LOGI(TAG, "=== ENHANCED QUEUE SYSTEM READY ===");
    ESP_LOGI(TAG, "Total queue memory: %u bytes (%.1f KB)", 
        total_queue_memory, total_queue_memory / 1024.0f);
    ESP_LOGI(TAG, "Queue overflow protection: %s", 
        QUEUE_OVERFLOW_PROTECTION ? "ENABLED" : "DISABLED");
    ESP_LOGI(TAG, "High watermark threshold: %d%%", QUEUE_HIGH_WATERMARK_PERCENT);
    
    return ESP_OK;
}

static esp_err_t init_tasks(void) {
    ESP_LOGI(TAG, "=== INITIALIZING ENHANCED TASK ARCHITECTURE ===");
    ESP_LOGI(TAG, "Task Configuration:");
    ESP_LOGI(TAG, "  Core 0: Sensor (%d), Monitor (%d), Output (%d), Power (%d)", 
        SENSOR_TASK_PRIORITY, SYSTEM_MONITOR_TASK_PRIORITY, 
        OUTPUT_TASK_PRIORITY, POWER_TASK_PRIORITY);
    ESP_LOGI(TAG, "  Core 1: Processing (%d), Communication (%d)", 
        PROCESSING_TASK_PRIORITY, COMMUNICATION_TASK_PRIORITY);
    
    // === PHASE 1: INITIALIZE SYSTEM MONITOR FIRST ===
    ESP_LOGI(TAG, "Phase 1: Initializing system monitor...");
    esp_err_t ret = system_monitor_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize system monitor: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Wait for monitor to be ready
    if (g_system_event_group != NULL) {
        EventBits_t bits = xEventGroupWaitBits(
            g_system_event_group,
            SYSTEM_EVENT_MONITOR_READY,
            pdFALSE,
            pdTRUE,
            pdMS_TO_TICKS(1000)
        );
        
        if (!(bits & SYSTEM_EVENT_MONITOR_READY)) {
            ESP_LOGW(TAG, "System monitor ready timeout");
        }
    }
    
    // === PHASE 2: INITIALIZE CORE 0 TASKS (Real-time + Control) ===
    ESP_LOGI(TAG, "Phase 2: Initializing Core 0 tasks (Real-time + Control)...");
    
    // Initialize sensor task (highest priority)
    ret = sensor_task_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize sensor task: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Register sensor task with monitor
    void* sensor_handle = sensor_task_get_handle();
    if (sensor_handle != NULL) {
        system_monitor_register_task((TaskHandle_t)sensor_handle, "sensor_task");
    }
    
    // Initialize output task 
    ret = output_task_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize output task: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Register output task with monitor
    void* output_handle = output_task_get_handle();
    if (output_handle != NULL) {
        system_monitor_register_task((TaskHandle_t)output_handle, "output_task");
    }
    
    // Initialize power task (lowest priority on Core 0)
    ret = power_task_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize power task: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Register power task with monitor
    void* power_handle = power_task_get_handle();
    if (power_handle != NULL) {
        system_monitor_register_task((TaskHandle_t)power_handle, "power_task");
    }
    
    ESP_LOGI(TAG, "Core 0 tasks initialized successfully");
    
    // === PHASE 3: INITIALIZE CORE 1 TASKS (Processing + Communication) ===
    ESP_LOGI(TAG, "Phase 3: Initializing Core 1 tasks (Processing + Communication)...");
    
    // Initialize processing task (highest priority on Core 1)
    ret = processing_task_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize processing task: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Register processing task with monitor
    void* processing_handle = processing_task_get_handle();
    if (processing_handle != NULL) {
        system_monitor_register_task((TaskHandle_t)processing_handle, "processing_task");
    }
    
    // Initialize communication task
    ret = communication_task_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize communication task: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Register communication task with monitor
    void* comm_handle = communication_task_get_handle();
    if (comm_handle != NULL) {
        system_monitor_register_task((TaskHandle_t)comm_handle, "comm_task");
    }
    
    ESP_LOGI(TAG, "Core 1 tasks initialized successfully");
    
    // === PHASE 4: ENABLE ENHANCED MONITORING ===
    ESP_LOGI(TAG, "Phase 4: Enabling enhanced monitoring features...");
    
    // Enable adaptive monitoring intervals
    system_monitor_set_adaptive_intervals(true);
    
    // Set initial queue monitoring
    if (g_sensor_data_queue != NULL) {
        ESP_LOGI(TAG, "Sensor queue configured: %d items", SENSOR_QUEUE_SIZE);
    }
    if (g_processing_result_queue != NULL) {
        ESP_LOGI(TAG, "Processing queue configured: %d items", PROCESSING_QUEUE_SIZE);
    }
    if (g_output_command_queue != NULL) {
        ESP_LOGI(TAG, "Output queue configured: %d items", OUTPUT_QUEUE_SIZE);
    }
    if (g_system_command_queue != NULL) {
        ESP_LOGI(TAG, "System queue configured: %d items", COMMAND_QUEUE_SIZE);
    }
    
    // === PHASE 5: VERIFY TASK ARCHITECTURE ===
    ESP_LOGI(TAG, "Phase 5: Verifying task architecture...");
    
    // Check that all critical tasks are registered
    system_metrics_t initial_metrics;
    ret = system_monitor_get_metrics(&initial_metrics);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Architecture verification:");
        ESP_LOGI(TAG, "  Total tasks: %u", initial_metrics.task_count);
        ESP_LOGI(TAG, "  Critical tasks monitored: %u", initial_metrics.critical_task_count);
        ESP_LOGI(TAG, "  Free heap: %u bytes", initial_metrics.free_heap);
        ESP_LOGI(TAG, "  System health: %s", 
            (initial_metrics.health_level == SYSTEM_HEALTH_OK) ? "OK" :
            (initial_metrics.health_level == SYSTEM_HEALTH_WARNING) ? "WARNING" : "CRITICAL");
    }
    
    // === PHASE 6: SET SYSTEM EVENTS ===
    if (g_system_event_group != NULL) {
        xEventGroupSetBits(g_system_event_group, 
            SYSTEM_EVENT_SENSOR_READY | 
            SYSTEM_EVENT_PROCESSING_READY | 
            SYSTEM_EVENT_OUTPUT_READY |
            SYSTEM_EVENT_POWER_READY);
    }
    
    ESP_LOGI(TAG, "=== ENHANCED TASK ARCHITECTURE INITIALIZED SUCCESSFULLY ===");
    ESP_LOGI(TAG, "Architecture Summary:");
    ESP_LOGI(TAG, "  Core 0 Load: Sensor + Monitor + Output + Power");
    ESP_LOGI(TAG, "  Core 1 Load: Processing + Communication");
    ESP_LOGI(TAG, "  Total Stack Allocated: ~%d KB", 
        (SENSOR_TASK_STACK_SIZE + SYSTEM_MONITOR_TASK_STACK + OUTPUT_TASK_STACK_SIZE + 
         POWER_TASK_STACK_SIZE + PROCESSING_TASK_STACK_SIZE + COMMUNICATION_TASK_STACK_SIZE) / 1024);
    
    return ESP_OK;
}

static esp_err_t register_critical_tasks(void) {
    ESP_LOGI(TAG, "Registering critical tasks with system monitor");
    
    // Register all critical tasks for monitoring
    // Note: Task handles should be made available from respective task modules
    
    // System monitor itself is automatically monitored
    
    // Register sensor task if available
    void* sensor_task_handle = sensor_task_get_handle();  // You'll need to add this function
    if (sensor_task_handle != NULL) {
        system_monitor_register_task((TaskHandle_t)sensor_task_handle, "sensor_task");
    }
    
    // Register processing task if available  
    void* processing_task_handle = processing_task_get_handle();  // You'll need to add this function
    if (processing_task_handle != NULL) {
        system_monitor_register_task((TaskHandle_t)processing_task_handle, "proc_task");
    }
    
    // Register output task if available
    void* output_task_handle = output_task_get_handle();  // You'll need to add this function
    if (output_task_handle != NULL) {
        system_monitor_register_task((TaskHandle_t)output_task_handle, "output_task");
    }
    
    // Register communication task if available
    void* comm_task_handle = communication_task_get_handle();  // You'll need to add this function
    if (comm_task_handle != NULL) {
        system_monitor_register_task((TaskHandle_t)comm_task_handle, "comm_task");
    }
    
    // Register power task if available
    void* power_task_handle = power_task_get_handle();  // You'll need to add this function
    if (power_task_handle != NULL) {
        system_monitor_register_task((TaskHandle_t)power_task_handle, "power_task");
    }
    
    // Enable adaptive monitoring intervals for better responsiveness
    system_monitor_set_adaptive_intervals(true);
    
    ESP_LOGI(TAG, "Critical task registration completed");
    return ESP_OK;
}


static void debug_mode_run(void) {
    ESP_LOGI(TAG, "=== STARTING DEBUG MODE ===");
    
    // Display system information
    //debug_display_system_info();
    
    uint32_t loop_count = 0;
    
    while (1) {
        
        ESP_LOGI("INIT", "\033[2J\033[H"); 
        ESP_LOGI(TAG, "=== DEBUG LOOP %lu ===\n", loop_count++);
        
        // Test all sensors every loop
        ESP_LOGI(TAG, "Testing Flex Sensors...");
        debug_test_flex_sensors();
        
        ESP_LOGI(TAG, "Testing IMU...");
        debug_test_imu();
        
        ESP_LOGI(TAG, "Testing Touch Sensors...");
        debug_test_touch_sensors();

        ESP_LOGI(TAG, "Testing System Monitor...");
        debug_test_enhanced_monitor();

        ESP_LOGI(TAG, "Testing System Monitor Health Check...");
        system_monitor_health_check();

        ESP_LOGI(TAG, "Testing System Monitor Metrics...");
        system_monitor_print_metrics();

        // Test output devices every 10 seconds
        /*
        if (current_time - last_full_test_time > 10000) {
            ESP_LOGI(TAG, "Testing Display...");
            debug_test_display();
            
            ESP_LOGI(TAG, "Testing Audio...");
            debug_test_audio();
            
            ESP_LOGI(TAG, "Testing Haptic...");
            debug_test_haptic();

            ESP_LOGI(TAG, "Testing ML Status...");
            debug_ml_status();

            ESP_LOGI(TAG, "Testing ML Inference Status...");
            debug_ml_inference_test();

            
            
            // Test camera less frequently (every 20 seconds) due to performance
            static uint32_t last_camera_test = 0;
            if (current_time - last_camera_test > 20000) {
                ESP_LOGI(TAG, "Testing Camera...");
                debug_test_ble_camera();
                last_camera_test = current_time;
            }
            
           
            last_full_test_time = current_time;
            
        }
        
        */   
        
        ESP_LOGI(TAG, "=== DEBUG LOOP %lu COMPLETE ===\n", loop_count - 1);
        
        vTaskDelay(pdMS_TO_TICKS(DEBUG_MODE_TIME_INTERVAL));
    }
}

static void debug_test_flex_sensors(void) {
    uint16_t raw_values[5];
    float angles[5];
    
    esp_err_t ret = flex_sensor_read_raw(raw_values);
    if (ret == ESP_OK) {
        ret = flex_sensor_read_angles(angles);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "✓ Flex Sensors OK:");
            ESP_LOGI(TAG, "  Thumb:  Raw=%4d, Angle=%.1f°", raw_values[0], angles[0]);
            ESP_LOGI(TAG, "  Index:  Raw=%4d, Angle=%.1f°", raw_values[1], angles[1]);
            ESP_LOGI(TAG, "  Middle: Raw=%4d, Angle=%.1f°", raw_values[2], angles[2]);
            ESP_LOGI(TAG, "  Ring:   Raw=%4d, Angle=%.1f°", raw_values[3], angles[3]);
            ESP_LOGI(TAG, "  Pinky:  Raw=%4d, Angle=%.1f°", raw_values[4], angles[4]);

            /*
            ESP_LOGI(TAG, "Testing calibration math:");
            for (int i = 0; i < FINGER_COUNT; i++) {
                flex_sensor_test_calibration_math(i);
            }
            */
        } else {
            ESP_LOGE(TAG, "✗ Flex Sensors: Failed to read angles: %s", esp_err_to_name(ret));
        }
    } else {
        ESP_LOGE(TAG, "✗ Flex Sensors: Failed to read raw values: %s", esp_err_to_name(ret));
    }
}

static void debug_test_imu(void) {
    imu_data_t imu_data;
    
    esp_err_t ret = imu_read(&imu_data);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "✓ IMU OK:");
        ESP_LOGI(TAG, "  Accel: X=%.2f, Y=%.2f, Z=%.2f (m/s²)", 
                 imu_data.accel[0], imu_data.accel[1], imu_data.accel[2]);
        ESP_LOGI(TAG, "  Gyro:  X=%.2f, Y=%.2f, Z=%.2f (°/s)", 
                 imu_data.gyro[0], imu_data.gyro[1], imu_data.gyro[2]);
        ESP_LOGI(TAG, "  Orient: Roll=%.1f, Pitch=%.1f, Yaw=%.1f (°)", 
                 imu_data.orientation[0], imu_data.orientation[1], imu_data.orientation[2]);
        ESP_LOGI(TAG, "  Temp: %.1f°C", imu_data.temp);
    } else {
        ESP_LOGE(TAG, "✗ IMU: Failed to read data: %s", esp_err_to_name(ret));
    }
}

static void debug_test_ble_camera(void) {
    ESP_LOGI(TAG, "--- BLE Camera Test ---");
    
    // Initialize BLE camera
    esp_err_t ret = ble_camera_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "✗ BLE Camera: Failed to initialize: %s", esp_err_to_name(ret));
        return;
    }
    
    ESP_LOGI(TAG, "✓ BLE Camera: Initialized successfully");
    
    // Attempt to connect
    ESP_LOGI(TAG, "Attempting to connect to ESP32 CAMERA...");
    ret = ble_camera_connect(DEVICE_NAME);
    debug_camera_state();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "⚠ BLE Camera: Connection failed (camera may not be available)");
        return;
    }
    
    // Wait for connection
    int timeout = 10; // 10 seconds timeout
    ble_camera_status_t status;
    while (timeout > 0) {
        ret = ble_camera_get_status(&status);
        if (ret == ESP_OK && status == BLE_CAMERA_STATUS_CONNECTED) {
            ESP_LOGI(TAG, "✓ BLE Camera: Connected successfully");
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
        timeout--;
    }
    
    if (timeout == 0) {
        ESP_LOGW(TAG, "⚠ BLE Camera: Connection timeout");
        return;
    }
    
    // Test frame capture
    ble_camera_frame_t frame;
    ret = ble_camera_capture_frame(&frame);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "✓ BLE Camera: Frame captured successfully");
        ESP_LOGI(TAG, "  Frame: %dx%d, Size=%lu bytes, Format=%d", 
                 frame.width, frame.height, frame.buffer_size, frame.format);
    } else {
        ESP_LOGW(TAG, "⚠ BLE Camera: Frame capture failed: %s", esp_err_to_name(ret));
    }
    
    // Test streaming
    ESP_LOGI(TAG, "Testing streaming mode...");
    ret = ble_camera_start_streaming();
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "✓ BLE Camera: Streaming started");
        
        // Capture a few frames
        for (int i = 0; i < 3; i++) {
            vTaskDelay(pdMS_TO_TICKS(500));
            ret = ble_camera_capture_frame(&frame);
            if (ret == ESP_OK) {
                ESP_LOGI(TAG, "  Stream frame %d: %dx%d, %lu bytes", 
                         i+1, frame.width, frame.height, frame.buffer_size);
            }
        }
        
        ble_camera_stop_streaming();
        ESP_LOGI(TAG, "✓ BLE Camera: Streaming stopped");
    } else {
        ESP_LOGW(TAG, "⚠ BLE Camera: Streaming failed to start");
    }
    
    // Get statistics
    ble_camera_stats_t stats;
    ret = ble_camera_get_stats(&stats);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "✓ BLE Camera Statistics:");
        ESP_LOGI(TAG, "  Frames received: %lu", stats.frames_received);
        ESP_LOGI(TAG, "  Frames dropped: %lu", stats.frames_dropped);
        ESP_LOGI(TAG, "  Bytes received: %lu", stats.bytes_received);
        ESP_LOGI(TAG, "  Frame rate: %.1f fps", stats.frame_rate);
        ESP_LOGI(TAG, "  Signal strength: %d dBm", stats.signal_strength);
    }
    
    // Disconnect
    ble_camera_disconnect();
    ESP_LOGI(TAG, "BLE Camera test completed");
}

static void debug_test_touch_sensors(void) {
    bool touch_status[5];
    uint16_t touch_values[5];
    
    esp_err_t ret = touch_get_status(touch_status);
    if (ret == ESP_OK) {
        ret = touch_get_values(touch_values);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "✓ Touch Sensors OK:");
            ESP_LOGI(TAG, "  Thumb:  %s (Raw=%d)", touch_status[0] ? "TOUCHED" : "FREE   ", touch_values[0]);
            ESP_LOGI(TAG, "  Index:  %s (Raw=%d)", touch_status[1] ? "TOUCHED" : "FREE   ", touch_values[1]);
            ESP_LOGI(TAG, "  Middle: %s (Raw=%d)", touch_status[2] ? "TOUCHED" : "FREE   ", touch_values[2]);
            ESP_LOGI(TAG, "  Ring:   %s (Raw=%d)", touch_status[3] ? "TOUCHED" : "FREE   ", touch_values[3]);
            ESP_LOGI(TAG, "  Pinky:  %s (Raw=%d)", touch_status[4] ? "TOUCHED" : "FREE   ", touch_values[4]);
        } else {
            ESP_LOGE(TAG, "✗ Touch Sensors: Failed to read values: %s", esp_err_to_name(ret));
        }
    } else {
        ESP_LOGE(TAG, "✗ Touch Sensors: Failed to read status: %s", esp_err_to_name(ret));
    }
}

static void debug_test_display(void) {
    esp_err_t ret = display_clear();
    if (ret == ESP_OK) {
        // Draw text to the buffer
        ret = display_draw_text("DEBUG MODE", 0, 0, DISPLAY_FONT_SMALL, DISPLAY_ALIGN_CENTER);
        if (ret == ESP_OK) {
            ret = display_draw_text("All Systems", 0, 16, DISPLAY_FONT_SMALL, DISPLAY_ALIGN_CENTER);
            if (ret == ESP_OK) {
                ret = display_draw_text("Testing...", 0, 32, DISPLAY_FONT_SMALL, DISPLAY_ALIGN_CENTER);
                if (ret == ESP_OK) {
                    // **CRITICAL: Update the display to show the content**
                    ret = display_update();
                    if (ret == ESP_OK) {
                        ESP_LOGI(TAG, "✓ Display OK: Text rendered and updated successfully");
                    } else {
                        ESP_LOGE(TAG, "✗ Display: Failed to update display: %s", esp_err_to_name(ret));
                    }
                } else {
                    ESP_LOGE(TAG, "✗ Display: Failed to draw text line 3: %s", esp_err_to_name(ret));
                }
            } else {
                ESP_LOGE(TAG, "✗ Display: Failed to draw text line 2: %s", esp_err_to_name(ret));
            }
        } else {
            ESP_LOGE(TAG, "✗ Display: Failed to draw text line 1: %s", esp_err_to_name(ret));
        }
    } else {
        ESP_LOGE(TAG, "✗ Display: Failed to clear: %s", esp_err_to_name(ret));
    }
}

static void debug_test_audio(void) {
    // Test a short beep or tone
    esp_err_t ret = audio_play_beep(5000, 100); // 1kHz tone for 100ms
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "✓ Audio OK: Test tone played");
    } else {
        ESP_LOGE(TAG, "✗ Audio: Failed to play test tone: %s", esp_err_to_name(ret));
    }
}

static void debug_test_haptic(void) {
    esp_err_t ret = haptic_vibrate(500); // 100ms pulse at 50% intensity
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "✓ Haptic OK: Test pulse sent");
    } else {
        ESP_LOGE(TAG, "✗ Haptic: Failed to send test pulse: %s", esp_err_to_name(ret));
    }
}

static void debug_display_system_info(void) {
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    
    ESP_LOGI(TAG, "=== SYSTEM INFORMATION ===");
    ESP_LOGI(TAG, "Chip: %s", CONFIG_IDF_TARGET);
    ESP_LOGI(TAG, "Cores: %d", chip_info.cores);
    ESP_LOGI(TAG, "Revision: %d", chip_info.revision);

    uint32_t flash_size = 0;
    esp_err_t ret = esp_flash_get_size(esp_flash_default_chip, &flash_size);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Flash: %luMB %s", flash_size / (1024 * 1024),
                 (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");
    } else {
        ESP_LOGE(TAG, "Failed to get flash size: %s", esp_err_to_name(ret));
    }

    // Display free heap memory
    ESP_LOGI(TAG, "Free heap: %lu bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "Min free heap: %lu bytes", esp_get_minimum_free_heap_size());
    
    // Display system uptime
    int64_t uptime = esp_timer_get_time() / 1000000; // Convert to seconds
    ESP_LOGI(TAG, "Uptime: %lld seconds", uptime);
    
    ESP_LOGI(TAG, "=== HARDWARE CONFIGURATION ===");
    ESP_LOGI(TAG, "Flex Sensors: 5 (GPIO1-5)");
    ESP_LOGI(TAG, "IMU: MPU6050 (I2C)");
    ESP_LOGI(TAG, "Display: OLED SSD1306 (I2C)");
    ESP_LOGI(TAG, "Audio: MAX98357A (I2S)");
    ESP_LOGI(TAG, "Camera: OV2640");
    ESP_LOGI(TAG, "Touch: 5 sensors (GPIO9-13)");
    ESP_LOGI(TAG, "=========================");
}

static void debug_test_enhanced_monitor(void) {
    ESP_LOGI(TAG, "=== ENHANCED SYSTEM MONITOR & TASK ARCHITECTURE TEST ===");
    
    // Test current system status
    system_metrics_t metrics;
    if (system_monitor_get_metrics(&metrics) == ESP_OK) {
        ESP_LOGI(TAG, "Current System Status:");
        ESP_LOGI(TAG, "  Health Level: %s", 
            (metrics.health_level == SYSTEM_HEALTH_OK) ? "🟢 OK" :
            (metrics.health_level == SYSTEM_HEALTH_WARNING) ? "🟡 WARNING" : "🔴 CRITICAL");
        
        ESP_LOGI(TAG, "Memory: %u/%u bytes (%.1f%% free)", 
            metrics.free_heap, metrics.total_heap,
            metrics.total_heap > 0 ? (float)metrics.free_heap / metrics.total_heap * 100 : 0.0f);
            
        ESP_LOGI(TAG, "CPU: %u%% usage, %.1f°C, %u MHz", 
            metrics.cpu_usage_percent, metrics.cpu_temperature, metrics.cpu_frequency);
            
        ESP_LOGI(TAG, "Tasks: %u total, %u critical monitored", 
            metrics.task_count, metrics.critical_task_count);
        
        ESP_LOGI(TAG, "Uptime: %.1f minutes", metrics.uptime_ms / 60000.0f);
        ESP_LOGI(TAG, "Errors: %u, Recoveries: %u", 
            metrics.error_count, metrics.recovery_count);
        
        ESP_LOGI(TAG, "Queues: Max %u%% usage, %u overflows", 
            metrics.queue_usage_max, metrics.queue_overflows);
    }
    
    // Test task architecture
    ESP_LOGI(TAG, "Task Architecture Verification:");
    ESP_LOGI(TAG, "  Core 0 Tasks: Sensor(%d), Monitor(%d), Output(%d), Power(%d)", 
        SENSOR_TASK_PRIORITY, SYSTEM_MONITOR_TASK_PRIORITY, 
        OUTPUT_TASK_PRIORITY, POWER_TASK_PRIORITY);
    ESP_LOGI(TAG, "  Core 1 Tasks: Processing(%d), Communication(%d)", 
        PROCESSING_TASK_PRIORITY, COMMUNICATION_TASK_PRIORITY);
    
    // Test queue health reporting
    ESP_LOGI(TAG, "Testing queue health reporting...");
    system_monitor_update_queue_health(75, false);  // 75% usage
    system_monitor_update_queue_health(95, false);  // High usage warning
    
    // Test recovery system
    ESP_LOGI(TAG, "Testing recovery system...");
    system_monitor_recovery_action(RECOVERY_MEMORY_CLEANUP, NULL);
    
    // Test adaptive intervals
    ESP_LOGI(TAG, "Testing adaptive monitoring...");
    system_health_level_t current_health = system_monitor_get_health_level();
    ESP_LOGI(TAG, "  Current health: %d", current_health);
    
    // Display stack usage for all tasks
    ESP_LOGI(TAG, "📚 Stack Usage Analysis:");
    ESP_LOGI(TAG, "  Sensor Task: %d bytes allocated", SENSOR_TASK_STACK_SIZE);
    ESP_LOGI(TAG, "  Processing Task: %d bytes allocated", PROCESSING_TASK_STACK_SIZE);
    ESP_LOGI(TAG, "  System Monitor: %d bytes allocated", SYSTEM_MONITOR_TASK_STACK);
    ESP_LOGI(TAG, "  Communication Task: %d bytes allocated", COMMUNICATION_TASK_STACK_SIZE);
    ESP_LOGI(TAG, "  Output Task: %d bytes allocated", OUTPUT_TASK_STACK_SIZE);
    ESP_LOGI(TAG, "  Power Task: %d bytes allocated", POWER_TASK_STACK_SIZE);
    
    uint32_t total_stack = SENSOR_TASK_STACK_SIZE + PROCESSING_TASK_STACK_SIZE + 
                          SYSTEM_MONITOR_TASK_STACK + COMMUNICATION_TASK_STACK_SIZE + 
                          OUTPUT_TASK_STACK_SIZE + POWER_TASK_STACK_SIZE;
    ESP_LOGI(TAG, "  Total Stack Allocated: %u bytes (%.1f KB)", total_stack, total_stack / 1024.0f);
    
    vTaskDelay(pdMS_TO_TICKS(1000));
    ESP_LOGI(TAG, "Enhanced system monitor and task architecture test completed");
}

// Main application entry point
void app_main(void) {

    ESP_LOGI(TAG, "=== Sign Language Translation Glove starting ===");
    
    // Initialize the application
    esp_err_t ret = app_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Application initialization failed! Error: %s", esp_err_to_name(ret));
        return;
    }
    
    ESP_LOGI(TAG, "Application initialized successfully, system running");

    // Main application code runs in FreeRTOS tasks
    // No need for further code here as all functionality is handled by tasks
    
    // Check if debug mode is enabled
    if (DEBUG_MODE_ENABLED) {
        ESP_LOGI(TAG, "=== DEBUG MODE ENABLED ===");
        ESP_LOGI(TAG, "Running system diagnostics...");
        debug_mode_run();
    } else {
        ESP_LOGI(TAG, "=== NORMAL MODE ===");
        ESP_LOGI(TAG, "System running with tasks");
        ESP_LOGI(TAG, "Waiting for BLE camera connection...");
        
        // Normal operation - system runs via tasks
        // Main thread can be used for monitoring if needed
        while (1) {
            // Periodic system health check
            ESP_LOGI(TAG, "System running - Free heap: %lu bytes", esp_get_free_heap_size());
            
            // Check BLE camera status (NEW)
            ble_camera_status_t camera_status;
            if (ble_camera_get_status(&camera_status) == ESP_OK) {
                static ble_camera_status_t prev_status = BLE_CAMERA_STATUS_DISCONNECTED;
                if (camera_status != prev_status) {
                    const char* status_str[] = {"DISCONNECTED", "CONNECTING", "CONNECTED", "STREAMING", "ERROR"};
                    ESP_LOGI(TAG, "BLE Camera status: %s", status_str[camera_status]);
                    prev_status = camera_status;
                    
                    // Optional: Log camera statistics when connected
                    if (camera_status == BLE_CAMERA_STATUS_CONNECTED || camera_status == BLE_CAMERA_STATUS_STREAMING) {
                        ble_camera_stats_t stats;
                        if (ble_camera_get_stats(&stats) == ESP_OK) {
                            ESP_LOGI(TAG, "Camera stats: %lu frames, %.1f fps, %d dBm", 
                                     stats.frames_received, stats.frame_rate, stats.signal_strength);
                        }
                    }
                }
            }
            
            // Check battery status periodically (NEW - every 5 cycles = 25 seconds)
            static int battery_check_counter = 0;
            if (++battery_check_counter >= 5) {
                battery_status_t battery;
                if (power_management_get_battery_status(&battery) == ESP_OK) {
                    ESP_LOGI(TAG, "Battery: %.2fV (%d%%) %s", 
                             battery.voltage_mv / 1000.0f, 
                             battery.percentage,
                             battery.is_charging ? "[CHARGING]" : "");
                    
                    if (battery.is_low) {
                        ESP_LOGW(TAG, "⚠️  LOW BATTERY WARNING");
                    }
                    if (battery.is_critical) {
                        ESP_LOGE(TAG, "🔋 CRITICAL BATTERY - SAVE WORK!");
                    }
                }
                battery_check_counter = 0;
            }
            
            vTaskDelay(pdMS_TO_TICKS(5000));  // Check every 5 seconds
        }
    }
}
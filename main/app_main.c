#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "nvs_flash.h"
#include "esp_spiffs.h"
#include "driver/i2c.h"

// Include all subsystems
#include "config/system_config.h"
#include "config/pin_definitions.h"
#include "core/power_management.h"
#include "core/system_monitor.h"
#include "drivers/flex_sensor.h"
#include "drivers/imu.h"
#include "drivers/camera.h"
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
    
    // Initialize drivers
    ret = init_drivers();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize drivers: %s", esp_err_to_name(ret));
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
    
    // Initialize system tasks
    ret = init_tasks();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize tasks: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Set system initialization complete
    xEventGroupSetBits(g_system_event_group, SYSTEM_EVENT_INIT_COMPLETE);
    
    ESP_LOGI(TAG, "Application initialized successfully");
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
    // Configure I2C controller for the common bus (IMU, display)
    i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ
    };
    
    esp_err_t ret = i2c_param_config(I2C_MASTER_NUM, &i2c_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure I2C parameters: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = i2c_driver_install(I2C_MASTER_NUM, I2C_MODE_MASTER, 0, 0, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install I2C driver: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "I2C master initialized successfully");
    return ESP_OK;
}

static esp_err_t init_system_config(void) {
    // Initialize default system configuration
    g_system_config.system_state = SYSTEM_STATE_INIT;
    g_system_config.last_error = SYSTEM_ERROR_NONE;
    g_system_config.output_mode = OUTPUT_MODE_TEXT_AND_AUDIO;
    g_system_config.display_brightness = 100;
    g_system_config.audio_volume = 80;
    g_system_config.haptic_intensity = 80;
    g_system_config.bluetooth_enabled = true;
    g_system_config.power_save_enabled = true;
    g_system_config.touch_enabled = true;
    g_system_config.camera_enabled = false; // Camera initially disabled to save power
    g_system_config.calibration_required = true;
    
    // Load configuration from NVS if available
    esp_err_t ret = system_config_load();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to load system configuration, using defaults");
        
        // If loading failed, save the default configuration
        ret = system_config_save();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to save default system configuration: %s", esp_err_to_name(ret));
            return ret;
        }
    }
    
    ESP_LOGI(TAG, "System configuration initialized");
    return ESP_OK;
}

static esp_err_t init_drivers(void) {
    esp_err_t ret;
    
    // Initialize display first to show progress
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
    
    // Initialize IMU
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
    if (g_system_config.camera_enabled) {
        ret = camera_init();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to initialize camera: %s", esp_err_to_name(ret));
            // Camera is optional, so we continue even if it fails
            g_system_config.camera_enabled = false;
        }
    }
    
    // Initialize power management
    ret = power_management_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize power management: %s", esp_err_to_name(ret));
        return ret;
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
    
    // Initialize sensor fusion
    ret = sensor_fusion_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize sensor fusion: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Initialize feature extraction
    ret = feature_extraction_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize feature extraction: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Initialize gesture detection
    ret = gesture_detection_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize gesture detection: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "Processing modules initialized successfully");
    return ESP_OK;
}

static esp_err_t init_communication(void) {
    esp_err_t ret;
    
    // Initialize BLE service if enabled
    if (g_system_config.bluetooth_enabled) {
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
    // Create sensor data queue
    g_sensor_data_queue = xQueueCreate(SENSOR_QUEUE_SIZE, sizeof(sensor_data_t));
    if (g_sensor_data_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create sensor data queue");
        return ESP_FAIL;
    }
    
    // Create processing result queue
    g_processing_result_queue = xQueueCreate(PROCESSING_QUEUE_SIZE, sizeof(processing_result_t));
    if (g_processing_result_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create processing result queue");
        return ESP_FAIL;
    }
    
    // Create output command queue
    g_output_command_queue = xQueueCreate(OUTPUT_QUEUE_SIZE, sizeof(output_command_t));
    if (g_output_command_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create output command queue");
        return ESP_FAIL;
    }
    
    // Create system command queue
    g_system_command_queue = xQueueCreate(COMMAND_QUEUE_SIZE, sizeof(system_command_t));
    if (g_system_command_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create system command queue");
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "All queues created successfully");
    return ESP_OK;
}

static esp_err_t init_tasks(void) {
    esp_err_t ret;
    
    // Initialize sensor task
    ret = sensor_task_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize sensor task: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Initialize processing task
    ret = processing_task_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize processing task: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Initialize output task
    ret = output_task_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize output task: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Initialize communication task
    ret = communication_task_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize communication task: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Initialize power task
    ret = power_task_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize power task: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "All tasks initialized successfully");
    return ESP_OK;
}

// Main application entry point
void app_main(void)
{
    ESP_LOGI(TAG, "Sign Language Translation Glove starting...");
    
    // Initialize the application
    esp_err_t ret = app_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Application initialization failed! Error: %s", esp_err_to_name(ret));
        return;
    }
    
    ESP_LOGI(TAG, "Application initialized successfully, system running");
    
    // Main application code runs in FreeRTOS tasks
    // No need for further code here as all functionality is handled by tasks
    
    // Main thread can be used for monitoring if needed
    while (1) {
        // Periodic system checks could be done here, but we'll leave it empty
        // as all functionality is in the tasks
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}
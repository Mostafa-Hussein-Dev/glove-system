idf_component_register(
    COMPILE_OPTIONS "-Wno-error=deprecated-declarations" "-Wno-deprecated-declarations"
    SRCS 
        "app_main.c"
        "config/system_config.c"
        "core/power_management.c"
        "core/system_monitor.c"
        "drivers/flex_sensor.c"
        "drivers/imu.c"
        "drivers/camera.c"
        "drivers/touch.c"
        "drivers/display.c"
        "drivers/audio.c"
        "drivers/haptic.c"
        "processing/sensor_fusion.c"
        "processing/feature_extraction.c"
        "processing/gesture_detection.c"
        "communication/ble_service.c"
        "output/text_generation.c"
        "output/output_manager.c"
        "tasks/sensor_task.c"
        "tasks/processing_task.c"
        "tasks/output_task.c"
        "tasks/communication_task.c"
        "tasks/power_task.c"
        "util/buffer.c"
        "util/debug.c"
    INCLUDE_DIRS "." "config" "core" "drivers" "processing" "communication" "output" "tasks" "util" "include"
    REQUIRES
    driver
    esp_timer
    esp_adc
    esp_wifi
    bt
    esp-tls
    esp_system 
    esp32-camera
    spiffs
    esp_hw_support
    esp_common
    esp_event
    nvs_flash
    esp_netif
    esp_eth
    esp_http_client
    esp_https_server
    freertos
)

# Suppress unused-but-set-variable warning for this file
set_source_files_properties(core/system_monitor.c PROPERTIES COMPILE_FLAGS "-Wno-unused-but-set-variable")


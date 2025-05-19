#include "drivers/camera.h"
#include <string.h>
#include "esp_log.h"
#include "esp_camera.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "config/pin_definitions.h"
#include "util/debug.h"

static const char *TAG = "CAMERA";

// Camera state
static bool camera_initialized = false;
static bool camera_streaming = false;

// Camera resolution
static camera_resolution_t current_resolution = CAMERA_RESOLUTION_QVGA;

// Frame buffer
static camera_fb_t *current_frame = NULL;

esp_err_t camera_init(void) {
    if (camera_initialized) {
        return ESP_OK;  // Already initialized
    }
    
    // Configure camera
    camera_config_t config = {
        .pin_pwdn = -1,
        .pin_reset = -1,
        .pin_xclk = CAMERA_XCLK_PIN,
        .pin_sccb_sda = I2C_MASTER_SDA_IO,  // Share with I2C bus
        .pin_sccb_scl = I2C_MASTER_SCL_IO,  // Share with I2C bus
        
        .pin_d7 = CAMERA_D7_PIN,
        .pin_d6 = CAMERA_D6_PIN,
        .pin_d5 = CAMERA_D5_PIN,
        .pin_d4 = CAMERA_D4_PIN,
        .pin_d3 = CAMERA_D3_PIN,
        .pin_d2 = CAMERA_D2_PIN,
        .pin_d1 = CAMERA_D1_PIN,
        .pin_d0 = CAMERA_D0_PIN,
        
        .pin_vsync = CAMERA_VSYNC_PIN,
        .pin_href = CAMERA_HREF_PIN,
        .pin_pclk = CAMERA_PCLK_PIN,
        
        .xclk_freq_hz = 20000000,  // 20MHz XCLK
        .ledc_timer = LEDC_TIMER_0,
        .ledc_channel = LEDC_CHANNEL_0,
        
        .pixel_format = PIXFORMAT_RGB565,
        .frame_size = FRAMESIZE_QVGA,  // 320x240
        
        .jpeg_quality = 12,  // 0-63, lower is higher quality
        .fb_count = 2,  // Number of frame buffers
        .fb_location = CAMERA_FB_IN_PSRAM,
        .grab_mode = CAMERA_GRAB_WHEN_EMPTY
    };
    
    // Init camera
    esp_err_t ret = esp_camera_init(&config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Camera init failed with error 0x%x", ret);
        return ret;
    }
    
    // Set initial parameters
    camera_initialized = true;
    camera_streaming = false;
    
    ESP_LOGI(TAG, "Camera initialized successfully");
    return ESP_OK;
}

esp_err_t camera_deinit(void) {
    if (!camera_initialized) {
        return ESP_OK;  // Already deinitialized
    }
    
    // Release frame if any
    if (current_frame != NULL) {
        esp_camera_fb_return(current_frame);
        current_frame = NULL;
    }
    
    // Deinitialize camera
    esp_err_t ret = esp_camera_deinit();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Camera deinit failed with error 0x%x", ret);
        return ret;
    }
    
    camera_initialized = false;
    camera_streaming = false;
    
    ESP_LOGI(TAG, "Camera deinitialized");
    return ESP_OK;
}

esp_err_t camera_set_resolution(camera_resolution_t resolution) {
    if (!camera_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Map to ESP32 Camera framesize
    framesize_t frame_size;
    switch (resolution) {
        case CAMERA_RESOLUTION_QQVGA:  // 160x120
            frame_size = FRAMESIZE_QQVGA;
            break;
        case CAMERA_RESOLUTION_QVGA:   // 320x240
            frame_size = FRAMESIZE_QVGA;
            break;
        case CAMERA_RESOLUTION_VGA:    // 640x480
            frame_size = FRAMESIZE_VGA;
            break;
        default:
            ESP_LOGE(TAG, "Invalid resolution: %d", resolution);
            return ESP_ERR_INVALID_ARG;
    }
    
    // Get camera sensor
    sensor_t *sensor = esp_camera_sensor_get();
    if (sensor == NULL) {
        ESP_LOGE(TAG, "Failed to get camera sensor");
        return ESP_FAIL;
    }
    
    // Set frame size
    if (sensor->set_framesize(sensor, frame_size) != 0) {
        ESP_LOGE(TAG, "Failed to set frame size");
        return ESP_FAIL;
    }
    
    current_resolution = resolution;
    ESP_LOGI(TAG, "Camera resolution set to %d", resolution);
    
    return ESP_OK;
}

esp_err_t camera_capture_frame(camera_frame_t *frame) {
    if (!camera_initialized || frame == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Release previous frame if any
    if (current_frame != NULL) {
        esp_camera_fb_return(current_frame);
        current_frame = NULL;
    }
    
    // Capture a frame
    current_frame = esp_camera_fb_get();
    if (current_frame == NULL) {
        ESP_LOGE(TAG, "Failed to capture frame");
        return ESP_FAIL;
    }
    
    // Fill in frame info
    frame->buffer = current_frame->buf;
    frame->buffer_size = current_frame->len;
    frame->width = current_frame->width;
    frame->height = current_frame->height;
    frame->format = (current_frame->format == PIXFORMAT_RGB565) ? 
                   CAMERA_FORMAT_RGB565 : CAMERA_FORMAT_JPEG;
    frame->timestamp = esp_timer_get_time() / 1000;  // Convert to milliseconds
    
    return ESP_OK;
}

esp_err_t camera_release_frame(void) {
    if (!camera_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (current_frame != NULL) {
        esp_camera_fb_return(current_frame);
        current_frame = NULL;
    }
    
    return ESP_OK;
}

esp_err_t camera_get_info(camera_info_t *info) {
    if (!camera_initialized || info == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Get camera sensor
    sensor_t *sensor = esp_camera_sensor_get();
    if (sensor == NULL) {
        ESP_LOGE(TAG, "Failed to get camera sensor");
        return ESP_FAIL;
    }
    
    // Fill info structure
    info->resolution = current_resolution;
    
    switch (current_resolution) {
        case CAMERA_RESOLUTION_QQVGA:
            info->width = 160;
            info->height = 120;
            break;
        case CAMERA_RESOLUTION_QVGA:
            info->width = 320;
            info->height = 240;
            break;
        case CAMERA_RESOLUTION_VGA:
            info->width = 640;
            info->height = 480;
            break;
        default:
            info->width = 0;
            info->height = 0;
            break;
    }
    
    return ESP_OK;
}

esp_err_t camera_start_streaming(void) {
    if (!camera_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (camera_streaming) {
        return ESP_OK;  // Already streaming
    }
    
    // No specific action needed for now, just update state
    camera_streaming = true;
    ESP_LOGI(TAG, "Camera streaming started");
    
    return ESP_OK;
}

esp_err_t camera_stop_streaming(void) {
    if (!camera_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (!camera_streaming) {
        return ESP_OK;  // Already stopped
    }
    
    // Release frame if any
    if (current_frame != NULL) {
        esp_camera_fb_return(current_frame);
        current_frame = NULL;
    }
    
    camera_streaming = false;
    ESP_LOGI(TAG, "Camera streaming stopped");
    
    return ESP_OK;
}

bool camera_is_streaming(void) {
    return camera_streaming;
}
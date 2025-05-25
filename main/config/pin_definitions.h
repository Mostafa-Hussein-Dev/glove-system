#ifndef PIN_DEFINITIONS_H
#define PIN_DEFINITIONS_H

#include "esp_adc/adc_oneshot.h"

/**
 * @brief GPIO pin assignments for all hardware components
 */

/* Flex Sensors ADC Pins (10 sensors, 2 per finger) */
#define FLEX_SENSOR_ADC_UNIT               ADC_UNIT_1
#define FLEX_SENSOR_ADC_ATTENUATION        ADC_ATTEN_DB_12
#define FLEX_SENSOR_ADC_BIT_WIDTH          ADC_BITWIDTH_12

#define FLEX_SENSOR_THUMB_MCP_ADC_CHANNEL  ADC_CHANNEL_0   // GPIO1
#define FLEX_SENSOR_THUMB_PIP_ADC_CHANNEL  ADC_CHANNEL_1   // GPIO2
#define FLEX_SENSOR_INDEX_MCP_ADC_CHANNEL  ADC_CHANNEL_2   // GPIO3
#define FLEX_SENSOR_INDEX_PIP_ADC_CHANNEL  ADC_CHANNEL_3   // GPIO4
#define FLEX_SENSOR_MIDDLE_MCP_ADC_CHANNEL ADC_CHANNEL_4   // GPIO5
#define FLEX_SENSOR_MIDDLE_PIP_ADC_CHANNEL ADC_CHANNEL_5   // GPIO6
#define FLEX_SENSOR_RING_MCP_ADC_CHANNEL   ADC_CHANNEL_6   // GPIO7
#define FLEX_SENSOR_RING_PIP_ADC_CHANNEL   ADC_CHANNEL_7   // GPIO8
#define FLEX_SENSOR_PINKY_MCP_ADC_CHANNEL  ADC_CHANNEL_8   // GPIO9
#define FLEX_SENSOR_PINKY_PIP_ADC_CHANNEL  ADC_CHANNEL_9   // GPIO10



/* IMU (MPU6050) I2C Pins */
#define I2C_MASTER_NUM              I2C_NUM_0
#define I2C_MASTER_SDA_IO           21
#define I2C_MASTER_SCL_IO           20
#define I2C_MASTER_FREQ_HZ          400000
#define IMU_INT_PIN                 4

/* Camera Interface Pins */
#define CAMERA_XCLK_PIN             15
#define CAMERA_PCLK_PIN             14
#define CAMERA_VSYNC_PIN            13
#define CAMERA_HREF_PIN             12
#define CAMERA_D0_PIN               11
#define CAMERA_D1_PIN               10
#define CAMERA_D2_PIN               9
#define CAMERA_D3_PIN               8
#define CAMERA_D4_PIN               7
#define CAMERA_D5_PIN               6
#define CAMERA_D6_PIN               5
#define CAMERA_D7_PIN               4

/* Touch Sensor Pins (Using ESP32-S3 Touch Sensors) */
#define TOUCH_THUMB_PIN             9
#define TOUCH_INDEX_PIN             10
#define TOUCH_MIDDLE_PIN            11
#define TOUCH_RING_PIN              12
#define TOUCH_PINKY_PIN             13

/* OLED Display Pins (I2C shared with IMU) */
#define DISPLAY_RST_PIN             18
// SDA and SCL shared with I2C_MASTER

/* Audio Output (I2S) Pins */
#define I2S_MCLK_PIN                0  // Not used for MAX98357A
#define I2S_BCK_PIN                 27
#define I2S_WS_PIN                  25
#define I2S_DATA_OUT_PIN            26
#define I2S_SD_PIN                  19  // Shutdown control for MAX98357A

/* Haptic Feedback Motor Pin */
#define HAPTIC_PIN                  23

/* Battery Monitoring Pin */
#define BATTERY_ADC_CHANNEL         ADC_CHANNEL_1  // GPIO16
#define BATTERY_ADC_UNIT            ADC_UNIT_1
#define BATTERY_ADC_ATTENUATION     ADC_ATTEN_DB_12

/* Power Control Pins */
#define SENSOR_POWER_CTRL_PIN       17  // Controls power to sensors
#define SYSTEM_POWER_CTRL_PIN       19  // Not used (placeholder)

/* Debug and Development Pins */
#define DEBUG_LED_PIN               33
#define DEBUG_UART_TX_PIN           43
#define DEBUG_UART_RX_PIN           44

#endif /* PIN_DEFINITIONS_H */
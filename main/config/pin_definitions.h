#ifndef PIN_DEFINITIONS_H
#define PIN_DEFINITIONS_H

#include "esp_adc/adc_oneshot.h"

/**
 * @brief GPIO pin assignments for all hardware components
 */

/* Flex Sensors ADC Pins (5 sensors, 1 per finger) */
#define FLEX_SENSOR_ADC_UNIT               ADC_UNIT_1
#define FLEX_SENSOR_ADC_ATTENUATION        ADC_ATTEN_DB_6
#define FLEX_SENSOR_ADC_BIT_WIDTH          ADC_BITWIDTH_12

#define FLEX_SENSOR_THUMB_ADC_CHANNEL      ADC_CHANNEL_0   // GPIO1
#define FLEX_SENSOR_INDEX_ADC_CHANNEL      ADC_CHANNEL_1   // GPIO2
#define FLEX_SENSOR_MIDDLE_ADC_CHANNEL     ADC_CHANNEL_2   // GPIO4
#define FLEX_SENSOR_RING_ADC_CHANNEL       ADC_CHANNEL_3   // GPIO5
#define FLEX_SENSOR_PINKY_ADC_CHANNEL      ADC_CHANNEL_4   // GPIO6

/* IMU (MPU6050) I2C Pins */
#define I2C_MASTER_NUM              I2C_NUM_0
#define I2C_MASTER_SDA_IO           21
#define I2C_MASTER_SCL_IO           20
#define I2C_MASTER_FREQ_HZ          100000
#define IMU_INT_PIN                 16

/* Touch Sensor Pins (Using ESP32-S3 Touch Sensors) */
#define TOUCH_THUMB_PIN             TOUCH_PAD_NUM9   
#define TOUCH_INDEX_PIN             TOUCH_PAD_NUM10     
#define TOUCH_MIDDLE_PIN            TOUCH_PAD_NUM11  
#define TOUCH_RING_PIN              TOUCH_PAD_NUM12     
#define TOUCH_PINKY_PIN             TOUCH_PAD_NUM13

/* OLED Display Pins (I2C shared with IMU) */
#define DISPLAY_RST_PIN             -1 // WAS GPIO 18
// SDA and SCL shared with I2C_MASTER

/* Audio Output (I2S) Pins */
#define I2S_MCLK_PIN                0  // Not used for MAX98357A
#define I2S_BCK_PIN                 45  
#define I2S_WS_PIN                  46  
#define I2S_DATA_OUT_PIN            47   
#define I2S_SD_PIN                  48  

/* Haptic Feedback Motor Pin */
#define HAPTIC_PIN                  18

/* Battery Monitoring Pin */
#define BATTERY_ADC_CHANNEL         ADC_CHANNEL_7  // GPIO20 (changed from GPIO6)
#define BATTERY_ADC_UNIT            ADC_UNIT_2
#define BATTERY_ADC_ATTENUATION     ADC_ATTEN_DB_12

/* Power Control Pins */
#define SENSOR_POWER_CTRL_PIN       17  // Controls power to sensors

/* Debug and Development Pins */
#define DEBUG_LED_PIN               33
#define DEBUG_UART_TX_PIN           43
#define DEBUG_UART_RX_PIN           44

/* Optional USB Detection Pin */
#define USB_DETECT_PIN              12  // NEW: Detect USB charging (was camera HREF)

#endif /* PIN_DEFINITIONS_H */
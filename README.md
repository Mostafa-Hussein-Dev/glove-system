# Sign Language Translation Glove

A real-time sign language translation system based on ESP32-S3 that uses flex sensors, IMUs, and machine learning to recognize sign language gestures and convert them to text and speech.

![Sign Language Glove System](docs/images/sign_language_glove_system.jpg)

## 🚀 Project Overview

This advanced wearable device bridges communication barriers for deaf and hard-of-hearing individuals by:

- Detecting finger positions and hand movements using multiple sensors
- Processing sensor data with embedded ML algorithms for accurate sign language recognition
- Providing real-time translation through visual, audio, and mobile interfaces
- Operating independently with efficient power management for extended battery life

## ✨ Features

- **Multi-modal sensing**: Combines flex sensors, IMU, camera, and touch sensors for robust gesture detection
- **Multi-output translation**: Displays results on OLED screen, speaks through text-to-speech, and sends to mobile app
- **Smart power management**: Adaptive power modes to maximize battery life based on usage patterns
- **Bluetooth connectivity**: Pairs with mobile app for extended functionality and settings control
- **Expandable vocabulary**: Supports up to 50 customizable ASL gestures with high recognition accuracy
- **Real-time performance**: Processes gestures with minimal latency for natural conversation flow
- **Compact & comfortable**: Ergonomic design with lightweight components for extended wear

## 🔧 Hardware Components

- **ESP32-S3** microcontroller (dual-core with WiFi/BLE)
- **10x Flex sensors** (Spectra Symbol Bend Sensors, 2 per finger)
- **MPU-6050** 6-axis IMU for hand motion tracking
- **OV2640** camera module for visual gesture validation
- **0.96" OLED** SSD1306 display for visual feedback
- **MAX98357A** I2S audio amplifier with small speaker
- **LiPo battery** with charging and protection circuits

## 📁 Project Structure

```
sign_language_glove/
├── components/                # Custom components
│   └── ml_inference/          # ML inference engine 
├── main/                      # Main application code
│   ├── config/                # Configuration files and pin definitions
│   ├── core/                  # Core system modules
│   ├── drivers/               # Hardware driver implementations
│   ├── processing/            # Sensor fusion and gesture recognition
│   ├── communication/         # BLE communication services
│   ├── output/                # Display, audio, and haptic feedback
│   ├── tasks/                 # FreeRTOS task implementations
│   └── util/                  # Utility functions and data structures
├── CMakeLists.txt             # Project CMake configuration
├── partitions.csv             # Flash partition table
├── README.md                  # Project documentation
└── sdkconfig.defaults         # Default SDK configuration
```

## 🛠️ Setup Instructions

### Prerequisites

- ESP-IDF v4.4 or later (with ESP32-S3 support)
- ESP32-S3 development board
- Hardware components per schematic
- Visual Studio Code with PlatformIO or Eclipse with ESP-IDF plugin

### Build and Flash

1. **Clone the repository**:
   ```
   git clone https://github.com/yourusername/sign-language-glove.git
   cd sign-language-glove
   ```

2. **Set up ESP-IDF environment**:
   ```
   # Linux/macOS
   . $HOME/esp/esp-idf/export.sh
   
   # Windows
   %userprofile%\esp\esp-idf\export.bat
   ```

3. **Configure the project** (optional, defaults provided):
   ```
   idf.py menuconfig
   ```

4. **Build the project**:
   ```
   idf.py build
   ```

5. **Flash to ESP32-S3**:
   ```
   idf.py -p PORT flash monitor
   ```
   Replace `PORT` with your ESP32-S3's serial port (e.g., COM3 on Windows, /dev/ttyUSB0 on Linux)

### Hardware Setup

Refer to the [circuit diagram](docs/schematics/circuit_design.pdf) and [hardware assembly guide](docs/hardware_assembly.md) for detailed instructions on building the hardware. The basic connections are:

- **Flex sensors**: ADC channels (GPIO1-10)
- **IMU**: I2C bus (SDA: GPIO21, SCL: GPIO22)
- **OLED display**: I2C bus (shared with IMU)
- **Camera**: Dedicated camera interface (GPIO4-15)
- **Audio**: I2S interface (GPIO25-27)
- **Haptic motor**: GPIO23

## 📱 Mobile Application

The companion mobile app enhances functionality with:
- Expanded text display 
- Voice output through phone speakers
- Gesture vocabulary management
- System configuration and calibration
- Battery monitoring and power settings

The mobile app source code is available in a [separate repository](https://github.com/yourusername/sign-language-glove-app).

## 📚 Documentation

- [User Manual](docs/user_manual.md): Complete usage instructions
- [Hardware Guide](docs/hardware_guide.md): Component details and assembly
- [Software Architecture](docs/software_architecture.md): Code structure and design
- [API Reference](docs/api_reference.md): Interface documentation
- [Calibration Guide](docs/calibration_guide.md): Sensor calibration procedures
- [Troubleshooting](docs/troubleshooting.md): Common issues and solutions

## 🌟 Current Status

- ✅ Hardware design and component selection complete
- ✅ Software architecture and framework implemented
- ✅ Core sensor drivers and processing pipeline
- ✅ Communication and output systems
- ⬜ ML model training and implementation (coming soon)
- ⬜ Mobile app integration and user testing
- ⬜ Comprehensive documentation and tutorials

## 🤝 Contributing

Contributions are welcome! Please check the [contributing guidelines](CONTRIBUTING.md) before submitting pull requests.

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 🙏 Acknowledgments

- Inspired by research on assistive technology for deaf and hard-of-hearing communities
- Special thanks to all contributors and testers
- Built with ESP-IDF and TensorFlow Lite for Microcontrollers
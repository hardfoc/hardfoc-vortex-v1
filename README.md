# 🔌 HardFOC Vortex V1 - Unified API Motor Control Platform

<div align="center">

![Version](https://img.shields.io/badge/version-2.0-blue.svg)
![Platform](https://img.shields.io/badge/platform-ESP32--C6-green.svg)
![Board](https://img.shields.io/badge/Board-HardFOC%20Vortex%20V1-orange.svg)
![ESP-IDF](https://img.shields.io/badge/ESP--IDF-v5.5+-brightgreen.svg)
![License](https://img.shields.io/badge/license-MIT-yellow.svg)
![Build](https://img.shields.io/badge/build-passing-brightgreen.svg)
![Vortex API](https://img.shields.io/badge/Vortex%20API-Unified%20Interface-brightgreen.svg)

**Advanced motor control platform with unified Vortex API for the HardFOC Vortex V1 board**

[🚀 Quick Start](#-quick-start) • [🔌 Vortex API](#-vortex-api) • [🏗️ Features](#️-features) • [📚 Documentation](#-documentation) • [🧪 Examples](#-examples)

</div>

## 🎯 Overview

The **HardFOC Vortex V1** platform provides a comprehensive, thread-safe hardware abstraction layer specifically designed for advanced motor control applications. At its core is the **Vortex API** - a unified singleton interface that provides elegant access to all component handlers including GPIO, ADC, communication interfaces, motor controllers, IMU sensors, encoders, LED management, and temperature monitoring.

### ✨ Key Features

- **🔌 Unified Vortex API**: Single point of access to all HardFOC Vortex V1 components
- **🏗️ ESP-IDF v5.5+ Compatible**: Built for the latest ESP-IDF framework
- **🎛️ Multi-Component Support**: ESP32-C6, TMC9660, PCAL95555, BNO08x, AS5047U, WS2812
- **🔒 Thread-Safe Operations**: Concurrent access from multiple FreeRTOS tasks
- **📊 Advanced Diagnostics**: Real-time health monitoring and comprehensive error tracking
- **⚡ High Performance**: Optimized for real-time control loops with minimal latency
- **🛡️ Safety Features**: Pin validation, conflict detection, and automatic fault recovery
- **🚀 Lazy Initialization**: Components initialized only when needed with proper dependencies

## 🔌 Vortex API - The Heart of the System

The **Vortex API** is the cornerstone of the HardFOC Vortex V1 platform, providing a beautiful, unified interface to all component handlers. Named after the **HardFOC Vortex V1** board, it implements lazy initialization with proper dependency management.

### 🚀 Quick Start with Vortex API

```cpp
#include "API/Vortex.h"

extern "C" void app_main(void) {
    // Get the unified Vortex API instance
    auto& vortex = Vortex::GetInstance();
    
    // Initialize all systems with proper dependency management
    if (vortex.EnsureInitialized()) {
        // Access all component handlers through unified interface
        auto& gpio = vortex.gpio;        // GPIO management
        auto& adc = vortex.adc;          // ADC operations
        auto& motors = vortex.motors;    // Motor controllers
        auto& imu = vortex.imu;          // IMU sensors
        auto& encoders = vortex.encoders; // Position encoders
        auto& leds = vortex.leds;        // LED management
        auto& temp = vortex.temp;        // Temperature monitoring
        
        // Use components naturally
        gpio.Set("GPIO_EXT_GPIO_CS_1", true);
        float voltage;
        adc.ReadChannelV("ADC_TMC9660_AIN3", voltage);
        leds.SetStatus(LedAnimation::STATUS_OK);
        
        // Get comprehensive system diagnostics
        VortexSystemDiagnostics diagnostics;
        vortex.GetSystemDiagnostics(diagnostics);
        ESP_LOGI("MAIN", "System Health: %s", 
                diagnostics.system_healthy ? "HEALTHY" : "UNHEALTHY");
    }
}
```

## 🏗️ Hardware Architecture

### Supported Hardware Components

| Component | Hardware | Interface | Purpose | Pins/Channels |
|-----------|----------|-----------|---------|---------------|
| **MCU** | ESP32-C6 | Native | Main controller | 2 GPIO pins |
| **Motor Controller** | TMC9660 | SPI/UART | FOC motor control | 8 GPIO + 4 ADC |
| **GPIO Expander** | PCAL95555 | I2C | I/O expansion | 16 GPIO pins |
| **IMU Sensor** | BNO08x | I2C | Motion sensing | 9-axis IMU |
| **Position Encoder** | AS5047U | SPI | Position feedback | 14-bit magnetic |
| **Status LED** | WS2812 | Digital | System indication | RGB LED |
| **Temperature** | NTC + Internal | ADC/Digital | Thermal monitoring | Multiple sensors |

### Available GPIO Pins

#### ESP32-C6 GPIO Pins
- `GPIO_EXT_GPIO_CS_1` - External GPIO chip select 1
- `GPIO_EXT_GPIO_CS_2` - External GPIO chip select 2

#### PCAL95555 GPIO Expander Pins
- `GPIO_PCAL_GPIO17` - General purpose GPIO 17
- `GPIO_PCAL_GPIO18` - General purpose GPIO 18
- `GPIO_PCAL_FAULT_STATUS` - Fault status indicator
- `GPIO_PCAL_DRV_EN` - Driver enable control
- `GPIO_PCAL_PWR_GOOD` - Power good indicator
- `GPIO_PCAL_IMU_INT` - IMU interrupt signal
- And 6 more specialized control pins

#### TMC9660 ADC Channels
- `ADC_TMC9660_AIN0-3` - External ADC inputs
- `TMC9660_CURRENT_I0-3` - Motor current sensing
- `TMC9660_SUPPLY_VOLTAGE` - Supply voltage monitoring
- `TMC9660_CHIP_TEMPERATURE` - Internal temperature
- `TMC9660_MOTOR_VELOCITY` - Motor velocity feedback
- `TMC9660_MOTOR_POSITION` - Motor position feedback

## 🔧 Component Access Through Vortex API

| Component | Access | Purpose | Key Methods |
|-----------|--------|---------|-------------|
| **Communication** | `vortex.comms` | SPI, I2C, UART, CAN interfaces | `GetSpiDevice()`, `GetI2cDevice()` |
| **GPIO Management** | `vortex.gpio` | Pin control and configuration | `Set()`, `Read()`, `SetDirection()` |
| **Motor Controllers** | `vortex.motors` | Motor control and management | `handler()`, `driver()`, `EnableMotor()` |
| **ADC Management** | `vortex.adc` | Analog-to-digital conversion | `ReadChannelV()`, `BatchRead()` |
| **IMU Sensors** | `vortex.imu` | Motion and orientation sensing | `GetBno08xHandler()`, `ReadAccel()` |
| **Encoders** | `vortex.encoders` | Position and velocity sensing | `ReadAngle()`, `ReadVelocityRPM()` |
| **LED Management** | `vortex.leds` | Status indication and animation | `SetStatus()`, `SetColor()` |
| **Temperature** | `vortex.temp` | Temperature monitoring | `ReadTemperatureCelsius()` |

## ⚡ Performance Considerations

### String-Based API vs Cached Access

#### 🔍 Convenience API (String-Based)
```cpp
// Easy to use and understand - perfect for configuration and setup
gpio.Set("GPIO_EXT_GPIO_CS_1", true);
adc.ReadChannelV("ADC_TMC9660_AIN3", voltage);
```

#### ⚡ High-Performance API (Cached Access)
```cpp
// Cache component pointers for maximum performance in control loops
auto gpio_cs1 = vortex.gpio.Get("GPIO_EXT_GPIO_CS_1");
auto* adc_temp = vortex.adc.Get("ADC_TMC9660_AIN3");

// Direct access - much faster for real-time operations >1kHz
gpio_cs1->SetActive();
adc_temp->ReadChannelV(0, voltage);
```

**Performance Impact:**
- String lookups: ~100-500ns (hash map lookup)
- Cached access: ~10-50ns (direct pointer access)
- **Recommendation**: Use cached access for control loops >1kHz

## 🔄 Initialization Order and Dependencies

The Vortex API manages initialization in the correct dependency order:

```
1. CommChannelsManager (foundation - SPI, I2C, UART, CAN)
   ↓
2. GpioManager (depends on CommChannelsManager)
   ↓
3. MotorController (depends on CommChannelsManager)
   ↓
4. AdcManager (depends on MotorController)
   ↓
5. ImuManager (depends on CommChannelsManager, GpioManager)
   ↓
6. EncoderManager (depends on CommChannelsManager, GpioManager)
   ↓
7. LedManager (independent)
   ↓
8. TemperatureManager (depends on AdcManager, MotorController)
```

## 📊 System Diagnostics and Health Monitoring

```cpp
// Get comprehensive system diagnostics
VortexSystemDiagnostics diagnostics;
if (vortex.GetSystemDiagnostics(diagnostics)) {
    ESP_LOGI("MAIN", "Overall Health: %s", 
             diagnostics.system_healthy ? "HEALTHY" : "UNHEALTHY");
    ESP_LOGI("MAIN", "Initialized Components: %u/%u", 
             diagnostics.initialized_components, diagnostics.total_components);
    ESP_LOGI("MAIN", "Initialization Time: %llu ms", 
             diagnostics.initialization_time_ms);
}

// Perform health check
if (vortex.PerformHealthCheck()) {
    ESP_LOGI("MAIN", "All systems operational");
}

// Get failed components
auto failed_components = vortex.GetFailedComponents();
for (const auto& component : failed_components) {
    ESP_LOGE("MAIN", "Failed: %s", component.c_str());
}
```

## 🚀 Getting Started

### Prerequisites

- **ESP-IDF v5.5 or newer**
- **HardFOC Vortex V1 board**
- **USB-C cable for programming and power**

### 1. Clone and Build

```bash
git clone --recursive https://github.com/hardfoc/hardfoc-vortex-v1-demo.git
cd hardfoc-vortex-v1-demo
idf.py set-target esp32c6
idf.py build
```

### 2. Flash and Monitor

```bash
idf.py flash monitor
```

### 3. Expected Output

```
========================================
  HardFOC Vortex V1 - Unified API Demo
========================================
ESP-IDF Version: v5.5.0
Compile Time: Jan 15 2025 10:30:45
========================================

I (100) VortexMain: Starting HardFOC Vortex V1 application
I (150) VortexMain: ✓ Vortex API instance obtained
I (200) VortexMain: ✓ HardFOC Vortex V1 initialization successful!
I (250) VortexMain: === HardFOC Vortex V1 System Diagnostics ===
I (300) VortexMain: Overall System Health: HEALTHY
I (350) VortexMain: Initialized Components: 8/8
```

## 🧪 Examples

### Basic GPIO Control

```cpp
auto& gpio = vortex.gpio;

// Configure ESP32 GPIO as output
gpio.SetDirection("GPIO_EXT_GPIO_CS_1", hf_gpio_direction_t::GPIO_DIRECTION_OUTPUT);

// Blink LED pattern
for (int i = 0; i < 10; i++) {
    gpio.Set("GPIO_EXT_GPIO_CS_1", true);
    vTaskDelay(pdMS_TO_TICKS(200));
    gpio.Set("GPIO_EXT_GPIO_CS_1", false);
    vTaskDelay(pdMS_TO_TICKS(200));
}
```

### Multi-Channel ADC Reading

```cpp
auto& adc = vortex.adc;

// Read multiple TMC9660 channels simultaneously
std::array<std::string_view, 4> channels = {
    "ADC_TMC9660_AIN0", "ADC_TMC9660_AIN1", 
    "ADC_TMC9660_AIN2", "ADC_TMC9660_AIN3"
};

auto readings = adc.BatchRead(channels);
for (size_t i = 0; i < channels.size(); i++) {
    ESP_LOGI("ADC", "%s: %.3fV", channels[i].data(), readings.voltages[i]);
}
```

### Motor Control

```cpp
auto& motors = vortex.motors;

// Get motor controller and configure
auto* handler = motors.handler(0);
if (handler) {
    auto driver = motors.driver(0);
    if (driver) {
        driver->SetTargetVelocity(1000);  // 1000 RPM
        driver->SetMaxCurrent(2000);      // 2A
        driver->EnableMotor(true);
    }
}
```

## 📁 Project Structure

```
├── main/                          # Main application
│   ├── main.cpp                   # Vortex API demonstration
│   ├── vortex_demos.h            # Demo function declarations
│   └── CMakeLists.txt             # Main component build config
├── hf-hal-vortex-v1/             # Vortex HAL (submodule)
│   ├── API/                       # Unified Vortex API
│   ├── component-handlers/        # Component managers
│   ├── utils-and-drivers/         # Drivers and utilities
│   └── examples/                  # Usage examples
├── threads/                       # Legacy thread system
├── CMakeLists.txt                 # Root project configuration
├── sdkconfig                      # ESP-IDF configuration
└── README.md                      # This file
```

## 🛠️ Build Configuration

The project is configured for ESP-IDF v5.5+ with the following key settings:

- **Target**: ESP32-C6
- **C++ Standard**: C++17
- **Build Type**: Release (optimized)
- **Components**: Full Vortex HAL + WS2812 + Threads
- **Logging Level**: INFO (configurable)

## 📚 Documentation

- **[📋 Complete Documentation](hf-hal-vortex-v1/DOCUMENTATION_INDEX.md)** - Full API reference
- **[🔌 Vortex API Guide](hf-hal-vortex-v1/API/README.md)** - Comprehensive API documentation
- **[🏗️ Hardware Architecture](hf-hal-vortex-v1/docs/development/ARCHITECTURE_GUIDELINES.md)** - System design
- **[🎯 GPIO Reference](hf-hal-vortex-v1/docs/component-handlers/GPIO_MANAGER_README.md)** - GPIO system guide
- **[📊 ADC Reference](hf-hal-vortex-v1/docs/component-handlers/ADC_MANAGER_README.md)** - ADC system guide

## 🤝 Contributing

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 🆘 Support

- **📚 Documentation**: Start with the [Documentation Index](hf-hal-vortex-v1/DOCUMENTATION_INDEX.md)
- **🐛 Issues**: Report bugs via [GitHub Issues](https://github.com/hardfoc/hardfoc-vortex-v1-demo/issues)
- **💬 Discussions**: Use [GitHub Discussions](https://github.com/hardfoc/hardfoc-vortex-v1-demo/discussions) for questions
- **📧 Contact**: HardFOC Team

---

<div align="center">

**Built with ❤️ by the HardFOC Team**

[⭐ Star us on GitHub](https://github.com/hardfoc/hardfoc-vortex-v1-demo) • [🐛 Report Bug](https://github.com/hardfoc/hardfoc-vortex-v1-demo/issues) • [💡 Request Feature](https://github.com/hardfoc/hardfoc-vortex-v1-demo/issues)

**The Vortex API provides a unified, elegant interface to the HardFOC Vortex V1 platform!**

</div>



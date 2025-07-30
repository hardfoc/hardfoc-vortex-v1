# 🔧 HardFOC Vortex V1 - Build Guide

## 📋 Prerequisites

### Required Software
- **ESP-IDF v5.5.0 or newer** (Latest stable version recommended)
- **Python 3.8+** (for ESP-IDF tools)
- **Git with submodule support**
- **CMake 3.16+** (usually included with ESP-IDF)

### Supported Platforms
- **Windows 10/11** (WSL2 recommended)
- **macOS 10.15+** (Intel and Apple Silicon)
- **Linux** (Ubuntu 20.04+, Debian 11+, etc.)

### Hardware Requirements
- **HardFOC Vortex V1 board**
- **USB-C cable** for programming and debugging
- **Power supply** (if using external power)

## 🚀 Quick Start Guide

### 1. Install ESP-IDF v5.5+

#### Option A: ESP-IDF Installer (Recommended)
```bash
# Download and run the ESP-IDF installer from:
# https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/

# For Linux/macOS:
mkdir -p ~/esp
cd ~/esp
git clone -b v5.5.0 --recursive https://github.com/espressif/esp-idf.git
cd esp-idf
./install.sh esp32c6
source ./export.sh
```

#### Option B: Using VS Code Extension
Install the "ESP-IDF" extension in VS Code and follow the setup wizard.

### 2. Clone the Project

```bash
# Clone with all submodules (important!)
git clone --recursive https://github.com/hardfoc/hardfoc-vortex-v1-demo.git
cd hardfoc-vortex-v1-demo

# If you forgot --recursive, initialize submodules manually:
git submodule update --init --recursive
```

### 3. Set Target and Configure

```bash
# Set the target MCU
idf.py set-target esp32c6

# Optional: Open configuration menu
idf.py menuconfig
```

### 4. Build the Project

```bash
# Full clean build
idf.py fullclean build

# Or regular build
idf.py build
```

### 5. Flash and Monitor

```bash
# Flash the firmware
idf.py flash

# Monitor serial output
idf.py monitor

# Or combine both
idf.py flash monitor
```

## 🔧 Build Configuration Details

### Project Structure
```
hardfoc-vortex-v1-demo/
├── main/                          # Main application
│   ├── main.cpp                   # Vortex API demo
│   ├── vortex_demos.h            # Demo declarations
│   └── CMakeLists.txt             # Main component config
├── hf-hal-vortex-v1/             # Vortex HAL (submodule)
│   ├── API/                       # Unified Vortex API
│   ├── component-handlers/        # Component managers
│   ├── utils-and-drivers/         # Drivers and utilities
│   └── CMakeLists.txt             # HAL component config
├── threads/                       # Legacy thread system
├── CMakeLists.txt                 # Root project config
├── sdkconfig                      # ESP-IDF configuration
└── README.md                      # Project documentation
```

### Key Build Settings

#### CMake Configuration
- **C Standard**: C11
- **C++ Standard**: C++17
- **Build Type**: Release (optimized)
- **Target**: ESP32-C6

#### Component Dependencies
- **hf-hal-vortex-v1**: Main Vortex HAL component
- **hf-ws2812-rmt-driver**: WS2812 LED driver
- **threads**: Legacy thread system
- **freertos**: FreeRTOS kernel
- **esp_driver_***: ESP-IDF v5.5+ driver components

#### Compile Definitions
```cmake
HARDFOC_VORTEX_V1=1
HARDFOC_VORTEX_API=1
ESP_IDF_VERSION_MAJOR=5
ESP_IDF_VERSION_MINOR=5
```

## 🎯 Build Targets

### Standard Targets
```bash
# Clean build
idf.py clean

# Full clean (removes all build artifacts)
idf.py fullclean

# Build only
idf.py build

# Flash firmware
idf.py flash

# Monitor serial output
idf.py monitor

# Build size analysis
idf.py size

# Build size components
idf.py size-components
```

### Advanced Targets
```bash
# Build with verbose output
idf.py -v build

# Build specific component only
idf.py build --only-components main

# Flash with custom port
idf.py -p /dev/ttyUSB1 flash

# Monitor with custom baud rate
idf.py monitor -b 921600

# Generate compilation database
idf.py compiledb
```

## 🔍 Troubleshooting

### Common Build Issues

#### 1. Submodule Not Initialized
```bash
# Error: No such file or directory: 'hf-hal-vortex-v1/...'
# Solution:
git submodule update --init --recursive
```

#### 2. ESP-IDF Version Mismatch
```bash
# Error: ESP-IDF version 4.x is not supported
# Solution: Install ESP-IDF v5.5+
source ~/esp/esp-idf/export.sh
```

#### 3. Component Not Found
```bash
# Error: Component 'hf-hal-vortex-v1' not found
# Solution: Check EXTRA_COMPONENT_DIRS in CMakeLists.txt
```

#### 4. C++ Standard Issues
```bash
# Error: C++17 features not available
# Solution: Ensure CMakeLists.txt sets CXX_STANDARD to 17
```

### Build Verification

#### Expected Build Output
```
[1/2] Building CXX object esp-idf/main/CMakeFiles/...
[2/2] Linking CXX executable hardfoc_vortex_v1.elf
Project build complete. To flash, run:
  idf.py flash
```

#### Memory Usage (Typical)
```
Total sizes:
Used static DRAM:   45678 bytes
Available static DRAM: 150000+ bytes
Used static IRAM:   12345 bytes
Available static IRAM: 50000+ bytes
Flash code:         234567 bytes
```

## 📊 Performance Optimization

### Build Performance
```bash
# Use multiple cores for building
idf.py -j8 build

# Use ccache for faster rebuilds
export IDF_CCACHE_ENABLE=1
idf.py build
```

### Runtime Optimization
```bash
# Enable compiler optimizations in menuconfig:
idf.py menuconfig
# Navigate to: Compiler options → Optimization Level → Release (-O2)
```

### Memory Optimization
```bash
# Configure memory settings in menuconfig:
idf.py menuconfig
# Navigate to: Component config → ESP32C6-specific
```

## 🧪 Testing the Build

### 1. Basic Functionality Test
```bash
# Flash and monitor
idf.py flash monitor

# Expected output:
# I (100) VortexMain: Starting HardFOC Vortex V1 application
# I (150) VortexMain: ✓ Vortex API instance obtained
# I (200) VortexMain: ✓ HardFOC Vortex V1 initialization successful!
```

### 2. Component Health Check
Look for initialization messages for each component:
- ✓ Communication Channels: OK
- ✓ GPIO Management: OK
- ✓ Motor Controllers: OK
- ✓ ADC Management: OK
- ✓ LED Management: OK

### 3. Interactive Testing
The main application provides several demonstration modes:
- GPIO control demo (every 20 seconds)
- ADC reading demo (every 20 seconds)
- LED animation demo (continuous)
- System health monitoring (every 5 seconds)

## 🔧 Development Environment Setup

### VS Code Configuration
Create `.vscode/settings.json`:
```json
{
    "C_Cpp.default.configurationProvider": "ms-vscode.cpptools",
    "idf.adapterTargetName": "esp32c6",
    "idf.customExtraPaths": "",
    "idf.customExtraVars": {},
    "idf.port": "/dev/ttyUSB0",
    "idf.flashType": "UART"
}
```

### Git Configuration
```bash
# Configure git for submodules
git config submodule.recurse true
git config push.recurseSubmodules check
```

## 📝 Build Script Examples

### Linux/macOS Build Script
```bash
#!/bin/bash
# build.sh

set -e

echo "🔧 Building HardFOC Vortex V1..."

# Source ESP-IDF environment
source ~/esp/esp-idf/export.sh

# Update submodules
git submodule update --init --recursive

# Set target
idf.py set-target esp32c6

# Clean and build
idf.py fullclean build

# Optional: Flash and monitor
if [ "$1" = "flash" ]; then
    idf.py flash monitor
fi

echo "✅ Build complete!"
```

### Windows PowerShell Build Script
```powershell
# build.ps1

Write-Host "🔧 Building HardFOC Vortex V1..." -ForegroundColor Green

# Source ESP-IDF environment
& "$env:IDF_PATH\export.ps1"

# Update submodules
git submodule update --init --recursive

# Set target
idf.py set-target esp32c6

# Clean and build
idf.py fullclean build

# Optional: Flash and monitor
if ($args[0] -eq "flash") {
    idf.py flash monitor
}

Write-Host "✅ Build complete!" -ForegroundColor Green
```

## 🆘 Support

### Documentation Resources
- [ESP-IDF Documentation](https://docs.espressif.com/projects/esp-idf/en/latest/)
- [HardFOC Vortex V1 API Reference](hf-hal-vortex-v1/API/README.md)
- [Component Documentation](hf-hal-vortex-v1/DOCUMENTATION_INDEX.md)

### Community Support
- **GitHub Issues**: Report build problems
- **GitHub Discussions**: Ask questions
- **ESP32 Forum**: ESP-IDF specific issues

### Common Commands Reference
```bash
# Project management
idf.py create-project my-project
idf.py set-target esp32c6
idf.py menuconfig

# Building
idf.py build
idf.py clean
idf.py fullclean

# Flashing and monitoring
idf.py flash
idf.py monitor
idf.py flash monitor

# Analysis
idf.py size
idf.py size-components
idf.py size-files

# Environment
idf.py --version
idf.py --help
```

---

**Build with confidence! 🚀**
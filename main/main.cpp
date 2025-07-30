/**
 * @file main.cpp
 * @brief HardFOC Vortex V1 Main Application using Unified Vortex API
 * 
 * @details This application demonstrates the complete Vortex API for the HardFOC Vortex V1
 *          motor controller board. It showcases all component handlers including GPIO, ADC,
 *          motor controllers, IMU, encoders, LEDs, and temperature sensors through a
 *          unified, elegant interface.
 * 
 * @author HardFOC Team
 * @date 2025
 * @version 2.0
 * 
 * Key Features:
 * - Unified Vortex API access to all HardFOC Vortex V1 components
 * - Comprehensive system initialization with proper dependency management
 * - Real-time system health monitoring and diagnostics
 * - Demonstration of all available hardware interfaces
 * - Thread-safe operations with proper error handling
 * - ESP-IDF v5.5+ compatibility
 * 
 * Hardware Support:
 * - ESP32-C6 microcontroller
 * - TMC9660 motor controller with integrated ADC
 * - PCAL95555 I2C GPIO expander
 * - BNO08x IMU sensor
 * - AS5047U magnetic encoder
 * - WS2812 status LED
 * - NTC temperature sensors
 * 
 * Usage:
 * 1. Build with ESP-IDF v5.5 or newer
 * 2. Flash to HardFOC Vortex V1 board
 * 3. Monitor serial output for system status
 * 4. Observe LED animations indicating system health
 */

#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_system.h>
#include <esp_chip_info.h>
#include <esp_flash.h>

// Vortex API - Unified interface to all HardFOC Vortex V1 components
#include "API/Vortex.h"

// HardFOC Logger - Advanced logging system
#include "Logger.h"

// Thread system (kept for compatibility but will be integrated with Vortex)
#include "CANOpenBLDCThread.h"
#include "WS2812TestThread.h"

//==============================================================================
// CONSTANTS AND CONFIGURATION
//==============================================================================

static const char* TAG = "VortexMain";

// Logger instance
static Logger& logger = Logger::GetInstance();

// Application timing constants
static const uint32_t MAIN_LOOP_DELAY_MS = 100;
static const uint32_t SYSTEM_HEALTH_CHECK_INTERVAL = 50;  // Every 5 seconds
static const uint32_t STATUS_LOG_INTERVAL = 100;           // Every 10 seconds
static const uint32_t DEMO_INTERVAL = 200;                 // Every 20 seconds
static const uint32_t MOTOR_DEMO_INTERVAL = 300;           // Every 30 seconds

// LED animation timing
static const uint32_t LED_BLINK_INTERVAL = 20;             // Every 2 seconds

//==============================================================================
// GLOBAL VARIABLES
//==============================================================================

// Thread instances (legacy compatibility)
static WS2812TestThread ws2812_thread;
static CANOpenBLDCThread canopen_bldc_thread;

// System statistics
static uint32_t main_loop_count = 0;
static uint64_t last_health_check_time = 0;

//==============================================================================
// UTILITY FUNCTIONS
//==============================================================================

/**
 * @brief Print comprehensive chip information
 */
void PrintChipInfo(void) {
    logger.Info(TAG, "=== ESP32-C6 Chip Information ===");
    
    esp_chip_info_t chip_info;
    uint32_t flash_size;
    esp_chip_info(&chip_info);
    
    logger.Info(TAG, "Chip: %s with %d CPU core(s)", CONFIG_IDF_TARGET, chip_info.cores);
    logger.Info(TAG, "Features: %s%s%s%s", 
                (chip_info.features & CHIP_FEATURE_WIFI_BGN) ? "WiFi/" : "",
                (chip_info.features & CHIP_FEATURE_BT) ? "BT" : "",
                (chip_info.features & CHIP_FEATURE_BLE) ? "BLE" : "",
                (chip_info.features & CHIP_FEATURE_IEEE802154) ? ", 802.15.4 (Zigbee/Thread)" : "");

    unsigned major_rev = chip_info.revision / 100;
    unsigned minor_rev = chip_info.revision % 100;
    logger.Info(TAG, "Silicon revision: v%d.%d", major_rev, minor_rev);
    
    if (esp_flash_get_size(NULL, &flash_size) == ESP_OK) {
        logger.Info(TAG, "Flash size: %" PRIu32 "MB %s", 
                    flash_size / (uint32_t)(1024 * 1024),
                    (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");
    }
    
    logger.Info(TAG, "Free heap: %" PRIu32 " bytes", esp_get_free_heap_size());
    logger.Info(TAG, "Min free heap: %" PRIu32 " bytes", esp_get_minimum_free_heap_size());
}

/**
 * @brief Print detailed system diagnostics
 */
void PrintSystemDiagnostics(const VortexSystemDiagnostics& diagnostics) {
    logger.Info(TAG, LogColor::BRIGHT_CYAN, LogStyle::BOLD, "=== HardFOC Vortex V1 System Diagnostics ===");
    
    // Use colors to highlight system health status
    LogColor health_color = diagnostics.system_healthy ? LogColor::BRIGHT_GREEN : LogColor::BRIGHT_RED;
    logger.Info(TAG, health_color, LogStyle::BOLD, "Overall System Health: %s", 
                diagnostics.system_healthy ? "HEALTHY" : "UNHEALTHY");
    
    logger.Info(TAG, "Initialized Components: %u/%u", diagnostics.initialized_components, diagnostics.total_components);
    logger.Info(TAG, "Failed Components: %u", diagnostics.failed_components);
    logger.Info(TAG, "Initialization Time: %llu ms", diagnostics.initialization_time_ms);
    logger.Info(TAG, "System Uptime: %llu ms", diagnostics.system_uptime_ms);
    
    logger.Info(TAG, LogColor::BRIGHT_BLUE, LogStyle::BOLD, "Component Status:");
    logger.Info(TAG, "  Communication Channels: %s", diagnostics.comms_initialized ? "✓ OK" : "✗ FAIL");
    logger.Info(TAG, "  GPIO Management: %s", diagnostics.gpio_initialized ? "✓ OK" : "✗ FAIL");
    logger.Info(TAG, "  Motor Controllers: %s", diagnostics.motors_initialized ? "✓ OK" : "✗ FAIL");
    logger.Info(TAG, "  ADC Management: %s", diagnostics.adc_initialized ? "✓ OK" : "✗ FAIL");
    logger.Info(TAG, "  IMU Management: %s", diagnostics.imu_initialized ? "✓ OK" : "✗ FAIL");
    logger.Info(TAG, "  Encoder Management: %s", diagnostics.encoders_initialized ? "✓ OK" : "✗ FAIL");
    logger.Info(TAG, "  LED Management: %s", diagnostics.leds_initialized ? "✓ OK" : "✗ FAIL");
    logger.Info(TAG, "  Temperature Management: %s", diagnostics.temp_initialized ? "✓ OK" : "✗ FAIL");
    
    if (!diagnostics.failed_components_list.empty()) {
        logger.Warn(TAG, LogColor::BRIGHT_RED, LogStyle::BOLD, "Failed Components:");
        for (const auto& component : diagnostics.failed_components_list) {
            logger.Warn(TAG, "  - %s", component.c_str());
        }
    }
    
    if (!diagnostics.warnings.empty()) {
        logger.Warn(TAG, LogColor::BRIGHT_YELLOW, LogStyle::BOLD, "System Warnings:");
        for (const auto& warning : diagnostics.warnings) {
            logger.Warn(TAG, "  - %s", warning.c_str());
        }
    }
}

//==============================================================================
// COMPONENT DEMONSTRATION FUNCTIONS
//==============================================================================

/**
 * @brief Demonstrate GPIO functionality
 */
void DemonstrateGpio(Vortex& vortex) {
    logger.Info(TAG, LogColor::BRIGHT_GREEN, LogStyle::BOLD, "=== GPIO Management Demo ===");
    
    auto& gpio = vortex.gpio;
    
    // Configure and control ESP32 GPIO pins
    if (gpio.Set("GPIO_EXT_GPIO_CS_1", true) == hf_gpio_err_t::GPIO_SUCCESS) {
        logger.Info(TAG, LogColor::GREEN, LogStyle::NORMAL, "✓ ESP32 CS1 pin set HIGH");
    } else {
        logger.Warn(TAG, LogColor::YELLOW, LogStyle::NORMAL, "✗ Failed to set ESP32 CS1 pin");
    }
    
    vTaskDelay(pdMS_TO_TICKS(200));
    
    if (gpio.Set("GPIO_EXT_GPIO_CS_1", false) == hf_gpio_err_t::GPIO_SUCCESS) {
        logger.Info(TAG, LogColor::GREEN, LogStyle::NORMAL, "✓ ESP32 CS1 pin set LOW");
    }
    
    // Read PCAL95555 GPIO pins
    bool gpio17_state, gpio18_state;
    if (gpio.Read("GPIO_PCAL_GPIO17", gpio17_state) == hf_gpio_err_t::GPIO_SUCCESS) {
        logger.Info(TAG, LogColor::CYAN, LogStyle::NORMAL, "✓ PCAL95555 GPIO17: %s", 
                    gpio17_state ? "HIGH" : "LOW");
    }
    
    if (gpio.Read("GPIO_PCAL_GPIO18", gpio18_state) == hf_gpio_err_t::GPIO_SUCCESS) {
        logger.Info(TAG, LogColor::CYAN, LogStyle::NORMAL, "✓ PCAL95555 GPIO18: %s", 
                    gpio18_state ? "HIGH" : "LOW");
    }
    
    logger.Info(TAG, LogColor::BRIGHT_GREEN, LogStyle::NORMAL, "GPIO demo completed");
}

/**
 * @brief Demonstrate ADC functionality
 */
void DemonstrateAdc(Vortex& vortex) {
    logger.Info(TAG, LogColor::BRIGHT_MAGENTA, LogStyle::BOLD, "=== ADC Management Demo ===");
    
    auto& adc = vortex.adc;
    
    // Read TMC9660 ADC channels
    float voltage;
    
    if (adc.ReadChannelV("ADC_TMC9660_AIN0", voltage) == hf_adc_err_t::ADC_SUCCESS) {
        logger.Info(TAG, LogColor::MAGENTA, LogStyle::NORMAL, "✓ TMC9660 AIN0: %.3f V", voltage);
    } else {
        logger.Warn(TAG, LogColor::YELLOW, LogStyle::NORMAL, "✗ TMC9660 AIN0 read failed");
    }
    
    if (adc.ReadChannelV("ADC_TMC9660_AIN3", voltage) == hf_adc_err_t::ADC_SUCCESS) {
        logger.Info(TAG, LogColor::MAGENTA, LogStyle::NORMAL, "✓ TMC9660 AIN3 (Temp): %.3f V", voltage);
    } else {
        logger.Warn(TAG, LogColor::YELLOW, LogStyle::NORMAL, "✗ TMC9660 AIN3 read failed");
    }
    
    // Read TMC9660 internal monitoring channels
    if (adc.ReadChannelV("TMC9660_CURRENT_I0", voltage) == hf_adc_err_t::ADC_SUCCESS) {
        logger.Info(TAG, LogColor::BRIGHT_YELLOW, LogStyle::NORMAL, "✓ TMC9660 Current I0: %.3f A", voltage);
    }
    
    if (adc.ReadChannelV("TMC9660_CHIP_TEMPERATURE", voltage) == hf_adc_err_t::ADC_SUCCESS) {
        logger.Info(TAG, LogColor::BRIGHT_RED, LogStyle::NORMAL, "✓ TMC9660 Chip Temp: %.1f°C", voltage);
    }
    
    if (adc.ReadChannelV("TMC9660_SUPPLY_VOLTAGE", voltage) == hf_adc_err_t::ADC_SUCCESS) {
        logger.Info(TAG, LogColor::BRIGHT_CYAN, LogStyle::NORMAL, "✓ TMC9660 Supply: %.3f V", voltage);
    }
    
    logger.Info(TAG, LogColor::BRIGHT_MAGENTA, LogStyle::NORMAL, "ADC demo completed");
}

/**
 * @brief Demonstrate motor controller functionality
 */
void DemonstrateMotorController(Vortex& vortex) {
    logger.Info(TAG, LogColor::BRIGHT_BLUE, LogStyle::BOLD, "=== Motor Controller Demo ===");
    
    auto& motors = vortex.motors;
    
    // Get onboard TMC9660 handler
    auto* handler = motors.handler(0);
    if (handler) {
        logger.Info(TAG, LogColor::BLUE, LogStyle::NORMAL, "✓ TMC9660 handler available");
        
        // Get underlying driver
        auto driver = motors.driver(0);
        if (driver) {
            logger.Info(TAG, LogColor::BLUE, LogStyle::NORMAL, "✓ TMC9660 driver available");
            
            // Configure motor parameters (safe values)
            driver->SetTargetVelocity(100);  // 100 RPM
            driver->SetMaxCurrent(500);      // 500 mA
            
            logger.Info(TAG, LogColor::GREEN, LogStyle::NORMAL, "✓ Motor parameters configured");
            
            // Note: Motor enabling would be done here in a real application
            // driver->EnableMotor(true);
        } else {
            logger.Warn(TAG, LogColor::YELLOW, LogStyle::NORMAL, "✗ TMC9660 driver not available");
        }
    } else {
        logger.Warn(TAG, LogColor::YELLOW, LogStyle::NORMAL, "✗ TMC9660 handler not available");
    }
    
    uint8_t device_count = motors.GetDeviceCount();
    logger.Info(TAG, LogColor::CYAN, LogStyle::NORMAL, "Active motor devices: %u", device_count);
    
    logger.Info(TAG, LogColor::BRIGHT_BLUE, LogStyle::NORMAL, "Motor controller demo completed");
}

/**
 * @brief Demonstrate IMU functionality
 */
void DemonstrateImu(Vortex& vortex) {
    logger.Info(TAG, LogColor::BRIGHT_YELLOW, LogStyle::BOLD, "=== IMU Management Demo ===");
    
    auto& imu = vortex.imu;
    
    // Get BNO08x handler
    auto* handler = imu.GetBno08xHandler(0);
    if (handler) {
        logger.Info(TAG, LogColor::YELLOW, LogStyle::NORMAL, "✓ BNO08x handler available");
        
        // Get underlying driver
        auto driver = imu.GetBno085Driver(0);
        if (driver) {
            logger.Info(TAG, LogColor::YELLOW, LogStyle::NORMAL, "✓ BNO085 driver available");
        } else {
            logger.Warn(TAG, LogColor::YELLOW, LogStyle::NORMAL, "✗ BNO085 driver not available");
        }
    } else {
        logger.Warn(TAG, LogColor::YELLOW, LogStyle::NORMAL, "✗ BNO08x handler not available");
    }
    
    uint8_t device_count = imu.GetDeviceCount();
    logger.Info(TAG, LogColor::CYAN, LogStyle::NORMAL, "Active IMU devices: %u", device_count);
    
    logger.Info(TAG, LogColor::BRIGHT_YELLOW, LogStyle::NORMAL, "IMU demo completed");
}

/**
 * @brief Demonstrate encoder functionality
 */
void DemonstrateEncoders(Vortex& vortex) {
    logger.Info(TAG, LogColor::BRIGHT_WHITE, LogStyle::BOLD, "=== Encoder Management Demo ===");
    
    auto& encoders = vortex.encoders;
    
    // Get AS5047U handler
    auto* handler = encoders.GetAs5047uHandler(0);
    if (handler) {
        logger.Info(TAG, LogColor::WHITE, LogStyle::NORMAL, "✓ AS5047U handler available");
        
        // Try to read encoder angle
        uint16_t angle;
        if (encoders.ReadAngle(0, angle) == As5047uError::SUCCESS) {
            logger.Info(TAG, LogColor::BRIGHT_GREEN, LogStyle::NORMAL, 
                       "✓ Encoder angle: %u LSB (%.2f°)", angle, (float)angle * 360.0f / 16384.0f);
        } else {
            logger.Warn(TAG, LogColor::YELLOW, LogStyle::NORMAL, "✗ Encoder angle read failed");
        }
        
        // Try to read encoder velocity
        double velocity_rpm;
        if (encoders.ReadVelocityRPM(0, velocity_rpm) == As5047uError::SUCCESS) {
            logger.Info(TAG, LogColor::BRIGHT_MAGENTA, LogStyle::NORMAL, 
                       "✓ Encoder velocity: %.2f RPM", velocity_rpm);
        } else {
            logger.Warn(TAG, LogColor::YELLOW, LogStyle::NORMAL, "✗ Encoder velocity read failed");
        }
    } else {
        logger.Warn(TAG, LogColor::YELLOW, LogStyle::NORMAL, "✗ AS5047U handler not available");
    }
    
    uint8_t device_count = encoders.GetDeviceCount();
    logger.Info(TAG, LogColor::CYAN, LogStyle::NORMAL, "Active encoder devices: %u", device_count);
    
    logger.Info(TAG, LogColor::BRIGHT_WHITE, LogStyle::NORMAL, "Encoder demo completed");
}

/**
 * @brief Demonstrate LED functionality
 */
void DemonstrateLeds(Vortex& vortex) {
    logger.Info(TAG, LogColor::BRIGHT_GREEN, LogStyle::BOLD, "=== LED Management Demo ===");
    
    auto& leds = vortex.leds;
    
    // Cycle through different LED animations
    const LedAnimation animations[] = {
        LedAnimation::STATUS_OK,
        LedAnimation::STATUS_WARN,
        LedAnimation::STATUS_ERROR,
        LedAnimation::STATUS_INIT
    };
    
    const char* animation_names[] = {
        "OK (Green)",
        "Warning (Yellow)",
        "Error (Red)",
        "Initializing (Blue)"
    };
    
    // Use matching colors for the log messages
    const LogColor led_colors[] = {
        LogColor::BRIGHT_GREEN,
        LogColor::BRIGHT_YELLOW,
        LogColor::BRIGHT_RED,
        LogColor::BRIGHT_BLUE
    };
    
    for (size_t i = 0; i < sizeof(animations) / sizeof(animations[0]); i++) {
        if (leds.SetStatus(animations[i]) == LedError::SUCCESS) {
            logger.Info(TAG, led_colors[i], LogStyle::NORMAL, "✓ LED set to %s", animation_names[i]);
        } else {
            logger.Warn(TAG, LogColor::YELLOW, LogStyle::NORMAL, "✗ Failed to set LED to %s", animation_names[i]);
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    
    // Set back to OK status
    leds.SetStatus(LedAnimation::STATUS_OK);
    
    logger.Info(TAG, LogColor::BRIGHT_GREEN, LogStyle::NORMAL, "LED demo completed");
}

/**
 * @brief Demonstrate temperature monitoring
 */
void DemonstrateTemperature(Vortex& vortex) {
    logger.Info(TAG, LogColor::BRIGHT_RED, LogStyle::BOLD, "=== Temperature Management Demo ===");
    
    auto& temp = vortex.temp;
    
    float temperature;
    
    // Read ESP32 internal temperature
    if (temp.ReadTemperatureCelsius("ESP32_INTERNAL", temperature) == hf_temp_err_t::HF_TEMP_SUCCESS) {
        LogColor temp_color = (temperature > 60.0f) ? LogColor::BRIGHT_RED : 
                             (temperature > 40.0f) ? LogColor::YELLOW : LogColor::CYAN;
        logger.Info(TAG, temp_color, LogStyle::NORMAL, "✓ ESP32 Internal: %.2f°C", temperature);
    } else {
        logger.Warn(TAG, LogColor::YELLOW, LogStyle::NORMAL, "✗ ESP32 Internal temperature read failed");
    }
    
    // Read motor temperature
    if (temp.ReadTemperatureCelsius("MOTOR_TEMP", temperature) == hf_temp_err_t::HF_TEMP_SUCCESS) {
        LogColor temp_color = (temperature > 80.0f) ? LogColor::BRIGHT_RED : 
                             (temperature > 60.0f) ? LogColor::YELLOW : LogColor::GREEN;
        logger.Info(TAG, temp_color, LogStyle::NORMAL, "✓ Motor Temperature: %.2f°C", temperature);
    } else {
        logger.Warn(TAG, LogColor::YELLOW, LogStyle::NORMAL, "✗ Motor temperature read failed");
    }
    
    // Read NTC thermistor
    if (temp.ReadTemperatureCelsius("NTC_THERMISTOR", temperature) == hf_temp_err_t::HF_TEMP_SUCCESS) {
        LogColor temp_color = (temperature > 70.0f) ? LogColor::BRIGHT_RED : 
                             (temperature > 50.0f) ? LogColor::YELLOW : LogColor::MAGENTA;
        logger.Info(TAG, temp_color, LogStyle::NORMAL, "✓ NTC Thermistor: %.2f°C", temperature);
    } else {
        logger.Warn(TAG, LogColor::YELLOW, LogStyle::NORMAL, "✗ NTC thermistor read failed");
    }
    
    logger.Info(TAG, LogColor::BRIGHT_RED, LogStyle::NORMAL, "Temperature demo completed");
}

//==============================================================================
// THREAD INTEGRATION (Legacy Compatibility)
//==============================================================================

/**
 * @brief Initialize and start legacy thread system
 */
void InitializeLegacyThreads() {
    logger.Info(TAG, LogColor::BRIGHT_CYAN, LogStyle::BOLD, "Initializing legacy thread system...");
    
    // Initialize WS2812 test thread
    if (ws2812_thread.EnsureInitialized()) {
        logger.Info(TAG, LogColor::GREEN, LogStyle::NORMAL, "✓ WS2812 test thread initialized");
        if (ws2812_thread.Start()) {
            logger.Info(TAG, LogColor::GREEN, LogStyle::NORMAL, "✓ WS2812 test thread started");
        } else {
            logger.Warn(TAG, LogColor::YELLOW, LogStyle::NORMAL, "✗ Failed to start WS2812 test thread");
        }
    } else {
        logger.Warn(TAG, LogColor::YELLOW, LogStyle::NORMAL, "✗ Failed to initialize WS2812 test thread");
    }
    
    // Initialize CANOpen BLDC thread
    if (canopen_bldc_thread.EnsureInitialized()) {
        logger.Info(TAG, LogColor::GREEN, LogStyle::NORMAL, "✓ CANOpen BLDC thread initialized");
        if (canopen_bldc_thread.Start()) {
            logger.Info(TAG, LogColor::GREEN, LogStyle::NORMAL, "✓ CANOpen BLDC thread started");
        } else {
            logger.Warn(TAG, LogColor::YELLOW, LogStyle::NORMAL, "✗ Failed to start CANOpen BLDC thread");
        }
    } else {
        logger.Warn(TAG, LogColor::YELLOW, LogStyle::NORMAL, "✗ Failed to initialize CANOpen BLDC thread");
    }
}

/**
 * @brief Monitor legacy thread health
 */
void MonitorLegacyThreads() {
    bool ws2812_running = ws2812_thread.IsThreadRunning();
    bool canopen_running = canopen_bldc_thread.IsThreadRunning();
    
    logger.Info(TAG, LogColor::CYAN, LogStyle::NORMAL, "Thread Status - WS2812: %s, CANOpen: %s",
                ws2812_running ? "Running" : "Stopped",
                canopen_running ? "Running" : "Stopped");
    
    // Print motor status if CANOpen thread is running
    if (canopen_running) {
        auto motor_status = canopen_bldc_thread.GetMotorStatus();
        logger.Info(TAG, LogColor::BLUE, LogStyle::NORMAL, 
                   "Motor - Present: %s, Enabled: %s, Pos: %d, Vel: %d",
                   motor_status.nodePresent ? "Yes" : "No",
                   motor_status.isEnabled ? "Yes" : "No",
                   motor_status.actualPosition, 
                   motor_status.actualVelocity);
    }
}

//==============================================================================
// MAIN APPLICATION ENTRY POINT
//==============================================================================

extern "C" void app_main(void) {
    // Initialize the HardFOC Logger first
    logger.Initialize();
    
    // Print banner using Logger with ASCII art
    const std::string banner = 
        "========================================\n"
        "  HardFOC Vortex V1 - Unified API Demo\n"
        "========================================";
    
    logger.LogBanner(TAG, banner, AsciiArtFormat{
        .color = LogColor::BRIGHT_CYAN,
        .style = LogStyle::BOLD,
        .center_art = true,
        .add_border = true,
        .border_char = '=',
        .max_width = 80
    });
    
    logger.Info(TAG, LogColor::WHITE, LogStyle::NORMAL, "ESP-IDF Version: %s", esp_get_idf_version());
    logger.Info(TAG, LogColor::WHITE, LogStyle::NORMAL, "Compile Time: %s %s", __DATE__, __TIME__);
    logger.Info(TAG, LogColor::BRIGHT_CYAN, LogStyle::BOLD, "========================================");
    
    logger.Info(TAG, LogColor::BRIGHT_GREEN, LogStyle::BOLD, "Starting HardFOC Vortex V1 application");
    
    // Print chip information
    PrintChipInfo();
    
    // Get the unified Vortex API instance
    logger.Info(TAG, LogColor::BRIGHT_YELLOW, LogStyle::NORMAL, "Obtaining Vortex API instance...");
    auto& vortex = Vortex::GetInstance();
    logger.Info(TAG, LogColor::GREEN, LogStyle::BOLD, "✓ Vortex API instance obtained");
    
    // Initialize all Vortex systems with proper dependency management
    logger.Info(TAG, LogColor::BRIGHT_BLUE, LogStyle::BOLD, "Initializing HardFOC Vortex V1 systems...");
    if (vortex.EnsureInitialized()) {
        logger.Info(TAG, LogColor::BRIGHT_GREEN, LogStyle::BOLD, "✓ HardFOC Vortex V1 initialization successful!");
        
        // Get and display system diagnostics
        VortexSystemDiagnostics diagnostics;
        if (vortex.GetSystemDiagnostics(diagnostics)) {
            PrintSystemDiagnostics(diagnostics);
        }
        
        // Set initial LED status to indicate successful initialization
        vortex.leds.SetStatus(LedAnimation::STATUS_OK);
        
        // Initialize legacy thread system for compatibility
        InitializeLegacyThreads();
        
        logger.Info(TAG, LogColor::BRIGHT_MAGENTA, LogStyle::BOLD, "System Version: %s", vortex.GetSystemVersion().c_str());
        logger.Info(TAG, LogColor::BRIGHT_GREEN, LogStyle::BOLD, "All systems initialized successfully!");
        
        // Main system loop
        logger.Info(TAG, LogColor::BRIGHT_CYAN, LogStyle::BOLD, "Entering main system loop...");
        
        while (true) {
            // Periodic system health monitoring
            if (main_loop_count % SYSTEM_HEALTH_CHECK_INTERVAL == 0) {
                if (!vortex.PerformHealthCheck()) {
                    logger.Warn(TAG, LogColor::BRIGHT_RED, LogStyle::BOLD, "System health check failed!");
                    vortex.leds.SetStatus(LedAnimation::STATUS_WARN);
                } else {
                    vortex.leds.SetStatus(LedAnimation::STATUS_OK);
                }
                last_health_check_time = vortex.GetSystemUptimeMs();
            }
            
            // Periodic status logging
            if (main_loop_count % STATUS_LOG_INTERVAL == 0) {
                logger.Info(TAG, LogColor::CYAN, LogStyle::NORMAL, 
                           "System Status - Uptime: %llu ms, Free Heap: %" PRIu32 " bytes",
                           vortex.GetSystemUptimeMs(), esp_get_free_heap_size());
                
                // Monitor legacy threads
                MonitorLegacyThreads();
            }
            
            // Demonstrate component functionality
            if (main_loop_count % DEMO_INTERVAL == 0) {
                logger.Info(TAG, LogColor::BRIGHT_WHITE, LogStyle::BOLD, "Running component demonstrations...");
                DemonstrateGpio(vortex);
                DemonstrateAdc(vortex);
                DemonstrateLeds(vortex);
                DemonstrateTemperature(vortex);
            }
            
            // Advanced component demonstrations
            if (main_loop_count % DEMO_INTERVAL == DEMO_INTERVAL / 2) {
                logger.Info(TAG, LogColor::BRIGHT_WHITE, LogStyle::BOLD, "Running advanced component demonstrations...");
                DemonstrateMotorController(vortex);
                DemonstrateImu(vortex);
                DemonstrateEncoders(vortex);
            }
            
            // Motor control demonstration (if CANOpen thread is running)
            if (main_loop_count % MOTOR_DEMO_INTERVAL == 150 && canopen_bldc_thread.IsThreadRunning()) {
                logger.Info(TAG, LogColor::BRIGHT_BLUE, LogStyle::BOLD, "Demonstrating motor control commands");
                
                // Enable motor
                canopen_bldc_thread.EnableMotor();
                vTaskDelay(pdMS_TO_TICKS(1000));
                
                // Try velocity mode
                canopen_bldc_thread.SetVelocityMode(100); // 100 RPM
                vTaskDelay(pdMS_TO_TICKS(3000));
                
                // Try position mode
                canopen_bldc_thread.SetPositionMode(1000); // Move to position 1000
                vTaskDelay(pdMS_TO_TICKS(3000));
                
                // Disable motor
                canopen_bldc_thread.DisableMotor();
            }
            
            // LED status blinking
            if (main_loop_count % LED_BLINK_INTERVAL == 0) {
                // Brief status blink to show system is alive
                static bool blink_state = false;
                if (blink_state) {
                    vortex.leds.SetStatus(LedAnimation::STATUS_OK);
                } else {
                    vortex.leds.SetStatus(LedAnimation::STATUS_INIT);
                }
                blink_state = !blink_state;
            }
            
            // Increment loop counter and delay
            main_loop_count++;
            vTaskDelay(pdMS_TO_TICKS(MAIN_LOOP_DELAY_MS));
        }
        
    } else {
        logger.Error(TAG, LogColor::BRIGHT_RED, LogStyle::BOLD, "✗ HardFOC Vortex V1 initialization failed!");
        
        // Show detailed failure information
        auto failed_components = vortex.GetFailedComponents();
        if (!failed_components.empty()) {
            logger.Error(TAG, LogColor::RED, LogStyle::BOLD, "Failed components:");
            for (const auto& component : failed_components) {
                logger.Error(TAG, LogColor::RED, LogStyle::NORMAL, "  - %s", component.c_str());
            }
        }
        
        auto warnings = vortex.GetSystemWarnings();
        if (!warnings.empty()) {
            logger.Error(TAG, LogColor::YELLOW, LogStyle::BOLD, "System warnings:");
            for (const auto& warning : warnings) {
                logger.Error(TAG, LogColor::YELLOW, LogStyle::NORMAL, "  - %s", warning.c_str());
            }
        }
        
        // Set LED to error status
        vortex.leds.SetStatus(LedAnimation::STATUS_ERROR);
        
        // Stay in error state with periodic retries
        while (true) {
            logger.Error(TAG, LogColor::BRIGHT_RED, LogStyle::BOLD, "System in error state - attempting recovery...");
            vTaskDelay(pdMS_TO_TICKS(5000));
            
            // Attempt recovery
            if (vortex.EnsureInitialized()) {
                logger.Info(TAG, LogColor::BRIGHT_GREEN, LogStyle::BOLD, "✓ System recovery successful!");
                break;
            }
        }
    }
}

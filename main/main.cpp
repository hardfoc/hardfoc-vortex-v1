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
#include <esp_log.h>

// Vortex API - Unified interface to all HardFOC Vortex V1 components
#include "API/Vortex.h"

// Thread system (kept for compatibility but will be integrated with Vortex)
#include "CANOpenBLDCThread.h"
#include "WS2812TestThread.h"

//==============================================================================
// CONSTANTS AND CONFIGURATION
//==============================================================================

static const char* TAG = "VortexMain";

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
    ESP_LOGI(TAG, "=== ESP32-C6 Chip Information ===");
    
    esp_chip_info_t chip_info;
    uint32_t flash_size;
    esp_chip_info(&chip_info);
    
    ESP_LOGI(TAG, "Chip: %s with %d CPU core(s)", CONFIG_IDF_TARGET, chip_info.cores);
    ESP_LOGI(TAG, "Features: %s%s%s%s", 
             (chip_info.features & CHIP_FEATURE_WIFI_BGN) ? "WiFi/" : "",
             (chip_info.features & CHIP_FEATURE_BT) ? "BT" : "",
             (chip_info.features & CHIP_FEATURE_BLE) ? "BLE" : "",
             (chip_info.features & CHIP_FEATURE_IEEE802154) ? ", 802.15.4 (Zigbee/Thread)" : "");

    unsigned major_rev = chip_info.revision / 100;
    unsigned minor_rev = chip_info.revision % 100;
    ESP_LOGI(TAG, "Silicon revision: v%d.%d", major_rev, minor_rev);
    
    if (esp_flash_get_size(NULL, &flash_size) == ESP_OK) {
        ESP_LOGI(TAG, "Flash size: %" PRIu32 "MB %s", 
                 flash_size / (uint32_t)(1024 * 1024),
                 (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");
    }
    
    ESP_LOGI(TAG, "Free heap: %" PRIu32 " bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "Min free heap: %" PRIu32 " bytes", esp_get_minimum_free_heap_size());
}

/**
 * @brief Print detailed system diagnostics
 */
void PrintSystemDiagnostics(const VortexSystemDiagnostics& diagnostics) {
    ESP_LOGI(TAG, "=== HardFOC Vortex V1 System Diagnostics ===");
    ESP_LOGI(TAG, "Overall System Health: %s", diagnostics.system_healthy ? "HEALTHY" : "UNHEALTHY");
    ESP_LOGI(TAG, "Initialized Components: %u/%u", diagnostics.initialized_components, diagnostics.total_components);
    ESP_LOGI(TAG, "Failed Components: %u", diagnostics.failed_components);
    ESP_LOGI(TAG, "Initialization Time: %llu ms", diagnostics.initialization_time_ms);
    ESP_LOGI(TAG, "System Uptime: %llu ms", diagnostics.system_uptime_ms);
    
    ESP_LOGI(TAG, "Component Status:");
    ESP_LOGI(TAG, "  Communication Channels: %s", diagnostics.comms_initialized ? "✓ OK" : "✗ FAIL");
    ESP_LOGI(TAG, "  GPIO Management: %s", diagnostics.gpio_initialized ? "✓ OK" : "✗ FAIL");
    ESP_LOGI(TAG, "  Motor Controllers: %s", diagnostics.motors_initialized ? "✓ OK" : "✗ FAIL");
    ESP_LOGI(TAG, "  ADC Management: %s", diagnostics.adc_initialized ? "✓ OK" : "✗ FAIL");
    ESP_LOGI(TAG, "  IMU Management: %s", diagnostics.imu_initialized ? "✓ OK" : "✗ FAIL");
    ESP_LOGI(TAG, "  Encoder Management: %s", diagnostics.encoders_initialized ? "✓ OK" : "✗ FAIL");
    ESP_LOGI(TAG, "  LED Management: %s", diagnostics.leds_initialized ? "✓ OK" : "✗ FAIL");
    ESP_LOGI(TAG, "  Temperature Management: %s", diagnostics.temp_initialized ? "✓ OK" : "✗ FAIL");
    
    if (!diagnostics.failed_components_list.empty()) {
        ESP_LOGW(TAG, "Failed Components:");
        for (const auto& component : diagnostics.failed_components_list) {
            ESP_LOGW(TAG, "  - %s", component.c_str());
        }
    }
    
    if (!diagnostics.warnings.empty()) {
        ESP_LOGW(TAG, "System Warnings:");
        for (const auto& warning : diagnostics.warnings) {
            ESP_LOGW(TAG, "  - %s", warning.c_str());
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
    ESP_LOGI(TAG, "=== GPIO Management Demo ===");
    
    auto& gpio = vortex.gpio;
    
    // Configure and control ESP32 GPIO pins
    if (gpio.Set("GPIO_EXT_GPIO_CS_1", true) == hf_gpio_err_t::GPIO_SUCCESS) {
        ESP_LOGI(TAG, "✓ ESP32 CS1 pin set HIGH");
    } else {
        ESP_LOGW(TAG, "✗ Failed to set ESP32 CS1 pin");
    }
    
    vTaskDelay(pdMS_TO_TICKS(200));
    
    if (gpio.Set("GPIO_EXT_GPIO_CS_1", false) == hf_gpio_err_t::GPIO_SUCCESS) {
        ESP_LOGI(TAG, "✓ ESP32 CS1 pin set LOW");
    }
    
    // Read PCAL95555 GPIO pins
    bool gpio17_state, gpio18_state;
    if (gpio.Read("GPIO_PCAL_GPIO17", gpio17_state) == hf_gpio_err_t::GPIO_SUCCESS) {
        ESP_LOGI(TAG, "✓ PCAL95555 GPIO17: %s", gpio17_state ? "HIGH" : "LOW");
    }
    
    if (gpio.Read("GPIO_PCAL_GPIO18", gpio18_state) == hf_gpio_err_t::GPIO_SUCCESS) {
        ESP_LOGI(TAG, "✓ PCAL95555 GPIO18: %s", gpio18_state ? "HIGH" : "LOW");
    }
    
    ESP_LOGI(TAG, "GPIO demo completed");
}

/**
 * @brief Demonstrate ADC functionality
 */
void DemonstrateAdc(Vortex& vortex) {
    ESP_LOGI(TAG, "=== ADC Management Demo ===");
    
    auto& adc = vortex.adc;
    
    // Read TMC9660 ADC channels
    float voltage;
    
    if (adc.ReadChannelV("ADC_TMC9660_AIN0", voltage) == hf_adc_err_t::ADC_SUCCESS) {
        ESP_LOGI(TAG, "✓ TMC9660 AIN0: %.3f V", voltage);
    } else {
        ESP_LOGW(TAG, "✗ TMC9660 AIN0 read failed");
    }
    
    if (adc.ReadChannelV("ADC_TMC9660_AIN3", voltage) == hf_adc_err_t::ADC_SUCCESS) {
        ESP_LOGI(TAG, "✓ TMC9660 AIN3 (Temp): %.3f V", voltage);
    } else {
        ESP_LOGW(TAG, "✗ TMC9660 AIN3 read failed");
    }
    
    // Read TMC9660 internal monitoring channels
    if (adc.ReadChannelV("TMC9660_CURRENT_I0", voltage) == hf_adc_err_t::ADC_SUCCESS) {
        ESP_LOGI(TAG, "✓ TMC9660 Current I0: %.3f A", voltage);
    }
    
    if (adc.ReadChannelV("TMC9660_CHIP_TEMPERATURE", voltage) == hf_adc_err_t::ADC_SUCCESS) {
        ESP_LOGI(TAG, "✓ TMC9660 Chip Temp: %.1f°C", voltage);
    }
    
    if (adc.ReadChannelV("TMC9660_SUPPLY_VOLTAGE", voltage) == hf_adc_err_t::ADC_SUCCESS) {
        ESP_LOGI(TAG, "✓ TMC9660 Supply: %.3f V", voltage);
    }
    
    ESP_LOGI(TAG, "ADC demo completed");
}

/**
 * @brief Demonstrate motor controller functionality
 */
void DemonstrateMotorController(Vortex& vortex) {
    ESP_LOGI(TAG, "=== Motor Controller Demo ===");
    
    auto& motors = vortex.motors;
    
    // Get onboard TMC9660 handler
    auto* handler = motors.handler(0);
    if (handler) {
        ESP_LOGI(TAG, "✓ TMC9660 handler available");
        
        // Get underlying driver
        auto driver = motors.driver(0);
        if (driver) {
            ESP_LOGI(TAG, "✓ TMC9660 driver available");
            
            // Configure motor parameters (safe values)
            driver->SetTargetVelocity(100);  // 100 RPM
            driver->SetMaxCurrent(500);      // 500 mA
            
            ESP_LOGI(TAG, "✓ Motor parameters configured");
            
            // Note: Motor enabling would be done here in a real application
            // driver->EnableMotor(true);
        } else {
            ESP_LOGW(TAG, "✗ TMC9660 driver not available");
        }
    } else {
        ESP_LOGW(TAG, "✗ TMC9660 handler not available");
    }
    
    uint8_t device_count = motors.GetDeviceCount();
    ESP_LOGI(TAG, "Active motor devices: %u", device_count);
    
    ESP_LOGI(TAG, "Motor controller demo completed");
}

/**
 * @brief Demonstrate IMU functionality
 */
void DemonstrateImu(Vortex& vortex) {
    ESP_LOGI(TAG, "=== IMU Management Demo ===");
    
    auto& imu = vortex.imu;
    
    // Get BNO08x handler
    auto* handler = imu.GetBno08xHandler(0);
    if (handler) {
        ESP_LOGI(TAG, "✓ BNO08x handler available");
        
        // Get underlying driver
        auto driver = imu.GetBno085Driver(0);
        if (driver) {
            ESP_LOGI(TAG, "✓ BNO085 driver available");
        } else {
            ESP_LOGW(TAG, "✗ BNO085 driver not available");
        }
    } else {
        ESP_LOGW(TAG, "✗ BNO08x handler not available");
    }
    
    uint8_t device_count = imu.GetDeviceCount();
    ESP_LOGI(TAG, "Active IMU devices: %u", device_count);
    
    ESP_LOGI(TAG, "IMU demo completed");
}

/**
 * @brief Demonstrate encoder functionality
 */
void DemonstrateEncoders(Vortex& vortex) {
    ESP_LOGI(TAG, "=== Encoder Management Demo ===");
    
    auto& encoders = vortex.encoders;
    
    // Get AS5047U handler
    auto* handler = encoders.GetAs5047uHandler(0);
    if (handler) {
        ESP_LOGI(TAG, "✓ AS5047U handler available");
        
        // Try to read encoder angle
        uint16_t angle;
        if (encoders.ReadAngle(0, angle) == As5047uError::SUCCESS) {
            ESP_LOGI(TAG, "✓ Encoder angle: %u LSB (%.2f°)", angle, (float)angle * 360.0f / 16384.0f);
        } else {
            ESP_LOGW(TAG, "✗ Encoder angle read failed");
        }
        
        // Try to read encoder velocity
        double velocity_rpm;
        if (encoders.ReadVelocityRPM(0, velocity_rpm) == As5047uError::SUCCESS) {
            ESP_LOGI(TAG, "✓ Encoder velocity: %.2f RPM", velocity_rpm);
        } else {
            ESP_LOGW(TAG, "✗ Encoder velocity read failed");
        }
    } else {
        ESP_LOGW(TAG, "✗ AS5047U handler not available");
    }
    
    uint8_t device_count = encoders.GetDeviceCount();
    ESP_LOGI(TAG, "Active encoder devices: %u", device_count);
    
    ESP_LOGI(TAG, "Encoder demo completed");
}

/**
 * @brief Demonstrate LED functionality
 */
void DemonstrateLeds(Vortex& vortex) {
    ESP_LOGI(TAG, "=== LED Management Demo ===");
    
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
    
    for (size_t i = 0; i < sizeof(animations) / sizeof(animations[0]); i++) {
        if (leds.SetStatus(animations[i]) == LedError::SUCCESS) {
            ESP_LOGI(TAG, "✓ LED set to %s", animation_names[i]);
        } else {
            ESP_LOGW(TAG, "✗ Failed to set LED to %s", animation_names[i]);
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    
    // Set back to OK status
    leds.SetStatus(LedAnimation::STATUS_OK);
    
    ESP_LOGI(TAG, "LED demo completed");
}

/**
 * @brief Demonstrate temperature monitoring
 */
void DemonstrateTemperature(Vortex& vortex) {
    ESP_LOGI(TAG, "=== Temperature Management Demo ===");
    
    auto& temp = vortex.temp;
    
    float temperature;
    
    // Read ESP32 internal temperature
    if (temp.ReadTemperatureCelsius("ESP32_INTERNAL", temperature) == hf_temp_err_t::HF_TEMP_SUCCESS) {
        ESP_LOGI(TAG, "✓ ESP32 Internal: %.2f°C", temperature);
    } else {
        ESP_LOGW(TAG, "✗ ESP32 Internal temperature read failed");
    }
    
    // Read motor temperature
    if (temp.ReadTemperatureCelsius("MOTOR_TEMP", temperature) == hf_temp_err_t::HF_TEMP_SUCCESS) {
        ESP_LOGI(TAG, "✓ Motor Temperature: %.2f°C", temperature);
    } else {
        ESP_LOGW(TAG, "✗ Motor temperature read failed");
    }
    
    // Read NTC thermistor
    if (temp.ReadTemperatureCelsius("NTC_THERMISTOR", temperature) == hf_temp_err_t::HF_TEMP_SUCCESS) {
        ESP_LOGI(TAG, "✓ NTC Thermistor: %.2f°C", temperature);
    } else {
        ESP_LOGW(TAG, "✗ NTC thermistor read failed");
    }
    
    ESP_LOGI(TAG, "Temperature demo completed");
}

//==============================================================================
// THREAD INTEGRATION (Legacy Compatibility)
//==============================================================================

/**
 * @brief Initialize and start legacy thread system
 */
void InitializeLegacyThreads() {
    ESP_LOGI(TAG, "Initializing legacy thread system...");
    
    // Initialize WS2812 test thread
    if (ws2812_thread.EnsureInitialized()) {
        ESP_LOGI(TAG, "✓ WS2812 test thread initialized");
        if (ws2812_thread.Start()) {
            ESP_LOGI(TAG, "✓ WS2812 test thread started");
        } else {
            ESP_LOGW(TAG, "✗ Failed to start WS2812 test thread");
        }
    } else {
        ESP_LOGW(TAG, "✗ Failed to initialize WS2812 test thread");
    }
    
    // Initialize CANOpen BLDC thread
    if (canopen_bldc_thread.EnsureInitialized()) {
        ESP_LOGI(TAG, "✓ CANOpen BLDC thread initialized");
        if (canopen_bldc_thread.Start()) {
            ESP_LOGI(TAG, "✓ CANOpen BLDC thread started");
        } else {
            ESP_LOGW(TAG, "✗ Failed to start CANOpen BLDC thread");
        }
    } else {
        ESP_LOGW(TAG, "✗ Failed to initialize CANOpen BLDC thread");
    }
}

/**
 * @brief Monitor legacy thread health
 */
void MonitorLegacyThreads() {
    bool ws2812_running = ws2812_thread.IsThreadRunning();
    bool canopen_running = canopen_bldc_thread.IsThreadRunning();
    
    ESP_LOGI(TAG, "Thread Status - WS2812: %s, CANOpen: %s",
             ws2812_running ? "Running" : "Stopped",
             canopen_running ? "Running" : "Stopped");
    
    // Print motor status if CANOpen thread is running
    if (canopen_running) {
        auto motor_status = canopen_bldc_thread.GetMotorStatus();
        ESP_LOGI(TAG, "Motor - Present: %s, Enabled: %s, Pos: %d, Vel: %d",
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
    // Print banner
    printf("\n");
    printf("========================================\n");
    printf("  HardFOC Vortex V1 - Unified API Demo\n");
    printf("========================================\n");
    printf("ESP-IDF Version: %s\n", esp_get_idf_version());
    printf("Compile Time: %s %s\n", __DATE__, __TIME__);
    printf("========================================\n\n");
    
    ESP_LOGI(TAG, "Starting HardFOC Vortex V1 application");
    
    // Print chip information
    PrintChipInfo();
    
    // Get the unified Vortex API instance
    ESP_LOGI(TAG, "Obtaining Vortex API instance...");
    auto& vortex = Vortex::GetInstance();
    ESP_LOGI(TAG, "✓ Vortex API instance obtained");
    
    // Initialize all Vortex systems with proper dependency management
    ESP_LOGI(TAG, "Initializing HardFOC Vortex V1 systems...");
    if (vortex.EnsureInitialized()) {
        ESP_LOGI(TAG, "✓ HardFOC Vortex V1 initialization successful!");
        
        // Get and display system diagnostics
        VortexSystemDiagnostics diagnostics;
        if (vortex.GetSystemDiagnostics(diagnostics)) {
            PrintSystemDiagnostics(diagnostics);
        }
        
        // Set initial LED status to indicate successful initialization
        vortex.leds.SetStatus(LedAnimation::STATUS_OK);
        
        // Initialize legacy thread system for compatibility
        InitializeLegacyThreads();
        
        ESP_LOGI(TAG, "System Version: %s", vortex.GetSystemVersion().c_str());
        ESP_LOGI(TAG, "All systems initialized successfully!");
        
        // Main system loop
        ESP_LOGI(TAG, "Entering main system loop...");
        
        while (true) {
            // Periodic system health monitoring
            if (main_loop_count % SYSTEM_HEALTH_CHECK_INTERVAL == 0) {
                if (!vortex.PerformHealthCheck()) {
                    ESP_LOGW(TAG, "System health check failed!");
                    vortex.leds.SetStatus(LedAnimation::STATUS_WARN);
                } else {
                    vortex.leds.SetStatus(LedAnimation::STATUS_OK);
                }
                last_health_check_time = vortex.GetSystemUptimeMs();
            }
            
            // Periodic status logging
            if (main_loop_count % STATUS_LOG_INTERVAL == 0) {
                ESP_LOGI(TAG, "System Status - Uptime: %llu ms, Free Heap: %" PRIu32 " bytes",
                         vortex.GetSystemUptimeMs(), esp_get_free_heap_size());
                
                // Monitor legacy threads
                MonitorLegacyThreads();
            }
            
            // Demonstrate component functionality
            if (main_loop_count % DEMO_INTERVAL == 0) {
                ESP_LOGI(TAG, "Running component demonstrations...");
                DemonstrateGpio(vortex);
                DemonstrateAdc(vortex);
                DemonstrateLeds(vortex);
                DemonstrateTemperature(vortex);
            }
            
            // Advanced component demonstrations
            if (main_loop_count % DEMO_INTERVAL == DEMO_INTERVAL / 2) {
                ESP_LOGI(TAG, "Running advanced component demonstrations...");
                DemonstrateMotorController(vortex);
                DemonstrateImu(vortex);
                DemonstrateEncoders(vortex);
            }
            
            // Motor control demonstration (if CANOpen thread is running)
            if (main_loop_count % MOTOR_DEMO_INTERVAL == 150 && canopen_bldc_thread.IsThreadRunning()) {
                ESP_LOGI(TAG, "Demonstrating motor control commands");
                
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
        ESP_LOGE(TAG, "✗ HardFOC Vortex V1 initialization failed!");
        
        // Show detailed failure information
        auto failed_components = vortex.GetFailedComponents();
        if (!failed_components.empty()) {
            ESP_LOGE(TAG, "Failed components:");
            for (const auto& component : failed_components) {
                ESP_LOGE(TAG, "  - %s", component.c_str());
            }
        }
        
        auto warnings = vortex.GetSystemWarnings();
        if (!warnings.empty()) {
            ESP_LOGE(TAG, "System warnings:");
            for (const auto& warning : warnings) {
                ESP_LOGE(TAG, "  - %s", warning.c_str());
            }
        }
        
        // Set LED to error status
        vortex.leds.SetStatus(LedAnimation::STATUS_ERROR);
        
        // Stay in error state with periodic retries
        while (true) {
            ESP_LOGE(TAG, "System in error state - attempting recovery...");
            vTaskDelay(pdMS_TO_TICKS(5000));
            
            // Attempt recovery
            if (vortex.EnsureInitialized()) {
                ESP_LOGI(TAG, "✓ System recovery successful!");
                break;
            }
        }
    }
}

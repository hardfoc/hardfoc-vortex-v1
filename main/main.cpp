
#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "esp_log.h"

// Include thread headers
#include "WS2812TestThread.h"
#include "CANOpenBLDCThread.h"

// Include GPIO configuration
#include "hf_gpio_config.hpp"

static const char* TAG = "HardFOC";

void printChipInfo(void)
{
    /* Print chip information */
    esp_chip_info_t chip_info;
    uint32_t flash_size;
    esp_chip_info(&chip_info);
    printf("This is %s chip with %d CPU core(s), %s%s%s%s, ",
           CONFIG_IDF_TARGET,
           chip_info.cores,
           (chip_info.features & CHIP_FEATURE_WIFI_BGN) ? "WiFi/" : "",
           (chip_info.features & CHIP_FEATURE_BT) ? "BT" : "",
           (chip_info.features & CHIP_FEATURE_BLE) ? "BLE" : "",
           (chip_info.features & CHIP_FEATURE_IEEE802154) ? ", 802.15.4 (Zigbee/Thread)" : "");

    unsigned major_rev = chip_info.revision / 100;
    unsigned minor_rev = chip_info.revision % 100;
    printf("silicon revision v%d.%d, ", major_rev, minor_rev);
    if(esp_flash_get_size(NULL, &flash_size) != ESP_OK) {
        printf("Get flash size failed");
        return;
    }

    printf("%" PRIu32 "MB %s flash\n", flash_size / (uint32_t)(1024 * 1024),
           (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    printf("Minimum free heap size: %" PRIu32 " bytes\n", esp_get_minimum_free_heap_size());
}

// Global thread instances
WS2812TestThread ws2812_thread;
CANOpenBLDCThread canopen_bldc_thread;

extern "C" void app_main(void)
{
    printf("HardFOC - Multi-threaded Motor Control System\n");
    ESP_LOGI(TAG, "Starting HardFOC application");

    printChipInfo();

    // Initialize GPIO configuration
    ESP_LOGI(TAG, "Configuring GPIO pins");
    init_mcu_pinconfig();

    ESP_LOGI(TAG, "Initializing and starting threads...");

    // Initialize WS2812 test thread
    if (ws2812_thread.EnsureInitialized()) {
        ESP_LOGI(TAG, "WS2812 test thread initialized successfully");
        if (ws2812_thread.Start()) {
            ESP_LOGI(TAG, "WS2812 test thread started");
        } else {
            ESP_LOGE(TAG, "Failed to start WS2812 test thread");
        }
    } else {
        ESP_LOGE(TAG, "Failed to initialize WS2812 test thread");
    }

    // Initialize CANOpen BLDC thread
    if (canopen_bldc_thread.EnsureInitialized()) {
        ESP_LOGI(TAG, "CANOpen BLDC thread initialized successfully");
        if (canopen_bldc_thread.Start()) {
            ESP_LOGI(TAG, "CANOpen BLDC thread started");
        } else {
            ESP_LOGE(TAG, "Failed to start CANOpen BLDC thread");
        }
    } else {
        ESP_LOGE(TAG, "Failed to initialize CANOpen BLDC thread");
    }

    ESP_LOGI(TAG, "All threads started successfully");

    // Main loop - monitor threads and perform system-level tasks
    uint32_t loopCount = 0;
    while (true) {
        // Check thread health
        bool ws2812Running = ws2812_thread.IsThreadRunning();
        bool canOpenRunning = canopen_bldc_thread.IsThreadRunning();

        // Log status periodically
        if (loopCount % 100 == 0) {  // Every 10 seconds
            ESP_LOGI(TAG, "System Status - WS2812: %s, CANOpen: %s", 
                     ws2812Running ? "Running" : "Stopped",
                     canOpenRunning ? "Running" : "Stopped");

            // Print motor status if CANOpen thread is running
            if (canOpenRunning) {
                auto motorStatus = canopen_bldc_thread.GetMotorStatus();
                ESP_LOGI(TAG, "Motor - Present: %s, Enabled: %s, Pos: %d, Vel: %d",
                         motorStatus.nodePresent ? "Yes" : "No",
                         motorStatus.isEnabled ? "Yes" : "No",
                         motorStatus.actualPosition, motorStatus.actualVelocity);
            }

            // Print free heap
            printf("Free heap: %" PRIu32 " bytes\n", esp_get_free_heap_size());
        }

        // Demonstrate motor control every 30 seconds
        if (loopCount % 300 == 150 && canopen_bldc_thread.IsThreadRunning()) {
            ESP_LOGI(TAG, "Demonstrating motor control commands");
            
            // Enable motor
            canopen_bldc_thread.EnableMotor();
            vTaskDelay(pdMS_TO_TICKS(1000));
            
            // Try velocity mode
            canopen_bldc_thread.SetVelocityMode(100);  // 100 RPM
            vTaskDelay(pdMS_TO_TICKS(5000));
            
            // Try position mode
            canopen_bldc_thread.SetPositionMode(1000);  // Move to position 1000
            vTaskDelay(pdMS_TO_TICKS(3000));
            
            // Disable motor
            canopen_bldc_thread.DisableMotor();
        }

        loopCount++;
        vTaskDelay(pdMS_TO_TICKS(100));  // 100ms main loop
    }

    // This code would only be reached if the main loop exits (shouldn't happen in normal operation)
    ESP_LOGI(TAG, "Stopping threads");
    ws2812_thread.Stop();
    canopen_bldc_thread.Stop();
}
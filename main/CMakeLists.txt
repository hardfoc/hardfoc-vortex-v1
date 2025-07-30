# Main component for HardFOC Vortex V1 application

idf_component_register(
    SRCS "main.cpp"
    
    # Public requirements - components that main depends on and that should be
    # available to any component that depends on main
    REQUIRES 
        freertos
        esp_system
        esp_common
        esp_timer
        esp_driver_gpio
        esp_driver_spi_master
        esp_driver_i2c_master
        esp_driver_uart
        esp_driver_ledc
        esp_adc
        log
        hf-hal-vortex-v1
        hf-ws2812-rmt-driver
        threads
    
    # Private requirements - components that main uses internally but doesn't
    # expose to dependent components
    PRIV_REQUIRES 
        spi_flash
        nvs_flash
        esp_partition
        hal
        soc
        esp_hw_support
        esp_rom
        esp_netif
        esp_event
        esp_wifi
        esp_pm
        esp_ota
        esp_http_client
        esp_https_ota
    
    # Include directories
    INCLUDE_DIRS "."
)

# Set C++ standard for this component
target_compile_features(${COMPONENT_LIB} PRIVATE cxx_std_17)

# Component-specific compile options for better debugging and performance
target_compile_options(${COMPONENT_LIB} PRIVATE
    -Wall
    -Wextra
    -Wno-unused-parameter
    -Wno-missing-field-initializers
    -O2
    -g
)

# Component-specific compile definitions
target_compile_definitions(${COMPONENT_LIB} PRIVATE
    HARDFOC_MAIN_APPLICATION=1
    HARDFOC_VORTEX_API_MAIN=1
    ESP_IDF_VERSION_MAJOR=5
    ESP_IDF_VERSION_MINOR=5
    LOG_LOCAL_LEVEL=ESP_LOG_INFO
)

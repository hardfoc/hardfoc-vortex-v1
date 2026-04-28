# HardFOC Vortex V1 â€” `main` component (Flux-style layout, no `middleware/`)
#
#   main/
#     app_main.cpp                 Thin IDF entry (like Flux `app_main.cpp`)
#     apps/
#       system/api/                `VortexSystemStateMachine`
#       system/sub_threads/        `VortexSystemThread` (boot table)
#       supervisor/                Main-task loop after boot (`VortexSupervisor`)
#       node/sub_threads/          Device workers: WS2812, CAN, CANopen host
#     utils/
#       CANopen transport: `CanOpenBaseCanLink` in hf-core `handlers/common/canopen/`
#       vortex_demos.h             Optional demo declarations (legacy)

set(MAIN_SRCS
    "app_main.cpp"
    "apps/supervisor/VortexSupervisor.cpp"
    "apps/system/api/VortexSystemStateMachine.cpp"
    "apps/system/sub_threads/VortexSystemThread.cpp"
    "apps/node/sub_threads/ws2812/WS2812TestThread.cpp"
    "apps/node/sub_threads/canopen_bldc/CANOpenBLDCThread.cpp"
    "apps/node/sub_threads/canopen_bldc/EnhancedCANOpenUtils.cpp"
    "apps/node/sub_threads/canopen_host/CanOpenHostServiceThread.cpp"
    "apps/node/sub_threads/vortex_host/VortexCanHostThread.cpp"
)
if(CONFIG_VORTEX_APP_CAN_BOOT_CANOPEN_SLAVE)
    list(APPEND MAIN_SRCS "apps/node/canopen/CanopenSlaveRuntime.cpp")
endif()

idf_component_register(
    SRCS ${MAIN_SRCS}
    REQUIRES
        freertos
        esp_system
        esp_common
        esp_timer
        esp_driver_gpio
        esp_driver_spi
        esp_driver_i2c
        esp_driver_uart
        esp_driver_ledc
        esp_adc
        log
        hf-hal-vortex-v1
        hf-ws2812-rmt-driver
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
        app_update
        esp_http_client
        esp_https_ota
    INCLUDE_DIRS
        "."
        "apps/supervisor"
        "apps/system/api"
        "apps/system/sub_threads"
        "apps/node/sub_threads/ws2812"
        "apps/node/sub_threads/canopen_bldc"
        "apps/node/sub_threads/canopen_host"
        "apps/node/sub_threads/vortex_host"
        "apps/node/canopen"
        "utils"
)

target_compile_features(${COMPONENT_LIB} PRIVATE cxx_std_17)

target_compile_options(${COMPONENT_LIB} PRIVATE
    -Wall
    -Wextra
    -Wno-unused-parameter
    -Wno-missing-field-initializers
    -O2
    -g
)

target_compile_definitions(${COMPONENT_LIB} PRIVATE
    HARDFOC_MAIN_APPLICATION=1
    HARDFOC_VORTEX_API_MAIN=1
    LOG_LOCAL_LEVEL=ESP_LOG_INFO
)


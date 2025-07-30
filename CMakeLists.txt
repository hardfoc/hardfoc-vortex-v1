# HardFOC Vortex V1 - ESP-IDF v5.5+ Project Configuration
# This project demonstrates the unified Vortex API for HardFOC Vortex V1 motor controller board

cmake_minimum_required(VERSION 3.16)

# Set project name and version
set(PROJECT_NAME "HardFOC_Vortex_V1")
set(PROJECT_VER "2.0.0")

# Tell ESP-IDF to treat hf-hal-vortex-v1 and threads as extra components
set(EXTRA_COMPONENT_DIRS
    ${CMAKE_CURRENT_LIST_DIR}/hf-hal-vortex-v1
    ${CMAKE_CURRENT_LIST_DIR}/hf-hal-vortex-v1/utils-and-drivers/hf-core-drivers/external
    ${CMAKE_CURRENT_LIST_DIR}/hf-hal-vortex-v1/utils-and-drivers/hf-core-drivers/external/hf-ws2812-rmt-driver
    ${CMAKE_CURRENT_LIST_DIR}/threads
)

# Include ESP-IDF project configuration
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# Set compiler standards for ESP-IDF v5.5+
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Enable verbose build output for debugging
set(CMAKE_VERBOSE_MAKEFILE ON)

# Set build type for better optimization
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# Project-specific compile definitions
add_compile_definitions(
    HARDFOC_VORTEX_V1=1
    HARDFOC_VORTEX_API=1
    ESP_IDF_VERSION_MAJOR=5
    ESP_IDF_VERSION_MINOR=5
)

# Create the project
project(${PROJECT_NAME} VERSION ${PROJECT_VER})

# Print build information
message(STATUS "=== HardFOC Vortex V1 Build Configuration ===")
message(STATUS "Project: ${PROJECT_NAME} v${PROJECT_VER}")
message(STATUS "ESP-IDF Version: $ENV{IDF_VER}")
message(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")
message(STATUS "C Standard: ${CMAKE_C_STANDARD}")
message(STATUS "C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "Extra Components: ${EXTRA_COMPONENT_DIRS}")
message(STATUS "=== End Build Configuration ===")

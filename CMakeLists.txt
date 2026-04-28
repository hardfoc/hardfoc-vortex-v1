# =============================================================================
# HardFOC Vortex V1 — ESP-IDF project (v5.5+)
# =============================================================================
# Layout matches Flux: HAL is the `hf-hal-vortex-v1` component under `hal/`.
# =============================================================================

cmake_minimum_required(VERSION 3.16)

set(PROJECT_NAME "HardFOC_Vortex_V1")
set(PROJECT_VER "2.0.0")

# --- HAL location (canonical) -------------------------------------------------
set(HF_HAL_VORTEX_DIR "${CMAKE_CURRENT_LIST_DIR}/hal/hf-hal-vortex-v1")
if(NOT EXISTS "${HF_HAL_VORTEX_DIR}/CMakeLists.txt")
    message(FATAL_ERROR
        "Vortex HAL not found at:\n"
        "  ${HF_HAL_VORTEX_DIR}\n"
        "Initialize the submodule from the repo root:\n"
        "  git submodule update --init --recursive\n"
        "Expected layout: hal/hf-hal-vortex-v1/ (see .gitmodules).")
endif()

# Core ships many optional ESP-IDF components from this single directory.
set(HF_CORE_DRIVER_EXT "${HF_HAL_VORTEX_DIR}/lib/core/hf-core-drivers/external")

list(APPEND EXTRA_COMPONENT_DIRS
    "${HF_HAL_VORTEX_DIR}"
    "${HF_CORE_DRIVER_EXT}"
    "${HF_CORE_DRIVER_EXT}/hf-ws2812-rmt-driver"
)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_VERBOSE_MAKEFILE ON)

# Language standards (after ESP-IDF `project.cmake` include; consumed by subprojects)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(${PROJECT_NAME} VERSION ${PROJECT_VER})

add_compile_definitions(
    HARDFOC_VORTEX_V1=1
    HARDFOC_VORTEX_API=1
    ESP_IDF_VERSION_MAJOR=5
    ESP_IDF_VERSION_MINOR=5
)

message(STATUS "=== HardFOC Vortex V1 ===")
message(STATUS "  HAL:           ${HF_HAL_VORTEX_DIR}")
message(STATUS "  Extra comps:   ${EXTRA_COMPONENT_DIRS}")
message(STATUS "  Build type:    ${CMAKE_BUILD_TYPE}")
message(STATUS "  C / C++:       ${CMAKE_C_STANDARD} / ${CMAKE_CXX_STANDARD}")
message(STATUS "=========================")

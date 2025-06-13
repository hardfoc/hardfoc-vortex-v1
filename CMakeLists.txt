# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# Minimum required by IDF tool‑chain
cmake_minimum_required(VERSION 3.16)

set(HF_HAL_ROOT ${CMAKE_CURRENT_SOURCE_DIR})

# Collect all C/C++ sources in this repo
file(GLOB_RECURSE HF_HAL_SRCS
     "${HF_HAL_ROOT}/utils-and-drivers/**/*.c"
     "${HF_HAL_ROOT}/utils-and-drivers/**/*.cpp")

idf_component_register(
        SRCS ${HF_HAL_SRCS}
        INCLUDE_DIRS "include" "utils-and-drivers/include"
        REQUIRES driver freertos        # public deps
        PRIV_REQUIRES hal esp_timer     # internal deps
)

# Optional: tighten warnings for this lib only
target_compile_options(${COMPONENT_LIB} PRIVATE -Wall -Wextra -Werror)

# "Trim" the build. Include the minimal set of components, main, and anything it depends on.
idf_build_set_property(MINIMAL_BUILD ON)
project(HardFOC)

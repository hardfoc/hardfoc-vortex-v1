# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

set(EXTRA_COMPONENT_DIRS
    "${CMAKE_CURRENT_SOURCE_DIR}/components"
    "${CMAKE_CURRENT_SOURCE_DIR}/HAL"
    "${CMAKE_CURRENT_SOURCE_DIR}/HAL/component_handlers"
    "${CMAKE_CURRENT_SOURCE_DIR}/HAL/internal_interface_drivers"
    "${CMAKE_CURRENT_SOURCE_DIR}/HAL/external_chip_drivers"
    "${CMAKE_CURRENT_SOURCE_DIR}/UTILITIES"
)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# "Trim" the build. Include the minimal set of components, main, and anything it depends on.
idf_build_set_property(MINIMAL_BUILD ON)
project(HardFOC)

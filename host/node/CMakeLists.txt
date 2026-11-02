# =============================================================================
# vortex_node_host: the Vortex CANopen device on a Linux / macOS host
# =============================================================================
# The same CANopenNode sources, generated OD.c, stack configuration, contracts
# and node application (CoNode, FrameRouter) as the firmware; only the CAN
# port (LinuxSocketCanPort or PipePort), the service thread (std::thread) and
# the HAL (sim/) differ. Design §9.4.
#
#   cmake -S host/node -B build/node-host -G Ninja
#   cmake --build build/node-host
#   build/node-host/vortex_node_host --node-id 5 --port vcan0
#   build/node-host/vortex_node_host --node-id 5 --port pipe:/tmp/vortex.sock
#
# Needs the HAL submodule with lib/core and third_party/CANopenNode.
# =============================================================================
cmake_minimum_required(VERSION 3.21)
project(vortex_node_host LANGUAGES C CXX)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)          # the firmware builds as C++17
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

get_filename_component(VORTEX_REPO_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
set(VORTEX_HAL_DIR "${VORTEX_REPO_ROOT}/hal/hf-hal-vortex-v1")
set(HF_CORE_ROOT "${VORTEX_HAL_DIR}/lib/core")
if(NOT EXISTS "${HF_CORE_ROOT}/cmake/hf_core_canopennode.cmake")
  message(FATAL_ERROR "hf-core not found at ${HF_CORE_ROOT}; run\n"
                      "  git submodule update --init hal/hf-hal-vortex-v1\n"
                      "  git -C hal/hf-hal-vortex-v1 submodule update --init lib/core third_party/CANopenNode")
endif()

option(VORTEX_NODE_SANITIZE "Build with AddressSanitizer + UBSan" OFF)

set(HF_CORE_CANOPENNODE_ROOT "${VORTEX_HAL_DIR}/third_party/CANopenNode")
set(HF_CORE_CANOPENNODE_OD_DIR "${VORTEX_REPO_ROOT}/od/canopennode")
set(HF_CORE_CANOPENNODE_CONFIG_DIR "${VORTEX_REPO_ROOT}/main/apps/node/canopen/config")
set(HF_CORE_CANOPENNODE_PLATFORM "POSIX")
set(HF_CORE_CANOPENNODE_RTOS "POSIX")
include("${HF_CORE_ROOT}/cmake/hf_core_canopennode.cmake")

find_package(Threads REQUIRED)

set(_san "")
if(VORTEX_NODE_SANITIZE)
  set(_san -fsanitize=address,undefined -fno-omit-frame-pointer)
endif()

# CANopenNode and the generated OD: third-party / generated code, warnings off.
add_library(vortex_co_stack STATIC ${HF_CORE_CANOPENNODE_STACK_SOURCES})
target_include_directories(vortex_co_stack PUBLIC ${HF_CORE_CANOPENNODE_INCLUDE_DIRS})
target_compile_definitions(vortex_co_stack PUBLIC ${HF_CORE_CANOPENNODE_DEFINITIONS})
target_compile_options(vortex_co_stack PRIVATE -w ${_san})

# hf-core's driver and host ports, and the node application: warning-clean.
set(_strict -Wall -Wextra -Werror)
add_library(vortex_co_driver STATIC ${HF_CORE_CANOPENNODE_DRIVER_SOURCES} ${HF_CORE_CANOPENNODE_PORT_SOURCES})
target_link_libraries(vortex_co_driver PUBLIC vortex_co_stack Threads::Threads)
target_compile_options(vortex_co_driver PRIVATE ${_strict} ${_san})

set(_node "${VORTEX_REPO_ROOT}/main/apps/node/canopen")
add_executable(vortex_node_host
  main.cpp
  "${_node}/CoNode.cpp"
  "${_node}/FrameRouter.cpp"
  "${_node}/CoSvcStd.cpp")
target_include_directories(vortex_node_host PRIVATE
  "${VORTEX_REPO_ROOT}"   # contracts/
  "${_node}"
  "${CMAKE_CURRENT_SOURCE_DIR}/sim")
target_link_libraries(vortex_node_host PRIVATE vortex_co_driver)
target_compile_options(vortex_node_host PRIVATE ${_strict} ${_san})
target_link_options(vortex_node_host PRIVATE ${_san})

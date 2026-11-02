# Host build of the generated OD.c with the full CANopenNode v4 stack and the
# upstream blank driver. Driven by tools/odgen/tests/test_od_c_build.py.
#   -DCANOPENNODE_DIR=<CANopenNode>  -DOD_DIR=<dir with OD.c / OD.h>  -DEXPECT_DIR=<dir with od_expect.h>
cmake_minimum_required(VERSION 3.20)
project(vortex_od_host_build C)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
option(MULTIPLE_OD "Build with CO_MULTIPLE_OD (exercises OD_INIT_CONFIG)" OFF)
option(ENTRIES_ONLY "Check the entries only (a fixture OD without the objects the stack needs to start)" OFF)

file(GLOB STACK_SOURCES
  "${CANOPENNODE_DIR}/301/*.c" "${CANOPENNODE_DIR}/303/*.c" "${CANOPENNODE_DIR}/304/*.c"
  "${CANOPENNODE_DIR}/305/*.c" "${CANOPENNODE_DIR}/309/*.c" "${CANOPENNODE_DIR}/storage/CO_storage.c"
  "${CANOPENNODE_DIR}/extra/*.c")

add_executable(od_stack_init
  od_stack_init.c
  "${OD_DIR}/OD.c"
  "${CANOPENNODE_DIR}/CANopen.c"
  "${CANOPENNODE_DIR}/example/CO_driver_blank.c"
  ${STACK_SOURCES})
target_include_directories(od_stack_init PRIVATE
  "${OD_DIR}" "${EXPECT_DIR}" "${CANOPENNODE_DIR}" "${CANOPENNODE_DIR}/example")
if(ENTRIES_ONLY)
  target_compile_definitions(od_stack_init PRIVATE ENTRIES_ONLY)
endif()
if(MULTIPLE_OD)
  target_compile_definitions(od_stack_init PRIVATE CO_MULTIPLE_OD)
endif()
# The generated OD must be warning-clean; the stack itself is third-party.
set_source_files_properties("${OD_DIR}/OD.c" od_stack_init.c PROPERTIES
  COMPILE_OPTIONS "-Wall;-Wextra;-Werror;-Wno-unused-parameter")

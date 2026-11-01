# =============================================================================
# Vortex V1 host tests: header-only contracts (design §4.1, §9.4)
# =============================================================================
#   cmake --preset host-debug
#   cmake --build --preset host-debug
#   ctest --preset host-debug
#
# No third-party test framework: the tests use the small CHECK runner in
# support/ (the pattern of the pw reference host tests), so configuring needs
# no network access.
# =============================================================================
cmake_minimum_required(VERSION 3.21)
project(vortex_host_tests LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)          # the firmware builds as C++17
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

get_filename_component(VORTEX_REPO_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)

option(VORTEX_HOST_SANITIZE "Build host tests with ASan + UBSan" ON)

add_library(vortex_contracts INTERFACE)
target_include_directories(vortex_contracts INTERFACE "${VORTEX_REPO_ROOT}")

add_library(vortex_host_warnings INTERFACE)
target_compile_options(vortex_host_warnings INTERFACE
  -Wall -Wextra -Wpedantic -Werror -Wshadow -Wconversion -Wsign-conversion -Wold-style-cast)
if(VORTEX_HOST_SANITIZE)
  target_compile_options(vortex_host_warnings INTERFACE -fsanitize=address,undefined -fno-omit-frame-pointer
                                                        -fno-sanitize-recover=all)
  target_link_options(vortex_host_warnings INTERFACE -fsanitize=address,undefined)
endif()

add_library(vortex_check STATIC support/check_main.cpp)
target_include_directories(vortex_check PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(vortex_check PUBLIC vortex_host_warnings)

enable_testing()

# Every contracts header must compile on its own.
file(GLOB VORTEX_CONTRACT_HEADERS CONFIGURE_DEPENDS "${VORTEX_REPO_ROOT}/contracts/*.hpp")
set(VORTEX_STANDALONE_SOURCES "")
foreach(hdr IN LISTS VORTEX_CONTRACT_HEADERS)
  get_filename_component(name "${hdr}" NAME_WE)
  set(src "${CMAKE_CURRENT_BINARY_DIR}/standalone/${name}.cpp")
  file(CONFIGURE OUTPUT "${src}" CONTENT "#include \"contracts/${name}.hpp\"\n#include \"contracts/${name}.hpp\"\n")
  list(APPEND VORTEX_STANDALONE_SOURCES "${src}")
endforeach()
add_library(vortex_contracts_standalone OBJECT ${VORTEX_STANDALONE_SOURCES})
target_link_libraries(vortex_contracts_standalone PRIVATE vortex_contracts vortex_host_warnings)

function(vortex_host_test name)
  add_executable(${name} ${name}.cpp)
  target_link_libraries(${name} PRIVATE vortex_contracts vortex_check)
  add_test(NAME contracts.${name} COMMAND ${name})
endfunction()

vortex_host_test(test_router_policy)
vortex_host_test(test_cia402_rules)

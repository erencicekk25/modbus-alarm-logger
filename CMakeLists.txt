cmake_minimum_required(VERSION 3.16)

project(modbus_alarm_logger
    VERSION 1.0.0
    DESCRIPTION "Threshold based alarm logger for a simulated Modbus TCP device"
    LANGUAGES CXX)

option(MAL_BUILD_TESTS "Build the unit and integration tests" ON)
option(MAL_ENABLE_SANITIZERS "Build with AddressSanitizer and UndefinedBehaviorSanitizer (GCC/Clang, non-Windows)" OFF)
option(MAL_WARNINGS_AS_ERRORS "Treat compiler warnings as errors" OFF)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(Threads REQUIRED)

# ---------------------------------------------------------------------------
# Shared compiler settings, applied through an interface target
# ---------------------------------------------------------------------------
add_library(mal_project_options INTERFACE)
target_compile_features(mal_project_options INTERFACE cxx_std_17)

if(MSVC)
    target_compile_options(mal_project_options INTERFACE /W4 /permissive- /utf-8)
    if(MAL_WARNINGS_AS_ERRORS)
        target_compile_options(mal_project_options INTERFACE /WX)
    endif()
else()
    target_compile_options(mal_project_options INTERFACE
        -Wall -Wextra -Wpedantic -Wshadow -Wconversion
        -Wnon-virtual-dtor -Wold-style-cast -Woverloaded-virtual)
    if(MAL_WARNINGS_AS_ERRORS)
        target_compile_options(mal_project_options INTERFACE -Werror)
    endif()
    if(MAL_ENABLE_SANITIZERS)
        target_compile_options(mal_project_options INTERFACE
            -fsanitize=address,undefined -fno-omit-frame-pointer)
        target_link_options(mal_project_options INTERFACE -fsanitize=address,undefined)
    endif()
endif()

if(WIN32)
    target_compile_definitions(mal_project_options INTERFACE WIN32_LEAN_AND_MEAN NOMINMAX)
endif()

# ---------------------------------------------------------------------------
# Core library: networking, virtual server and alarm client
# ---------------------------------------------------------------------------
add_library(modbus_core STATIC
    src/TcpSocket.cpp
    src/VirtualModbusServer.cpp
    src/ModbusAlarmClient.cpp)

target_include_directories(modbus_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(modbus_core
    PUBLIC
        Threads::Threads
        $<$<PLATFORM_ID:Windows>:ws2_32>
    PRIVATE
        mal_project_options)
target_compile_features(modbus_core PUBLIC cxx_std_17)
set_target_properties(modbus_core PROPERTIES CXX_EXTENSIONS OFF)

# ---------------------------------------------------------------------------
# Command line application
# ---------------------------------------------------------------------------
add_executable(modbus-alarm-logger src/main.cpp)
target_link_libraries(modbus-alarm-logger PRIVATE modbus_core mal_project_options)
set_target_properties(modbus-alarm-logger PROPERTIES CXX_EXTENSIONS OFF)

# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
if(MAL_BUILD_TESTS)
    enable_testing()
    add_executable(modbus_tests tests/ModbusTests.cpp)
    target_link_libraries(modbus_tests PRIVATE modbus_core mal_project_options)
    set_target_properties(modbus_tests PROPERTIES CXX_EXTENSIONS OFF)
    add_test(NAME modbus_tests COMMAND modbus_tests)
endif()

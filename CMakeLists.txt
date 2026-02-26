# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(ENABLE_PSRAM "0" CACHE STRING "Enables PSRAM support (0/1)")
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# If this directory is built standalone, import the Pico SDK. When used as a
# subdirectory of a top-level Pico project the SDK will already be available,
# so avoid re-importing an SDK configured for a different version.
if(NOT TARGET pico_sdk_init)
    include(pico_sdk_import.cmake)
endif()

project(pico_ov5640 C CXX ASM)

# initialize the Pico SDK if not already done
pico_sdk_init()

# Provide an interface library so other projects can link against this camera driver
add_library(pico_ov5640 INTERFACE)

target_sources(pico_ov5640 INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/ov5640.c
    ${CMAKE_CURRENT_LIST_DIR}/imagecapture/StateMachine.c
    ${CMAKE_CURRENT_LIST_DIR}/imagecapture/ParallelImageCapture.c
)

target_include_directories(pico_ov5640 INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/imagecapture
)

target_link_libraries(pico_ov5640 INTERFACE
    hardware_i2c
    hardware_gpio
    hardware_pwm
    hardware_pio
    hardware_dma
    hardware_clocks
    pico_stdlib
)

# Optionally provide the example executable when this directory is built standalone
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_LIST_DIR)
    add_executable(camera camera.c)
    pico_enable_stdio_uart(camera 1)
    pico_enable_stdio_usb(camera 0)
    target_link_libraries(camera PRIVATE pico_stdlib pico_ov5640)
    pico_add_extra_outputs(camera)
endif()


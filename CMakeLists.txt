# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0-a4)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(SX1276_HAB_SSTV_TX C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(SX1276_HAB_SSTV_TX 
        main.c 
        usb_descriptors.c
        msc_disk.c
    src/tjpgd.c
    src/tjpgd.h
    src/tjpgdcnf.h
    jpeg_tjpgdec.c
)

pico_set_program_name(SX1276_HAB_SSTV_TX "SX1276_HAB_SSTV_TX")
pico_set_program_version(SX1276_HAB_SSTV_TX "0.1")

# UART for debug (GP0=TX, GP1=RX), USB for Mass Storage
pico_enable_stdio_uart(SX1276_HAB_SSTV_TX 1)
pico_enable_stdio_usb(SX1276_HAB_SSTV_TX 0)

# Add the standard library to the build
target_link_libraries(SX1276_HAB_SSTV_TX
        pico_stdlib
        hardware_spi
        hardware_pwm
        hardware_dma
        hardware_flash
        hardware_sync
        tinyusb_device
        tinyusb_board)

# Add the standard include files to the build
target_include_directories(SX1276_HAB_SSTV_TX PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/src
)

pico_add_extra_outputs(SX1276_HAB_SSTV_TX)


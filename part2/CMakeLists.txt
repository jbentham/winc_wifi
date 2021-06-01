# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# initalize pico_sdk from installed location
# (note this can come from environment, CMake cache etc)
set(PICO_SDK_PATH "C:/Users/jb/Documents/Pico/pico-sdk")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(winc_wifi C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(winc_wifi winc_pico_part2.c winc_wifi.c winc_sock.c)

pico_set_program_name(winc_wifi "winc_wifi")
pico_set_program_version(winc_wifi "0.1")

pico_enable_stdio_uart(winc_wifi 1)
pico_enable_stdio_usb(winc_wifi 0)

# Add the standard library to the build
target_link_libraries(winc_wifi pico_stdlib)

# Add any user requested libraries
target_link_libraries(winc_wifi
	pico_stdlib
        hardware_spi
        )

pico_add_extra_outputs(winc_wifi)


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
set(sdkVersion 2.1.1)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.1.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
#set(PICO_BOARD pico2 CACHE STRING "Board type")

#set(PICO_BOARD_HEADER_DIRS ${CMAKE_SOURCE_DIR})
set(PICO_BOARD pimoroni_pga2350 CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(Framethrower_Denise C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()


find_package(Git)

if(GIT_FOUND)
    execute_process(
        COMMAND ${GIT_EXECUTABLE} rev-parse --short HEAD
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        OUTPUT_VARIABLE GIT_HASH
        OUTPUT_STRIP_TRAILING_WHITESPACE
    )
else()
    set(GIT_HASH "unknown")
endif()

# 3. Die Header-Datei generieren
# CMake nimmt version.h.in, ersetzt @GIT_HASH@ und speichert es als version.h
configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/version.h.in
    ${CMAKE_CURRENT_BINARY_DIR}/fwversion.h
)

# Add executable. Default name is the project name, version 0.1 

add_executable(Framethrower_Denise Framethrower_Denise.c mipi.c rga_device.c)

pico_set_program_name(Framethrower_Denise "Framethrower_Denise")
pico_set_program_version(Framethrower_Denise "0.1")

# Generate PIO headers
pico_generate_pio_header(Framethrower_Denise ${CMAKE_CURRENT_LIST_DIR}/video_capture.pio)
pico_generate_pio_header(Framethrower_Denise ${CMAKE_CURRENT_LIST_DIR}/rga_access.pio)

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(Framethrower_Denise 0)
pico_enable_stdio_usb(Framethrower_Denise 0)

# Add the standard library to the build
target_link_libraries(Framethrower_Denise
        pico_stdlib)

# Add the standard include files to the build
target_include_directories(Framethrower_Denise PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
)

# Add any user requested libraries
target_link_libraries(Framethrower_Denise 
        hardware_dma
        hardware_pio
        hardware_adc
        hardware_flash
        hardware_watchdog
        pico_multicore
        hardware_clocks
        )

target_compile_definitions(Framethrower_Denise PRIVATE
        #PLL_SYS_REFDIV=1
        #PLL_SYS_VCO_FREQ_HZ=1344000000
        #PLL_SYS_POSTDIV1=4
        #PLL_SYS_POSTDIV2=1
        #SYS_CLK_HZ=336000000
        PICO_XOSC_STARTUP_DELAY_MULTIPLIER=128 #this gives the cheap jlcpcb crystal a bit time to stabilize...
        #PICO_PIO_USE_GPIO_BASE=1

        #PLL_USB_REFDIV=2
        #PLL_USB_VCO_FREQ_HZ=1350000000
        #PLL_USB_POSTDIV1=5
        #PLL_USB_POSTDIV2=1
        #USB_CLK_HZ=270000000
)
pico_set_binary_type(Framethrower_Denise copy_to_ram)
set_property(TARGET Framethrower_Denise APPEND_STRING PROPERTY LINK_FLAGS "-Wl,--print-memory-usage")
set_property(TARGET Framethrower_Denise PROPERTY COMPILE_WARNING_AS_ERROR ON)
add_compile_definitions(PICO_PIO_USE_GPIO_BASE=1)
#add_compile_definitions(PICO_EMBED_XIP_SETUP=1)
pico_add_extra_outputs(Framethrower_Denise)

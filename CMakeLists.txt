# ===========================================================================
# DSPico — RP2350 USB-to-USB parametric EQ bridge (Phase 0 + Phase 1)
# ===========================================================================
cmake_minimum_required(VERSION 3.13)

# Waveshare RP2350-USB-CM is an RP2350A board; build for the generic pico2
# target (RP2350, ARM Cortex-M33). Override with -DPICO_BOARD=... if needed.
if(NOT DEFINED PICO_BOARD)
    set(PICO_BOARD pico2)
endif()

# Pull in the SDK (needs PICO_SDK_PATH env var, or set -DPICO_SDK_FETCH_FROM_GIT=ON).
include(pico_sdk_import.cmake)

project(dspico C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

# Second USB port: Pico-PIO-USB. Cloned by ./setup.sh into lib/Pico-PIO-USB.
set(PICO_PIO_USB_DIR ${CMAKE_CURRENT_LIST_DIR}/lib/Pico-PIO-USB)
if(NOT EXISTS ${PICO_PIO_USB_DIR}/CMakeLists.txt)
    message(FATAL_ERROR
        "Pico-PIO-USB not found at ${PICO_PIO_USB_DIR}.\n"
        "Run ./setup.sh (or: git clone https://github.com/sekigon-gonnoc/Pico-PIO-USB lib/Pico-PIO-USB).")
endif()
add_subdirectory(${PICO_PIO_USB_DIR} pico_pio_usb)

add_executable(dspico
    src/main.c
    src/usb_descriptors.c
    src/uac2_device.c
    src/uac_host.c
    src/test_tone.c
    src/status_led.c
    src/dsp_peq.c
    src/signal_path.c
)

# Generate ws2812.pio.h from the PIO source (uses the SDK's pioasm).
pico_generate_pio_header(dspico ${CMAKE_CURRENT_LIST_DIR}/src/ws2812.pio)

# tusb_config.h + our headers live in src/.
target_include_directories(dspico PRIVATE ${CMAKE_CURRENT_LIST_DIR}/src)

target_link_libraries(dspico
    pico_stdlib
    pico_multicore
    pico_unique_id
    hardware_pio
    hardware_clocks
    tinyusb_device
    tinyusb_host
    pico_pio_usb
)

# Debug console on UART (GPIO0/1), NOT USB — the USB ports are the audio path.
pico_enable_stdio_uart(dspico 1)
pico_enable_stdio_usb(dspico 0)

# Produce the .uf2 (drag-and-drop to the BOOTSEL drive) plus .elf/.bin/.map.
pico_add_extra_outputs(dspico)

cmake_minimum_required(VERSION 3.13)

# Set board to Pico 2 (RP2350)
set(PICO_BOARD pico2 CACHE STRING "Pico board type")

# Board configuration variant (M1, M2)
set(BOARD_VARIANT "M2" CACHE STRING "Board variant: M1 or M2")

# PSRAM speed configuration (84, 100, 133 MHz target)
set(PSRAM_SPEED "133" CACHE STRING "PSRAM max frequency in MHz: 84, 100, 133")

# USB HID gamepad/keyboard support (disabled by default, conflicts with USB CDC)
option(USB_HID_ENABLED "Enable USB HID host for gamepads and keyboards" OFF)

message(STATUS "Board: ${BOARD_VARIANT}, PSRAM: ${PSRAM_SPEED} MHz")

include(pico_sdk_import.cmake)

project(kickstarter C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_subdirectory(src/fatfs)
add_subdirectory(drivers/sdcard)
add_subdirectory(drivers/nespad)
add_subdirectory(drivers/ps2kbd)
add_subdirectory(drivers/usbhid)

# Driver library
add_library(drivers
    drivers/HDMI.c
    drivers/hdmi_scanline.S
)
target_include_directories(drivers PUBLIC drivers src)
target_compile_definitions(drivers PRIVATE
    BOARD_${BOARD_VARIANT}
    PSRAM_MAX_FREQ_MHZ=${PSRAM_SPEED}
)

target_link_libraries(drivers pico_stdlib hardware_dma hardware_pio hardware_spi hardware_gpio)

add_executable(kickstarter
    src/main.c
)

target_include_directories(kickstarter PRIVATE
    src
    drivers
    drivers/sdcard
    drivers/nespad
    drivers/ps2kbd
)

target_compile_definitions(kickstarter PRIVATE
    BOARD_${BOARD_VARIANT}
    PSRAM_MAX_FREQ_MHZ=${PSRAM_SPEED}
    PICO_ON_DEVICE=1
    SDCARD_PIN_SPI0_CS=5
    SDCARD_PIN_SPI0_SCK=6
    SDCARD_PIN_SPI0_MOSI=7
    SDCARD_PIN_SPI0_MISO=4
    NESPAD_GPIO_CLK=20
    NESPAD_GPIO_LATCH=21
    NESPAD_GPIO_DATA=26
)

target_link_libraries(kickstarter
    pico_stdlib
    pico_multicore
    hardware_spi
    hardware_dma
    hardware_pio
    hardware_pwm
    hardware_irq
    hardware_clocks
    sdcard
    drivers
    nespad
    ps2kbd
    usbhid
)

# USB serial console for debug output
pico_enable_stdio_uart(kickstarter 0)
pico_enable_stdio_usb(kickstarter 1)

pico_add_extra_outputs(kickstarter)

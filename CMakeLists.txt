cmake_minimum_required(VERSION 3.13)

set(PICO_BOARD pico2 CACHE STRING "Pico board type")
set(BOARD_VARIANT "M2" CACHE STRING "Board variant: M1 or M2")
set(DVI_DEFAULT_SERIAL_CONFIG "murmulator2_cfg" CACHE STRING "DVI pin config")

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
add_subdirectory(drivers/libdvi)

add_executable(kickstarter
    src/main.c
)

target_include_directories(kickstarter PRIVATE
    src
    drivers
    drivers/sdcard
    drivers/nespad
    drivers/ps2kbd
    drivers/libdvi
)

target_compile_definitions(kickstarter PRIVATE
    BOARD_${BOARD_VARIANT}
    PICO_ON_DEVICE=1
    DVI_DEFAULT_SERIAL_CONFIG=${DVI_DEFAULT_SERIAL_CONFIG}
    PICO_CORE1_STACK_SIZE=0x200
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
    hardware_vreg
    hardware_interp
    sdcard
    libdvi
    nespad
    ps2kbd
    usbhid
)

pico_enable_stdio_uart(kickstarter 0)
pico_enable_stdio_usb(kickstarter 1)

pico_add_extra_outputs(kickstarter)

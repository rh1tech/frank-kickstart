cmake_minimum_required(VERSION 3.13)

# M1: Murmulator 1 (HDMI on GPIO 6-13, SD on SPI0 2/3/4/5, PS/2 0/1, NES 14/15/16)
# M2: Murmulator 2 (HDMI on GPIO 12-19, SD on SPI0 6/7/4/5, PS/2 2/3, NES 20/21/26)
# Z0: Waveshare RP2350-PiZero (HDMI on GPIO 32-39, SD on SPI1 30/31/40/43,
#     PS/2 2/3, NES 4/5/7)
set(BOARD_VARIANT "M2" CACHE STRING "Board variant: M1, M2 or Z0")
set_property(CACHE BOARD_VARIANT PROPERTY STRINGS M1 M2 Z0)
if(BOARD_VARIANT STREQUAL "M1")
    set(PICO_BOARD pico2)
    set(VIDEO_BACKEND_DEFAULT PIO)
    set(HDMI_PIO_BASE_PIN 6)
    set(HDMI_PIO_Z0_LAYOUT 0)
    set(SD_SPI_BUS spi0)
    set(SD_PIN_SCK 2)
    set(SD_PIN_MOSI 3)
    set(SD_PIN_MISO 4)
    set(SD_PIN_CS 5)
    set(NES_PIN_CLK 14)
    set(NES_PIN_LATCH 15)
    set(NES_PIN_DATA 16)
    set(PS2_PIN_CLK 0)
elseif(BOARD_VARIANT STREQUAL "M2")
    set(PICO_BOARD pico2)
    set(VIDEO_BACKEND_DEFAULT HSTX)
    set(HDMI_PIO_BASE_PIN 12)
    set(HDMI_PIO_Z0_LAYOUT 0)
    set(SD_SPI_BUS spi0)
    set(SD_PIN_SCK 6)
    set(SD_PIN_MOSI 7)
    set(SD_PIN_MISO 4)
    set(SD_PIN_CS 5)
    set(NES_PIN_CLK 20)
    set(NES_PIN_LATCH 21)
    set(NES_PIN_DATA 26)
    set(PS2_PIN_CLK 2)
elseif(BOARD_VARIANT STREQUAL "Z0")
    set(PICO_BOARD frank_z0_rp2350b)
    set(VIDEO_BACKEND_DEFAULT PIO)
    set(HDMI_PIO_BASE_PIN 32)
    set(HDMI_PIO_Z0_LAYOUT 1)
    set(SD_SPI_BUS spi1)
    set(SD_PIN_SCK 30)
    set(SD_PIN_MOSI 31)
    set(SD_PIN_MISO 40)
    set(SD_PIN_CS 43)
    set(NES_PIN_CLK 4)
    set(NES_PIN_LATCH 5)
    set(NES_PIN_DATA 7)
    set(PS2_PIN_CLK 2)
else()
    message(FATAL_ERROR "BOARD_VARIANT must be M1, M2 or Z0")
endif()
set(PICO_BOARD_HEADER_DIRS ${CMAKE_CURRENT_LIST_DIR}/boards)

# HSTX: M2 only. PIO: every board (M2 with -DVIDEO_BACKEND=PIO tests the
# M1/Z0 driver on the M2's HDMI pins).
set(VIDEO_BACKEND "${VIDEO_BACKEND_DEFAULT}" CACHE STRING "Video driver: HSTX (M2) or PIO")
set_property(CACHE VIDEO_BACKEND PROPERTY STRINGS HSTX PIO)
if(VIDEO_BACKEND STREQUAL "HSTX" AND NOT BOARD_VARIANT STREQUAL "M2")
    message(FATAL_ERROR "VIDEO_BACKEND=HSTX needs BOARD_VARIANT=M2")
endif()

# 640x480@60: 25.2 MHz pixel clock, so clk_sys is a multiple of 252 MHz
set(CPU_SPEED "252" CACHE STRING "CPU clock in MHz: 252 or 504")
if(CPU_SPEED GREATER_EQUAL 504)
    set(CPU_VOLTAGE "VREG_VOLTAGE_1_65")
else()
    set(CPU_VOLTAGE "VREG_VOLTAGE_1_50")
endif()
set(PSRAM_SPEED "133" CACHE STRING "PSRAM max frequency in MHz")
set(SD_SPEED "24" CACHE STRING "SD card SPI clock in MHz (clk_peri 48 MHz / an even divider)")

include(pico_sdk_import.cmake)

project(frank-kickstart C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Read version from version.txt (format: "MAJOR MINOR" e.g. "0 1" = v0.01)
file(READ "${CMAKE_CURRENT_LIST_DIR}/version.txt" VERSION_CONTENT)
string(STRIP "${VERSION_CONTENT}" VERSION_CONTENT)
string(REPLACE " " ";" VERSION_LIST "${VERSION_CONTENT}")
list(GET VERSION_LIST 0 FK_VERSION_MAJOR)
list(GET VERSION_LIST 1 FK_VERSION_MINOR)
string(LENGTH "${FK_VERSION_MINOR}" _minor_len)
if(_minor_len LESS 2)
    set(FIRMWARE_VERSION "v${FK_VERSION_MAJOR}.0${FK_VERSION_MINOR}")
else()
    set(FIRMWARE_VERSION "v${FK_VERSION_MAJOR}.${FK_VERSION_MINOR}")
endif()
message(STATUS "frank-kickstart version: ${FIRMWARE_VERSION}, board ${BOARD_VARIANT}, video ${VIDEO_BACKEND}")

pico_sdk_init()

# USB HID host for gamepads and keyboards (uses native USB port)
option(USB_HID_ENABLED "Enable USB HID host for gamepads and keyboards" ON)
# boot stages as white squares on the screen (bring-up)
option(FK_BOOT_TRACE "Show the boot stages on the screen" OFF)
if(FK_BOOT_TRACE)
    add_compile_definitions(FK_BOOT_TRACE=1)
endif()

add_subdirectory(src/fatfs)
add_subdirectory(drivers/sdcard)
add_subdirectory(drivers/nespad)
add_subdirectory(drivers/ps2kbd)
add_subdirectory(drivers/usbhid)

if(VIDEO_BACKEND STREQUAL "HSTX")
    set(VIDEO_SOURCES
        src/video/video_hstx.c
        drivers/hdmi/psx_video.c
        drivers/hdmi/hstx_packet.c
        drivers/hdmi/hstx_data_island_queue.c
    )
    set_source_files_properties(src/video/video_hstx.c
        PROPERTIES COMPILE_OPTIONS "-fno-tree-loop-distribute-patterns")
else()
    set(VIDEO_SOURCES
        src/video/video_pio.c
        src/video/tmds_balanced.c
        drivers/hdmi_pio640/hdmi_pio640.c
        drivers/hdmi/hstx_packet.c
    )
    # plain DVI (no HDMI data islands), for sinks that reject them
    option(HDMI_PIO_DVI "PIO video: DVI only, no data islands" OFF)
    if(HDMI_PIO_DVI)
        add_compile_definitions(HDMI_PIO640_DVI=1)
    endif()
    # the PIO HDMI's IRQ code must stay in SRAM: no loops turned into newlib calls
    set_source_files_properties(drivers/hdmi_pio640/hdmi_pio640.c src/video/video_pio.c
        PROPERTIES COMPILE_OPTIONS "-fno-tree-loop-distribute-patterns")
endif()

set(FK_COMMON_DEFINES
    BOARD_${BOARD_VARIANT}
    BOARD_NAME="${BOARD_VARIANT}"
    VIDEO_BACKEND_${VIDEO_BACKEND}=1
    PICO_ON_DEVICE=1
    PICO_FLASH_SIZE_BYTES=16777216
    PICO_STACK_SIZE=0x2000
    PICO_CORE1_STACK_SIZE=0x800
    CPU_CLOCK_MHZ=${CPU_SPEED}
    CPU_VOLTAGE=${CPU_VOLTAGE}
    PSRAM_MAX_FREQ_MHZ=${PSRAM_SPEED}
    HDMI_PIO_BASE_PIN=${HDMI_PIO_BASE_PIN}
    HDMI_PIO_Z0_LAYOUT=${HDMI_PIO_Z0_LAYOUT}
    NESPAD_GPIO_CLK=${NES_PIN_CLK}
    NESPAD_GPIO_LATCH=${NES_PIN_LATCH}
    NESPAD_GPIO_DATA=${NES_PIN_DATA}
    PS2_PIN_CLK=${PS2_PIN_CLK}
    PS2_PIN_DATA=${PS2_PIN_CLK}+1
    FK_VERSION_MAJOR=${FK_VERSION_MAJOR}
    FK_VERSION_MINOR=${FK_VERSION_MINOR}
    PICO_STDIO_USB_CONNECT_WAIT_TIMEOUT_MS=0
)

set(FK_INCLUDES
    src
    src/video
    src/gfx
    src/ui
    src/data
    src/audio
    src/flash
    drivers
    drivers/hdmi
    drivers/hdmi_pio640
    drivers/sdcard
    drivers/nespad
    drivers/ps2kbd
)

set(FK_LIBS
    pico_stdlib
    pico_multicore
    hardware_spi
    hardware_dma
    hardware_pio
    hardware_irq
    hardware_clocks
    hardware_vreg
    hardware_flash
    hardware_xip_cache
    hardware_watchdog
    sdcard
    nespad
    ps2kbd
)

# ---------------------------------------------------------------------
# The launcher
# ---------------------------------------------------------------------
add_executable(frank-kickstart
    src/main.c
    src/fw_boot_redirect.c
    src/psram_init.c
    src/platform.c
    src/gfx/gfx.c
    src/gfx/palette.c
    src/fonts/fonts.c
    src/ui/ui.c
    src/ui/welcome.c
    src/ui/input.c
    src/data/catalog.c
    src/data/fsio_fatfs.c
    src/flash/flash_uf2.c
    src/audio/music.c
    src/cxx_stubs.cpp
    ${VIDEO_SOURCES}
)

# Custom linker script: frank-kickstart at 0x10FC0000, firmware area at 0x10000000
pico_set_linker_script(frank-kickstart ${CMAKE_SOURCE_DIR}/memmap.frank-kickstart.ld)
target_include_directories(frank-kickstart PRIVATE ${FK_INCLUDES})
target_compile_definitions(frank-kickstart PRIVATE ${FK_COMMON_DEFINES})
target_link_libraries(frank-kickstart ${FK_LIBS} usbhid)

pico_enable_stdio_uart(frank-kickstart 1)
if(USB_HID_ENABLED)
    pico_enable_stdio_usb(frank-kickstart 0)
else()
    pico_enable_stdio_usb(frank-kickstart 1)
endif()
pico_add_extra_outputs(frank-kickstart)

# ---------------------------------------------------------------------
# Video bring-up: test patterns and a once-a-second health line on the UART
# (make frank-kickstart-vtest). Linked like the launcher, at 0x10FC0000.
# ---------------------------------------------------------------------
add_executable(frank-kickstart-vtest
    tools/video_test.c
    src/fw_boot_redirect.c
    src/psram_init.c
    src/platform.c
    ${VIDEO_SOURCES}
)
pico_set_linker_script(frank-kickstart-vtest ${CMAKE_SOURCE_DIR}/memmap.frank-kickstart.ld)
target_include_directories(frank-kickstart-vtest PRIVATE ${FK_INCLUDES})
target_compile_definitions(frank-kickstart-vtest PRIVATE ${FK_COMMON_DEFINES})
target_link_libraries(frank-kickstart-vtest ${FK_LIBS})
# console on the native USB port (CDC) and the UART
pico_enable_stdio_uart(frank-kickstart-vtest 1)
pico_enable_stdio_usb(frank-kickstart-vtest 1)
pico_add_extra_outputs(frank-kickstart-vtest)

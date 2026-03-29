# FRANK Kickstart

UF2 firmware launcher for Raspberry Pi Pico 2 (RP2350) with HDMI output, SD card browser, NES/SNES gamepad, USB gamepad, PS/2 keyboard, and USB keyboard support.

Scans the `kickstart` directory on an SD card for `.uf2` firmware files, displays cover art and metadata on an 800x600 HDMI screen, and flashes the selected firmware directly from the launcher. Based on the boot redirect mechanism from [pico-launcher](https://github.com/DnCraptor/pico-launcher) by [xrip](https://github.com/xrip) and [DnCraptor](https://github.com/DnCraptor).

## Supported Boards

This firmware is designed for the **M1** and **M2** board layouts on RP2350-based boards with integrated HDMI and SD card:

- **[FRANK](https://rh1.tech/projects/frank?area=about)** -- A versatile development board based on RP Pico 2 with HDMI output and extensive I/O options.
- **[Murmulator](https://murmulator.ru)** -- A compact retro-computing platform based on RP Pico 2, designed for emulators and retro computing.

Both boards provide all necessary peripherals out of the box (no additional wiring required).

## Features

- Native 800x600 HDMI video output via PicoDVI (400x300 with 2x pixel doubling)
- SD card firmware browser with cover art and XML metadata
- UF2 firmware flashing directly to on-board flash (no BOOTSEL mode required)
- Persistent boot redirect -- flashed firmware runs on reboot, Select/Space triggers return to launcher
- PSRAM support: up to 128 firmware entries with PSRAM, 16 without
- NES and SNES gamepad support (directly connected)
- USB gamepad support (via native USB Host)
- USB and PS/2 keyboard support

## How It Works

FRANK Kickstart lives in the last 1MB of a 16MB flash chip (at `0x10F00000`). User firmware occupies the beginning of flash. A `before_main()` constructor checks watchdog scratch registers to decide whether to run the flashed firmware or the launcher UI.

### Flash Layout (16MB)

| Region | Address | Size | Purpose |
|--------|---------|------|---------|
| BOOT2 | `0x10000000` | 4KB | Vectors + IMAGE_DEF |
| Firmware | `0x10001000` | ~15.7MB | User firmware area |
| ZERO_BLOCK | `0x10FBF000` | 4KB | Original vector table backup |
| FRANK Kickstart | `0x10FC0000` | 256KB | Launcher code |

### Boot Flow

1. On power-up, `before_main()` runs before `main()`
2. If watchdog scratch[0] contains `SCRATCH_MAGIC_BOOT` -- jump to flashed firmware
3. If watchdog scratch[0] contains `SCRATCH_MAGIC_UI` or no valid firmware -- run launcher
4. Press **Select** (gamepad) or **Space** (keyboard) in a running firmware to reboot into launcher

## Hardware Requirements

- **Raspberry Pi Pico 2** (RP2350) or compatible board
- **16MB flash** (required for dual-firmware layout)
- **HDMI connector** (directly connected via resistors, no HDMI encoder needed)
- **SD card module** (SPI mode)
- **8MB QSPI PSRAM** (optional, enables 128 firmware entries instead of 16)
- **NES or SNES gamepad** (directly connected) -- OR --
- **USB gamepad** (via native USB port) -- OR --
- **USB or PS/2 keyboard**

## Pin Assignment

### HDMI (via 270 Ohm resistors)

| Signal | M1 GPIO | M2 GPIO |
|--------|---------|---------|
| CLK-   | 6       | 12      |
| CLK+   | 7       | 13      |
| D0-    | 8       | 14      |
| D0+    | 9       | 15      |
| D1-    | 10      | 16      |
| D1+    | 11      | 17      |
| D2-    | 12      | 18      |
| D2+    | 13      | 19      |

### SD Card (SPI mode)

| Signal | M1 GPIO | M2 GPIO |
|--------|---------|---------|
| CLK    | 2       | 6       |
| CMD    | 3       | 7       |
| DAT0   | 4       | 4       |
| DAT3   | 5       | 5       |

### NES/SNES Gamepad

| Signal | GPIO |
|--------|------|
| CLK    | 20   |
| LATCH  | 21   |
| DATA 1 | 26   |
| DATA 2 | 27   |

### PS/2 Keyboard

| Signal | M1 GPIO | M2 GPIO |
|--------|---------|---------|
| CLK    | 0       | 2       |
| DATA   | 1       | 3       |

### PSRAM (auto-detected)

| Chip Package | GPIO |
|--------------|------|
| RP2350B      | 47   |
| RP2350A (M1) | 19   |
| RP2350A (M2) | 8    |

## SD Card Setup

1. Format an SD card as **FAT32**
2. Create a `kickstart` directory in the root
3. For each firmware, place three files in `kickstart/`:
   - `name.uf2` -- the firmware binary
   - `name.bmp` -- cover art (8-bit indexed BMP, uncompressed, max 200x200)
   - `name.xml` -- metadata in the following format:

```xml
<firmware>
  <title>Firmware Name</title>
  <version>1.00</version>
  <author>Author Name</author>
  <website>https://example.com</website>
  <description>Description of the firmware.</description>
  <controls>Control scheme description.</controls>
</firmware>
```

### Cover Art

BMP files must be 8-bit indexed (palette), uncompressed, max 200x200 pixels. A convenience script is included to batch-convert all PNG images in `sdcard/kickstart/`:

```bash
./convert_images.sh
```

This converts each `.png` to a 100x150 8-bit indexed BMP with 239 colors using ImageMagick. You can also convert manually:

```bash
magick input.png -resize 100x150! -colors 239 -type palette -compress none BMP3:output.bmp
```

## Controls

### NES/SNES Gamepad

| Button | Action |
|--------|--------|
| Left / Right | Browse firmware |
| A / Start | Flash and launch selected firmware |
| Select | Return to launcher (via reboot) |

### PS/2 / USB Keyboard

| Key | Action |
|-----|--------|
| Left / Right arrow | Browse firmware |
| Enter | Flash and launch selected firmware |
| Space | Return to launcher (via reboot) |

## Building

### Prerequisites

1. Install the [Raspberry Pi Pico SDK](https://github.com/raspberrypi/pico-sdk) (version 2.0+)
2. Set environment variable: `export PICO_SDK_PATH=/path/to/pico-sdk`
3. Install ARM GCC toolchain

### Build

```bash
git clone https://github.com/rh1tech/frank-kickstart.git
cd frank-kickstart
./build.sh
```

The build script compiles for the M2 board layout by default. To build for M1:

```bash
./build.sh M1
```

Output: `build/frank-kickstart.uf2`

### Release Build

To build release firmware for all board variants:

```bash
./release.sh
```

Output files are placed in `release/` as `frank-kickstart_<board>_<version>.uf2`.

### Flashing

```bash
# With device connected:
./flash.sh

# Or manually with picotool:
picotool load -f build/frank-kickstart.elf && picotool reboot -f
```

## Technical Notes

- SPI and DVI are incompatible at 400 MHz due to APB bus contention. All SD card reads happen before DVI starts (data is pre-loaded, then SD is unmounted).
- DMA-based SPI bulk reads are used in the SD card driver for performance.
- Core 0 runs the UI and input handling; Core 1 runs TMDS encoding and DVI output.
- Watchdog scratch registers are used for boot state instead of SRAM, as SRAM near the top can be corrupted by the RP2350 boot ROM's stack during reboot.
- Flash timings are configured for 400 MHz operation (max flash freq = 66 MHz).

## License

Copyright (c) 2026 Mikhail Matveev <<xtreme@rh1.tech>>

This project is licensed under the GNU General Public License v3.0. See [LICENSE](LICENSE) for details.

Note: Individual drivers and libraries included in this project may have their own licenses as noted in the acknowledgments below.

## Acknowledgments

This project incorporates code and ideas from the following projects:

| Project | Author(s) | License | Used For |
|---------|-----------|---------|----------|
| [pico-launcher](https://github.com/DnCraptor/pico-launcher) | [xrip](https://github.com/xrip), [DnCraptor](https://github.com/DnCraptor) | GPL-2.0 | Boot redirect mechanism, UF2 flashing pattern, flash layout |
| [PicoDVI](https://github.com/Wren6991/PicoDVI) | Luke Wren (Wren6991) | BSD-3-Clause | Original DVI/TMDS library for RP2040/RP2350 |
| [FatFS](http://elm-chan.org/fsw/ff/) | ChaN | Custom permissive | FAT32 filesystem for SD card |
| [pico-infonesPlus](https://github.com/fhoedemakers/pico-infonesPlus) | shuichitakano, fhoedemakers | MIT | NES/SNES gamepad PIO driver |
| [TinyUSB](https://github.com/hathach/tinyusb) | Ha Thach | MIT | USB HID host driver |
| [Raspberry Pi Pico SDK](https://github.com/raspberrypi/pico-sdk) | Raspberry Pi Foundation | BSD-3-Clause | Hardware abstraction layer |

Special thanks to:

- **[xrip](https://github.com/xrip)** and **[DnCraptor](https://github.com/DnCraptor)** for [pico-launcher](https://github.com/DnCraptor/pico-launcher), whose boot redirect and UF2 flashing mechanism forms the foundation of this project
- **nickoala** for the PICO-BK RP2350-patched PicoDVI driver
- **Luke Wren** for the original PicoDVI library
- **shuichitakano** and **fhoedemakers** for the NES/SNES gamepad PIO driver
- The Raspberry Pi Foundation for the RP2350 and Pico SDK
- The Murmulator community for hardware designs and testing

## Author

Mikhail Matveev <<xtreme@rh1.tech>>

[https://rh1.tech](https://rh1.tech) | [GitHub](https://github.com/rh1tech/frank-kickstart)

# FRANK Kickstart

Official page: **[frank.rh1.tech](https://frank.rh1.tech/)** — hub for all FRANK boards and firmware.

UF2 firmware launcher for RP2350 boards with HDMI output: an SD card browser with cover art, NES/SNES gamepad, USB gamepad, PS/2 and USB keyboard support.

It lists the `.uf2` firmware files in the `kickstart` directory of an SD card, shows their cover art and details on a 640x480 HDMI screen, and writes the chosen one into the board's flash, without BOOTSEL mode or a computer. Based on the boot redirect mechanism from [pico-launcher](https://github.com/DnCraptor/pico-launcher) by [xrip](https://github.com/xrip) and [DnCraptor](https://github.com/DnCraptor).

## Supported Boards

| Board | Release file | Video |
|-------|--------------|-------|
| **M1** -- [Murmulator](https://murmulator.ru) 1.x layout | `frank-kickstart_m1_X_YY.uf2` | PIO HDMI, GPIO 6-13 |
| **M2** -- [FRANK](https://rh1.tech/projects/frank?area=about) / Murmulator 2 layout | `frank-kickstart_m2_X_YY.uf2` | HSTX HDMI, GPIO 12-19 |
| **Z0** -- Waveshare RP2350-PiZero | `frank-kickstart_z0_X_YY.uf2` | PIO HDMI (mini-HDMI), GPIO 32-39 |

Every board needs **16 MB of flash** and **8 MB of PSRAM**.

## Features

- 640x480@60 HDMI with a true 24-bit palette (M2: 15-bit output through the HSTX; M1/Z0: a new PIO driver sending every pixel as its own DC-balanced TMDS symbol, so lines are 640 real pixels)
- Anti-aliased proportional fonts (Inter, Latin and Cyrillic), cover art up to 240x320 with 128 colours, slide and fade transitions
- An animated welcome screen
- Background music over HDMI (M2): `kickstart/music.wav`, looped
- UF2 checks before flashing: RP2040 and RISC-V images, and images that would reach the launcher's flash area, are refused; firmware built for another board asks for a second press
- Writes the firmware with the screen still running (progress bar)
- Up to 128 firmware entries
- NES and SNES gamepads, USB gamepads (native USB host), USB and PS/2 keyboards

## How It Works

FRANK Kickstart lives in the last 256 KB of a 16 MB flash chip (at `0x10FC0000`). User firmware occupies the beginning of flash. A `before_main()` constructor checks the watchdog scratch registers to decide whether to run the flashed firmware or the launcher.

### Flash Layout (16MB)

| Region | Address | Size | Purpose |
|--------|---------|------|---------|
| BOOT2 | `0x10000000` | 4KB | Vectors + IMAGE_DEF |
| Firmware | `0x10001000` | ~15.7MB | User firmware area |
| ZERO_BLOCK | `0x10FBF000` | 4KB | Original vector table backup |
| FRANK Kickstart | `0x10FC0000` | 256KB | Launcher code |

A firmware must not write flash at or above `0x10FBF000`; see [docs/FIRMWARE.md](docs/FIRMWARE.md) for which ones do.

### Boot Flow

1. On power-up, `before_main()` runs before `main()`
2. If watchdog scratch[0] contains `SCRATCH_MAGIC_BOOT` (set by the launcher just before it reboots) -- jump to the flashed firmware
3. Otherwise (power-on, reset, or `SCRATCH_MAGIC_UI` 0x17F00FFF from a firmware) -- run the launcher
4. With a firmware in the flash, after a power-on or reset, the launcher shows "Loading <name>" with a 4-second countdown and then starts it; any key opens the menu instead. A firmware that asks for the launcher (`SCRATCH_MAGIC_UI`) gets the menu at once.

Once the countdown has been cut short, later starts go straight to the menu (`menu=1` in `kickstart/.config`) until a firmware is launched from it again. Choosing the firmware that is already in the flash starts it at once, without writing anything.

## Pin Assignment

| Signal | M1 | M2 | Z0 |
|--------|----|----|----|
| HDMI CLK-/+, D0-/+, D1-/+, D2-/+ | 6..13 | 12..19 | 32..39 (PIO, BGR order) |
| SD card SCK / MOSI / MISO / CS | SPI0 2 / 3 / 4 / 5 | SPI0 6 / 7 / 4 / 5 | SPI1 30 / 31 / 40 / 43 |
| NES/SNES CLK / LATCH / DATA | 14 / 15 / 16 | 20 / 21 / 26 | 4 / 5 / 7 |
| PS/2 CLK / DATA | 0 / 1 | 2 / 3 | 2 / 3 |
| PSRAM CS | 19 (RP2350A), 47 (RP2350B) | 8 (RP2350A), 47 (RP2350B) | 47 |
| Debug UART TX / RX | -- (PS/2 uses 0/1) | 0 / 1 | 0 / 1 |

## SD Card Setup

1. Format an SD card as **FAT32** or **exFAT**
2. Create a `kickstart` directory in the root
3. For each firmware, place up to three files in `kickstart/`:
   - `name.uf2` -- the firmware binary
   - `name.bmp` -- cover art (8-bit indexed BMP, uncompressed, 200x300 recommended, at most 240x320; older 100x150 covers are shown at 2x)
   - `name.xml` -- metadata:

```xml
<firmware>
  <title>Firmware Name</title>
  <version>1.00</version>
  <author>Author Name</author>
  <website>https://example.com</website>
  <description>Description of the firmware.</description>
  <controls>Control scheme description.</controls>
  <boards>m1 m2 z0</boards>
</firmware>
```

`<boards>` is optional: the boards the UF2 is built for. Without it the launcher reads board names in the file name (`frank-nes_m2p2_hstx.uf2` is for M2). The `sdcard/` folder of this repository has metadata and covers for the FRANK firmware; [docs/FIRMWARE.md](docs/FIRMWARE.md) lists them with the release file to use on each board.

### Background Music (M2)

On M2 the launcher plays `kickstart/music.wav` in a loop over HDMI audio (it fades out before installing). The file must be IMA ADPCM, stereo, 44.1 kHz, 1024-byte blocks (about 44 KB/s):

```bash
ffmpeg -i track.mp3 -ac 2 -ar 44100 -c:a adpcm_ima_wav -block_size 1024 kickstart/music.wav
```

Without the file the launcher is silent. M1 and Z0 have no sound (their PIO HDMI sends no audio). The **M** key switches the music off and on; the choice is kept in `kickstart/.config`.

The bundled `sdcard/kickstart/music.wav` is "Echoes of Neon Dreams". Song by <a href="https://stocktune.com/free-music/echoes-of-neon-dreams-263948-157491?signup=true">StockTune</a>.

### Cover Art

```bash
# any picture, cropped to 200x300 and reduced to 128 colours
tools/mkcover.py image input.png kickstart/name.bmp
# a generated cover from a screenshot
tools/mkcover.py screenshot screen.png kickstart/name.bmp --title "Name" --kind GAME
```

```bash
# an object cut out from its background (a PNG with alpha, or a photo on a
# plain background): shown floating on the page with a soft shadow
tools/mkcover.py cutout photo.png kickstart/name.bmp --remove-bg 40
```

Transparency is the palette colour `#FF00FF`; covers with more than 128 colours are reduced on the board. The sources and licences of the bundled covers are listed in [docs/COVERS.md](docs/COVERS.md).

## Controls

| Gamepad | Keyboard | Action |
|---------|----------|--------|
| Left / Right (Up / Down) | Arrow keys | Browse firmware |
| A / Start | Enter | Install and launch (or launch, if it is already in the flash) |
| B | Esc | Cancel the "install anyway" question |
| -- | M | Music off / on (M2) |
| Select + Start for 3 s | -- | Reboot into BOOTSEL (USB) mode |

## Building

### Prerequisites

1. Install the [Raspberry Pi Pico SDK](https://github.com/raspberrypi/pico-sdk) (version 2.1+)
2. Set environment variable: `export PICO_SDK_PATH=/path/to/pico-sdk`
3. Install ARM GCC toolchain

### Build

```bash
git clone https://github.com/rh1tech/frank-kickstart.git
cd frank-kickstart
./build.sh            # M2
./build.sh M1         # or M1, Z0
./build.sh M2 -DVIDEO_BACKEND=PIO   # M2 with the M1/Z0 video driver
```

Output: `build/frank-kickstart.uf2`. `make -C build frank-kickstart-vtest` builds a video test firmware (colour bars, ramps, encoder stress patterns, and a once-a-second health line on the UART).

### Release Build

```bash
./release.sh
```

Output files are placed in `release/` as `frank-kickstart_<board>_<version>.uf2`.

### Flashing

```bash
./flash.sh                       # picotool, board in BOOTSEL mode
```

To reach BOOTSEL without the button: send `~BOOT~` on the debug UART (115200 baud), or hold Select + Start for 3 seconds in the launcher. `~RST~` reboots.

### Screens without a board

```bash
tools/host_render.sh             # PNGs of every screen in build-host/ (FK_PIO=1: M1/Z0 colours)
```

## Technical Notes

- The system clock is 252 MHz (10x the 25.2 MHz pixel clock); the flash runs at 63 MHz.
- Core 1 runs only the video scan-out, entirely from SRAM, so the flash is written while the picture stays up (each erase/program runs from RAM with core 0's interrupts off, the QMI timing restored after).
- The 300 KB framebuffer is in SRAM; screens are composed in PSRAM and copied in, the catalogue and a 16-cover cache are in PSRAM too.
- Every software reboot stops core 1 and takes the PSRAM out of QPI mode first, or the boot ROM cannot read the flash after a warm reset.
- The installed firmware's sector 0 is written with the launcher's stack pointer and reset vector (the original goes to ZERO_BLOCK), and at start the launcher puts its own exception handlers back in the vector table it copied from there. The PSRAM is reset to SPI mode before it is set up, as a firmware may have left it in QPI mode.
- Watchdog scratch registers are used for boot state instead of SRAM, as SRAM near the top can be corrupted by the RP2350 boot ROM's stack during reboot.
- Fonts are generated from Inter by `tools/mkfonts.sh` (the C headers are committed).

## License

Copyright (c) 2026 Mikhail Matveev <<xtreme@rh1.tech>>

This project is licensed under the GNU General Public License v3.0. See [LICENSE](LICENSE) for details.

Note: Individual drivers and libraries included in this project may have their own licenses as noted in the acknowledgments below.

## Acknowledgments

This project incorporates code and ideas from the following projects:

| Project | Author(s) | License | Used For |
|---------|-----------|---------|----------|
| [pico-launcher](https://github.com/DnCraptor/pico-launcher) | [xrip](https://github.com/xrip), [DnCraptor](https://github.com/DnCraptor) | GPL-2.0 | Boot redirect mechanism, UF2 flashing pattern, flash layout |
| [pico_hdmi](https://github.com/fliperama86/pico_hdmi) | fliperama86 | Public domain | HSTX HDMI output (via frank-quake2's driver) |
| [Inter](https://github.com/rsms/inter) | Rasmus Andersson | SIL OFL 1.1 | The UI fonts (`fonts/OFL.txt`) |
| [FatFS](http://elm-chan.org/fsw/ff/) | ChaN | Custom permissive | FAT32 filesystem for SD card |
| [pico-infonesPlus](https://github.com/fhoedemakers/pico-infonesPlus) | shuichitakano, fhoedemakers | MIT | NES/SNES gamepad PIO driver |
| [TinyUSB](https://github.com/hathach/tinyusb) | Ha Thach | MIT | USB HID host driver |
| [Raspberry Pi Pico SDK](https://github.com/raspberrypi/pico-sdk) | Raspberry Pi Foundation | BSD-3-Clause | Hardware abstraction layer |

Background music: "Echoes of Neon Dreams". Song by <a href="https://stocktune.com/free-music/echoes-of-neon-dreams-263948-157491?signup=true">StockTune</a>.

Special thanks to:

- **[xrip](https://github.com/xrip)** and **[DnCraptor](https://github.com/DnCraptor)** for [pico-launcher](https://github.com/DnCraptor/pico-launcher), whose boot redirect and UF2 flashing mechanism forms the foundation of this project
- **Luke Wren** for PicoDVI, which this launcher used up to version 1.04
- **shuichitakano** and **fhoedemakers** for the NES/SNES gamepad PIO driver
- The Raspberry Pi Foundation for the RP2350 and Pico SDK
- The Murmulator community for hardware designs and testing

## Author

Mikhail Matveev <<xtreme@rh1.tech>>

[https://rh1.tech](https://rh1.tech) | [GitHub](https://github.com/rh1tech/frank-kickstart)

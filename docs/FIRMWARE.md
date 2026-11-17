# Firmware compatibility

What frank-kickstart can install and launch, checked against each repository's
latest GitHub release (16 November 2026). The launcher runs only on the **M1,
M2 and Z0** RP2350 boards. RP2040 and RISC-V images are refused, and so is any
image that reaches the launcher's flash area.

Flash layout (16 MB): the user image starts at `0x10000000`. The launcher owns
`0x10FBF000`–`0x10FFFFFF`: a 4 KB ZERO_BLOCK at `0x10FBF000`, which holds the
image's original sector 0, then 256 KB of launcher code at `0x10FC0000`. A
firmware that erases or programs anything in that range at run time destroys
the launcher. The launcher's own UF2 check only covers the image, not what the
firmware writes later.

## All rh1tech firmware repositories

Board tokens come from the release asset names: `m1`/`m1p2` → M1,
`m2`/`m2p2` → M2, `z0`/`z0p2` → Z0. Core 2 (`core2`, `c2`), `z2`, `pc`, `dv`,
`tw`, `s2`, RP2040 (`p1`, `rp2040`) and RISC-V builds are ignored. The asset
shown is the one to use: the HDMI build with sound over HDMI where one exists
(HSTX on M2, `hdmi-pio-audio` on M1/Z0), otherwise the plain HDMI build. "+N"
counts the other variants for that board (VGA, TV, PIO/HSTX, audio path or
clock).

| Firmware | Boards | Release asset per board (latest) | Flash collision | Status | Notes |
|---|---|---|---|---|---|
| frank-386 (v1.08) | M1 M2 | M1 `frank-386_m1_1_08.uf2`; M2 `frank-386_m2_1_08.uf2` | none | Supported | SD entry `386`. Needs 8 MB PSRAM. `frank-386_z2_1_08.uf2` is built for the Waveshare RP2350-PiZero, the same board as Z0, so it is a Z0 candidate; the token rules ignore `z2` and it is untested with the launcher |
| frank-amiga | – | no release | – | Out of scope | FRANK Core 2 (dual RP2350) only. For M1/M2/Z0 see drewpo28/pico-amiga below |
| frank-apple (release_1_04) | M1 M2 | M1/M2 `frank_apple_<m1\|m2>_hdmi_i2s_rp2350_1_04.uf2` (+3 each) | none | Supported | SD entry `apple`. The `_rp2040_` builds are refused |
| frank-blood (v1.01) | M2 | M2 `m2_frank-blood_1_01.uf2` | none | Supported | SD entry `blood` |
| frank-c64 (v1.07) | M1 M2 Z0 | `frank-c64_<m1\|m2\|z0>_rp2350_hdmi_i2s_378mhz_1_07.uf2` (+9 each) | none | Supported | SD entry `c64`. No HDMI audio: on Z0 fit an I2S DAC or use `hdmi_pwm`. The `_rp2040_` builds are refused |
| frank-castle (v1.00) | M2 | M2 `m2p2_frank-castle_1_00.uf2` | none (13.3 MB image, ends below `0x10FBF000`) | Supported | SD entry `castle` |
| frank-cabal | – | no release | – | Out of scope | ScummVM work in progress; frank-quest is the released ScummVM port |
| frank-cpc (v1.00) | M1 M2 Z0 | `<m1p2\|m2p2\|z0p2>_frank-cpc_1_00_hdmi-pio-audio.uf2` (+2 M1, +2 M2, +1 Z0) | none | Supported | SD entry `cpc`. No HSTX build; `hdmi-pio-audio` carries sound over HDMI (Z0 has no DAC) |
| frank-digger (v1.01) | M1 M2 | M1 `frank-digger_m1_1_01.uf2`; M2 `frank-digger_m2_1_01.uf2` | conditional: high scores at `PICO_FLASH_SIZE_BYTES - 4K` | Supported | The release is built with a 4 MB flash size, so scores land at `0x3FF000`, well below the launcher. A 16 MB build would erase `0xFFF000` (launcher code) |
| frank-dino (v1.00) | own board | `frank-dino-1.00.uf2` | – | Out of scope | dino_z80 board (real Z80), not M1/M2/Z0 |
| frank-doom (release_1_12) | M1 M2 | archived | – | Out of scope | Archived; superseded by frank-idtech1 |
| frank-duke3d (v1.06) | M1 M2 | `frank-duke3d_<m1\|m2>_378_133_1_06.uf2` (+1 each, 504 MHz) | none | Supported | SD entry `duke3d` |
| frank-ecat (v1.00) | M2 | M2 `m2p2_frank-ecat-<machine>_1_00.uf2`, 11 machines | none (its own data hole at `0x10F00000`, below the launcher) | Supported | SD entries `ecat-agat`, `ecat-bk`, `ecat-dvk`, `ecat-e85`, `ecat-irisha`, `ecat-orion`, `ecat-poisk`, `ecat-pravetz`, `ecat-rk86`, `ecat-uknc`, `ecat-unior`. Software from `frank-ecat-sdcard_1_00.zip` goes in `/ecat/` |
| frank-fodder (v1.00) | M1 M2 | `<m1p2\|m2p2>_frank-fodder_1_00.uf2` | none | Supported | SD entry `fodder` |
| frank-gamepad (v1.02) | M2 | `frank-gamepad.uf2` (no board token; the README says M2 only) | none | Supported | SD entry `gamepad` |
| frank-genesis (v1.02) | M1 M2 | `frank-genesis_<m1\|m2>_378_133_1_02.uf2` (+1 each, 504 MHz) | none | Supported | SD entry `genesis` |
| frank-heretic (release_1_03) | M1 M2 | archived | – | Out of scope | Archived; superseded by frank-idtech1 |
| frank-idtech1 (v1.01) | M1 M2 Z0 | `frank-idtech1_<m1\|m2\|z0>_1_01.uf2` | none | Supported | SD entry `idtech1` (DOOM, Heretic, Hexen, Strife). On Z0 the keyboard goes on the PIO-USB socket |
| frank-linux | – | no release | – | Out of scope | FRANK Core 2 only |
| frank-manul (v1.00) | M2 | M2 `frank-manul_m2_1_00.uf2` | **yes**: settings in `0xFF0000`–`0xFFFFFF` | Unsupported | See the fixes below |
| frank-micro (v1.00) | M1 M2 Z0 | `<m1p2\|m2p2\|z0p2>_frank-micro_1_00_hdmi-pio-audio.uf2` (+2 M1, +2 M2, +1 Z0) | none | Supported | SD entry `micro`. `hdmi-pio-audio` is the README's default build |
| frank-msx (v0.01) | M1 M2 Z0 | M1 `frank-msx_m1-hdmi-pio_0_01.uf2`; M2 `frank-msx_m2-hdmi-hstx_0_01.uf2`; Z0 `frank-msx_z0-hdmi-pio_0_01.uf2` (+1 M1, +3 M2) | none | Supported | SD entry `msx`. HSTX is the release's default M2 build (HDMI audio). The unreleased 0.02 writes cartridges to flash `0x100000`–`0x380000`, still below the launcher |
| frank-nes (v1.12) | M1 M2 Z0 | M1 `m1p2_frank-nes_1_12_hdmi_vga.uf2`; M2 `m2p2_frank-nes_1_12_hdmi_hstx.uf2`; Z0 `z0p2_frank-nes_1_12_hdmi_vga.uf2` (+1 M1, +3 M2) | none | Supported | SD entry `nes`. HSTX on M2 carries HDMI audio |
| frank-netcard (v1.01) | – | `frank-netcard_1_01.bin` | – | Out of scope | ESP8266 firmware |
| frank-os (v1.04) | M2 | M2 `frankos_m2_1_04_hdmi.uf2` (+1 VGA) | **yes**: a rival launcher with its own layout in the top 1 MB | Unsupported | See the fixes below |
| frank-prince (release_1_04) | M1 M2 | `frank-prince_<m1\|m2>_378_133_1_04.uf2` (+2 each) | none | Supported | SD entry `prince` |
| frank-quake2 (v1.04) | M2 Z0 | `<m2p2\|z0p2>_frank-quake2_hdmi_1_04.uf2` (+1 each, I2S) | **yes**: asset image from `0x1C0000` to the end of flash | Unsupported | See the fixes below |
| frank-quest (v1.00) | M1 M2 | `<m1p2\|m2p2>_frank-quest_1_00.uf2` | none | Supported | SD entry `quest` |
| frank-snes (v1.11) | M1 M2 | `frank-snes_<m1\|m2>_1_11.uf2` | none | Supported | SD entry `snes` |
| frank-test (v1.09) | (all RP2350) | `frank-test_1_09_rp2350a.uf2`, `frank-test_1_09_rp2350b.uf2` | **yes**: settings in the last sector of the chip | Unsupported | See the fixes below |
| frank-video (v1.00) | M1 M2 | M1 `m1p2_hdmi_pio_frank-video_1_00.uf2`; M2 `m2p2_hdmi_hstx_frank-video_1_00.uf2` (+1 PIO) | none | Supported | SD entry `video`. HSTX on M2 carries sound over HDMI; `hdmi_pio` uses the I2S DAC |
| frank-war1 (v1.02) | M2 | M2 `m2_frank-war1_1_02.uf2` | none | Supported | SD entry `war1` |
| frank-war2 (v1.00, private, no GitHub release) | M2 | M2 `release/m2_frank-war2_1_00.uf2` (PSX edition), `build/m2-hid/frank-war2-pc.uf2` (PC edition) | none | Supported | SD entries `war2`, `war2pc`; data in `/war2` (`disc/` from the PSX disc, `dos/` from the PC CD, `music/`) |
| frank-wolf3d (v1.01) | M1 M2 | `frank-wolf3d_<m1\|m2>_1_01.uf2` (+1 each, 504 MHz) | none | Supported | SD entry `wolf3d` |
| frank-xt8086 (v1.00) | own board | `frank-xt8086-1.00.uf2` | – | Out of scope | xt8086 beta board (real 8086) |
| murmulator-os2 (fork) | – | no release in the fork | – | Out of scope | Fork of DnCraptor's OS, which is its own launcher (see the incompatible list) |
| SpeccyP (fork) | – | no release in the fork | – | (see below) | The catalogue uses upstream billgilbert7000/SpeccyP |

Not firmware, so not listed above: frank, frank-lab, frank-archive,
frank-firmware, frank-rh1-tech, frank-hdmi-audio (a library), frank-kickstart
itself, sotn-decomp and psyz (game-code forks used by frank-castle), and the
RP2040/ESP32 projects (hecate, iris, hermes, nyx, echo, keyboard, ps2lib,
protea32).

### Other authors

DnCraptor, drewpo28, xrip and billgilbert7000. xrip's own releases are 2023–2024
RP2040 builds without a board token; the current RP2350 builds of his emulators
are published from the DnCraptor forks, so they are listed under DnCraptor.
"Image end" is the highest address in the M2 UF2 (the single RP2350-E10
"absolute" block at `0x10FFFF00` ignored); every image listed is RP2350 ARM-S.

| Firmware (version) | SD entry | M1 asset | M2 asset | Z0 asset | Flash risk | Notes |
|---|---|---|---|---|---|---|
| DnCraptor/PICO-BK (v.1.5.6) | `bk` | `m1p2-bk-VGA-HDMI-400MHz-I2S-1.5.6.uf2` | `m2p2-bk-VGA-HDMI-400MHz-I2S-1.5.6.uf2` | `z0p2-bk-VGA-HDMI-400MHz-I2S-1.5.6.uf2` | none (no flash writes) | z0p2 builds are new in 1.5.6 (I2S and PWM). The `m1p1` builds are RP2040 |
| drewpo28/pico-spec (1.2.30) | `picospec` | `m1p2-spec-VGA-HDMI-1.2.30.uf2` | `m2p2-spec-VGA-HDMI-1.2.30.uf2` | `z0p2-spec-VGA-HDMI-1.2.30.uf2` | low: 4 MB link map; the MIDI (GM) bank sits at `0x180000`–`0x400000` and is written only when the bank is provisioned; custom ROMs stay inside the image. Ends `0x1017AA00` | drewpo28's fork carries pico-spec on (1.2.18–1.2.30); DnCraptor/pico-spec is still at v.1.2.17. Asset names no longer carry the clock. Credits DnCraptor and drewpo28 |
| drewpo28/pico-speccy (1.1.1) | `speccy` | `m1p2-speccy-VGA-HDMI-4MB-1.1.1.uf2` | `m2p2-speccy-VGA-HDMI-HSTX-4MB-1.1.1.uf2` | `z0p2-speccy-VGA-HDMI-4MB-1.1.1.uf2` | low with the **-4MB** builds (see below); **the -16MB builds collide** | Successor of pico-spec: 48K to +3, Pentagon, Profi, Quorum, ALF; MIDI and Betadisk. PSRAM recommended. 1.1.2 is a pre-release |
| drewpo28/pico-amiga (0.1.0) | `amiga` | `m1p2-amiga-0.1.0.uf2` | `m2p2-amiga-0.1.0.uf2` | `z0p2-amiga-0.1.0.uf2` | none: no flash writes on the device. Ends `0x1035C900` | Needs 8 MB QSPI PSRAM and a Kickstart ROM (built in, or a 256/512 KB file in `/pico-amiga`); set 504 MHz in Options > Overclock. M2 uses HSTX, M1 PIO. The 0.1.1 pre-release also fits (ends `0x105DDD00`) |
| billgilbert7000/SpeccyP (1.8.0, `release/` folder) | `speccyp` | `SpeccyP_1.8.0_m1p2.uf2` | `SpeccyP_1.8.0_m2p2.uf2` (+1 HDMI audio) | `SpeccyP_1.8.0_z0p2.uf2` (+1 PCM5122) | none (no flash writes) | No GitHub releases; the `release/` folder still tops out at 1.8.0 |
| DnCraptor/quakegeneric (RC2, assets 1.09) | `quake` | `m1p2-quake-VGA-HDMI-…-504MHz-P100MHz-1.09.uf2` | `m2p2-quake-VGA-HDMI-HSTX-504MHz-P100MHz-1.09.uf2` | `z0p2-quake-VGA-HDMI-252MHz-P100MHz-1.09.uf2` | none (writes only inside its own image) | No release after RC2 |
| DnCraptor/pico-gameboy (v.3.2.9) | `gameboy` | `m1p2-gameboy-color-HDMI-378-I2S-3.2.9.uf2` | `m2p2-gameboy-color-HDMI-378-I2S-3.2.9.uf2` | `z0p2-gameboy-color-HDMI-378-I2S-3.2.9.uf2` | none: ROM cache after the image, stops 512 KB below the flash end on purpose. Ends `0x10051A00` | Peanut-GB core. Also 252/504 MHz, PWM and VGA variants |
| DnCraptor/pico-wonderswan (v.1.0.6) | `wswan` | `m1p2-wonderswan-HDMI-I2S-1.0.6.uf2` | `m2p2-wonderswan-HDMI-I2S-1.0.6.uf2` | `z0p2-wonderswan-HDMI-I2S-1.0.6.uf2` | none: flash fallback slot at 2 MB, refused past flash end − 512 KB. Ends `0x10087300` | Oswan core. PSRAM recommended |
| DnCraptor/pico-watara (v.2.2.1) | `watara` | `m1p2-watara-HDMI-I2S-2.2.1.uf2` | `m2p2-watara-HDMI-I2S-2.2.1.uf2` | `z0p2-watara-HDMI-I2S-2.2.1.uf2` | none: ROMs in PSRAM or a ≤64 KB slot after the image. Ends `0x10040E00` | Potator core. 48 variants (TV, VGA too) |
| DnCraptor/pico-gamate (v.3.1.2) | `gamate` | `m1p2-gamate-HDMI-I2S-3.1.2.uf2` | `m2p2-gamate-HDMI-I2S-3.1.2.uf2` | `z0p2-gamate-HDMI-I2S-3.1.2.uf2` | none: small slot after the image. Ends `0x10050B00` | The M2 list also has a stray 3.1.1 and a 3.1.2-1 asset; use the plain 3.1.2 |
| DnCraptor/pico-z26 (v.4.0.7) | `atari2600` | `m1p2-z26-HDMI-I2S-4.0.7.uf2` | `m2p2-z26-HDMI-I2S-4.0.7.uf2` | `z0p2-z26-HDMI-I2S-4.0.7.uf2` | none: last ROM kept just after the image (≤512 KB). Ends `0x10047700` | z26 core. DnCraptor/pico-atari2600 has no releases |
| DnCraptor/pico-j2me (v.0.1.1) | `j2me` | `m1p2-j2me-HDMI-I2S-378MHz-0.1.1.uf2` | `m2p2-j2me-HDMI-I2S-378MHz-0.1.1.uf2` | `z0p2-j2me-HDMI-I2S-378MHz-0.1.1.uf2` | none: no flash writes. Ends `0x1007DB00` | Nano Java VM. QSPI PSRAM needed for real games; JARs in `/j2me`. 504 MHz variants exist |
| DnCraptor/emu80v4 (v.0.7.6-vector) | `vector06c` | `m1p2-v06c-400-HDMI-DVI-0.7.6.uf2` | `m2p2-v06c-400-HDMI-DVI-0.7.6.uf2` | `z0p2-v06c-400-HDMI-DVI-0.7.6.uf2` | none: the flash-writing USB drivers are not linked. Ends `0x10063600` | One repository, one tag per machine; this tag is not marked Latest |
| DnCraptor/emu80v4 (v.0.7.1-korvet) | `korvet` | `m1p2-korvet-400-HDMI-DVI-0.7.1.uf2` | `m2p2-korvet-400-HDMI-DVI-0.7.1.uf2` | `z0p2-korvet-400-HDMI-DVI-0.7.1.uf2` | none. Ends `0x10064B00` | Korvet PK8020, boots CP/M disk images |
| DnCraptor/emu80v4 (v.0.5.4-lvov) | `lvov` | `m1p2-lvov-400-HDMI-DVI-0.5.4.uf2` | `m2p2-lvov-400-HDMI-DVI-0.5.4.uf2` | – | none. Ends `0x1002EC00` | PK-01 Lvov. No Z0 build |
| DnCraptor/pico-alf (v1.2.B) | `alf` | `m1p2-alf-378-PWM-HDMI-1.2.B.uf2` | `m2p2-alf-378-PWM-HDMI-1.2.B.uf2` | – (`z0p1` only, RP2040) | none: cartridges go to ROM arrays inside the image. Ends `0x1019F100` | ALF TV Game. The only HDMI build has PWM sound (I2S builds are VGA only) |
| DnCraptor/murmulator-tester (v.1.2.2) | `tester` | `m1p2-murmulator-tester-VGA-HDMI-1.2.2.uf2` | `m2p2-murmulator-tester-VGA-HDMI-1.2.2.uf2` | – | low: the flash self-test erases and programs one sector at `0x20000`, inside the app area. Ends `0x1001C900` | v.1.2.4 is a pre-release with fewer assets. Overlaps frank-gamepad / frank-test |
| DnCraptor/pico-lynx (v.2.1.3) | `lynx` | `m1pX-lynx-2.1.3.zip` → `m1p2-lynx-HDMI-I2S-TDA1387.uf2` | `m2pX-lynx-2.1.3.zip` → `m2p2-lynx-HDMI-I2S-TDA1387.uf2` | – | none: ROM slot after the image, refused past flash end − 512 KB. Ends `0x1002D100` | Handy core. Old release (March 2025), UF2s inside a zip |
| DnCraptor/pico-sms (v.2.1.3) | `sms` | `m1pX-sms-gg-2.1.3.zip` → `m1p2-sms-HDMI-I2S-TDA1387.uf2` | `m2pX-sms-gg-2.1.3.zip` → `m2p2-sms-HDMI-I2S-TDA1387.uf2` | – | low: the ROM is written right after the image; the size check would allow ~16 MB but SMS/GG ROMs are ≤1 MB. Ends `0x1003D100` | SMS Plus core. xrip/pico-sms releases are RP2040 only |

Skipped although compatible: DnCraptor/pico-nes v.3.0.4 (`dendy`; InfoNES, a
duplicate of frank-nes), and DnCraptor's forks of frank-386 (murm386 v.1.18),
frank-c64 (murmc64 v.1.05, older than 1.07) and frank-apple (murmapple RC6),
which duplicate the rh1tech entries.

**pico-speccy: use the `-4MB` builds.** Without PSRAM it writes its General
MIDI wavetable bank into flash, from the end of the image up to the build's
flash size. In the `-4MB` builds that ends at the 4 MB mark, far below the
launcher. The `-16MB` builds run the bank up to 16 MB and overwrite the
launcher at `0x10FBF000`+ the first time the bank is written; do not use them
with frank-kickstart.

## Unsupported: fixes needed

None of these four has a catalogue entry. Each one destroys the launcher the
first time it writes its data, and the board then has to be reflashed over
BOOTSEL.

- **frank-test**, `core/settings.c:48-58`. `settings_offset()` returns
  `capacity - FLASH_SECTOR_SIZE` (`0xFFF000` on 16 MB), which is launcher
  code. `settings_save()` erases it (`core/settings.c:119`, `:147`).
  **Fix:** keep the settings below the launcher area: return
  `capacity - (260u << 10) - FLASH_SECTOR_SIZE` (`0xFBE000`), or check for
  the launcher's ZERO_BLOCK (`0x10FBF000`, word 1023 = `0x3836D91A`) and step
  under it only when it is there.
- **frank-manul** (local clone `frank-lynx`), `src/flash.c:16-17`.
  `FLASH_TARGET_OFFSET` is `16 MB - 64 KB`, so its 16 storage sectors sit in
  `0xFF0000`–`0xFFFFFF`, inside the launcher.
  **Fix:** `#define FLASH_TARGET_OFFSET (16 * 1024 * 1024 - (260 * 1024) - FLASH_STORAGE_SIZE)`
  (`0xFAF000`–`0xFBEFFF`).
- **frank-quake2**, `src/rp2350/q2_assets_flash.c:41` and `:189`. The model
  and skin image is written from `Q2_ASSETS_OFFSET` (`0x1C0000`) to
  `flash_size()`, the end of the chip.
  **Fix:** cap the end at the launcher, e.g.
  `assets_size = MIN(flash, 0xFBF000u) - Q2_ASSETS_OFFSET;` (about 14 MB is
  still left for the image). The SD entry `quake2` (`quake2.xml`, `quake2.bmp`,
  data in `/quake2/baseq2/`) is in `sdcard/kickstart/` already; it shows in the
  launcher once a fixed `quake2.uf2` is copied next to it.
- **frank-os** (local clone `rhea`). It is a launcher too: its system table is
  at `0x10FFF000` (`rhea/src/sys_table.h:39`), its own ZERO_BLOCK is at
  `0xEFF000` (`rhea/src/app.c:158-159`), and the OS and its apps live in the
  top 1 MB. That overlaps the launcher, and the two boot redirects fight.
  **Fix:** none that is small. Use frank-os instead of frank-kickstart, not
  with it, or move its whole reserved area below `0xFBF000` and drop its boot
  redirect.

### Incompatible third-party firmware

- **DnCraptor/murmulator-os2** (v.2.3.2). The UF2 itself is placed at
  `0x10FBF000`–`0x10FEC000` and `0x10FFF000` (`ZERO_BLOCK_OFFSET` =
  16 MB − 256 KB − 4 KB): it is a launcher/OS living in the same slot, and its
  built-in UF2 flasher rewrites flash. The same goes for murmulator-os
  (v.0.3.5) and pico-launcher (v3), which are launchers too.
- **DnCraptor/pico-mm** (PicoMite MMBasic, V7.0.00b9 RC5). Its `A:` drive
  (LittleFS) runs from the system area to the detected flash size, so it
  formats and writes over `0xFBF000`. It would need a build with a capped
  flash size. The older PicoMiteAllVersions has the same drive.
- **DnCraptor/atari800** (v.3.2.6). The released builds are VGA only; HDMI
  code exists in the source but is not released. Recheck later.
- **M1 VGA only, no HDMI or M2 build**: pico-pce (v.1.1.1), pc (286 PC, RC1,
  superseded by frank-386), pico-mac- (Macintosh 128K/512K).
- **Other**: PicoVoxel (ST7789 LCD only), pico-rk86 (one RP2040 VGA build;
  ecat-rk86 covers the machine).

Not checked here (other authors, M2 builds published): PicoPlus-devel's
pico-outrun, pico-galagino and pico-genesisPlus.

## SD card layout & naming convention

```
/kickstart/<name>.uf2    the firmware (any release build, renamed)
/kickstart/<name>.xml    metadata (optional)
/kickstart/<name>.bmp    cover art (optional)
/kickstart/.last         written by the launcher: the firmware now in flash
```

**Convention: `<name>` is the firmware's short name, which is the repository
name without `frank-`.** For example `nes`, `snes`, `idtech1`, `war1`,
`castle`. frank-ecat has one firmware per machine family, so it gets one entry
per family: `ecat-<machine>` (`ecat-bk`, `ecat-uknc`, …), the machine part of
`frank-ecat-<machine>`. Third-party firmware gets a short name for what it
runs: `bk` (PICO-BK), `picospec`, `speccy` (pico-speccy), `speccyp`, `quake`,
`amiga`, `gameboy`, `wswan`, `watara`, `gamate`, `atari2600` (pico-z26),
`j2me`, `vector06c`, `korvet` and `lvov` (emu80v4 tags), `alf`, `tester`
(murmulator-tester), `lynx`, `sms`. Download the build for your board from the
tables above and rename the release UF2 to `<name>.uf2`, for example:

```
m2p2_frank-nes_1_12_hdmi_hstx.uf2           ->  /kickstart/nes.uf2
frank-idtech1_z0_1_01.uf2                   ->  /kickstart/idtech1.uf2
m2p2_frank-ecat-uknc_1_00.uf2               ->  /kickstart/ecat-uknc.uf2
m2p2-speccy-VGA-HDMI-HSTX-4MB-1.1.1.uf2     ->  /kickstart/speccy.uf2
```

The other option was to keep the release file name minus the version. It was
rejected for these reasons:

- The release names carry the board and build variant (`m1p2_…_hdmi_vga`,
  `…_378_133_…`) and follow a different pattern in almost every repository. One
  metadata pair per build would mean 9 XML files for NES alone.
- One file per firmware also means a new release only needs the UF2 swapped.
- The existing entries and `tools/host_render.sh` (which uses `apple`, `nes`
  and `bk`) already work this way.

Every XML lists its boards in `<boards>`. The launcher shows them as pills
and warns when the running board is not listed. When an XML has no `<boards>`,
the launcher reads board tokens from the file name. So someone who keeps the
release file name and has no XML still gets the per-build warning. With a
`<boards>` list, a wrong-board build (an M1 file on an M2 board, say) is not
caught. Pick the right build when you download.

XML format (UTF-8, under 4 KB, `&amp; &lt; &gt;` for `& < >`):

```xml
<firmware>
  <title>NES</title>
  <version>1.12</version>
  <author>Mikhail Matveev; QuickNES</author>
  <website>https://github.com/rh1tech/frank-nes</website>
  <description>Two or three plain sentences.</description>
  <controls>One short line.</controls>
  <boards>m1 m2 z0</boards>
</firmware>
```

What fits on screen (measured with `tools/host_render.sh`): the title on one
line is about 20 characters; a two-line title costs a line of description.
The `v<version> · <author>` line is about 40 characters. With the board pills
shown, the description gets 3 lines of about 45 characters (roughly 130
characters) and the controls 2 lines (about 90). Anything longer is cut with
an ellipsis.

Cover art: `python3 tools/mkcover.py image IN.png OUT.bmp` (fills the card),
`… cutout IN.png OUT.bmp [--remove-bg TOL]` (an object with transparency,
shown floating with a shadow), or
`… screenshot IN.png OUT.bmp --title "…" --kind GAME|CONSOLE|COMPUTER|OS|TOOL`
(200x300 at most 240x320, 8-bit, 128 colours; `#FF00FF` is transparent).
Bottom-up and top-down BMPs both load. Sources and licences of the bundled
covers: [COVERS.md](COVERS.md).

## Return to launcher

No firmware has a "back to the launcher" command yet. Power-cycling always
returns to the launcher. The flashed image's reset vector points at the
launcher (`src/flash/flash_uf2.c:73-80`), and the launcher's boot redirect
(`src/fw_boot_redirect.c:25-52`) jumps into the firmware only when
`watchdog_hw->scratch[0]` holds the one-shot boot magic `0x383DA910` that the
launcher sets just before it reboots into the firmware. In the launcher, the
installed firmware is preselected, and starting it again does not reflash it.

This also means any software reset by a firmware (a watchdog reboot) already
lands in the launcher, not back in the firmware. A firmware author can add an
explicit menu item. Set the launcher's UI magic and reboot, after the same
clean-up every warm reset needs (frank-ecat's `platform_reboot`):

```c
#include "hardware/watchdog.h"
#include "pico/multicore.h"

void return_to_launcher(void) {
    multicore_reset_core1();          /* stop core 1 first */
    psram_leave_qpi();                /* board code: PSRAM back to SPI mode,
                                         or the boot ROM cannot read flash */
    watchdog_hw->scratch[0] = 0x17F00FFF;   /* SCRATCH_MAGIC_UI */
    watchdog_reboot(0, 0, 0);
    for (;;) tight_loop_contents();
}
```

`0x17F00FFF` tells the launcher to show its UI (it clears the register). It
works the same on a board without the launcher, where it is just a reboot.

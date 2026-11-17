/*
 * frank-kickstart - background music over HDMI (M2): an IMA ADPCM WAV
 * (44.1 kHz stereo) streamed from the SD card, looped.
 *
 * Copyright (c) 2026 Mikhail Matveev <xtreme@rh1.tech>
 * https://rh1.tech
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Core 0 (music_pump, from video_wait_vblank) reads the file into a 32 KB
 * ring in SRAM; core 1 (music_core1, its idle loop) decodes whole blocks
 * into a PCM ring; the HDMI IRQ takes the samples. Nothing past the open
 * touches the flash or PSRAM, so the flash can be written while it plays
 * (the reads stop then: it runs dry into silence).
 */
#ifndef FK_MUSIC_H
#define FK_MUSIC_H

#include <stdbool.h>

/* Opens the WAV and starts it (fading in if on, silent if not); false if
 * there is no file, it is not IMA ADPCM 44.1 kHz stereo, or the video has
 * no sound. */
bool music_start(const char *path, bool on);

/* There is music (started): it can be switched on and off. */
bool music_available(void);

/* On: fades in; off: fades out (it keeps its place). */
void music_set_on(bool on);
bool music_is_on(void);

/* Fades out (about half a second, waiting on vblanks) and closes the file. */
void music_stop(void);

/* Core 0: the file into the ring (from the frame hook). */
void music_pump(void);

/* Core 1: decodes what fits (from its idle loop, in RAM). */
void music_core1(void);

#endif

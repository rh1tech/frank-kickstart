#!/bin/bash
# frank-kickstart - renders the launcher screens on the host (no board
# needed): builds tools/host_render.c with the firmware's UI code, runs it
# on a test SD card made from sdcard/kickstart, and writes PNGs.
#
#   tools/host_render.sh [OUT_DIR]      (default build-host/; FK_PIO=1: M1/Z0 colours)
set -e
cd "$(dirname "$0")/.."
OUT=${1:-build-host}
SD="$OUT/sd"
mkdir -p "$OUT" "$SD/kickstart"
cp sdcard/kickstart/*.xml sdcard/kickstart/*.bmp "$SD/kickstart/"
# a UF2 header for every entry (an RP2350 image at 0x10000000), a few odd ones
python3 - "$SD/kickstart" <<'PY'
import os, struct, sys
d = sys.argv[1]
def uf2(path, family, nblocks):
    with open(path, "wb") as f:
        for i in range(2):
            f.write(struct.pack("<8I", 0x0A324655, 0x9E5D5157, 0x2000, 0x10000000 + 256 * i,
                                256, i, nblocks, family) + bytes(476) + struct.pack("<I", 0x0AB16F30))
for name in os.listdir(d):
    if name.endswith(".xml"):
        uf2(os.path.join(d, name[:-4] + ".uf2"), 0xE48BFF59, 4000)
uf2(os.path.join(d, "bk.uf2"), 0xE48BFF56, 800)              # an RP2040 build
import re
x = os.path.join(d, "apple.xml")                             # as if built for M1 only
s = re.sub(r"<boards>.*?</boards>", "", open(x).read()).replace("</firmware>", "<boards>m1</boards></firmware>")
open(x, "w").write(s)
PY
cc -O1 -g -DFK_HOST -DBOARD_M2 -DBOARD_NAME='"M2"' -DFK_VERSION_MAJOR=2 -DFK_VERSION_MINOR=0 \
    -Isrc -Isrc/video -Isrc/gfx -Isrc/ui -Isrc/data -Isrc/fonts -Isrc/audio \
    tools/host_render.c src/gfx/gfx.c src/gfx/palette.c src/fonts/fonts.c src/ui/ui.c src/ui/welcome.c \
    src/data/catalog.c src/data/fsio_host.c src/video/tmds_balanced.c -lm -o "$OUT/host_render"
FK_SD="$SD" FK_OUT="$OUT" "$OUT/host_render"
python3 - "$OUT" <<'PY'
import glob, os, sys
from PIL import Image
for p in glob.glob(os.path.join(sys.argv[1], "*.ppm")):
    Image.open(p).save(p[:-4] + ".png")
    os.remove(p)
PY
echo "PNGs in $OUT/"

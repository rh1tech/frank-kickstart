#!/bin/bash
# frank-kickstart - regenerates src/fonts/*.h from fonts/*.ttf (Inter 4.1,
# SIL OFL 1.1). The headers are committed; run this only to change them.
set -e
cd "$(dirname "$0")/.."
F=fonts
# Latin, Latin-1, Cyrillic (Russian), and the symbols the UI uses
R="32-126,0xA0-0xBB,0xD7,0x401,0x410-0x44F,0x451,0x2013,0x2014,0x2018,0x2019,0x201C,0x201D,0x2022,0x2026,0x2116,0x2190-0x2193,0x25B2,0x25B6,0x25BC,0x25C0,0x2713"
python3 tools/mkfont.py $F/Inter-Medium.ttf 12 font_small src/fonts/font_small.h --ranges "$R" --tracking 0.2
python3 tools/mkfont.py $F/Inter-Regular.ttf 15 font_body src/fonts/font_body.h --ranges "$R"
python3 tools/mkfont.py $F/Inter-SemiBold.ttf 15 font_bold src/fonts/font_bold.h --ranges "$R"
python3 tools/mkfont.py $F/InterDisplay-Bold.ttf 30 font_title src/fonts/font_title.h \
    --ranges "32-126,0x401,0x410-0x44F,0x451,0x2013,0x2014,0x2019,0x2026"
# the welcome logo: the letters of FRANK only
python3 tools/mkfont.py $F/InterDisplay-Black.ttf 52 font_huge src/fonts/font_huge.h --ranges "32,0x41,0x46,0x4B,0x4E,0x52"
# the welcome screen's KICKSTART: its letters only
python3 tools/mkfont.py $F/InterDisplay-Black.ttf 16 font_kick src/fonts/font_kick.h --ranges "32,0x41,0x43,0x49,0x4B,0x52,0x53,0x54"
# the start countdown: 1..4 only (flash is short)
python3 tools/mkfont.py $F/InterDisplay-Black.ttf 84 font_count src/fonts/font_count.h --ranges "32,0x31-0x34"

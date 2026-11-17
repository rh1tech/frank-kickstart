#!/bin/bash
# frank-kickstart build: ./build.sh [M1|M2|Z0] [extra cmake options]
#   M2 (default): HSTX video; M1, Z0: PIO video. -DVIDEO_BACKEND=PIO on M2
#   tests the M1/Z0 driver. The video test patterns: make -C build frank-kickstart-vtest
set -e
BOARD_VARIANT="${1:-M2}"
case "$BOARD_VARIANT" in
    M1|M2|Z0) ;;
    *) echo "usage: $0 [M1|M2|Z0] [cmake options]"; exit 1 ;;
esac
shift || true
rm -rf ./build
mkdir build
cd build
cmake -DBOARD_VARIANT=${BOARD_VARIANT} "$@" ..
make -j$(sysctl -n hw.ncpu 2>/dev/null || nproc) frank-kickstart

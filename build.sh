#!/bin/bash
rm -rf ./build
mkdir build
cd build

# Board variant: default M2
BOARD_VARIANT="${1:-M2}"

cmake \
	-DPICO_PLATFORM=rp2350 \
	-DBOARD_VARIANT=${BOARD_VARIANT} \
	..
make -j$(sysctl -n hw.ncpu 2>/dev/null || nproc)

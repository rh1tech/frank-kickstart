#!/bin/bash
# Convert all PNG images in ./firmware to 100x150 8-bit indexed BMP

IMGDIR="./sdcard/kickstart"
COUNT=0

for png in "$IMGDIR"/*.png; do
    [ -f "$png" ] || continue
    bmp="${png%.png}.bmp"
    echo "Converting: $(basename "$png") -> $(basename "$bmp")"
    magick "$png" -resize 100x150! -colors 239 -type palette -compress none BMP3:"$bmp"
    COUNT=$((COUNT + 1))
done

if [ "$COUNT" -eq 0 ]; then
    echo "No PNG files found in $IMGDIR"
    exit 1
fi

echo "Done: $COUNT image(s) converted."

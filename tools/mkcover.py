#!/usr/bin/env python3
#
# frank-kickstart - cover art for the SD card: a 200x300, 112-colour,
# 8-bit BMP (what the launcher shows without scaling).
#
#   tools/mkcover.py image IN.png OUT.bmp
#       any picture, scaled to cover 200x300 (cropped to fit)
#   tools/mkcover.py screenshot IN.png OUT.bmp --title "Quake II" [--kind GAME]
#       a generated cover: the screenshot on top, the title under it
#   tools/mkcover.py cutout IN.png OUT.bmp [--remove-bg 40] [--margin 8]
#       an object with transparency (the PNG's alpha, or a plain background
#       taken away from the edges): fitted inside 200x300, see-through
#       around it. The launcher shows it on the page with a soft shadow.
#
# Transparency in the BMP is the palette colour #FF00FF; the soft edges are
# blended into the launcher's page colour (MATTE) and the rest is cut.
#
# SPDX-License-Identifier: GPL-3.0-or-later
import argparse
import os

from PIL import Image, ImageDraw, ImageFilter, ImageFont

W, H = 200, 300
COLORS = 112  # ART_NCOLORS in src/data/catalog.h
KEY = (255, 0, 255)
MATTE = (0x0F, 0x14, 0x21)  # RGB_PAGE in src/ui/theme.h
HERE = os.path.dirname(os.path.abspath(__file__))
FONT_DIR = os.path.join(HERE, "..", "fonts")


def font(name, size):
    return ImageFont.truetype(os.path.join(FONT_DIR, name), size)


def fit_cover(im, w, h):
    """Scale to cover w x h, then crop the middle."""
    im = im.convert("RGB")
    s = max(w / im.width, h / im.height)
    nw, nh = max(w, round(im.width * s)), max(h, round(im.height * s))
    im = im.resize((nw, nh), Image.LANCZOS)
    x, y = (nw - w) // 2, (nh - h) // 2
    return im.crop((x, y, x + w, y + h))


def wrap(draw, text, f, max_w):
    words, lines, cur = text.split(), [], ""
    for wd in words:
        t = (cur + " " + wd).strip()
        if draw.textlength(t, font=f) <= max_w or not cur:
            cur = t
        else:
            lines.append(cur)
            cur = wd
    if cur:
        lines.append(cur)
    return lines


def screenshot_cover(shot, title, kind):
    top = (16, 21, 36)
    bottom = (30, 38, 62)
    im = Image.new("RGB", (W, H))
    d = ImageDraw.Draw(im)
    for y in range(H):
        t = y / (H - 1)
        d.line([(0, y), (W, y)], fill=tuple(int(a + (b - a) * t) for a, b in zip(top, bottom)))
    # the screenshot, 4:3, framed
    sw, sh = W - 24, (W - 24) * 3 // 4
    shot = fit_cover(shot, sw, sh)
    glow = Image.new("RGB", (W, H), (0, 0, 0))
    glow.paste(shot.resize((sw // 4, sh // 4)).resize((sw, sh)), (12, 20))
    glow = glow.filter(ImageFilter.GaussianBlur(10))
    im = Image.blend(im, glow, 0.35)
    im.paste(shot, (12, 20))
    d = ImageDraw.Draw(im)
    d.rectangle([11, 19, 12 + sw, 20 + sh], outline=(70, 82, 110))
    y = 20 + sh + 22
    if kind:
        f = font("Inter-SemiBold.ttf", 11)
        d.text((14, y), " ".join(kind.upper()), font=f, fill=(169, 124, 255))
        y += 22
    f = font("InterDisplay-Bold.ttf", 26)
    for line in wrap(d, title, f, W - 28)[:3]:
        d.text((13, y), line, font=f, fill=(238, 241, 248))
        y += 30
    return im


def remove_background(im, tol):
    """Alpha from a flood fill of the background from every edge pixel whose
    colour is within tol of the corners' (a product photo on white, say)."""
    from collections import deque
    rgb = im.convert("RGB")
    w, h = rgb.size
    px = rgb.load()
    corners = [px[0, 0], px[w - 1, 0], px[0, h - 1], px[w - 1, h - 1]]
    ref = tuple(sum(c[k] for c in corners) // 4 for k in range(3))
    def near(c):
        return sum((c[k] - ref[k]) ** 2 for k in range(3)) <= tol * tol
    alpha = Image.new("L", (w, h), 255)
    a = alpha.load()
    q = deque()
    for x in range(w):
        q.append((x, 0)); q.append((x, h - 1))
    for y in range(h):
        q.append((0, y)); q.append((w - 1, y))
    while q:
        x, y = q.popleft()
        if a[x, y] == 0 or not near(px[x, y]):
            continue
        a[x, y] = 0
        if x > 0: q.append((x - 1, y))
        if x < w - 1: q.append((x + 1, y))
        if y > 0: q.append((x, y - 1))
        if y < h - 1: q.append((x, y + 1))
    # soften the cut by a pixel
    alpha = alpha.filter(ImageFilter.GaussianBlur(0.8))
    out = rgb.convert("RGBA")
    out.putalpha(alpha)
    return out


def cutout(im, margin, remove_bg):
    im = im.convert("RGBA")
    if remove_bg is not None:
        im = remove_background(im, remove_bg)
    box = im.getchannel("A").point(lambda v: 255 if v > 24 else 0).getbbox()
    if box:
        im = im.crop(box)
    # fit inside, never upscaled past 2x
    s = min((W - 2 * margin) / im.width, (H - 2 * margin) / im.height, 2.0)
    im = im.resize((max(1, round(im.width * s)), max(1, round(im.height * s))), Image.LANCZOS)
    return im


def save_cutout_bmp(im, path):
    """127 colours for the object, index 127 the key."""
    a = im.getchannel("A")
    matte = Image.new("RGB", im.size, MATTE)
    rgb = Image.composite(im.convert("RGB"), matte, a)
    q = rgb.quantize(colors=COLORS - 1, method=Image.Quantize.MEDIANCUT, dither=Image.Dither.FLOYDSTEINBERG)
    pal = q.getpalette()[: 3 * (COLORS - 1)]
    pal += [0, 0, 0] * (COLORS - 1 - len(pal) // 3)
    pal += list(KEY)
    q.putpalette(pal)
    qp = q.load()
    ap = a.load()
    for y in range(q.height):
        for x in range(q.width):
            if ap[x, y] < 96:
                qp[x, y] = COLORS - 1
    q.save(path, format="BMP")
    print("%s: %dx%d cut-out, %d colours + key" % (path, q.width, q.height, COLORS - 1))


def save_bmp(im, path):
    q = im.quantize(colors=COLORS, method=Image.Quantize.MEDIANCUT, dither=Image.Dither.FLOYDSTEINBERG)
    q.save(path, format="BMP")
    print("%s: %dx%d, %d colours" % (path, q.width, q.height, len(q.getcolors(256) or [])))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("mode", choices=["image", "screenshot", "cutout"])
    ap.add_argument("input")
    ap.add_argument("output")
    ap.add_argument("--title", default="")
    ap.add_argument("--kind", default="")
    ap.add_argument("--remove-bg", type=int, default=None, metavar="TOL",
                    help="cutout: take away the background from the edges (colour distance)")
    ap.add_argument("--margin", type=int, default=8)
    a = ap.parse_args()
    src = Image.open(a.input)
    if a.mode == "cutout":
        save_cutout_bmp(cutout(src, a.margin, a.remove_bg), a.output)
        return
    if a.mode == "image":
        im = fit_cover(src, W, H)
    else:
        im = screenshot_cover(src, a.title, a.kind)
    save_bmp(im, a.output)


if __name__ == "__main__":
    main()

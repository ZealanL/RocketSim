#!/usr/bin/env python3
"""XWD (TrueColor ZPixmap) -> PNG for headless screenshot review.

Handles the Xephyr output used by tools/vis-shot.sh (depth 24, 24/32 bpp).
Vectorized with numpy so a 1280x720 capture converts in well under a second.
"""
import struct
import sys


def parse_header(data: bytes):
    hdr = struct.unpack(">25I", data[:100])
    (header_size, _version, pixmap_format, depth, width, height,
     _xoffset, byte_order, _bitmap_unit, _bit_order, _bitmap_pad, bpp,
     bytes_per_line, visual_class, red_mask, green_mask, blue_mask,
     _bits_per_rgb, _cmap_entries, ncolors, _win_w, _win_h, _win_x, _win_y,
     _win_bdr) = hdr
    return {
        "header_size": header_size,
        "pixmap_format": pixmap_format,
        "depth": depth,
        "width": width,
        "height": height,
        "byte_order": byte_order,
        "bpp": bpp,
        "bytes_per_line": bytes_per_line,
        "visual_class": visual_class,
        "red_mask": red_mask,
        "green_mask": green_mask,
        "blue_mask": blue_mask,
        "ncolors": ncolors,
    }


def channel_scale(v, mask):
    shift = (mask & -mask).bit_length() - 1
    bits = bin(mask).count("1")
    c = (v & mask) >> shift
    maxv = (1 << bits) - 1
    return (c * 255 // maxv) if maxv else c


def convert(src: str, dst: str):
    with open(src, "rb") as f:
        data = f.read()
    h = parse_header(data)
    assert h["pixmap_format"] == 2, f"need ZPixmap, got {h['pixmap_format']}"
    assert h["visual_class"] in (4, 5), f"need Pseudo/TrueColor, got {h['visual_class']}"
    print(
        f"xwd {h['width']}x{h['height']} depth={h['depth']} bpp={h['bpp']} "
        f"masks={h['red_mask']:08x}/{h['green_mask']:08x}/{h['blue_mask']:08x} "
        f"byte_order={h['byte_order']}",
        file=sys.stderr,
    )
    off = h["header_size"] + h["ncolors"] * 12
    width, height, stride = h["width"], h["height"], h["bytes_per_line"]
    rm, gm, bm = h["red_mask"], h["green_mask"], h["blue_mask"]
    msb = h["byte_order"] == 1

    import numpy as np
    from PIL import Image

    step = 4 if h["bpp"] in (24, 32) else h["bpp"] // 8
    buf = np.frombuffer(data, dtype=np.uint8, count=stride * height, offset=off)
    buf = buf.reshape(height, stride)[:, : width * step].reshape(height, width, step)
    if step >= 4:
        # 24/32bpp: bytes within each pixel follow the file byte order.
        if msb:
            v = (buf[:, :, 0].astype(np.uint32) << 24) | (buf[:, :, 1].astype(np.uint32) << 16) \
                | (buf[:, :, 2].astype(np.uint32) << 8) | buf[:, :, 3].astype(np.uint32)
        else:
            v = buf[:, :, 0].astype(np.uint32) | (buf[:, :, 1].astype(np.uint32) << 8) \
                | (buf[:, :, 2].astype(np.uint32) << 16) | (buf[:, :, 3].astype(np.uint32) << 24)
    else:
        v = buf[:, :, 0].astype(np.uint32)
        if step > 1:
            for i in range(1, step):
                if msb:
                    v = (v << 8) | buf[:, :, i].astype(np.uint32)
                else:
                    v = v | (buf[:, :, i].astype(np.uint32) << (8 * i))

    out = np.empty((height, width, 3), dtype=np.uint8)
    for i, mask in enumerate((rm, gm, bm)):
        shift = (mask & -mask).bit_length() - 1
        bits = bin(mask).count("1")
        maxv = (1 << bits) - 1
        c = ((v & np.uint32(mask)) >> np.uint32(shift)).astype(np.uint32)
        out[:, :, i] = ((c * 255 // maxv).astype(np.uint8) if maxv else c.astype(np.uint8))
    Image.fromarray(out, "RGB").save(dst)
    print(f"saved {dst}", file=sys.stderr)


if __name__ == "__main__":
    if len(sys.argv) != 3:
        print(f"usage: {sys.argv[0]} <in.xwd> <out.png>", file=sys.stderr)
        sys.exit(2)
    convert(sys.argv[1], sys.argv[2])

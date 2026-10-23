#!/usr/bin/env python3
"""
img2mem.py - convert an image into a $readmemh file for the projection engine.

Outputs (in the output directory):
  image.mem         W*H lines, one 8-bit grayscale pixel per line (2 hex digits),
                    raster order: addr = y*W + x
  golden_row.mem    H lines, 16-bit hex: horizontal projection (sum of each row)
  golden_col.mem    W lines, 16-bit hex: vertical projection (sum of each column)
  image_preview.png the exact image stored in the ROM, upscaled for viewing

Usage:
  python3 scripts/img2mem.py <input_image> [--width 64] [--height 64] [--out mem]
"""
import argparse
import os

import numpy as np
from PIL import Image


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("image")
    ap.add_argument("--width", type=int, default=64)
    ap.add_argument("--height", type=int, default=64)
    ap.add_argument("--out", default="mem")
    args = ap.parse_args()

    W, H = args.width, args.height
    os.makedirs(args.out, exist_ok=True)

    img = Image.open(args.image).convert("L")  # 8-bit grayscale

    # Centre-crop to the target aspect ratio so the face is not stretched.
    src_w, src_h = img.size
    target = W / H
    if src_w / src_h > target:
        new_w = int(round(src_h * target))
        left = (src_w - new_w) // 2
        img = img.crop((left, 0, left + new_w, src_h))
    else:
        new_h = int(round(src_w / target))
        top = (src_h - new_h) // 2
        img = img.crop((0, top, src_w, top + new_h))

    img = img.resize((W, H), Image.LANCZOS)
    pix = np.asarray(img, dtype=np.uint8)  # shape (H, W)

    with open(os.path.join(args.out, "image.mem"), "w") as f:
        for y in range(H):
            for x in range(W):
                f.write(f"{pix[y, x]:02x}\n")

    row = pix.astype(np.uint32).sum(axis=1)  # one value per row
    col = pix.astype(np.uint32).sum(axis=0)  # one value per column
    with open(os.path.join(args.out, "golden_row.mem"), "w") as f:
        f.writelines(f"{v:04x}\n" for v in row)
    with open(os.path.join(args.out, "golden_col.mem"), "w") as f:
        f.writelines(f"{v:04x}\n" for v in col)

    img.resize((W * 4, H * 4), Image.NEAREST).save(
        os.path.join(args.out, "image_preview.png"))

    print(f"Wrote {W}x{H} image ({W*H} bytes) to {args.out}/image.mem")
    print(f"Row sums: min {row.min()} max {row.max()}  "
          f"Col sums: min {col.min()} max {col.max()}  (max possible {255*max(W,H)})")


if __name__ == "__main__":
    main()

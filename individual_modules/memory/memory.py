#!/usr/bin/env python3
"""
memory.py - Generates vertices.mem and matrix.mem for the testbench.

Follows initial_modules.txt:
  * Q24.8 fixed point, 32-bit two's complement   (value = int / 256.0)
  * RGB565 colour, 16 bit
  * Vertex word = {X[31:0], Y[31:0], Z[31:0], W[31:0], R[4:0], G[5:0], B[4:0]}  (144 bit = 36 hex digits)
  * Cube = 6 faces x 2 triangles x 3 vertices = 36 vertices, one colour per face
  * Matrix = 16 x 32-bit Q24.8 elements

Usage:
  python3 memory.py                    # identity matrix, 36 vertices
  python3 memory.py --rot-z 30         # 30 degree rotation about Z instead of identity
  python3 memory.py --pad 64           # pad vertices.mem with zero words up to 64 lines
"""
import argparse
import math

FRAC = 8
MASK32 = 0xFFFFFFFF


def to_q24_8(x: float) -> int:
    """Float -> Q24.8 two's complement (round to nearest)."""
    return int(round(x * (1 << FRAC))) & MASK32


def rgb565(r8: int, g8: int, b8: int) -> int:
    return ((r8 >> 3) << 11) | ((g8 >> 2) << 5) | (b8 >> 3)


def pack_vertex(x, y, z, w, color16) -> int:
    return (
        (to_q24_8(x) << 112)
        | (to_q24_8(y) << 80)
        | (to_q24_8(z) << 48)
        | (to_q24_8(w) << 16)
        | (color16 & 0xFFFF)
    )


# Each face: 4 corners (CCW seen from outside) + a colour.
FACES = [
    ([(-1, -1, 1), (1, -1, 1), (1, 1, 1), (-1, 1, 1)],        rgb565(255, 0, 0)),     # +Z red
    ([(-1, -1, -1), (-1, 1, -1), (1, 1, -1), (1, -1, -1)],    rgb565(0, 255, 0)),     # -Z green
    ([(1, -1, -1), (1, 1, -1), (1, 1, 1), (1, -1, 1)],        rgb565(0, 0, 255)),     # +X blue
    ([(-1, -1, -1), (-1, -1, 1), (-1, 1, 1), (-1, 1, -1)],    rgb565(255, 255, 0)),   # -X yellow
    ([(-1, 1, -1), (-1, 1, 1), (1, 1, 1), (1, 1, -1)],        rgb565(0, 255, 255)),   # +Y cyan
    ([(-1, -1, -1), (1, -1, -1), (1, -1, 1), (-1, -1, 1)],    rgb565(255, 0, 255)),   # -Y magenta
]


def cube_vertices():
    words = []
    for corners, col in FACES:
        v0, v1, v2, v3 = corners
        for tri in ((v0, v1, v2), (v0, v2, v3)):
            for (x, y, z) in tri:
                words.append(pack_vertex(x, y, z, 1.0, col))
    assert len(words) == 36
    return words


def matrix_words(rot_z_deg):
    if rot_z_deg is None:
        m = [1, 0, 0, 0,
             0, 1, 0, 0,
             0, 0, 1, 0,
             0, 0, 0, 1]
    else:
        a = math.radians(rot_z_deg)
        c, s = math.cos(a), math.sin(a)
        m = [c, -s, 0, 0,
             s,  c, 0, 0,
             0,  0, 1, 0,
             0,  0, 0, 1]
    return [to_q24_8(e) for e in m]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--rot-z", type=float, default=None, help="use a Z rotation (deg) instead of identity")
    ap.add_argument("--pad", type=int, default=0, help="pad vertices.mem with zero words up to N lines")
    ap.add_argument("--outdir", default=".")
    args = ap.parse_args()

    verts = cube_vertices()
    with open(f"{args.outdir}/vertices.mem", "w") as f:
        for w in verts:
            f.write(f"{w:036x}\n")
        for _ in range(max(0, args.pad - len(verts))):
            f.write(f"{0:036x}\n")

    with open(f"{args.outdir}/matrix.mem", "w") as f:
        for w in matrix_words(args.rot_z):
            f.write(f"{w:08x}\n")

    print(f"vertices.mem: {max(len(verts), args.pad)} lines, matrix.mem: 16 lines "
          f"({'identity' if args.rot_z is None else f'rot-z {args.rot_z} deg'})")


if __name__ == "__main__":
    main()

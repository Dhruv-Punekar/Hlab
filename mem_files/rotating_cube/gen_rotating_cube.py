#!/usr/bin/env python3
"""
gen_rotating_cube.py  -  memory-file generator for the FPGA 3D pipeline
=======================================================================

Makes every .mem file the Nexys A7 pipeline needs, for a cube that rotates.
All frames are computed here once; they go into the bitstream through
$readmemh, so the FPGA itself does no trigonometry.

Files written to the output folder
----------------------------------
  vertices.mem         36 vertices (12 triangles, one colour per face) + zero
                       padding to 64 lines. 144 bits per line:
                       X | Y | Z | W (each 32-bit Q24.8) | colour (RGB565).
                       Every triangle is wound COUNTER-CLOCKWISE seen from
                       outside, as needed by back-face culling (CULL_MODE = 1).
  matrix.mem           ONE 4x4 matrix (16 lines, 32-bit Q24.8) = frame 0.
                       Works with the CURRENT hardware (single still picture).
  matrices.mem         ALL frames: frame f uses lines 16*f .. 16*f+15.
                       Needs the animation upgrade of matrix_memory / the
                       pipeline (one matrix per frame, chosen by a counter).
  recip_lut_q1_17.mem  1024-entry reciprocal table for triangle setup
                       (18-bit Q1.17, round(2^17 / (1 + (k + 0.5)/1024))).
  frames_info.txt      what was generated (settings, depth range per frame).
  preview/             (optional, needs Pillow) expected picture per frame
                       and an animated preview.gif of the whole rotation.

Matrix word order (IMPORTANT)
-----------------------------
transform_engine reads M00 from matrix_regs[15], so inside every frame the
16 words are written M33 first ... M00 last (reverse row-major). A file in
normal row-major order gives a black screen.

Number formats
--------------
  Q24.8 = signed 32-bit integer = value * 256, written as 8 hex digits
  (two's complement for negatives, e.g. -1.0 -> ffffff00).

Usage
-----
  python gen_rotating_cube.py                       # 120 frames, defaults
  python gen_rotating_cube.py --frames 60 --out mem_out
  python gen_rotating_cube.py --tilt 25 --spin-x 0 --spin-y 360 --preview
  python gen_rotating_cube.py --still-angle 35      # matrix.mem at 35 deg

Only the Python standard library is needed. Pillow (pip install pillow) is
used for the optional preview images.
"""

import argparse
import math
import os
import sys

# ----------------------------------------------------------------------------
# Hardware facts (keep in step with the Verilog)
# ----------------------------------------------------------------------------
SCREEN_W, SCREEN_H = 300, 400        # framebuffer size (pixels)
VERTEX_DEPTH = 64                    # vertex_memory DEPTH (lines in vertices.mem)
NUM_VERTICES = 36                    # hardware_pipeline NUM_VERTICES
Q = 256                              # Q24.8 scale
INT32_MIN, INT32_MAX = -2**31, 2**31 - 1

# one RGB565 colour per face (same as the verified demo)
FACE_COLORS = {
    (0, +1): 0xF800,   # +X red
    (0, -1): 0x07E0,   # -X green
    (1, +1): 0x001F,   # +Y blue
    (1, -1): 0xFFE0,   # -Y yellow
    (2, +1): 0xF81F,   # +Z magenta
    (2, -1): 0x07FF,   # -Z cyan
}


# ----------------------------------------------------------------------------
# Small helpers
# ----------------------------------------------------------------------------
def to_q24_8(value):
    """float -> signed Q24.8 integer, with a range check."""
    raw = int(round(value * Q))
    if not INT32_MIN <= raw <= INT32_MAX:
        raise ValueError(f"value {value} does not fit Q24.8")
    return raw


def hex32(raw):
    """signed 32-bit integer -> 8 hex digits (two's complement)."""
    return f"{raw & 0xFFFFFFFF:08x}"


def mat_mul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def rot_x(deg):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return [[1, 0, 0, 0], [0, c, -s, 0], [0, s, c, 0], [0, 0, 0, 1]]


def rot_y(deg):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return [[c, 0, s, 0], [0, 1, 0, 0], [-s, 0, c, 0], [0, 0, 0, 1]]


def rot_z(deg):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return [[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]


def translate(x, y, z):
    return [[1, 0, 0, x], [0, 1, 0, y], [0, 0, 1, z], [0, 0, 0, 1]]


def scale(s):
    return [[s, 0, 0, 0], [0, s, 0, 0], [0, 0, s, 0], [0, 0, 0, 1]]


def perspective(fov_y_deg, aspect, near, far):
    """OpenGL-style projection (camera looks down -Z, NDC z in [-1, 1])."""
    f = 1.0 / math.tan(math.radians(fov_y_deg) / 2)
    return [[f / aspect, 0, 0, 0],
            [0, f, 0, 0],
            [0, 0, (far + near) / (near - far), 2 * far * near / (near - far)],
            [0, 0, -1, 0]]


def quantise(m):
    """What the hardware really sees: every element rounded to Q24.8."""
    return [[to_q24_8(v) / Q for v in row] for row in m]


# ----------------------------------------------------------------------------
# Geometry: unit cube (-1..+1), 12 triangles, CCW seen from outside
# ----------------------------------------------------------------------------
def build_cube(size=1.0):
    tris = []
    for (axis, sign), colour in FACE_COLORS.items():
        u, v = [a for a in range(3) if a != axis]
        quad = []
        for a, b in [(-1, -1), (1, -1), (1, 1), (-1, 1)]:
            p = [0.0, 0.0, 0.0]
            p[axis], p[u], p[v] = sign * size, a * size, b * size
            quad.append(p)
        # make the corner order counter-clockwise when seen from outside
        e1 = [quad[1][i] - quad[0][i] for i in range(3)]
        e2 = [quad[2][i] - quad[0][i] for i in range(3)]
        normal = [e1[1] * e2[2] - e1[2] * e2[1],
                  e1[2] * e2[0] - e1[0] * e2[2],
                  e1[0] * e2[1] - e1[1] * e2[0]]
        if (normal[axis] > 0) != (sign > 0):
            quad = quad[::-1]
        tris.append((quad[0], quad[1], quad[2], colour))
        tris.append((quad[0], quad[2], quad[3], colour))
    return tris


def check_winding(tris):
    """Every triangle's normal must point away from the cube centre."""
    for a, b, c, _ in tris:
        e1 = [b[i] - a[i] for i in range(3)]
        e2 = [c[i] - a[i] for i in range(3)]
        n = [e1[1] * e2[2] - e1[2] * e2[1], e1[2] * e2[0] - e1[0] * e2[2], e1[0] * e2[1] - e1[1] * e2[0]]
        centre = [(a[i] + b[i] + c[i]) / 3 for i in range(3)]
        if sum(n[i] * centre[i] for i in range(3)) <= 0:
            return False
    return True


# ----------------------------------------------------------------------------
# Per-frame matrix
# ----------------------------------------------------------------------------
def frame_matrix(args, f):
    """Model-view-projection matrix for frame f (floating point)."""
    t = f / args.frames                          # 0 .. 1 over one loop
    ang_x = args.tilt + args.spin_x * t
    ang_y = args.start_y + args.spin_y * t
    ang_z = args.spin_z * t
    aspect = SCREEN_W / SCREEN_H                 # square pixels on 300 x 400
    P = perspective(args.fov, aspect, args.near, args.far)
    V = translate(0, 0, -args.distance)
    M = mat_mul(rot_x(ang_x), mat_mul(rot_y(ang_y), mat_mul(rot_z(ang_z), scale(args.size))))
    return mat_mul(P, mat_mul(V, M)), (ang_x, ang_y, ang_z)


def matrix_words(m):
    """16 hex words in HARDWARE order: M33 first ... M00 last."""
    row_major = [hex32(to_q24_8(m[r][c])) for r in range(4) for c in range(4)]
    return row_major[::-1]


# ----------------------------------------------------------------------------
# Checks done on every frame (same maths as the hardware, quantised matrix)
# ----------------------------------------------------------------------------
def project(mq, p):
    v = [p[0], p[1], p[2], 1.0]
    c = [sum(mq[r][k] * v[k] for k in range(4)) for r in range(4)]
    return c


def check_frame(mq, tris):
    """Returns (ok, message, z_min, z_max)."""
    zs = []
    det3 = (mq[0][0] * (mq[1][1] * mq[2][2] - mq[1][2] * mq[2][1])
            - mq[0][1] * (mq[1][0] * mq[2][2] - mq[1][2] * mq[2][0])
            + mq[0][2] * (mq[1][0] * mq[2][1] - mq[1][1] * mq[2][0]))
    if det3 >= 0:
        return False, "matrix contains a mirror (would flip winding / culling)", 0, 0
    for a, b, c, _ in tris:
        for p in (a, b, c):
            x, y, z, w = project(mq, p)
            if w <= 0:
                return False, "a corner is behind the camera (W <= 0)", 0, 0
            sx, sy, nz = (x / w + 1) * SCREEN_W / 2, (1 - y / w) * SCREEN_H / 2, z / w
            if not (0 <= sx <= SCREEN_W and 0 <= sy <= SCREEN_H):
                return False, f"a corner is off screen ({sx:.0f}, {sy:.0f})", 0, 0
            if not -1 <= nz <= 1:
                return False, f"a corner is outside near/far (z = {nz:.3f})", 0, 0
            zs.append(nz)
    return True, "ok", min(zs), max(zs)


# ----------------------------------------------------------------------------
# Optional preview (software copy of the pipeline: culling + z-buffer)
# ----------------------------------------------------------------------------
def render_preview(mq, tris):
    W, H = SCREEN_W, SCREEN_H
    zbuf = [float("inf")] * (W * H)
    fb = [(0, 0, 0)] * (W * H)
    for a, b, c, col in tris:
        pts = []
        for p in (a, b, c):
            x, y, z, w = project(mq, p)
            pts.append(((x / w + 1) * W / 2, (1 - y / w) * H / 2, z / w))
        (x0, y0, z0), (x1, y1, z1), (x2, y2, z2) = pts
        D = (x1 - x0) * (y2 - y0) - (x2 - x0) * (y1 - y0)
        if D >= 0:                      # back-facing on screen -> culled (CULL_MODE 1)
            continue
        rgb = (((col >> 11) & 31) * 255 // 31, ((col >> 5) & 63) * 255 // 63, (col & 31) * 255 // 31)
        for py in range(max(0, int(math.floor(min(y0, y1, y2)))), min(H - 1, int(math.ceil(max(y0, y1, y2)))) + 1):
            for px in range(max(0, int(math.floor(min(x0, x1, x2)))), min(W - 1, int(math.ceil(max(x0, x1, x2)))) + 1):
                cx, cy = px + 0.5, py + 0.5
                e0 = -((y0 - y1) * cx + (x1 - x0) * cy + (x0 * y1 - x1 * y0))
                e1 = -((y1 - y2) * cx + (x2 - x1) * cy + (x1 * y2 - x2 * y1))
                e2 = -((y2 - y0) * cx + (x0 - x2) * cy + (x2 * y0 - x0 * y2))
                if e0 >= 0 and e1 >= 0 and e2 >= 0:
                    l1 = ((cx - x0) * (y2 - y0) - (x2 - x0) * (cy - y0)) / D
                    l2 = ((x1 - x0) * (cy - y0) - (cx - x0) * (y1 - y0)) / D
                    z = z0 + l1 * (z1 - z0) + l2 * (z2 - z0)
                    i = py * W + px
                    if z < zbuf[i]:
                        zbuf[i] = z
                        fb[i] = rgb
    return fb


# ----------------------------------------------------------------------------
# Main
# ----------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description="Generate .mem files for a rotating cube on the FPGA pipeline.")
    ap.add_argument("--out", default="mem_out", help="output folder (default: mem_out)")
    ap.add_argument("--frames", type=int, default=120, help="frames in one full loop (default 120)")
    ap.add_argument("--tilt", type=float, default=25.0, help="fixed tilt about X in degrees (default 25)")
    ap.add_argument("--start-y", type=float, default=35.0, help="Y angle of frame 0 in degrees (default 35)")
    ap.add_argument("--spin-y", type=float, default=360.0, help="degrees turned about Y over the loop (default 360)")
    ap.add_argument("--spin-x", type=float, default=0.0, help="degrees turned about X over the loop (default 0)")
    ap.add_argument("--spin-z", type=float, default=0.0, help="degrees turned about Z over the loop (default 0)")
    ap.add_argument("--size", type=float, default=1.0, help="cube half-size (default 1.0 -> 2x2x2 cube)")
    ap.add_argument("--distance", type=float, default=6.0, help="camera distance (default 6)")
    ap.add_argument("--fov", type=float, default=60.0, help="vertical field of view in degrees (default 60)")
    ap.add_argument("--near", type=float, default=4.0, help="near plane (default 4)")
    ap.add_argument("--far", type=float, default=10.0, help="far plane (default 10)")
    ap.add_argument("--still-angle", type=float, default=None,
                    help="Y angle for matrix.mem (single still picture); default = frame 0")
    ap.add_argument("--preview", action="store_true", help="also write preview PNGs and preview.gif (needs Pillow)")
    args = ap.parse_args()

    if args.frames < 1:
        sys.exit("--frames must be at least 1")
    os.makedirs(args.out, exist_ok=True)

    # ---- vertices.mem -------------------------------------------------------
    tris = build_cube(1.0)          # unit cube; --size is applied in the matrix
    if not check_winding(tris):
        sys.exit("internal error: cube winding is not counter-clockwise")
    lines = []
    for a, b, c, col in tris:
        for p in (a, b, c):
            lines.append(hex32(to_q24_8(p[0])) + hex32(to_q24_8(p[1])) +
                         hex32(to_q24_8(p[2])) + hex32(to_q24_8(1.0)) + f"{col:04x}")
    assert len(lines) == NUM_VERTICES
    lines += ["0" * 36] * (VERTEX_DEPTH - len(lines))
    with open(os.path.join(args.out, "vertices.mem"), "w") as fh:
        fh.write("\n".join(lines) + "\n")

    # ---- recip_lut_q1_17.mem ------------------------------------------------
    with open(os.path.join(args.out, "recip_lut_q1_17.mem"), "w") as fh:
        for k in range(1024):
            fh.write(f"{round(2**17 / (1 + (k + 0.5) / 1024)):05x}\n")

    # ---- all frames -> matrices.mem ----------------------------------------
    report, all_words, bad = [], [], 0
    previews = []
    for f in range(args.frames):
        m, (ax, ay, az) = frame_matrix(args, f)
        mq = quantise(m)
        ok, msg, zmin, zmax = check_frame(mq, tris)
        if not ok:
            bad += 1
        report.append(f"frame {f:4d}  rotX {ax:7.2f}  rotY {ay:7.2f}  rotZ {az:7.2f}  "
                      f"depth [{zmin:+.3f}, {zmax:+.3f}]  {msg}")
        all_words += matrix_words(m)
        if args.preview:
            previews.append(render_preview(mq, tris))

    with open(os.path.join(args.out, "matrices.mem"), "w") as fh:
        fh.write("\n".join(all_words) + "\n")

    # ---- matrix.mem (one still picture, current hardware) ------------------
    if args.still_angle is None:
        still, _ = frame_matrix(args, 0)
    else:
        saved = args.start_y
        args.start_y = args.still_angle
        still, _ = frame_matrix(args, 0)
        args.start_y = saved
    ok, msg, _, _ = check_frame(quantise(still), tris)
    if not ok:
        sys.exit(f"still picture is not valid: {msg}")
    with open(os.path.join(args.out, "matrix.mem"), "w") as fh:
        fh.write("\n".join(matrix_words(still)) + "\n")

    # ---- report --------------------------------------------------------------
    words = 16 * args.frames
    header = [
        "Rotating cube memory files",
        f"frames: {args.frames}   matrices.mem: {words} words x 32 bit = {words * 32} bits "
        f"(~{math.ceil(words * 32 / 36864)} block RAM(s) if stored in BRAM)",
        f"tilt X {args.tilt} deg, start Y {args.start_y} deg, spin X/Y/Z {args.spin_x}/{args.spin_y}/{args.spin_z} deg per loop",
        f"camera: distance {args.distance}, fov {args.fov} deg, near {args.near}, far {args.far}, cube half-size {args.size}",
        "matrix word order per frame: M33 first ... M00 last (transform_engine reads M00 from word 15)",
        f"frames failing checks: {bad}",
        "",
    ]
    with open(os.path.join(args.out, "frames_info.txt"), "w") as fh:
        fh.write("\n".join(header + report) + "\n")

    # ---- previews ------------------------------------------------------------
    if args.preview:
        try:
            from PIL import Image
        except ImportError:
            print("Pillow not installed: skipping previews (pip install pillow)")
        else:
            pdir = os.path.join(args.out, "preview")
            os.makedirs(pdir, exist_ok=True)
            imgs = []
            for f, fb in enumerate(previews):
                im = Image.new("RGB", (SCREEN_W, SCREEN_H))
                im.putdata(fb)
                im.save(os.path.join(pdir, f"frame_{f:03d}.png"))
                imgs.append(im)
            imgs[0].save(os.path.join(pdir, "preview.gif"), save_all=True,
                         append_images=imgs[1:], duration=33, loop=0)

    print("\n".join(header[:6]))
    print(f"written to: {os.path.abspath(args.out)}")
    if bad:
        print("WARNING: some frames fail the checks; see frames_info.txt "
              "(move the camera back with --distance or widen --near/--far).")
        sys.exit(1)


if __name__ == "__main__":
    main()

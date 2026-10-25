#!/usr/bin/env python3
"""
image_to_mem.py - turn a picture (PNG/JPG) into memory files for hardware_pipeline.

The pipeline draws flat-coloured triangles, so the picture is converted into a
"low-poly" mosaic:
  1. points are placed on the picture, densely where there is detail (edges)
     and sparsely in flat areas, plus points along the border
  2. the points are joined into triangles (Delaunay triangulation)
  3. every triangle gets the average colour of the pixels it covers
  4. every triangle becomes 3 vertices in vertices.mem (144-bit words)

Outputs (in --out folder):
  vertices.mem        {X, Y, Z, W, RGB565} per vertex, Q24.8, 36 hex digits per line
  matrix.mem          identity matrix (16 lines, Q24.8)
  expected_frame.ppm  300x400 picture the pipeline should produce (RGB444, same
                      format as frame_capture.ppm from tb_pipeline_memfiles)
  preview.png         the same picture, 2x enlarged, for a quick look
  scene_info.txt      NUM_VERTICES / VERTEX_DEPTH to set in hardware_pipeline,
                      BRAM use and render-time estimate

Coordinate mapping (identity matrix, W = 1):
  projection_engine: x_screen = (X + 1) * 150,  y_screen = (1 - Y) * 200
  so X = x_screen/150 - 1, Y = 1 - y_screen/200   (X, Y in Q24.8)
  Every vertex is snapped to the Q24.8 grid BEFORE triangulating, so the
  triangles in the files are exactly the triangles the hardware sees.

expected_frame.ppm uses the rasterizer's own rules (pixel centres, edge test
E >= 0 with ties inside, edge constant rounded half away from zero, bounding
box floor/ceil clamped to the screen, first triangle wins on equal depth), so
it should match the simulated frame_capture.ppm pixel for pixel.

Usage:
  python3 image_to_mem.py face.png                      # ~1200 triangles
  python3 image_to_mem.py face.png --triangles 600 --drop-bg --out face_600
  python3 image_to_mem.py face.png --compare frame_capture.ppm   # check a simulation result
Needs: numpy, scipy, pillow   (pip install numpy scipy pillow)
"""
import argparse
import math
import os

import numpy as np
from PIL import Image, ImageFilter
from scipy.spatial import Delaunay

SCR_W, SCR_H = 300, 400          # framebuffer size
ONE = 256                        # 1.0 in Q24.8
MASK32 = 0xFFFFFFFF


# ----------------------------------------------------------------------------
# fixed-point helpers
# ----------------------------------------------------------------------------
def x_to_q(xs):
    """screen x (pixels, float) -> nearest Q24.8 NDC X"""
    return int(round(xs * ONE / 150.0 - ONE))


def y_to_q(ys):
    """screen y (pixels, float) -> nearest Q24.8 NDC Y"""
    return int(round(ONE - ys * ONE / 200.0))


def q_to_screen_raw(X, Y):
    """Q24.8 NDC -> exact screen position in Q24.8 raw units (as the hardware)"""
    return (X + ONE) * 150, (ONE - Y) * 200


def rgb_to_565(r, g, b):
    """
    The VGA output keeps only the top 4 bits of each RGB565 channel, so pick the
    nearest 4-bit level and repeat its top bits into the low bits.
    """
    r4, g4, b4 = (int(round(c * 15 / 255.0)) for c in (r, g, b))
    r5 = (r4 << 1) | (r4 >> 3)
    g6 = (g4 << 2) | (g4 >> 2)
    b5 = (b4 << 1) | (b4 >> 3)
    return (r5 << 11) | (g6 << 5) | b5


def c565_to_444(c):
    return ((c >> 12) & 0xF, (c >> 7) & 0xF, (c >> 1) & 0xF)


def pack_vertex(X, Y, Z, W, col):
    X, Y, Z, W, col = (int(v) for v in (X, Y, Z, W, col))   # Python ints: no 64-bit overflow
    return (((X & MASK32) << 112) | ((Y & MASK32) << 80) |
            ((Z & MASK32) << 48) | ((W & MASK32) << 16) | (col & 0xFFFF))


def round_half_away(num, den):
    """round(num/den), halves away from zero (rasterizer conv_c, C_ROUND = 1)"""
    q = (abs(num) + den // 2) // den
    return q if num >= 0 else -q


# ----------------------------------------------------------------------------
# reference rasterizer (same rules as triangle_setup + rasterizer)
# ----------------------------------------------------------------------------
def raster_triangle(v):
    """
    v: 3 screen positions in Q24.8 raw units [(x, y), ...]
    returns (xs, ys) arrays of covered pixels, or None if degenerate
    """
    (x0, y0), (x1, y1), (x2, y2) = v
    d = (x1 - x0) * (y2 - y0) - (x2 - x0) * (y1 - y0)
    if d == 0:
        return None
    s = -1 if d < 0 else 1                      # orient_neg flips all edges
    edges = []
    for (xa, ya), (xb, yb) in (((x0, y0), (x1, y1)), ((x1, y1), (x2, y2)),
                               ((x2, y2), (x0, y0))):
        A = s * (ya - yb)                       # Q24.8
        B = s * (xb - xa)                       # Q24.8
        C16 = s * (xa * yb - xb * ya)           # Q18.16
        C8 = round_half_away(C16, ONE)          # -> Q24.8 like conv_c
        edges.append((A, B, C8))

    xs_raw = (x0, x1, x2)
    ys_raw = (y0, y1, y2)
    xmin = min(max(min(xs_raw) // ONE, 0), SCR_W - 1)
    xmax = min(max(-((-max(xs_raw)) // ONE), 0), SCR_W - 1)
    ymin = min(max(min(ys_raw) // ONE, 0), SCR_H - 1)
    ymax = min(max(-((-max(ys_raw)) // ONE), 0), SCR_H - 1)

    px, py = np.meshgrid(np.arange(xmin, xmax + 1, dtype=np.int64),
                         np.arange(ymin, ymax + 1, dtype=np.int64))
    inside = np.ones(px.shape, dtype=bool)
    for A, B, C8 in edges:
        # doubled scale, exactly as the rasterizer accumulators: 2E at the centre
        e2 = A * (2 * px + 1) + B * (2 * py + 1) + 2 * C8
        inside &= e2 >= 0
    return px[inside], py[inside], (xmax - xmin + 1) * (ymax - ymin + 1)


def render(tris_raw):
    """triangle-id buffer, first triangle wins (all depths equal)"""
    ids = np.full((SCR_H, SCR_W), -1, dtype=np.int32)
    scanned = 0
    for t, v in enumerate(tris_raw):
        r = raster_triangle(v)
        if r is None:
            continue
        xs, ys, n_scan = r
        scanned += n_scan
        free = ids[ys, xs] == -1
        ids[ys[free], xs[free]] = t
    return ids, scanned


# ----------------------------------------------------------------------------
# point placement
# ----------------------------------------------------------------------------
def place_points(img, region, n_points, border_step, rng):
    """edge-weighted points with a minimum spacing, plus border points"""
    ox, oy, w, h = region
    gray = np.asarray(img.convert("L").filter(ImageFilter.GaussianBlur(1.2)),
                      dtype=np.float64)
    gy, gx = np.gradient(gray)
    weight = np.hypot(gx, gy) ** 1.2 + 2.0      # +2: some points in flat areas too
    prob = (weight / weight.sum()).ravel()

    # minimum spacing so points do not clump on strong edges
    min_d = 0.6 * math.sqrt(w * h / max(n_points, 1))
    cell = min_d / math.sqrt(2)
    grid = {}
    pts = []

    def ok(p):
        cx, cy = int(p[0] // cell), int(p[1] // cell)
        for i in range(cx - 2, cx + 3):
            for j in range(cy - 2, cy + 3):
                q = grid.get((i, j))
                if q is not None and (q[0] - p[0]) ** 2 + (q[1] - p[1]) ** 2 < min_d ** 2:
                    return False
        return True

    def add(p):
        grid[(int(p[0] // cell), int(p[1] // cell))] = p
        pts.append(p)

    # border first (keeps the picture rectangle filled)
    nx = max(2, int(round(w / border_step)))
    ny = max(2, int(round(h / border_step)))
    for i in range(nx + 1):
        for yy in (0.0, float(h)):
            add((i * w / nx, yy))
    for j in range(1, ny):
        for xx in (0.0, float(w)):
            add((xx, j * h / ny))

    tries = 0
    while len(pts) < n_points and tries < 60:
        idx = rng.choice(prob.size, size=4 * n_points, p=prob)
        for k in idx:
            p = (k % w + rng.random(), k // w + rng.random())
            if ok(p):
                add(p)
                if len(pts) >= n_points:
                    break
        tries += 1
    return [(ox + x, oy + y) for x, y in pts]


# ----------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("image")
    ap.add_argument("--triangles", type=int, default=1200,
                    help="approximate number of triangles (default 1200)")
    ap.add_argument("--out", default="mem_out", help="output folder")
    ap.add_argument("--seed", type=int, default=1, help="random seed (same seed = same files)")
    ap.add_argument("--winding", choices=["ccw", "cw"], default="ccw",
                    help="winding in vertices.mem: ccw for CULL_MODE=1, cw for CULL_MODE=2 "
                         "(either works with CULL_MODE=0)")
    ap.add_argument("--drop-bg", action="store_true",
                    help="drop triangles that match the picture's background colour (drawn black)")
    ap.add_argument("--bg-tol", type=float, default=18.0,
                    help="colour distance (0..441) for --drop-bg (default 18)")
    ap.add_argument("--max-size", type=int, default=SCR_W,
                    help="largest side of the picture on screen in pixels (default 300)")
    ap.add_argument("--compare", metavar="FRAME_PPM",
                    help="after generating, compare a simulated frame_capture.ppm with "
                         "expected_frame.ppm and write diff.png (use the same options/seed)")
    args = ap.parse_args()
    rng = np.random.default_rng(args.seed)
    os.makedirs(args.out, exist_ok=True)

    # ---- load, flatten transparency, fit into 300x400 keeping the aspect ----
    src = Image.open(args.image)
    if src.mode in ("RGBA", "LA", "P"):
        src = src.convert("RGBA")
        bg = Image.new("RGBA", src.size, (255, 255, 255, 255))
        src = Image.alpha_composite(bg, src)
    src = src.convert("RGB")
    scale = min(args.max_size / src.width, SCR_W / src.width, SCR_H / src.height)
    w = max(1, int(round(src.width * scale)))
    h = max(1, int(round(src.height * scale)))
    ox, oy = (SCR_W - w) / 2.0, (SCR_H - h) / 2.0
    img = src.resize((w, h), Image.LANCZOS)

    canvas = np.zeros((SCR_H, SCR_W, 3), dtype=np.float64)
    canvas[int(oy):int(oy) + h, int(ox):int(ox) + w] = np.asarray(img, dtype=np.float64)

    # background colour = median of the picture's border pixels
    a = np.asarray(img, dtype=np.float64)
    border = np.concatenate([a[0], a[-1], a[:, 0], a[:, -1]])
    bg_col = np.median(border, axis=0)

    # ---- points -> snapped to the Q24.8 grid -> unique ----
    n_pts = max(8, args.triangles // 2 + 2)
    pts = place_points(img, (ox, oy, w, h), n_pts, border_step=max(6.0, (w + h) / 40), rng=rng)
    q = sorted({(x_to_q(min(max(x, 0.0), SCR_W)), y_to_q(min(max(y, 0.0), SCR_H))) for x, y in pts})
    q = np.array(q, dtype=np.int64)
    scr = np.array([q_to_screen_raw(X, Y) for X, Y in q], dtype=np.float64) / ONE

    tri = Delaunay(scr).simplices

    # ---- consistent winding (in NDC, y up) and drop zero-area triangles ----
    tris_q = []
    for i, j, k in tri:
        (X0, Y0), (X1, Y1), (X2, Y2) = q[i], q[j], q[k]
        area2 = (X1 - X0) * (Y2 - Y0) - (X2 - X0) * (Y1 - Y0)
        if area2 == 0:
            continue
        if (area2 < 0) == (args.winding == "ccw"):
            j, k = k, j
        tris_q.append((tuple(q[i]), tuple(q[j]), tuple(q[k])))

    # ---- colour every triangle with the mean colour it covers ----
    tris_raw = [[q_to_screen_raw(X, Y) for X, Y in t] for t in tris_q]
    ids, _ = render(tris_raw)
    flat_ids = ids.ravel()
    sel = flat_ids >= 0
    n = len(tris_q)
    cnt = np.bincount(flat_ids[sel], minlength=n).astype(np.float64)
    sums = np.stack([np.bincount(flat_ids[sel], weights=canvas.reshape(-1, 3)[sel, c], minlength=n)
                     for c in range(3)], axis=1)
    colours = []
    keep = []
    for t in range(n):
        if cnt[t] > 0:
            col = sums[t] / cnt[t]
        else:                                    # tiny triangle: colour at its centroid
            cx = sum(p[0] for p in tris_raw[t]) / (3.0 * ONE)
            cy = sum(p[1] for p in tris_raw[t]) / (3.0 * ONE)
            col = canvas[min(int(cy), SCR_H - 1), min(int(cx), SCR_W - 1)]
        if args.drop_bg and np.linalg.norm(col - bg_col) < args.bg_tol:
            continue
        keep.append(t)
        colours.append(rgb_to_565(*col))

    tris_q = [tris_q[t] for t in keep]
    tris_raw = [tris_raw[t] for t in keep]

    # ---- the picture the pipeline should produce ----
    ids, scanned = render(tris_raw)
    out444 = np.zeros((SCR_H, SCR_W, 3), dtype=np.uint8)
    lut = np.array([c565_to_444(c) for c in colours], dtype=np.uint8).reshape(-1, 3)
    m = ids >= 0
    out444[m] = lut[ids[m]]

    with open(os.path.join(args.out, "expected_frame.ppm"), "w") as f:
        f.write(f"P3\n{SCR_W} {SCR_H}\n15\n")
        for r, g, b in out444.reshape(-1, 3):
            f.write(f"{r} {g} {b}\n")
    Image.fromarray(out444 * 17).resize((SCR_W * 2, SCR_H * 2), Image.NEAREST) \
         .save(os.path.join(args.out, "preview.png"))

    # ---- memory files ----
    n_vtx = 3 * len(tris_q)
    depth = 1 << max(6, (n_vtx - 1).bit_length())          # power of two, >= 64
    with open(os.path.join(args.out, "vertices.mem"), "w") as f:
        f.write(f"// vertices.mem - {len(tris_q)} triangles = {n_vtx} vertices from "
                f"{os.path.basename(args.image)}\n")
        f.write("// word = {X, Y, Z, W, RGB565}, Q24.8, identity matrix, z = 0, w = 1, "
                f"winding {args.winding.upper()}\n")
        for t, (tq, col) in enumerate(zip(tris_q, colours)):
            for k, (X, Y) in enumerate(tq):
                note = f"  // tri {t} v{k}" + (f" col {col:04X}" if k == 0 else "")
                f.write(f"{pack_vertex(X, Y, 0, ONE, col):036X}{note}\n")
        for _ in range(depth - n_vtx):                      # pad: no readmemh warnings
            f.write("0" * 36 + "\n")

    with open(os.path.join(args.out, "matrix.mem"), "w") as f:
        f.write("// matrix.mem - identity, Q24.8 (the same in row- and reverse order)\n")
        for k in range(16):
            f.write(f"{(ONE if k % 5 == 0 else 0):08X}\n")

    # ---- summary ----
    bram36 = 2 * math.ceil(depth / 512)                     # 144 bits = 2 x 72-bit BRAM36
    render_cycles = scanned + 6 * len(tris_q)               # 1 position/clock + per-tri overhead
    info = (
        f"source             : {args.image}  ({src.width}x{src.height}) -> {w}x{h} on screen at "
        f"({ox:.0f},{oy:.0f})\n"
        f"triangles          : {len(tris_q)}\n"
        f"NUM_VERTICES       : {n_vtx}      (hardware_pipeline parameter)\n"
        f"VERTEX_DEPTH       : {depth}      (hardware_pipeline parameter, vertices.mem padded to this)\n"
        f"vertex RAM         : 144 x {depth} bits = {144 * depth / 1024:.0f} Kbit, about {bram36} BRAM36\n"
        f"pixels covered     : {int(m.sum())} of {SCR_W * SCR_H}\n"
        f"positions scanned  : {scanned}  (sum of bounding boxes)\n"
        f"render estimate    : ~{render_cycles} cycles = {render_cycles / 1e5:.2f} ms at 100 MHz "
        f"(+ 1.2 ms z-buffer clear)\n"
        f"winding            : {args.winding.upper()} in NDC  ->  "
        f"{'CULL_MODE 1 or 0' if args.winding == 'ccw' else 'CULL_MODE 2 or 0'}\n"
    )
    if depth > 4096:
        info += ("WARNING            : the z-buffer + framebuffer already use ~108 of the 135 BRAM36 "
                 "on the XC7A100T;\n                     VERTEX_DEPTH > 4096 (> 1365 triangles) "
                 "will not fit. Use fewer --triangles.\n")
    with open(os.path.join(args.out, "scene_info.txt"), "w") as f:
        f.write(info)
    print(info, end="")
    print(f"files written to {args.out}/")

    # ---- optional: compare a simulated frame with the expected one ----
    if args.compare:
        got = read_ppm(args.compare)
        diff = (got != out444).any(axis=2)
        n_bad = int(diff.sum())
        vis = (out444 * 17 // 3).astype(np.uint8)            # dimmed expected picture
        vis[diff] = (255, 0, 255)                             # mismatches in magenta
        Image.fromarray(vis).resize((SCR_W * 2, SCR_H * 2), Image.NEAREST) \
             .save(os.path.join(args.out, "diff.png"))
        print(f"compare {args.compare}: {n_bad} of {SCR_W * SCR_H} pixels differ "
              f"({'MATCH' if n_bad == 0 else 'see diff.png, mismatches in magenta'})")


def read_ppm(path):
    """P3 300x400 file as written by tb_pipeline_memfiles (max value 15)"""
    tok = open(path).read().split()
    assert tok[0] == "P3" and int(tok[1]) == SCR_W and int(tok[2]) == SCR_H, "expected a P3 300x400 PPM"
    return np.array(tok[4:4 + 3 * SCR_W * SCR_H], dtype=np.int64).reshape(SCR_H, SCR_W, 3)


if __name__ == "__main__":
    main()

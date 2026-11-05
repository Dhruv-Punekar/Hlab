#!/usr/bin/env python3
"""
gen_tt_scene.py  -  memory files for the table-tennis demo scene
================================================================

Builds the static table-tennis scene for the Nexys A7 pipeline:

    sky (clear colour)                      light blue
    floor (clear colour) + grid lines       dark grey, light grey lines
    table: blue top, white lines, legs      ITTF blue, white, dark grey
    net + posts                             charcoal net, white top tape
    player racket (near), opponent racket   red / black blades, wooden handles
    ball                                    orange cube

Everything is drawn with one camera matrix (matrix.mem).  The sky / floor
split is done by the framebuffer clear (z_f_buffer CLEAR_* parameters), so the
large floor area costs no rasterizer time; only the grid lines are triangles.

Files written to the output folder
----------------------------------
  vertices.mem       144-bit vertices X|Y|Z|W (Q24.8) | RGB565, padded to
                     VERTEX_DEPTH lines. Triangles are wound counter-clockwise
                     seen from their visible side (CULL_MODE = 1).
  matrix.mem         camera (projection x view), 16 x 32-bit Q24.8,
                     hardware word order M33 first ... M00 last.
  tt_scene_map.vh    Verilog localparams: vertex counts, clear colours, horizon
                     row, and the vertex range / centre of every MOVING object
                     (rackets, ball) for the motion step that comes later.
  scene_info.txt     summary: triangle counts, render-time estimate, checks.
  preview.png        software render of the frame (needs Pillow + numpy).

World units: 1 unit = 10 cm (1 dm), so Q24.8 resolves 0.4 mm.
Axes: x to the right, y up, z towards the camera (player end at +z).
Table centre on the floor at x = z = 0.

Moving objects are stored as separate, contiguous vertex ranges (see
tt_scene_map.vh) and placed at their start positions; the motion step can
move them by rewriting those vertices or by giving them their own matrix.

Usage:  python gen_tt_scene.py [--out DIR] [--no-preview]
Only the standard library is needed; numpy + Pillow are used for preview.png.
"""

import argparse
import math
import os
import sys

# ----------------------------------------------------------------------------
# Hardware facts (keep in step with the Verilog)
# ----------------------------------------------------------------------------
SCREEN_W, SCREEN_H = 300, 400
Q = 256                                  # Q24.8
CLIP_MARGIN = 0.996                      # keep vertices strictly inside the screen
CYC_PER_TRI = 10                         # rasterizer overhead per drawn triangle (approx.)
CLEAR_CYCLES = SCREEN_W * SCREEN_H       # z/framebuffer clear, 1 pixel per clock
F_CLK = 100e6

# ----------------------------------------------------------------------------
# Colours (RGB444 -> exact RGB565, z_f_buffer keeps the top 4 bits)
# ----------------------------------------------------------------------------
def rgb444(r, g, b):
    return (r << 12) | (g << 7) | (b << 1)      # RGB565 whose top bits are r, g, b


C = {
    "sky":         (8, 12, 15),
    "floor":       (4, 4, 5),
    "grid":        (8, 8, 9),
    "table_top":   (1, 4, 10),
    "table_side":  (1, 2, 6),
    "line":        (15, 15, 15),
    "leg":         (2, 2, 2),
    "net":         (2, 2, 3),
    "net_strand":  (6, 6, 7),
    "net_tape":    (14, 14, 14),
    "post":        (5, 5, 5),
    "blade_red":   (13, 1, 1),
    "blade_black": (1, 1, 1),
    "blade_edge":  (12, 9, 5),
    "handle":      (10, 6, 3),
    "ball":        (15, 8, 0),
    "ball_shade":  (12, 6, 0),
}

# ----------------------------------------------------------------------------
# Scene dimensions (metres; converted to units below)
# ----------------------------------------------------------------------------
U = 10.0                                  # units per metre
TABLE_L, TABLE_W, TABLE_H = 2.74, 1.525, 0.76
TOP_T = 0.04                              # top thickness
LINE_W = 0.03                             # white lines (real: 2 cm, wider for 300 px)
NET_H, NET_OVER = 0.1525, 0.1525          # net height, overhang past the table side
NET_T = 0.01
NET_STRAND, NET_PITCH = 0.016, 0.15       # mesh look: strand width, strand spacing
POST_W = 0.03
LEG_W = 0.06
LEG_INSET = 0.25                          # legs inset from the table corners

BLADE = 0.17                              # square blade side (slightly enlarged)
BLADE_T = 0.015
HANDLE_L, HANDLE_W = 0.10, 0.03
BALL = 0.07                               # cube edge (real ball 4 cm; enlarged to stay visible)

GRID_STEP = 1.0                           # floor grid cell
FLOOR_X = 9.0                             # grid half-width
FLOOR_FAR = -9.0                          # far edge of the floor (z)
GRID_PX = 1.6                             # target on-screen thickness of grid lines

# start positions of the moving objects (metres)
RACKET_PLAYER = (0.30, 0.95, 1.55)        # near end, in front of the table
RACKET_OPP = (-0.25, 1.00, -1.60)         # far end
BALL_POS = (0.10, 1.05, -0.55)            # in the air over the opponent half

# camera
CAM_POS = (0.0, 2.05, 3.55)
CAM_LOOK = (0.0, 0.45, -0.6)
FOV_Y = 52.0
NEAR, FAR = 1.0, 16.0


# ----------------------------------------------------------------------------
# Small vector / matrix helpers
# ----------------------------------------------------------------------------
def sub(a, b): return [a[i] - b[i] for i in range(3)]
def add(a, b): return [a[i] + b[i] for i in range(3)]
def mul(a, s): return [a[i] * s for i in range(3)]
def dot(a, b): return sum(a[i] * b[i] for i in range(3))
def cross(a, b): return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
def norm(a):
    n = math.sqrt(dot(a, a))
    return [x / n for x in a]


def mat_mul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def mat_vec(m, p):
    v = [p[0], p[1], p[2], 1.0]
    return [sum(m[r][k] * v[k] for k in range(4)) for r in range(4)]


def perspective(fov_y_deg, aspect, near, far):
    f = 1.0 / math.tan(math.radians(fov_y_deg) / 2)
    return [[f / aspect, 0, 0, 0], [0, f, 0, 0],
            [0, 0, (far + near) / (near - far), 2 * far * near / (near - far)],
            [0, 0, -1, 0]]


def look_at(eye, target, up=(0, 1, 0)):
    f = norm(sub(target, eye))
    s = norm(cross(f, list(up)))
    u = cross(s, f)
    return [[s[0], s[1], s[2], -dot(s, eye)],
            [u[0], u[1], u[2], -dot(u, eye)],
            [-f[0], -f[1], -f[2], dot(f, eye)],
            [0, 0, 0, 1]]


def to_q(v):
    raw = int(round(v * Q))
    if not -2**31 <= raw < 2**31:
        raise ValueError(f"{v} does not fit Q24.8")
    return raw


def hex32(raw):
    return f"{raw & 0xFFFFFFFF:08x}"


def quantise_mat(m):
    return [[to_q(v) / Q for v in row] for row in m]


def matrix_words(m):
    return [hex32(to_q(m[r][c])) for r in range(4) for c in range(4)][::-1]


def m2u(p):
    return [p[0] * U, p[1] * U, p[2] * U]


# ----------------------------------------------------------------------------
# Geometry builders (all in units). A triangle = (p0, p1, p2, rgb565).
# ----------------------------------------------------------------------------
def quad(a, b, c, d, col, facing):
    """Quad a-b-c-d (any order around the edge), wound CCW seen from 'facing'
    (a direction pointing to the visible side)."""
    n = cross(sub(b, a), sub(c, a))
    if dot(n, facing) < 0:
        a, b, c, d = a, d, c, b
    return [(a, b, c, col), (a, c, d, col)]


def box(lo, hi, cols):
    """Axis-aligned box. cols: dict face -> colour, faces '+x','-x','+y','-y','+z','-z'.
    Faces missing from cols are not emitted (never visible)."""
    x0, y0, z0 = lo
    x1, y1, z1 = hi
    P = lambda x, y, z: [x, y, z]
    faces = {
        "+x": ([P(x1, y0, z0), P(x1, y1, z0), P(x1, y1, z1), P(x1, y0, z1)], [1, 0, 0]),
        "-x": ([P(x0, y0, z0), P(x0, y0, z1), P(x0, y1, z1), P(x0, y1, z0)], [-1, 0, 0]),
        "+y": ([P(x0, y1, z0), P(x0, y1, z1), P(x1, y1, z1), P(x1, y1, z0)], [0, 1, 0]),
        "-y": ([P(x0, y0, z0), P(x1, y0, z0), P(x1, y0, z1), P(x0, y0, z1)], [0, -1, 0]),
        "+z": ([P(x0, y0, z1), P(x1, y0, z1), P(x1, y1, z1), P(x0, y1, z1)], [0, 0, 1]),
        "-z": ([P(x0, y0, z0), P(x0, y1, z0), P(x1, y1, z0), P(x1, y0, z0)], [0, 0, -1]),
    }
    tris = []
    for name, col in cols.items():
        (a, b, c, d), n = faces[name]
        tris += quad(a, b, c, d, col, n)
    return tris


def box_c(center, size, cols):
    lo = [center[i] - size[i] / 2 for i in range(3)]
    hi = [center[i] + size[i] / 2 for i in range(3)]
    return box(lo, hi, cols)


def all_faces(col_side, **over):
    d = {k: col_side for k in ("+x", "-x", "+y", "-y", "+z", "-z")}
    d.update({k.replace("p", "+").replace("m", "-"): v for k, v in over.items()})
    return d


# ----------------------------------------------------------------------------
# Scene objects
# ----------------------------------------------------------------------------
def table():
    """Top surface split into blue panels and white lines (no overlap, so no
    z-fighting), side faces, legs."""
    col = {k: rgb444(*v) for k, v in C.items()}
    L, W, H, T, LW = TABLE_L * U, TABLE_W * U, TABLE_H * U, TOP_T * U, LINE_W * U
    x0, x1, z0, z1 = -W / 2, W / 2, -L / 2, L / 2
    y = H
    up = [0, 1, 0]
    tris = []
    rect = lambda xa, xb, za, zb, c: quad([xa, y, za], [xb, y, za], [xb, y, zb], [xa, y, zb], c, up)
    # white border (sides full length, ends between the side lines)
    tris += rect(x0, x0 + LW, z0, z1, col["line"])
    tris += rect(x1 - LW, x1, z0, z1, col["line"])
    tris += rect(x0 + LW, x1 - LW, z0, z0 + LW, col["line"])
    tris += rect(x0 + LW, x1 - LW, z1 - LW, z1, col["line"])
    # centre line (half width) along z
    cl = LW / 2
    tris += rect(-cl, cl, z0 + LW, z1 - LW, col["line"])
    # blue panels, split at the net (z = 0) so they stay small
    for (xa, xb) in ((x0 + LW, -cl), (cl, x1 - LW)):
        for (za, zb) in ((z0 + LW, 0.0), (0.0, z1 - LW)):
            tris += rect(xa, xb, za, zb, col["table_top"])
    # top slab sides (front end towards the camera, the two long sides)
    tris += box([x0, y - T, z0], [x1, y, z1],
                {"+z": col["table_side"], "+x": col["table_side"], "-x": col["table_side"],
                 "-z": col["table_side"], "-y": col["leg"]})
    # legs
    lw, li = LEG_W * U, LEG_INSET * U
    for sx in (-1, 1):
        for sz in (-1, 1):
            cx = sx * (W / 2 - li)
            cz = sz * (L / 2 - li)
            tris += box([cx - lw / 2, 0, cz - lw / 2], [cx + lw / 2, y - T, cz + lw / 2],
                        {"+z": col["leg"], "+x": col["leg"], "-x": col["leg"]})
    return tris


def net():
    col = {k: rgb444(*v) for k, v in C.items()}
    W, H = TABLE_W * U, TABLE_H * U
    half = W / 2 + NET_OVER * U
    nh, nt, tape = NET_H * U, NET_T * U, 0.015 * U
    tris = []
    # net body: front face (+z, towards the camera) built from dark mesh gaps and
    # lighter vertical strands side by side in one plane (no overlap -> no
    # z-fighting); plain back face
    zf, top = nt / 2, H + nh - tape
    sw, pitch = NET_STRAND * U, NET_PITCH * U
    x = -half
    k = 0
    while x < half - 1e-6:
        w = sw if k % 2 else pitch - sw
        xe = min(half, x + w)
        tris += quad([x, H, zf], [xe, H, zf], [xe, top, zf], [x, top, zf],
                     col["net_strand"] if k % 2 else col["net"], [0, 0, 1])
        x, k = xe, k + 1
    tris += quad([-half, H, -zf], [half, H, -zf], [half, top, -zf], [-half, top, -zf], col["net"], [0, 0, -1])
    tris += box([-half, H + nh - tape, -nt / 2], [half, H + nh, nt / 2],
                {"+z": col["net_tape"], "+y": col["net_tape"], "-z": col["net_tape"]})
    # posts
    pw = POST_W * U
    for sx in (-1, 1):
        cx = sx * (half + pw / 2)
        tris += box([cx - pw / 2, H - 0.05 * U, -pw / 2], [cx + pw / 2, H + nh, pw / 2],
                    {"+z": col["post"], "+y": col["post"], "+x": col["post"], "-x": col["post"]})
    return tris


def racket(pos_m, blade_col):
    """Square blade facing +z (towards the camera) with a handle below."""
    col = {k: rgb444(*v) for k, v in C.items()}
    c = m2u(pos_m)
    b, t = BLADE * U, BLADE_T * U
    hl, hw = HANDLE_L * U, HANDLE_W * U
    tris = box_c(c, [b, b, t], {"+z": blade_col, "-z": blade_col, "+x": col["blade_edge"],
                                "-x": col["blade_edge"], "+y": col["blade_edge"], "-y": col["blade_edge"]})
    hc = [c[0], c[1] - b / 2 - hl / 2, c[2]]
    tris += box_c(hc, [hw, hl, hw * 0.8], {"+z": col["handle"], "-z": col["handle"],
                                           "+x": col["handle"], "-x": col["handle"]})
    return tris, c


def ball(pos_m):
    col = {k: rgb444(*v) for k, v in C.items()}
    c = m2u(pos_m)
    s = BALL * U
    return box_c(c, [s, s, s], {"+z": col["ball"], "+y": col["ball"], "+x": col["ball_shade"],
                                "-x": col["ball_shade"], "-z": col["ball"], "-y": col["ball_shade"]}), c


# ----------------------------------------------------------------------------
# Clipping (no clipper in hardware: every vertex must land on screen)
# ----------------------------------------------------------------------------
def clip_planes(m):
    """Linear functions of the world point that must be >= 0 (in clip space)."""
    r = m
    k = CLIP_MARGIN
    planes = []
    for i in (0, 1):                                    # x and y: -k*w <= x <= k*w
        planes.append([k * r[3][j] - r[i][j] for j in range(4)])
        planes.append([k * r[3][j] + r[i][j] for j in range(4)])
    planes.append([r[3][j] + r[2][j] for j in range(4)])  # near: z >= -w
    planes.append([r[3][j] - r[2][j] for j in range(4)])  # far:  z <=  w
    return planes


def clip_tri(tri, planes):
    a, b, c, col = tri
    poly = [a, b, c]
    for pl in planes:
        f = lambda p: pl[0] * p[0] + pl[1] * p[1] + pl[2] * p[2] + pl[3]
        out = []
        n = len(poly)
        for i in range(n):
            p, q = poly[i], poly[(i + 1) % n]
            fp, fq = f(p), f(q)
            if fp >= 0:
                out.append(p)
            if (fp >= 0) != (fq >= 0):
                t = fp / (fp - fq)
                out.append(add(p, mul(sub(q, p), t)))
        poly = out
        if len(poly) < 3:
            return []
    return [(poly[0], poly[i], poly[i + 1], col) for i in range(1, len(poly) - 1)]


# ----------------------------------------------------------------------------
# Floor grid (thickness adapted to distance so far lines stay ~GRID_PX wide)
# ----------------------------------------------------------------------------
def screen_xy(m, p):
    x, y, z, w = mat_vec(m, p)
    return (x / w + 1) * SCREEN_W / 2, (1 - y / w) * SCREEN_H / 2, w


def floor_grid(m, cam_z=None, far_z=None, half_x=None):
    """Grid in the view's local frame (camera on +z looking to -z); m is the
    local view matrix. Defaults = the original single camera."""
    cam_z = CAM_POS[2] if cam_z is None else cam_z
    far_z = FLOOR_FAR if far_z is None else far_z
    col = rgb444(*C["grid"])
    up = [0, 1, 0]
    tris = []
    half_x = FLOOR_X if half_x is None else half_x
    X = half_x * U
    step = GRID_STEP * U
    zs = []
    z = math.floor(cam_z * U / step) * step
    while z >= far_z * U:
        zs.append(z)
        z -= step
    # lines across (constant z): thickness from the screen-space height of the line
    for z in zs:
        y_a = screen_xy(m, [0, 0, z])[1]
        y_b = screen_xy(m, [0, 0, z - 1.0])[1]          # px per unit of z here
        px_per_unit = abs(y_a - y_b)
        w = GRID_PX / max(px_per_unit, 1e-6)
        w = min(w, 0.25 * step)
        tris += quad([-X, 0, z + w / 2], [X, 0, z + w / 2], [X, 0, z - w / 2], [-X, 0, z - w / 2], col, up)
    # lines along z (constant x), split per cell, width grows with distance
    xs = [i * step for i in range(-int(half_x / GRID_STEP), int(half_x / GRID_STEP) + 1)]
    for x in xs:
        for za, zb in zip(zs[:-1], zs[1:]):
            wa = GRID_PX / max(abs(screen_xy(m, [x + 0.5, 0, za])[0] - screen_xy(m, [x - 0.5, 0, za])[0]), 1e-6)
            wb = GRID_PX / max(abs(screen_xy(m, [x + 0.5, 0, zb])[0] - screen_xy(m, [x - 0.5, 0, zb])[0]), 1e-6)
            tris += quad([x - wa / 2, 0, za], [x + wa / 2, 0, za], [x + wb / 2, 0, zb], [x - wb / 2, 0, zb], col, up)
    return tris


# ----------------------------------------------------------------------------
# Software model of the pipeline (for checks, cost and preview)
# ----------------------------------------------------------------------------
def to_screen(mq, p):
    pq = [to_q(v) / Q for v in p]
    x, y, z, w = mat_vec(mq, pq)
    return (x / w + 1) * SCREEN_W / 2, (1 - y / w) * SCREEN_H / 2, z / w, w


def analyse(mq, tris):
    """Per-triangle screen data, culling (CULL_MODE 1) and bbox cost."""
    out, cost, culled, bad = [], 0, 0, []
    for i, (a, b, c, col) in enumerate(tris):
        pts = [to_screen(mq, p) for p in (a, b, c)]
        for (sx, sy, nz, w) in pts:
            if w <= 0 or not (0 <= sx <= SCREEN_W and 0 <= sy <= SCREEN_H) or abs(nz) > 1.0:
                bad.append(i)
        (x0, y0, _, _), (x1, y1, _, _), (x2, y2, _, _) = pts
        D = (x1 - x0) * (y2 - y0) - (x2 - x0) * (y1 - y0)
        if D >= 0:
            culled += 1
            out.append(None)
            continue
        bw = min(299, math.ceil(max(x0, x1, x2))) - max(0, math.floor(min(x0, x1, x2))) + 1
        bh = min(399, math.ceil(max(y0, y1, y2))) - max(0, math.floor(min(y0, y1, y2))) + 1
        cost += bw * bh + CYC_PER_TRI
        out.append(pts)
    return out, cost, culled, bad


def render_preview(scr, tris, horizon_y, path):
    import numpy as np
    from PIL import Image
    W, H = SCREEN_W, SCREEN_H
    sky = np.array(C["sky"]) * 17
    flo = np.array(C["floor"]) * 17
    img = np.zeros((H, W, 3), dtype=np.uint8)
    img[:horizon_y] = sky
    img[horizon_y:] = flo
    zb = np.full((H, W), np.inf)
    ys, xs = np.mgrid[0:H, 0:W]
    cx, cy = xs + 0.5, ys + 0.5
    for pts, (_, _, _, col) in zip(scr, tris):
        if pts is None:
            continue
        (x0, y0, z0, _), (x1, y1, z1, _), (x2, y2, z2, _) = pts
        D = (x1 - x0) * (y2 - y0) - (x2 - x0) * (y1 - y0)
        xa, xb = max(0, math.floor(min(x0, x1, x2))), min(W - 1, math.ceil(max(x0, x1, x2)))
        ya, yb = max(0, math.floor(min(y0, y1, y2))), min(H - 1, math.ceil(max(y0, y1, y2)))
        sx, sy = cx[ya:yb + 1, xa:xb + 1], cy[ya:yb + 1, xa:xb + 1]
        e0 = -((y0 - y1) * sx + (x1 - x0) * sy + (x0 * y1 - x1 * y0))
        e1 = -((y1 - y2) * sx + (x2 - x1) * sy + (x1 * y2 - x2 * y1))
        e2 = -((y2 - y0) * sx + (x0 - x2) * sy + (x2 * y0 - x0 * y2))
        inside = (e0 >= 0) & (e1 >= 0) & (e2 >= 0)
        l1 = ((sx - x0) * (y2 - y0) - (x2 - x0) * (sy - y0)) / D
        l2 = ((x1 - x0) * (sy - y0) - (sx - x0) * (y1 - y0)) / D
        z = z0 + l1 * (z1 - z0) + l2 * (z2 - z0)
        zsub = zb[ya:yb + 1, xa:xb + 1]
        win = inside & (z < zsub)
        zsub[win] = z[win]
        r, g, b = (col >> 12) & 15, (col >> 7) & 15, (col >> 1) & 15
        img[ya:yb + 1, xa:xb + 1][win] = (r * 17, g * 17, b * 17)
    Image.fromarray(img).save(path)


# ----------------------------------------------------------------------------
# Main
# ----------------------------------------------------------------------------
# ----------------------------------------------------------------------------
# Camera views. All views share the pitch and the field of view of the original
# camera, so the horizon (sky / floor split of the background clear) stays on
# the same screen row. Each view is described in a local frame (camera on +z,
# looking towards -z) and turned about the vertical axis by k x 90 degrees.
# The floor grid's far edge is placed so that it projects onto that same row.
# ----------------------------------------------------------------------------
GRID_X_SIDE = 7.0                                                    # side view: narrower grid (vertex budget)
PITCH_T = (CAM_POS[1] - CAM_LOOK[1]) / (CAM_POS[2] - CAM_LOOK[2])   # tan(pitch) of view 0
FAR_D0 = CAM_POS[2] - FLOOR_FAR                                      # floor depth seen by view 0


def view_def(name, k, cam_d, look_z, grid_x=FLOOR_X, side_x=0.0):
    """side_x moves the camera sideways in its own frame (view direction kept)."""
    h = CAM_LOOK[1] + (cam_d - look_z) * PITCH_T
    return dict(name=name, k=k, cam=(side_x, h, cam_d), look=(side_x, CAM_LOOK[1], look_z),
                far_z=cam_d - FAR_D0 * h / CAM_POS[1], grid_x=grid_x)


VIEWS = [
    view_def("player 1 (near end)", 0, CAM_POS[2], CAM_LOOK[2]),   # = the original camera
    view_def("side",                1, 6.8, 0.0, grid_x=GRID_X_SIDE, side_x=-0.45),
    view_def("player 2 (far end)",  2, 4.6, CAM_LOOK[2]),
]


def rot_k(p, k):
    """Local -> world: turn about +y by k x 90 degrees (k=1: local +z -> world +x)."""
    x, y, z = p
    for _ in range(k % 4):
        x, z = z, -x
    return [x, y, z]


def build_views():
    """Per view: quantised matrix, static triangles (clipped to that view,
    back faces removed), horizon row. Shared: the moving objects."""
    col = {k: rgb444(*v) for k, v in C.items()}
    out = []
    for v in VIEWS:
        cam_w, look_w = rot_k(m2u(v["cam"]), v["k"]), rot_k(m2u(v["look"]), v["k"])
        P = perspective(FOV_Y, SCREEN_W / SCREEN_H, NEAR * U, FAR * U)
        M = mat_mul(P, look_at(cam_w, look_w))
        mq = quantise_mat(M)
        ML = mat_mul(P, look_at(m2u(v["cam"]), m2u(v["look"])))       # local frame, for the grid
        grid = [tuple(rot_k(p, v["k"]) for p in t[:3]) + (t[3],) for t in floor_grid(ML, v["cam"][2], v["far_z"], v["grid_x"])]
        planes = clip_planes(mq)
        static_c = [t for tri in table() + net() + grid for t in clip_tri(tri, planes)]
        scr_s, _, _, _ = analyse(mq, static_c)
        static_c = [t for t, s_ in zip(static_c, scr_s) if s_ is not None]
        far_c = rot_k([0, 0, v["far_z"] * U], v["k"])
        horizon = int(round(to_screen(mq, far_c)[1]))
        out.append(dict(v, M=M, mq=mq, static=static_c, horizon=max(0, min(SCREEN_H, horizon))))
    objects = [("RACKET_P", *racket(RACKET_PLAYER, col["blade_red"])),
               ("RACKET_O", *racket(RACKET_OPP, col["blade_black"])),
               ("BALL", *ball(BALL_POS))]
    return out, objects


def to_q16(v):
    raw = to_q(v)
    if not -2**15 <= raw < 2**15:
        raise ValueError(f"{v} does not fit Q8.8 (16 bit)")
    return raw & 0xFFFF


def main():
    ap = argparse.ArgumentParser(description="Generate the table-tennis scene .mem files.")
    ap.add_argument("--out", default="mem_out")
    ap.add_argument("--no-preview", action="store_true")
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)

    views, objects = build_views()
    v0 = views[0]
    for name, tris, _ in objects:
        for v in views:
            if len([t for tri in tris for t in clip_tri(tri, clip_planes(v["mq"]))]) != len(tris):
                sys.exit(f"{name} is (partly) off screen at its start position in view '{v['name']}'")

    # every view's static set is padded to the same length with zero-area
    # triangles (discarded by the setup unit); the moving objects follow
    n_static = 3 * max(len(v["static"]) for v in views)
    pad = ([0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], 0)
    for v in views:
        v["padded"] = v["static"] + [pad] * (n_static // 3 - len(v["static"]))
    ranges, obj_tris = [], []
    for name, tris, centre in objects:
        ranges.append((name, n_static + 3 * len(obj_tris), 3 * len(tris), centre))
        obj_tris += tris
    n_vert = n_static + 3 * len(obj_tris)
    depth = 1 << max(6, math.ceil(math.log2(n_vert)))

    info = ["Table-tennis scene"]
    for k, v in enumerate(views):
        all_tris = v["padded"] + obj_tris
        scr, cost, culled, bad = analyse(v["mq"], all_tris)
        if bad:
            sys.exit(f"view {k}: {len(bad)} triangle(s) off screen / outside depth: {bad[:10]}")
        v["scr"], v["all"] = scr, all_tris
        info.append(f"view {k} ({v['name']}): static triangles {len(v['static'])}, horizon row {v['horizon']}, "
                    f"raster cost {cost} cyc -> frame ~{(cost + CLEAR_CYCLES) / F_CLK * 1e3:.2f} ms (gap 3.97 ms); "
                    f"camera {tuple(round(c, 2) for c in rot_k(v['cam'], v['k']))} m")
    horizon_y = v0["horizon"]
    if any(abs(v["horizon"] - horizon_y) > 1 for v in views):
        sys.exit("views do not share the horizon row")

    # ---- vertices.mem (view 0, power-up) and matrix.mem (view 0)
    def word144(p, cc):
        return hex32(to_q(p[0])) + hex32(to_q(p[1])) + hex32(to_q(p[2])) + hex32(to_q(1.0)) + f"{cc:04x}"
    lines = [word144(p, t[3]) for t in v0["all"] for p in t[:3]]
    lines += ["0" * 36] * (depth - len(lines))
    with open(os.path.join(args.out, "vertices.mem"), "w") as fh:
        fh.write("\n".join(lines) + "\n")
    with open(os.path.join(args.out, "matrix.mem"), "w") as fh:
        fh.write("\n".join(matrix_words(v0["M"])) + "\n")
    # ---- tt_views.mem: static vertices of every view, {X, Y, Z Q8.8, RGB565}
    vl = [f"{to_q16(p[0]):04x}{to_q16(p[1]):04x}{to_q16(p[2]):04x}{t[3]:04x}"
          for v in views for t in v["padded"] for p in t[:3]]
    with open(os.path.join(args.out, "tt_views.mem"), "w") as fh:
        fh.write("\n".join(vl) + "\n")

    # ---- tt_scene_map.vh
    c12 = lambda k: "12'h%X%X%X" % C[k]
    vh = [
        "// tt_scene_map.vh - generated by gen_tt_scene.py, do not edit",
        "// Vertex ranges are in vertices.mem lines (3 per triangle).",
        f"localparam TT_NUM_VERTICES = {n_vert};",
        f"localparam TT_VERTEX_DEPTH = {depth};",
        f"localparam [11:0] TT_SKY_RGB444   = {c12('sky')};",
        f"localparam [11:0] TT_FLOOR_RGB444 = {c12('floor')};",
        f"localparam TT_HORIZON_Y = {horizon_y};      // first framebuffer row drawn in floor colour",
        f"localparam TT_NUM_VIEWS = {len(views)};      // camera views (tt_views.mem, tt_view_mat)",
        f"localparam TT_N_STATIC  = {n_static};    // static vertices per view (padded), at vertex 0",
        "// moving objects: first vertex, vertex count, start centre (Q24.8 world units, 1 unit = 10 cm)",
    ]
    for name, base, cnt, cen in ranges:
        vh.append(f"localparam TT_{name}_BASE = {base};  localparam TT_{name}_COUNT = {cnt};")
        sv = lambda v: ("-" if to_q(v) < 0 else "") + f"32'sd{abs(to_q(v))}"
        vh.append(f"localparam signed [31:0] TT_{name}_X0 = {sv(cen[0])};  "
                  f"localparam signed [31:0] TT_{name}_Y0 = {sv(cen[1])};  "
                  f"localparam signed [31:0] TT_{name}_Z0 = {sv(cen[2])};")
    vh.append("// camera matrix of view v, word i in matrix_memory order (word 0 = M33 ... word 15 = M00)")
    vh.append("function [31:0] tt_view_mat;  input [1:0] v;  input [3:0] i;  begin")
    vh.append("    case ({v, i})")
    for k, v in enumerate(views):
        for i, w in enumerate(matrix_words(v["M"])):
            vh.append(f"        6'd{k * 16 + i}: tt_view_mat = 32'h{w};")
    vh.append("        default: tt_view_mat = 32'h0;")
    vh.append("    endcase  end endfunction")
    with open(os.path.join(args.out, "tt_scene_map.vh"), "w") as fh:
        fh.write("\n".join(vh) + "\n")

    info += [
        f"vertices: {n_vert} ({n_static} static per view + {3 * len(obj_tris)} moving)  VERTEX_DEPTH: {depth}",
        f"tt_views.mem: {len(vl)} x 64 bit",
        f"horizon row: {horizon_y}  sky {C['sky']}  floor {C['floor']} (RGB444)",
        "moving objects (vertex base, count):",
    ] + [f"  {n:9s} base {b:5d}  count {c:3d}  centre (m) {tuple(round(v / U, 3) for v in cen)}" for n, b, c, cen in ranges]
    with open(os.path.join(args.out, "scene_info.txt"), "w") as fh:
        fh.write("\n".join(info) + "\n")
    print("\n".join(info))

    if not args.no_preview:
        try:
            for k, v in enumerate(views):
                render_preview(v["scr"], v["all"], horizon_y,
                               os.path.join(args.out, "preview.png" if k == 0 else f"preview_view{k}.png"))
            print("previews written")
        except ImportError:
            print("numpy/Pillow missing: no preview")


if __name__ == "__main__":
    main()

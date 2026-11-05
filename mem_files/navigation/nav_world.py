#!/usr/bin/env python3
"""
nav_world.py - memory-file generator for the "walk around a small city" demo
               (nav_top.v + nav_camera.v + nav_overlay.v).

WHAT IT MAKES (all in --outdir, default ./nav_mem):
  vertices.mem   the 3D objects: building walls, roofs, trees   -> vertex_memory
  palette.mem    the 16 colours of the double-buffered framebuffer -> z_f_buffer
  matrix.mem     camera matrix of the start position (power-up frame) -> matrix_memory
  nav_rot.mem    256 headings x 16 words: the part of the camera
                 matrix that only depends on the viewing direction -> nav_camera
  nav_trig.mem   256 x {sin, cos} (Q1.14)                         -> nav_camera, nav_overlay
  nav_floor.mem  200 screen rows x {distance, step, offset}       -> nav_overlay
  nav_map.mem    48 x 48 cells, what is on the ground of each 2 m cell
                 (grass, road, building, ...)                     -> nav_camera, nav_overlay
  scene_info.txt numbers that must match the Verilog parameters
  preview/       what the monitor should show (start view, a few
                 more views, and a short walk video)

THE IDEA IN ONE PARAGRAPH
  The pipeline only draws the things that stick up from the ground (walls,
  roofs, trees). The ground and the sky are NOT triangles: nav_overlay paints
  every pixel the pipeline left black. Above the screen middle that is sky;
  below it, it works out which spot of the ground you are looking at (the
  camera never tilts, so every screen row looks at the ground at one fixed
  distance, stored in nav_floor.mem) and colours it from nav_map.mem. This
  needs no triangles for the 96 m x 96 m ground, never shows holes, and the
  same map is used for walking collisions and for the mini-map.

COORDINATES
  x = east, z = south, y = up, 1 unit = 1 metre. World = 96 m x 96 m.
  Map cell (cx, cz) covers x in [2cx, 2cx+2), z in [2cz, 2cz+2).
  Heading h = 0..255, 0 = north, 64 = east, 128 = south, 192 = west.
  Camera position inside nav_camera: 1/4096 m units (Q.12);
  matrices / pipeline: Q24.8 (1/256 m), like everything else in the project.

Usage:
  python3 nav_world.py                 # mem files + start-view preview
  python3 nav_world.py --video         # + walk video (needs ffmpeg) / GIF
  python3 nav_world.py --check         # + artifact statistics over random spots
"""
import argparse
import math
import os
import random
import struct
import subprocess
import zlib

import numpy as np

# =============================================================================
# 1. SETTINGS (if you change one, keep the Verilog parameters in step;
#    scene_info.txt lists all of them)
# =============================================================================
CELL      = 2.0        # metres per map cell
MAP_N     = 48         # map is MAP_N x MAP_N cells -> 96 m x 96 m
WORLD     = CELL * MAP_N
EYE_H     = 1.7        # eye height (m)
FY        = 1.0        # vertical field of view 90 deg  (y_ndc = Y'/W')
FX        = 4.0 / 3.0  # square pixels on the 300 x 400 framebuffer -> 74 deg wide
N_HEAD    = 256        # headings per full turn (1.4 deg each)
POS_FRAC  = 12         # nav_camera position: 1/4096 m
PANEL     = 1.5        # walls are split into panels this wide (see --check)
TREE_B    = 0.9        # half width of a tree's foliage (m)
VERTEX_DEPTH = 4096    # vertex_memory depth (16 RAMB36)

# depth: z = Z'/W' = DEP_A - DEP_B / W'   maps W' = 0.5 m -> -48, 150 m -> +48
# (rasterizer depth is Q6.9 = +-64, the z-buffer clears to +64 = far)
W_NEAR, W_FAR, Z_SPAN = 0.5, 150.0, 48.0
DEP_B = 2 * Z_SPAN / (1 / W_NEAR - 1 / W_FAR)
DEP_A = -Z_SPAN + DEP_B / W_NEAR

# movement, per frame (60 frames per second)
WALK_STEP = round(3.0 / 60 * (1 << POS_FRAC))   # 3 m/s   (205 units)
TURN_STEP = 1                                     # 1.4 deg per frame = 84 deg/s
RADIUS    = round(1.0 * (1 << POS_FRAC))          # you are a 2 m wide box (keeps walls >= 1 m away)
FOG_DIST  = 45.0                                  # ground further than this is hazy

START_X, START_Z, START_H = 34.0, 52.0, 0         # on the main north-south road

# =============================================================================
# 2. THE MAP: one code per 2 m cell
# =============================================================================
GRASS, ROAD, WALK, WATER, BUILDING, TREE, PATH, OUTSIDE = range(8)
WALKABLE = {GRASS, ROAD, WALK, PATH}

# 16 palette colours (RGB444). Index 0 MUST be black: it is what the clear
# writes, and nav_overlay treats black as "nothing drawn here".
PALETTE = [
    0x000,                    # 0  background (cleared pixels)
    0xD43, 0x922,             # 1, 2   red brick   : sunny side, shadow side
    0xEC7, 0xA84,             # 3, 4   cream plaster
    0x7AD, 0x468,             # 5, 6   blue glass
    0xCCC, 0x888,             # 7, 8   concrete
    0xA43, 0x632,             # 9, 10  roof tiles
    0x742,                    # 11     tree trunk
    0x4B4, 0x283,             # 12, 13 leaves
    0xDB9, 0x975,             # 14, 15 sandstone
]
MAT = {"brick": (1, 2), "cream": (3, 4), "glass": (5, 6), "concrete": (7, 8),
       "roof": (9, 10), "trunk": (11, 11), "leaf": (12, 13), "sand": (14, 15)}

# ground / sky / mini-map colours, painted by nav_overlay (not in the palette)
FLOOR_RGB = {GRASS: 0x5A3, ROAD: 0x555, WALK: 0x999, WATER: 0x37C,
             BUILDING: 0x888, TREE: 0x5A3, PATH: 0xCB8, OUTSIDE: 0x483}
MINI_RGB = dict(FLOOR_RGB)
MINI_RGB.update({BUILDING: 0x336, TREE: 0x262})
SKY_RGB = [0x58E, 0x7AF, 0x9CF, 0xBDF]          # top band ... horizon band
FOG_RGB = 0xBDF

ROAD_LINES = [2, 16, 30, 44]       # each road is 2 cells (4 m) wide


def build_map():
    """Returns (map[cz][cx], building list, tree list)."""
    m = [[GRASS] * MAP_N for _ in range(MAP_N)]
    for r in ROAD_LINES:
        for k in range(MAP_N):
            for d in (0, 1):
                m[r + d][k] = ROAD
                m[k][r + d] = ROAD
    blocks = [4, 18, 32]            # first cell of each 12-cell block
    # sidewalk ring around every block
    for bz in blocks:
        for bx in blocks:
            for i in range(12):
                for (cx, cz) in ((bx + i, bz), (bx + i, bz + 11), (bx, bz + i), (bx + 11, bz + i)):
                    m[cz][cx] = WALK

    buildings, trees = [], []

    def bld(bx, bz, o0, o1, p0, p1, h, mat, roof=None):
        # cells bx+1+o0 .. bx+1+o1, bz+1+p0 .. bz+1+p1 (inclusive), inside the block
        c0, c1, r0, r1 = bx + 1 + o0, bx + 1 + o1, bz + 1 + p0, bz + 1 + p1
        buildings.append((c0, r0, c1, r1, h, mat, roof))
        for cz in range(r0, r1 + 1):
            for cx in range(c0, c1 + 1):
                m[cz][cx] = BUILDING

    def tree(bx, bz, o, p, size=1.0):
        cx, cz = bx + 1 + o, bz + 1 + p
        trees.append((cx, cz, size))
        m[cz][cx] = TREE

    B0, B1, B2 = blocks
    # north-west: red tower + small concrete shop
    bld(B0, B0, 1, 4, 1, 4, 24.0, "brick")
    bld(B0, B0, 6, 9, 1, 3, 5.0, "concrete")
    tree(B0, B0, 7, 7); tree(B0, B0, 2, 8)
    # north: two houses with roofs
    bld(B1, B0, 1, 3, 1, 3, 4.0, "cream", roof=2.5)
    bld(B1, B0, 6, 8, 1, 3, 4.0, "cream", roof=2.5)
    bld(B1, B0, 2, 7, 6, 8, 6.0, "sand")
    tree(B1, B0, 4, 4); tree(B1, B0, 9, 6)
    # north-east: glass office + concrete block
    bld(B2, B0, 1, 8, 1, 4, 16.0, "glass")
    bld(B2, B0, 1, 4, 7, 9, 8.0, "concrete")
    tree(B2, B0, 7, 7); tree(B2, B0, 9, 9)
    # west: long hall
    bld(B0, B1, 1, 8, 2, 6, 10.0, "concrete")
    tree(B0, B1, 2, 9); tree(B0, B1, 6, 9); tree(B0, B1, 9, 9)
    # centre: park with a pond, sand paths and trees
    for cz in range(B1 + 1, B1 + 11):
        for cx in range(B1 + 1, B1 + 11):
            if cx in (B1 + 5, B1 + 6) or cz in (B1 + 5, B1 + 6):
                m[cz][cx] = PATH
    for cz in range(B1 + 2, B1 + 5):
        for cx in range(B1 + 2, B1 + 5):
            m[cz][cx] = WATER
    for (o, p) in ((7, 1), (9, 3), (8, 8), (1, 8), (3, 9), (9, 9), (1, 6)):
        tree(B1, B1, o, p)
    # east: brick tower + houses
    bld(B2, B1, 1, 4, 1, 4, 18.0, "brick")
    bld(B2, B1, 7, 9, 1, 3, 4.0, "cream", roof=2.5)
    bld(B2, B1, 7, 9, 6, 8, 4.0, "cream", roof=2.5)
    tree(B2, B1, 2, 8)
    # south-west: houses + glass block
    bld(B0, B2, 1, 3, 1, 3, 4.0, "sand", roof=2.5)
    bld(B0, B2, 5, 7, 1, 3, 4.0, "cream", roof=2.5)
    bld(B0, B2, 1, 8, 6, 9, 12.0, "glass")
    # south: plaza with a fountain and trees (all sidewalk)
    for cz in range(B2 + 1, B2 + 11):
        for cx in range(B1 + 1, B1 + 11):
            m[cz][cx] = WALK
    for cz in range(B2 + 4, B2 + 7):
        for cx in range(B1 + 4, B1 + 7):
            m[cz][cx] = WATER
    for (o, p) in ((0, 0), (9, 0), (0, 9), (9, 9)):
        tree(B1, B2, o, p)
    # south-east: glass skyscraper + low sandstone
    bld(B2, B2, 2, 5, 2, 5, 30.0, "glass")
    bld(B2, B2, 7, 9, 6, 9, 7.0, "sand")
    tree(B2, B2, 8, 2); tree(B2, B2, 2, 8)
    return m, buildings, trees


# =============================================================================
# 3. GEOMETRY: every face counter-clockwise seen from outside (CULL_MODE 1)
# =============================================================================
def rgb565_from444(c):
    """An RGB565 value that z_f_buffer truncates back to exactly c (RGB444)."""
    r, g, b = (c >> 8) & 15, (c >> 4) & 15, c & 15
    return ((r << 1) << 11) | ((g << 2) << 5) | (b << 1)


class Mesh:
    def __init__(self):
        self.tris = []          # (p0, p1, p2, palette index)

    def tri(self, a, b, c, outward, idx):
        a, b, c = (np.array(p, float) for p in (a, b, c))
        if np.dot(np.cross(b - a, c - a), outward) < 0:
            b, c = c, b                      # make it counter-clockwise from outside
        self.tris.append((a, b, c, idx))

    def quad(self, p0, p1, p2, p3, outward, idx):
        self.tri(p0, p1, p2, outward, idx)
        self.tri(p0, p2, p3, outward, idx)

    def wall(self, x0, z0, x1, z1, h, outward, idx, panel=PANEL):
        """Vertical wall (x0,z0)->(x1,z1), 0..h high, cut into panels <= `panel`
        wide. Full height in one piece: a wall's vertical edges stay vertical on
        the screen, so clamping its top/bottom to the screen edge is exact."""
        n = max(1, math.ceil(math.hypot(x1 - x0, z1 - z0) / panel - 1e-9))
        for i in range(n):
            ax, az = x0 + (x1 - x0) * i / n, z0 + (z1 - z0) * i / n
            bx, bz = x0 + (x1 - x0) * (i + 1) / n, z0 + (z1 - z0) * (i + 1) / n
            self.quad((ax, 0, az), (bx, 0, bz), (bx, h, bz), (ax, h, az), outward, idx)


def build_mesh(buildings, trees, panel=PANEL):
    m = Mesh()
    for (c0, r0, c1, r1, h, mat, roof) in buildings:
        x0, z0, x1, z1 = c0 * CELL, r0 * CELL, (c1 + 1) * CELL, (r1 + 1) * CELL
        lit, shade = MAT[mat]
        # sun from the south-east: south and east walls bright, north and west dark
        m.wall(x0, z1, x1, z1, h, (0, 0, 1), lit, panel)       # south
        m.wall(x1, z1, x1, z0, h, (1, 0, 0), lit, panel)       # east
        m.wall(x1, z0, x0, z0, h, (0, 0, -1), shade, panel)    # north
        m.wall(x0, z0, x0, z1, h, (-1, 0, 0), shade, panel)    # west
        # flat roofs are higher than the eye -> never visible, so none are made
        if roof:
            rl, rs = MAT["roof"]
            ap = ((x0 + x1) / 2, h + roof, (z0 + z1) / 2)
            m.tri((x0, h, z1), (x1, h, z1), ap, (0, 1, 1), rl)
            m.tri((x1, h, z1), (x1, h, z0), ap, (1, 1, 0), rl)
            m.tri((x1, h, z0), (x0, h, z0), ap, (0, 1, -1), rs)
            m.tri((x0, h, z0), (x0, h, z1), ap, (-1, 1, 0), rs)
    for (cx, cz, s) in trees:
        x, z = cx * CELL + CELL / 2, cz * CELL + CELL / 2
        t, th = 0.25, 1.6
        tl, ts = MAT["trunk"]
        m.wall(x - t, z + t, x + t, z + t, th, (0, 0, 1), tl)
        m.wall(x + t, z + t, x + t, z - t, th, (1, 0, 0), tl)
        m.wall(x + t, z - t, x - t, z - t, th, (0, 0, -1), ts)
        m.wall(x - t, z - t, x - t, z + t, th, (-1, 0, 0), ts)
        b, top = TREE_B * s, 5.5 * s      # foliage stays inside its 2 m cell
        ll, ls = MAT["leaf"]
        ap = (x, top, z)
        # each side of the cone is split in two (smaller triangles = smaller
        # gaps when one of them has a corner behind your eye)
        for (p, q_, n, col) in (((x - b, z + b), (x + b, z + b), (0, 1, 1), ll),
                                ((x + b, z + b), (x + b, z - b), (1, 1, 0), ll),
                                ((x + b, z - b), (x - b, z - b), (0, 1, -1), ls),
                                ((x - b, z - b), (x - b, z + b), (-1, 1, 0), ls)):
            mid = ((p[0] + q_[0]) / 2, th, (p[1] + q_[1]) / 2)
            m.tri((p[0], th, p[1]), mid, ap, n, col)
            m.tri(mid, (q_[0], th, q_[1]), ap, n, col)
        # underside of the leaves (seen when you stand close)
        m.quad((x - b, th, z - b), (x + b, th, z - b), (x + b, th, z + b), (x - b, th, z + b), (0, -1, 0), ls)
    return m.tris


# =============================================================================
# 4. FIXED-POINT TABLES
# =============================================================================
def q(x, frac):
    return int(round(x * (1 << frac)))


def w32(v):
    return v & 0xFFFFFFFF


def rot_words(h):
    """16 Q24.8 words, ROW-MAJOR (element 4*row+col), for heading h.
    The 4th column holds only the part that does not depend on the position;
    nav_camera adds   - (m[r][0]*x + m[r][2]*z)   to it."""
    a = 2 * math.pi * h / N_HEAD
    s, c = math.sin(a), math.cos(a)
    # forward f = (s, 0, -c), right r = (c, 0, s), up = (0, 1, 0)
    rows = [
        [FX * c, 0.0, FX * s, 0.0],                 # X' = FX * (right . (p - eye))
        [0.0, FY, 0.0, -FY * EYE_H],                # Y' = FY * (y - eye height)
        [DEP_A * s, 0.0, -DEP_A * c, -DEP_B],       # Z' = DEP_A * W' - DEP_B
        [s, 0.0, -c, 0.0],                          # W' = forward . (p - eye)
    ]
    return [q(v, 8) for row in rows for v in row]


ROT = [rot_words(h) for h in range(N_HEAD)]
TRIG = [(q(math.sin(2 * math.pi * h / N_HEAD), 14), q(math.cos(2 * math.pi * h / N_HEAD), 14))
        for h in range(N_HEAD)]


def floor_row(v):
    """Ground distance / per-pixel step / start offset for framebuffer row v (200..399).
    Pixel centre (u+0.5, v+0.5): the ground there is at forward distance
    d = 200*FY*EYE_H / (v + 0.5 - 200) and sideways  d*(u + 0.5 - 150)/(150*FX)."""
    d = 200 * FY * EYE_H / (v + 0.5 - 200)
    k = d / (150 * FX)
    g = 149.5 * k
    return q(d, 16), q(k, 16), q(g, 16)


FLOOR = [floor_row(v) for v in range(200, 400)]
FOG_ROWS = sum(1 for (d, _, _) in FLOOR if d > q(FOG_DIST, 16))   # rows 200..200+FOG_ROWS-1


# =============================================================================
# 5. BIT-EXACT MODELS OF THE VERILOG (nav_camera / nav_overlay)
# =============================================================================
def asr(v, n):
    """arithmetic shift right (Verilog >>> on a signed value)"""
    return v >> n


class Camera:
    """Same integer steps as nav_camera.v."""

    def __init__(self, mp):
        self.map = mp
        self.x = q(START_X, POS_FRAC)
        self.z = q(START_Z, POS_FRAC)
        self.h = START_H
        self.bump = False

    def walkable(self, px, pz):
        lim = MAP_N << (POS_FRAC + 1)            # 96 m in 1/4096 m
        if px < 0 or pz < 0 or px >= lim or pz >= lim:
            return False
        return self.map[pz >> (POS_FRAC + 1)][px >> (POS_FRAC + 1)] in WALKABLE

    def box_ok(self, px, pz):
        R = RADIUS
        return (self.walkable(px - R, pz - R) and self.walkable(px + R, pz - R) and
                self.walkable(px - R, pz + R) and self.walkable(px + R, pz + R))

    def step(self, fwd=0, back=0, left=0, right=0, snap=0, run=0, strafe=0):
        """One frame. Returns True if the pose changed."""
        old = (self.x, self.z, self.h)
        mul = 2 if run else 1
        # 1) turning (left/right turn unless strafe mode), snap to the nearest 90 deg
        if not strafe:
            self.h = (self.h + mul * TURN_STEP * (right - left)) & (N_HEAD - 1)
        if snap:
            self.h = ((self.h + N_HEAD // 8) & (N_HEAD - 1)) & ~(N_HEAD // 4 - 1)
        # 2) which way to walk
        moving = True
        if fwd and not back:
            d = self.h
        elif back and not fwd:
            d = self.h + N_HEAD // 2
        elif strafe and right and not left:
            d = self.h + N_HEAD // 4
        elif strafe and left and not right:
            d = self.h + 3 * N_HEAD // 4
        else:
            moving = False
            d = self.h
        d &= N_HEAD - 1
        s, c = TRIG[d]
        stp = WALK_STEP * mul if moving else 0
        dx = asr(stp * s, 14)
        dz = -asr(stp * c, 14)
        # 3) collision: x first, then z (so you slide along walls)
        self.bump = False
        if dx != 0:
            if self.box_ok(self.x + dx, self.z):
                self.x += dx
            else:
                self.bump = True
        if dz != 0:
            if self.box_ok(self.x, self.z + dz):
                self.z += dz
            else:
                self.bump = True
        return (self.x, self.z, self.h) != old

    def matrix_rowmajor(self):
        """16 Q24.8 ints, element 4*r+c - exactly what nav_camera builds."""
        base = ROT[self.h]
        x8, z8 = asr(self.x, POS_FRAC - 8), asr(self.z, POS_FRAC - 8)
        m = list(base)
        for r in range(4):
            m[4 * r + 3] = base[4 * r + 3] - asr(base[4 * r + 0] * x8 + base[4 * r + 2] * z8, 8)
        return m

    def pose8(self):
        return asr(self.x, POS_FRAC - 8), asr(self.z, POS_FRAC - 8), self.h


def matrix_mem_words(m):
    """matrix_memory order: word k = element 15-k (M33 first ... M00 last)."""
    return [w32(m[15 - k]) for k in range(16)]


# ---- overlay (sky, ground, mini-map) --------------------------------------
MM_X0, MM_Y0, MM_CELL = 485, 60, 3            # mini-map: 48 x 3 = 144 px square
FB_X0, FB_Y0 = 170, 40


def rgb444_to_rgb(c):
    return ((c >> 8) & 15) * 17, ((c >> 4) & 15) * 17, (c & 15) * 17


def overlay_frame(fb444, mp, pose8):
    """fb444: 400x300 RGB444 ints from the pipeline (0 = nothing drawn).
    Returns the 480x640x3 picture nav_overlay sends to the monitor."""
    x8, z8, h = pose8
    s, c = TRIG[h]
    mapa = np.array(mp, dtype=np.int64)
    out = np.zeros((480, 640), np.int64)
    win = fb444.copy()
    # sky
    for v in range(200):
        row = win[v]
        row[row == 0] = SKY_RGB[min(3, v // 50)]
    # ground
    u = np.arange(300, dtype=np.int64)
    floor_lut = np.array([FLOOR_RGB[t] for t in range(8)])
    for v in range(200, 400):
        D, K, G = FLOOR[v - 200]
        ax = (x8 << 8) + asr(D * s, 14) - asr(G * c, 14)
        az = (z8 << 8) - asr(D * c, 14) - asr(G * s, 14)
        bx, bz = asr(K * c, 14), asr(K * s, 14)
        px, pz = ax + u * bx, az + u * bz
        lim = MAP_N << 17
        inside = (px >= 0) & (pz >= 0) & (px < lim) & (pz < lim)
        t = np.full(300, OUTSIDE)
        t[inside] = mapa[pz[inside] >> 17, px[inside] >> 17]
        col = floor_lut[t]
        if v - 200 < FOG_ROWS:                    # haze: average with the fog colour
            col = (((col >> 8 & 15) + (FOG_RGB >> 8 & 15)) >> 1 << 8 |
                   ((col >> 4 & 15) + (FOG_RGB >> 4 & 15)) >> 1 << 4 |
                   ((col & 15) + (FOG_RGB & 15)) >> 1)
        row = win[v]
        row[row == 0] = col[row == 0]
    out[FB_Y0:FB_Y0 + 400, FB_X0:FB_X0 + 300] = win
    # mini-map
    mini_lut = np.array([MINI_RGB[t] for t in range(8)])
    n = MAP_N * MM_CELL
    out[MM_Y0 - 1:MM_Y0 + n + 1, MM_X0 - 1:MM_X0 + n + 1] = 0xFFF
    out[MM_Y0:MM_Y0 + n, MM_X0:MM_X0 + n] = mini_lut[np.repeat(np.repeat(mapa, MM_CELL, 0), MM_CELL, 1)]
    mx = MM_X0 + ((x8 * 3) >> 9)                # 1.5 px per metre
    mz = MM_Y0 + ((z8 * 3) >> 9)
    for k in (4, 8, 12):                        # heading dots, 2 x 2 px
        dx, dz = asr(k * s, 14), -asr(k * c, 14)
        out[mz + dz - 1:mz + dz + 1, mx + dx - 1:mx + dx + 1] = 0xFF0
    out[mz - 2:mz + 3, mx - 2:mx + 3] = 0xF00     # you: 5 x 5 px red
    img = np.zeros((480, 640, 3), np.uint8)
    img[..., 0] = ((out >> 8) & 15) * 17
    img[..., 1] = ((out >> 4) & 15) * 17
    img[..., 2] = (out & 15) * 17
    return img


# ---- the 3D pipeline (approximate model of the project's pipeline) ---------
W, H = 300, 400
YS, XS = np.mgrid[0:H, 0:W]
PX, PY = XS + 0.5, YS + 0.5


def render(tris, mrow, exact=False):
    """Pipeline model: matrix (Q24.8 row-major ints) -> 400x300 RGB444 ints.
    exact=False: like the hardware (discard triangles with a vertex behind the
                 eye, clamp vertices to the screen, back-face cull, z-buffer).
    exact=True : reference image (proper near-plane clipping, no clamping)."""
    M = np.array(mrow, float).reshape(4, 4) / 256.0
    fb = np.zeros((H, W), np.int64)
    zb = np.full((H, W), 64.0)
    cycles = W * H
    for p0, p1, p2, idx in tris:
        cycles += 9
        V = np.array([[*p0, 1.0], [*p1, 1.0], [*p2, 1.0]]) @ M.T
        polys = [V]
        if (V[:, 3] <= 0).any():
            if not exact:
                continue
            polys = [clip_near(V)]
            if len(polys[0]) < 3:
                continue
        for P in polys:
            for k in range(1, len(P) - 1):
                cycles += draw(fb, zb, P[[0, k, k + 1]], PALETTE[idx], exact)
    return fb, cycles


def clip_near(V, eps=0.02):
    out = []
    n = len(V)
    for i in range(n):
        a, b = V[i], V[(i + 1) % n]
        ia, ib = a[3] >= eps, b[3] >= eps
        if ia:
            out.append(a)
        if ia != ib:
            t = (eps - a[3]) / (b[3] - a[3])
            out.append(a + t * (b - a))
    return np.array(out)


def draw(fb, zb, V, col, exact):
    Wc = V[:, 3]
    sx = (V[:, 0] / Wc + 1) * 150
    sy = (1 - V[:, 1] / Wc) * 200
    sz = V[:, 2] / Wc
    if not exact:                     # triangle setup clamps every vertex to the screen
        sx = np.round(np.clip(sx, 0, 300) * 256) / 256
        sy = np.round(np.clip(sy, 0, 400) * 256) / 256
    dx1, dy1, dx2, dy2 = sx[1] - sx[0], sy[1] - sy[0], sx[2] - sx[0], sy[2] - sy[0]
    D = dx1 * dy2 - dx2 * dy1
    if D >= 0:                        # degenerate or back face (CULL_MODE 1)
        return 0
    zA = ((sz[1] - sz[0]) * dy2 - (sz[2] - sz[0]) * dy1) / D
    zB = (dx1 * (sz[2] - sz[0]) - dx2 * (sz[1] - sz[0])) / D
    zC = sz[0] - zA * sx[0] - zB * sy[0]
    if not exact and max(abs(zA), abs(zB), abs(zC)) > 8192:
        return 0                      # setup: depth overflow -> discarded
    x0, x1 = max(0, int(math.floor(sx.min()))), min(299, int(math.ceil(sx.max())))
    y0, y1 = max(0, int(math.floor(sy.min()))), min(399, int(math.ceil(sy.max())))
    if x0 > x1 or y0 > y1:
        return 0
    X, Y = PX[y0:y1 + 1, x0:x1 + 1], PY[y0:y1 + 1, x0:x1 + 1]
    e0 = -((sy[0] - sy[1]) * X + (sx[1] - sx[0]) * Y + (sx[0] * sy[1] - sx[1] * sy[0]))
    e1 = -((sy[1] - sy[2]) * X + (sx[2] - sx[1]) * Y + (sx[1] * sy[2] - sx[2] * sy[1]))
    e2 = -((sy[2] - sy[0]) * X + (sx[0] - sx[2]) * Y + (sx[2] * sy[0] - sx[0] * sy[2]))
    Z = np.clip(sz[0] + zA * (X - sx[0]) + zB * (Y - sy[0]), -64, 63.99)
    zs = zb[y0:y1 + 1, x0:x1 + 1]
    win = (e0 >= 0) & (e1 >= 0) & (e2 >= 0) & (Z < zs)
    zs[win] = Z[win]
    fb[y0:y1 + 1, x0:x1 + 1][win] = col
    return (x1 - x0 + 1) * (y1 - y0 + 1) + 6


# =============================================================================
# 6. OUTPUT
# =============================================================================
def write_png(path, img):
    h, w, _ = img.shape
    raw = b"".join(b"\x00" + img[y].tobytes() for y in range(h))

    def chunk(t, d):
        c = struct.pack(">I", len(d)) + t + d
        return c + struct.pack(">I", zlib.crc32(t + d) & 0xFFFFFFFF)

    with open(path, "wb") as f:
        f.write(b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0))
                + chunk(b"IDAT", zlib.compress(raw, 9)) + chunk(b"IEND", b""))


# a scripted walk for the preview video: (frames, buttons)
WALK_SCRIPT = [
    (40, {}),
    (100, {"fwd": 1}),
    (64, {"right": 1}),                      # turn 90 deg right, standing
    (90, {"fwd": 1, "left": 1}),             # walk in a gentle left curve
    (60, {"fwd": 1, "run": 1}),
    (64, {"left": 1, "run": 1}),             # quick look around
    (1, {"snap": 1}),
    (80, {"fwd": 1}),
    (40, {"strafe": 1, "right": 1}),
    (60, {"back": 1}),
]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--outdir", default="nav_mem")
    ap.add_argument("--video", action="store_true")
    ap.add_argument("--check", type=int, default=0, metavar="N",
                    help="compare the hardware model with a correctly clipped "
                         "reference at N random spots")
    args = ap.parse_args()
    os.makedirs(os.path.join(args.outdir, "preview"), exist_ok=True)
    out = lambda name: os.path.join(args.outdir, name)

    mp, buildings, trees = build_map()
    tris = build_mesh(buildings, trees)
    nv = 3 * len(tris)
    assert nv <= VERTEX_DEPTH, f"{nv} vertices do not fit in {VERTEX_DEPTH}"

    # ---- vertices.mem: {X, Y, Z, W=1, RGB565}, padded with zero words ----
    with open(out("vertices.mem"), "w") as f:
        for p0, p1, p2, idx in tris:
            col = rgb565_from444(PALETTE[idx])
            for p in (p0, p1, p2):
                v = (w32(q(p[0], 8)) << 112) | (w32(q(p[1], 8)) << 80) | \
                    (w32(q(p[2], 8)) << 48) | (w32(q(1.0, 8)) << 16) | col
                f.write(f"{v:036x}\n")
        for _ in range(VERTEX_DEPTH - nv):
            f.write("0" * 36 + "\n")
    with open(out("palette.mem"), "w") as f:
        f.writelines(f"{c:03X}\n" for c in PALETTE)
    with open(out("nav_rot.mem"), "w") as f:
        for h in range(N_HEAD):
            f.writelines(f"{w32(v):08x}\n" for v in ROT[h])
    with open(out("nav_trig.mem"), "w") as f:
        f.writelines(f"{s & 0xFFFF:04x}{c & 0xFFFF:04x}\n" for s, c in TRIG)
    with open(out("nav_floor.mem"), "w") as f:
        f.writelines(f"{d:08x}{k:08x}{g:08x}\n" for d, k, g in FLOOR)
    with open(out("nav_map.mem"), "w") as f:
        f.writelines(f"{mp[cz][cx]:x}\n" for cz in range(MAP_N) for cx in range(MAP_N))

    cam = Camera(mp)
    assert cam.box_ok(cam.x, cam.z), "start position is not walkable"
    with open(out("matrix.mem"), "w") as f:
        f.writelines(f"{w:08x}\n" for w in matrix_mem_words(cam.matrix_rowmajor()))

    # ---- preview of the start view ----
    fb, cyc = render(tris, cam.matrix_rowmajor())
    write_png(out("preview/start_view.png"), overlay_frame(fb, mp, cam.pose8()))
    worst = cyc
    for k, (xx, zz, hh) in enumerate([(34, 52, 64), (20, 36, 160), (60, 33, 200), (75, 64, 0)]):
        c2 = Camera(mp)
        c2.x, c2.z, c2.h = q(xx, POS_FRAC), q(zz, POS_FRAC), hh
        fb, cyc = render(tris, c2.matrix_rowmajor())
        worst = max(worst, cyc)
        write_png(out(f"preview/view_{k + 1}.png"), overlay_frame(fb, mp, c2.pose8()))

    # ---- walk video ----
    if args.video:
        fdir = out("preview/frames")
        os.makedirs(fdir, exist_ok=True)
        cam = Camera(mp)
        n = 0
        for frames, btn in WALK_SCRIPT:
            for i in range(frames):
                b = dict(btn)
                if "snap" in b and i:
                    b.pop("snap")
                cam.step(**b)
                if n % 2 == 0:                      # 30 fps video
                    fb, cyc = render(tris, cam.matrix_rowmajor())
                    worst = max(worst, cyc)
                    write_png(os.path.join(fdir, f"f{n // 2:04d}.png"), overlay_frame(fb, mp, cam.pose8()))
                n += 1
        subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-framerate", "30", "-i",
                        os.path.join(fdir, "f%04d.png"), "-c:v", "libx264", "-preset", "veryfast",
                        "-pix_fmt", "yuv420p", out("preview/walk.mp4")], check=True)

    # ---- artifact check ----
    stats = ""
    if args.check:
        rng = random.Random(1)
        wrong, cam = [], Camera(mp)
        while len(wrong) < args.check:
            cam.x = rng.randrange(0, MAP_N << (POS_FRAC + 1))
            cam.z = rng.randrange(0, MAP_N << (POS_FRAC + 1))
            cam.h = rng.randrange(N_HEAD)
            if not cam.box_ok(cam.x, cam.z):
                continue
            m = cam.matrix_rowmajor()
            a, cyc = render(tris, m)
            b, _ = render(tris, m, exact=True)
            worst = max(worst, cyc)
            wrong.append(int((a != b).sum()))
        wrong = np.array(wrong) / 1200.0
        stats = (f"ARTIFACTS      = mean {wrong.mean():.2f}% of pixels, 95th pct {np.percentile(wrong, 95):.2f}%, "
                 f"worst {wrong.max():.2f}% ({args.check} random spots)")

    info = [
        f"NUM_VERTICES   = {nv}",
        f"VERTEX_DEPTH   = {VERTEX_DEPTH}",
        f"NUM_TRIANGLES  = {len(tris)}   ({len(buildings)} buildings, {len(trees)} trees, panels {PANEL} m)",
        f"WORLD          = {WORLD:.0f} m x {WORLD:.0f} m, map {MAP_N} x {MAP_N} cells of {CELL} m",
        f"START          = x {START_X} m, z {START_Z} m, heading {START_H}  "
        f"(START_X={q(START_X, POS_FRAC)}, START_Z={q(START_Z, POS_FRAC)} in 1/4096 m)",
        f"WALK_STEP      = {WALK_STEP}   (1/4096 m per frame, {WALK_STEP / 4096 * 60:.2f} m/s; run x2)",
        f"TURN_STEP      = {TURN_STEP}   ({TURN_STEP * 360 / N_HEAD * 60:.0f} deg/s; run x2)",
        f"RADIUS         = {RADIUS}   ({RADIUS / 4096:.2f} m)",
        f"FOG_ROWS       = {FOG_ROWS}   (ground rows further than {FOG_DIST} m)",
        f"DEPTH_MAP      = z = {DEP_A:.4f} - {DEP_B:.4f}/W'",
        f"WORST_RENDER   = {worst} clk = {worst / 1e5:.2f} ms at 100 MHz (a 60 fps frame is 16.7 ms)",
    ]
    if stats:
        info.append(stats)
    with open(out("scene_info.txt"), "w") as f:
        f.write("\n".join(info) + "\n")
    print("\n".join(info))


if __name__ == "__main__":
    main()

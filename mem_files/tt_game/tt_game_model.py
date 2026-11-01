#!/usr/bin/env python3
"""
tt_game_model.py  -  table-tennis gameplay: reference model + constants
=======================================================================

Bit-exact integer model of the gameplay RTL (tt_game_core.v, tt_obj_animator.v)
and the generator of tt_game_map.vh (every game constant, the swing tables, the
sin/cos table and the object ROM used by the RTL).

What it does
------------
  * writes tt_game_map.vh            (include for the RTL)
  * runs scripted / auto-play games  -> per-frame logs (golden reference for
                                        tb_tt_game_core.v)
  * checks every moving-object pose of those games: all vertices on screen,
    depth in range, render time per frame (clear + raster) under the 3.97 ms
    VGA gap
  * renders a preview GIF of a rally (needs numpy + Pillow)

World units: 1 unit = 10 cm.  Ball / velocity state: Q16.16 units.
Poses sent to the renderer: Q8.8 units (16 bit), yaw = 0..63 (5.625 deg).

Usage:
  python tt_game_model.py --scene <dir with vertices.mem/matrix.mem/tt_scene_map.vh>
                          --out <dir> [--frames N] [--gif]
  python tt_game_model.py --check-sim frames.txt      (output of tb_tt_game.v)
"""

import argparse
import math
import os
import sys

import gen_tt_scene as G

U = G.U                      # units per metre
Q16 = 1 << 16
Q8 = 1 << 8

# ----------------------------------------------------------------------------
# Game constants (metres / frames at 60 Hz)
# ----------------------------------------------------------------------------
Y_TABLE = G.TABLE_H
BALL_HALF = G.BALL / 2
Y_BOUNCE = Y_TABLE + BALL_HALF          # ball centre when it touches the table
Z_HIT = 1.50                            # hit plane (|z|) where the racket meets the ball
Y_HIT = 0.98
Z_SERVE_B1 = 0.60                       # serve: first bounce, server's own half
Z_SERVE_B2 = 0.75                       # serve: second bounce, receiver's half
Z_RALLY_B = 0.80                        # rally: bounce on the opponent's half
HOLD_Y = 1.03                           # ball held before the serve (bobs +-BOB/2)
BOB = 0.05
BOB_HALF = 24                           # frames per bob direction

# arcs: frames, height of the arc above the straight line (bump)
ARCS = {                                 # kind: (N, bump m)
    "SERVE1": (14, 0.03),
    "SERVE2": (26, 0.20),
    "RALLY":  (30, 0.26),
    "TOHIT":  (22, 0.16),
}
A_NONE, A_SERVE1, A_SERVE2, A_RALLY, A_TOHIT, A_FREE = range(6)
ARC_ID = {"SERVE1": A_SERVE1, "SERVE2": A_SERVE2, "RALLY": A_RALLY, "TOHIT": A_TOHIT}

# lateral positions (close to the middle line)
BX = [-0.12, -0.08, -0.04, 0.0, 0.0, 0.04, 0.08, 0.12]   # bounce x, lfsr[2:0]
HXD = [-0.04, 0.0, 0.04, 0.0]                              # hit x = bounce x + HXD[lfsr[4:3]]

# swing animation
SW_LEN = 24                             # frames
SW_CONTACT = 8                          # frame at which the blade meets the ball
HIT_EARLY = 8                           # contact may be up to 8 frames before arrival
HIT_LATE = 5                            # ... or 5 frames after
W_LO = SW_CONTACT - HIT_LATE            # press accepted when tta in [W_LO, W_HI]
W_HI = SW_CONTACT + HIT_EARLY
MISS_FRAMES = 10                        # ball keeps flying 10 frames past the racket, then hides
SPIN = 3                                # ball yaw step per frame in flight
TRACK_STEP = 0.012                      # idle racket follows the ball x (m / frame)

POINT_PAUSE = 50                        # frames between a point and the next serve
SERVE_AUTO = 45                         # computer serves after this many frames
OVER_PAUSE = 180                        # both computer players: new game after 3 s
WIN_POINTS = 11

# computer player: press when tta == SW_CONTACT + jitter; 1 in 8 is a deliberate miss
AI_JIT = [-2, -1, 0, 0, 1, 1, 2, 0]     # lfsr[7:5]
AI_MISS_T = W_HI + 2                    # too early -> whiff

S_SERVE, S_SERVING, S_FLIGHT, S_POINT, S_OVER = range(5)
STATE_NAME = ["SERVE", "SERVING", "FLIGHT", "POINT", "OVER"]

LFSR_SEED = 0xACE1
LFSR_TAPS = 0xB400                      # Galois, x^16 + x^14 + x^13 + x^11 + 1

# swing keyframes for the NEAR player (P1): frame, x offset from the racket's
# base x, y, z (absolute, metres), yaw (deg). Far player: x -> -x, z -> -z.
KEY_FORE = [(0, 0.10, 0.92, 1.62, 0), (4, 0.38, 0.90, 1.72, 45), (SW_CONTACT, 0.02, 0.98, 1.56, 0),
            (13, -0.28, 1.12, 1.42, -50), (SW_LEN - 1, 0.10, 0.92, 1.62, 0)]
KEY_BACK = [(0, 0.10, 0.92, 1.62, 0), (4, -0.32, 0.92, 1.70, -45), (SW_CONTACT, -0.02, 0.98, 1.56, 0),
            (13, 0.30, 1.10, 1.44, 50), (SW_LEN - 1, 0.10, 0.92, 1.62, 0)]

# ----------------------------------------------------------------------------
# Fixed-point helpers
# ----------------------------------------------------------------------------
def q16(m):              # metres -> Q16.16 units
    return int(round(m * U * Q16))


def q8(m):               # metres -> Q8.8 units
    return int(round(m * U * Q8))


def s32(v):
    v &= 0xFFFFFFFF
    return v - (1 << 32) if v & 0x80000000 else v


def arc_consts(kind):
    n, bump = ARCS[kind]
    g = int(round(8 * bump * U * Q16 / (n * n)))
    rn = int(round(Q16 / n))
    gh = g * (n - 1) // 2
    return n, g, rn, gh


ARC_C = {ARC_ID[k]: arc_consts(k) for k in ARCS}


def swing_table(keys):
    """Per-frame (x, y, z) Q8.8 and yaw 0..63, smoothstep between keyframes."""
    out = []
    for f in range(SW_LEN):
        for (f0, *a), (f1, *b) in zip(keys[:-1], keys[1:]):
            if f0 <= f <= f1:
                t = (f - f0) / (f1 - f0)
                t = t * t * (3 - 2 * t)
                v = [a[i] + (b[i] - a[i]) * t for i in range(4)]
                break
        yaw = int(round(v[3] / 5.625)) & 63
        out.append((q8(v[0]), q8(v[1]), q8(v[2]), yaw))
    return out


SWING = [swing_table(KEY_FORE), swing_table(KEY_BACK)]     # 0 forehand, 1 backhand
SINCOS = [(int(round(16384 * math.cos(2 * math.pi * k / 64))),
           int(round(16384 * math.sin(2 * math.pi * k / 64)))) for k in range(64)]


# ----------------------------------------------------------------------------
# Object ROM: local vertices of the moving objects (same order as vertices.mem)
# ----------------------------------------------------------------------------
def object_rom():
    col = {k: G.rgb444(*v) for k, v in G.C.items()}
    objs = [G.racket(G.RACKET_PLAYER, col["blade_red"]),
            G.racket(G.RACKET_OPP, col["blade_black"]),
            G.ball(G.BALL_POS)]
    rom = []
    for oid, (tris, c) in enumerate(objs):
        for a, b, cc, colour in tris:
            for p in (a, b, cc):
                l = [int(round((p[i] - c[i]) * Q8)) for i in range(3)]
                rom.append((oid, l[0], l[1], l[2], colour))
    return rom


def animate(rom, poses):
    """tt_obj_animator model. poses[oid] = (x, y, z, yaw, vis), Q8.8 / 0..63.
    Returns the Q24.8 (X, Y, Z) of every object vertex."""
    out = []
    for oid, lx, ly, lz, colour in rom:
        x, y, z, yaw, vis = poses[oid]
        if not vis:
            out.append((x, y, z, colour))
            continue
        c, s = SINCOS[yaw]
        rx = (c * lx + s * lz) >> 14
        rz = (-s * lx + c * lz) >> 14
        out.append((x + rx, y + ly, z + rz, colour))
    return out


# ----------------------------------------------------------------------------
# Game model (mirrors tt_game_core.v step by step)
# ----------------------------------------------------------------------------
class Game:
    def __init__(self):
        self.lfsr = LFSR_SEED
        self.score = [0, 0]
        self.first_server = 0
        self.frame = 0
        self.new_game(toggle=False)

    # -- helpers ------------------------------------------------------------
    def new_game(self, toggle=True):
        if toggle:
            self.first_server ^= 1
        self.score = [0, 0]
        self.server = self.first_server
        self.winner = 0
        self.state = S_SERVE
        self.wait = 0
        self.sw_act = [0, 0]
        self.sw_type = [0, 0]
        self.sw_frame = [0, 0]
        self.base_x = [0, 0]                 # Q8.8 world x of each racket's base
        self.hitter = 0
        self.recv = 1
        self.tta = 0
        self.swung = 0
        self.recv_ok = 0
        self.miss = 0
        self.ai_t = 0
        self.bx = 0
        self.hx = 0
        self.arc = A_NONE
        self.arc_left = 0
        self.p = [0, 0, 0]
        self.v = [0, 0, 0]
        self.g = 0
        self.tgt = [0, 0, 0]
        self.yaw = 0
        self.vis = 1
        self.reset_bob()
        self.hold_ball()

    def reset_bob(self):
        self.bob = 0
        self.bob_cnt = 0
        self.bob_dir = 0

    def side(self, p):                       # +1 near, -1 far
        return 1 if p == 0 else -1

    def hold_ball(self):
        s = self.side(self.server)
        self.p = [self.base_x[self.server] << 8, q16(HOLD_Y) + self.bob, s * q16(Z_HIT)]

    def lfsr_step(self):
        l = self.lfsr
        self.lfsr = (l >> 1) ^ (LFSR_TAPS if l & 1 else 0)

    def pick_lateral(self):
        """New bounce x / hit x / computer timing for the next incoming ball."""
        l = self.lfsr
        self.bx = q16(BX[l & 7])
        self.hx = q16(BX[l & 7] + HXD[(l >> 3) & 3])
        miss = ((l >> 8) & 7) == 0
        self.ai_t = AI_MISS_T if miss else SW_CONTACT + AI_JIT[(l >> 5) & 7]

    def launch(self, kind, tgt):
        n, g, rn, gh = ARC_C[kind]
        d = [tgt[i] - self.p[i] for i in range(3)]
        self.v = [(d[i] * rn) >> 16 for i in range(3)]
        self.v[1] += gh
        self.g = g
        self.arc = kind
        self.arc_left = n
        self.tgt = list(tgt)

    def start_swing(self, p, t):
        self.sw_act[p] = 1
        self.sw_type[p] = t
        self.sw_frame[p] = 0

    # -- one frame ----------------------------------------------------------
    def step(self, press, newgame=False, auto=(0, 0)):
        """press[p] = (forehand, backhand) button events since the last frame."""
        self.frame += 1
        self.lfsr_step()
        if newgame:
            self.new_game()
            return self.poses()

        # 1. swings advance
        contact = [0, 0]
        for p in (0, 1):
            if self.sw_act[p]:
                self.sw_frame[p] += 1
                if self.sw_frame[p] == SW_LEN:
                    self.sw_act[p] = 0
                    self.sw_frame[p] = 0
            contact[p] = self.sw_act[p] and self.sw_frame[p] == SW_CONTACT

        def pressed(p):
            f, b = press[p]
            return (1, 0) if f else (1, 1) if b else (0, 0)

        launched = 0
        st = self.state
        if st == S_SERVE:
            self.vis = 1
            if self.bob_dir == 0:
                self.bob += q16(BOB / BOB_HALF)
            else:
                self.bob -= q16(BOB / BOB_HALF)
            self.bob_cnt += 1
            if self.bob_cnt == BOB_HALF:
                self.bob_cnt = 0
                self.bob_dir ^= 1
            self.hold_ball()
            if self.wait < 255:
                self.wait += 1
            srv = self.server
            hit, t = pressed(srv)
            if auto[srv] and self.wait >= SERVE_AUTO:
                hit, t = 1, (self.lfsr >> 11) & 1
            if hit and not self.sw_act[srv]:
                self.start_swing(srv, t)
                self.state = S_SERVING

        elif st == S_SERVING:
            srv = self.server
            if contact[srv]:
                s = self.side(srv)
                self.pick_lateral()
                self.hitter = srv
                self.recv = srv ^ 1
                self.tta = ARC_C[A_SERVE1][0] + ARC_C[A_SERVE2][0] + ARC_C[A_TOHIT][0]
                self.swung = self.recv_ok = self.miss = 0
                self.launch(A_SERVE1, [self.p[0], q16(Y_BOUNCE), s * q16(Z_SERVE_B1)])
                self.state = S_FLIGHT

        elif st == S_FLIGHT:
            r = self.recv
            # a) receiver input / timing window
            if not self.miss and not self.swung:
                hit, t = pressed(r)
                if auto[r] and self.tta == self.ai_t:
                    hx_side = self.hx >= 0 if r == 0 else self.hx <= 0
                    hit, t = 1, (0 if hx_side else 1)
                if hit and not self.sw_act[r]:
                    self.start_swing(r, t)
                    self.swung = 1
                    if W_LO <= self.tta <= W_HI:
                        self.recv_ok = 1
                    else:
                        self.miss = 1
                elif self.tta < W_LO:
                    self.miss = 1
            # b) contact -> return
            if contact[r] and self.recv_ok:
                s = self.side(r)
                self.pick_lateral()
                self.hitter = r
                self.recv = r ^ 1
                self.tta = ARC_C[A_RALLY][0] + ARC_C[A_TOHIT][0]
                self.swung = self.recv_ok = self.miss = 0
                self.launch(A_RALLY, [self.bx, q16(Y_BOUNCE), -s * q16(Z_RALLY_B)])
                launched = 1
            # c) physics
            if not launched:
                self.p = [s32(self.p[i] + self.v[i]) for i in range(3)]
                self.v[1] = s32(self.v[1] - self.g)
                self.yaw = (self.yaw + SPIN) & 63
                if self.tta > -128:
                    self.tta -= 1
                if self.arc != A_FREE:
                    self.arc_left -= 1
                    if self.arc_left == 0:
                        sr = self.side(self.recv)
                        if self.arc == A_SERVE1:
                            self.p = list(self.tgt)
                            self.launch(A_SERVE2, [self.bx, q16(Y_BOUNCE), sr * q16(Z_SERVE_B2)])
                        elif self.arc in (A_SERVE2, A_RALLY):
                            self.p = list(self.tgt)
                            self.launch(A_TOHIT, [self.hx, q16(Y_HIT), sr * q16(Z_HIT)])
                        else:
                            self.arc = A_FREE
            # d) point over
            if self.miss and self.tta <= -MISS_FRAMES:
                self.vis = 0
                self.score[self.hitter] += 1
                self.state = S_POINT
                self.wait = 0

        elif st == S_POINT:
            self.vis = 0
            self.wait += 1
            if self.wait == POINT_PAUSE:
                a, b = self.score
                if (a >= WIN_POINTS or b >= WIN_POINTS) and abs(a - b) >= 2:
                    self.winner = 0 if a > b else 1
                    self.state = S_OVER
                else:
                    n = a + b
                    if a >= WIN_POINTS - 1 and b >= WIN_POINTS - 1:
                        self.server = self.first_server ^ (n & 1)
                    else:
                        self.server = self.first_server ^ ((n >> 1) & 1)
                    self.state = S_SERVE
                    self.reset_bob()
                    self.hold_ball()
                self.wait = 0

        elif st == S_OVER:
            self.vis = 0
            if self.wait < 255:
                self.wait += 1
            if auto[0] and auto[1] and self.wait == OVER_PAUSE:
                self.new_game()
                return self.poses()

        # racket base x follows the incoming ball while the racket is idle
        step = q8(TRACK_STEP)
        for p in (0, 1):
            if not self.sw_act[p]:
                tgt = (self.hx >> 8) if (self.state == S_FLIGHT and p == self.recv and not self.miss) else 0
                d = tgt - self.base_x[p]
                if abs(d) <= step:
                    self.base_x[p] = tgt
                else:
                    self.base_x[p] += step if d > 0 else -step
        return self.poses()

    def racket_pose(self, p):
        f = self.sw_frame[p] if self.sw_act[p] else 0
        tx, ty, tz, yaw = SWING[self.sw_type[p] if self.sw_act[p] else 0][f]
        if p == 0:
            return (self.base_x[0] + tx, ty, tz, yaw, 1)
        return (self.base_x[1] - tx, ty, -tz, yaw, 1)

    def poses(self):
        ball = (self.p[0] >> 8, self.p[1] >> 8, self.p[2] >> 8, self.yaw, self.vis)
        return [self.racket_pose(0), self.racket_pose(1), ball]

    def log_line(self):
        r0, r1, b = self.poses()
        f = lambda t: " ".join(str(v) for v in t)
        return (f"{self.frame} st {self.state} sc {self.score[0]} {self.score[1]} srv {self.server} "
                f"tta {self.tta} | P {f(r0)} | O {f(r1)} | B {f(b)}")


# ----------------------------------------------------------------------------
# Scenarios
# ----------------------------------------------------------------------------
def run(frames, script=None, auto=(1, 1), newgame_at=()):
    """script(frame, game) -> press [[f,b],[f,b]] or None."""
    g = Game()
    out = []
    for k in range(frames):
        press = script(k, g) if script else [[0, 0], [0, 0]]
        g.step(press, newgame=(k in newgame_at), auto=auto)
        out.append((g.log_line(), g.poses(), g.state, list(g.score)))
    return g, out


HUMAN_SEQ = [9, 4, 15, 9, 20, 9, 3, 2, 16, 8, 17, 6]   # good, late edge, early edge, good, too early, ...


def human_script(k, g, _n=[0]):
    """P1 (near) by 'buttons': serves after 30 frames, returns at the frames-to-
    arrival listed in HUMAN_SEQ in turn (in window 3..16 = hit, else miss);
    P2 is the computer."""
    if k == 0:
        _n[0] = 0
    press = [[0, 0], [0, 0]]
    if g.state == S_SERVE and g.server == 0 and g.wait == 30:
        press[0] = [0, 1]
    if g.state == S_FLIGHT and g.recv == 0 and not g.swung and not g.miss:
        if g.tta == HUMAN_SEQ[_n[0] % len(HUMAN_SEQ)]:
            press[0] = [1, 0] if (g.hx >= 0) else [0, 1]
            _n[0] += 1
    return press


def deuce_script(k, g):
    """Both players by buttons: the server serves after 20 frames (P1 forehand,
    P2 backhand), the receiver never returns -> the server wins every point:
    tests the serve rotation into and through deuce (10-10 and beyond)."""
    press = [[0, 0], [0, 0]]
    if g.state == S_SERVE and g.wait == 20:
        press[g.server] = [1, 0] if g.server == 0 else [0, 1]
    return press


# ----------------------------------------------------------------------------
# Checks: on screen, depth, render time
# ----------------------------------------------------------------------------
def load_scene(scene_dir):
    with open(os.path.join(scene_dir, "matrix.mem")) as fh:
        words = [int(l, 16) for l in fh.read().split()]
    words = words[::-1]
    sgn = lambda w: (w - (1 << 32) if w & 0x80000000 else w) / G.Q
    mq = [[sgn(words[r * 4 + c]) for c in range(4)] for r in range(4)]
    tris = []
    with open(os.path.join(scene_dir, "vertices.mem")) as fh:
        lines = [l.strip() for l in fh if l.strip()]
    return mq, lines


def static_tris(lines, n_static):
    tris = []
    for t in range(n_static // 3):
        pts = []
        for k in range(3):
            l = lines[3 * t + k]
            v = [int(l[i * 8:(i + 1) * 8], 16) for i in range(3)]
            v = [(x - (1 << 32) if x & 0x80000000 else x) / G.Q for x in v]
            pts.append(v)
            col = int(l[32:36], 16)
        tris.append((pts[0], pts[1], pts[2], col))
    return tris


def object_tris(rom, poses):
    verts = animate(rom, poses)
    tris = []
    for t in range(len(verts) // 3):
        a, b, c = verts[3 * t:3 * t + 3]
        tris.append(([v / G.Q for v in a[:3]], [v / G.Q for v in b[:3]], [v / G.Q for v in c[:3]], a[3]))
    return tris, verts


def check_frame(mq, stat, rom, poses):
    tris, verts = object_tris(rom, poses)
    worst = 1e9
    for x, y, z, _ in verts:
        sx, sy, nz, w = G.to_screen(mq, [x / G.Q, y / G.Q, z / G.Q])
        m = min(sx, G.SCREEN_W - sx, sy, G.SCREEN_H - sy)
        worst = min(worst, m)
        if w <= 0 or abs(nz) >= 1.0:
            worst = -1e9
    _, cost, _, _ = G.analyse(mq, stat + tris)
    return worst, cost


def write_vh(path, rom):
    L = []
    a = L.append
    a("// tt_game_map.vh - generated by tt_game_model.py, do not edit")
    a("// Gameplay constants, swing tables, sin/cos table and object ROM.")
    a("// Units: positions Q16.16 world units (1 unit = 10 cm) unless noted; frames at 60 Hz.")
    a("")
    a(f"localparam TT_OBJ_VERTS   = {len(rom)};   // object ROM entries (rackets + ball)")
    a(f"localparam TT_SW_LEN      = {SW_LEN};")
    a(f"localparam TT_SW_CONTACT  = {SW_CONTACT};")
    a(f"localparam TT_W_LO        = {W_LO};    // press accepted when frames-to-arrival in [W_LO, W_HI]")
    a(f"localparam TT_W_HI        = {W_HI};")
    a(f"localparam TT_MISS_FRAMES = {MISS_FRAMES};")
    a(f"localparam TT_SPIN        = {SPIN};")
    a(f"localparam TT_POINT_PAUSE = {POINT_PAUSE};")
    a(f"localparam TT_SERVE_AUTO  = {SERVE_AUTO};")
    a(f"localparam TT_OVER_PAUSE  = {OVER_PAUSE};")
    a(f"localparam TT_WIN_POINTS  = {WIN_POINTS};")
    a(f"localparam TT_AI_MISS_T   = {AI_MISS_T};")
    a(f"localparam TT_BOB_HALF    = {BOB_HALF};")
    a(f"localparam [15:0] TT_LFSR_SEED = 16'h{LFSR_SEED:04X};")
    a(f"localparam [15:0] TT_LFSR_TAPS = 16'h{LFSR_TAPS:04X};")
    sd = lambda v, w=32: ("-" if v < 0 else "") + f"{w}'sd{abs(v)}"
    a(f"localparam signed [31:0] TT_Y_BOUNCE  = {sd(q16(Y_BOUNCE))};")
    a(f"localparam signed [31:0] TT_Y_HIT     = {sd(q16(Y_HIT))};")
    a(f"localparam signed [31:0] TT_Z_HIT     = {sd(q16(Z_HIT))};")
    a(f"localparam signed [31:0] TT_Z_SB1     = {sd(q16(Z_SERVE_B1))};")
    a(f"localparam signed [31:0] TT_Z_SB2     = {sd(q16(Z_SERVE_B2))};")
    a(f"localparam signed [31:0] TT_Z_RB      = {sd(q16(Z_RALLY_B))};")
    a(f"localparam signed [31:0] TT_HOLD_Y    = {sd(q16(HOLD_Y))};")
    a(f"localparam signed [31:0] TT_BOB_STEP  = {sd(q16(BOB / BOB_HALF))};")
    a(f"localparam signed [15:0] TT_TRACK_STEP = {sd(q8(TRACK_STEP), 16)};   // Q8.8")
    a("")
    a("// arcs: kind 1 SERVE1, 2 SERVE2, 3 RALLY, 4 TOHIT -> frames N, gravity g, 65536/N, g*(N-1)/2")
    for k in (A_SERVE1, A_SERVE2, A_RALLY, A_TOHIT):
        n, g, rn, gh = ARC_C[k]
        name = [x for x, v in ARC_ID.items() if v == k][0]
        a(f"localparam TT_N_{name} = {n};  localparam signed [31:0] TT_G_{name} = {sd(g)};  "
          f"localparam [15:0] TT_RN_{name} = 16'd{rn};  localparam signed [31:0] TT_GH_{name} = {sd(gh)};")
    a("")
    a("// lateral positions, Q16.16: bounce x by lfsr[2:0], hit x by lfsr[2:0] and lfsr[4:3]")
    a("function signed [31:0] tt_bx;  input [2:0] i;  begin")
    a("    case (i)")
    for i, v in enumerate(BX):
        a(f"        3'd{i}: tt_bx = {sd(q16(v))};")
    a("    endcase  end endfunction")
    a("function signed [31:0] tt_hx;  input [4:0] i;  begin")
    a("    case (i)")
    for i in range(32):
        a(f"        5'd{i}: tt_hx = {sd(q16(BX[i & 7] + HXD[(i >> 3) & 3]))};")
    a("    endcase  end endfunction")
    a("// computer player press time (frames to arrival) by lfsr[7:5]")
    a("function [5:0] tt_ai_t;  input [2:0] i;  begin")
    a("    case (i)")
    for i, j in enumerate(AI_JIT):
        a(f"        3'd{i}: tt_ai_t = 6'd{SW_CONTACT + j};")
    a("    endcase  end endfunction")
    a("")
    a("// swing tables (near player): {x Q8.8 offset from base, y Q8.8, z Q8.8, yaw 6b}")
    a("// type 0 forehand, 1 backhand; far player mirrors x and z")
    a("function [53:0] tt_swing;  input type_b;  input [4:0] f;  begin")
    a("    case ({type_b, f})")
    for t in (0, 1):
        for f, (x, y, z, yaw) in enumerate(SWING[t]):
            a(f"        6'd{t * 32 + f}: tt_swing = {{16'h{x & 0xFFFF:04X}, 16'h{y & 0xFFFF:04X}, "
              f"16'h{z & 0xFFFF:04X}, 6'd{yaw}}};")
    a(f"        default: tt_swing = {{16'h{SWING[0][0][0] & 0xFFFF:04X}, 16'h{SWING[0][0][1] & 0xFFFF:04X}, "
      f"16'h{SWING[0][0][2] & 0xFFFF:04X}, 6'd0}};")
    a("    endcase  end endfunction")
    a("")
    a("// yaw table: {cos, sin} Q1.14, 64 steps per turn")
    a("function [31:0] tt_sincos;  input [5:0] k;  begin")
    a("    case (k)")
    for k, (c, s) in enumerate(SINCOS):
        a(f"        6'd{k}: tt_sincos = {{16'h{c & 0xFFFF:04X}, 16'h{s & 0xFFFF:04X}}};")
    a("    endcase  end endfunction")
    a("")
    a("// object ROM: {object id 2b, local x, y, z Q8.8, RGB565}; order = vertices.mem")
    a("function [65:0] tt_obj_rom;  input [7:0] i;  begin")
    a("    case (i)")
    for i, (oid, x, y, z, col) in enumerate(rom):
        a(f"        8'd{i}: tt_obj_rom = {{2'd{oid}, 16'h{x & 0xFFFF:04X}, 16'h{y & 0xFFFF:04X}, "
          f"16'h{z & 0xFFFF:04X}, 16'h{col:04X}}};")
    a("        default: tt_obj_rom = 66'd0;")
    a("    endcase  end endfunction")
    with open(path, "w") as fh:
        fh.write("\n".join(L) + "\n")


def gif(mq, stat, rom, frames, path, horizon, every=2, scale=2):
    import numpy as np
    from PIL import Image
    imgs = []
    tmp = path + ".tmp.png"
    for poses, score in frames[::every]:
        tris, _ = object_tris(rom, poses)
        allt = stat + tris
        scr, _, _, _ = G.analyse(mq, allt)
        G.render_preview(scr, allt, horizon, tmp)
        im = Image.open(tmp).convert("RGB")
        im = im.resize((im.width * scale, im.height * scale), Image.NEAREST)
        from PIL import ImageDraw
        d = ImageDraw.Draw(im)
        d.rectangle([im.width - 78, 6, im.width - 8, 26], fill=(17, 34, 68))
        d.text((im.width - 72, 10), f"{score[0]:02d} - {score[1]:02d}", fill=(255, 255, 255))
        imgs.append(im.convert("P", palette=Image.ADAPTIVE))
    os.remove(tmp)
    imgs[0].save(path, save_all=True, append_images=imgs[1:], duration=int(1000 / 60 * every), loop=0)


def check_sim(path):
    """Check frames.txt written by tb_tt_game.v: object vertices in vertex
    memory == animate(captured poses), render finished before VGA line 40."""
    rom = object_rom()
    s16 = lambda h: (int(h, 16) ^ 0x8000) - 0x8000
    recs, cur = [], None
    for line in open(path):
        t = line.split()
        if t[0] == "F":
            cur = {"hdr": dict(zip(t[1::2], t[2::2])), "V": [], "F": int(t[1])}
            recs.append(cur)
        elif t[0] == "P":
            v = t[1:]
            cur["poses"] = [(s16(v[0]), s16(v[1]), s16(v[2]), int(v[3], 16), 1),
                            (s16(v[4]), s16(v[5]), s16(v[6]), int(v[7], 16), 1),
                            (s16(v[8]), s16(v[9]), s16(v[10]), int(v[11], 16), int(v[12], 16))]
        else:
            cur["V"].append(t[1])
    ok = bool(recs)
    for r in recs:
        h = r["hdr"]
        exp = [f"{x & 0xFFFFFFFF:08x}{y & 0xFFFFFFFF:08x}{z & 0xFFFFFFFF:08x}00000100{c:04x}"
               for x, y, z, c in animate(rom, r["poses"])]
        vm = r["V"] == exp
        rc = int(h["render_cycles"])
        print(f"frame {r['F']:3d}: vertex memory {'== model' if vm else 'MISMATCH'}, last render {rc} clk = "
              f"{rc / 1e5:.3f} ms, busy at line 40 {h['busy']}, overrun {h['overrun']}, "
              f"renders {h['frames_rendered']}, score {h['score']}, state {h['state']}")
        if not vm or h["overrun"] != "0" or h["busy"] != "0" or rc > 397000:
            ok = False
    print("SYSTEM CHECK", "PASSED" if ok else "FAILED")
    return ok


def main():
    if len(sys.argv) == 3 and sys.argv[1] == "--check-sim":
        sys.exit(0 if check_sim(sys.argv[2]) else 1)
    ap = argparse.ArgumentParser()
    ap.add_argument("--scene", required=True, help="folder with vertices.mem, matrix.mem, scene_info")
    ap.add_argument("--out", required=True)
    ap.add_argument("--frames", type=int, default=3000)
    ap.add_argument("--long-frames", type=int, default=40000)
    ap.add_argument("--gif", action="store_true")
    ap.add_argument("--gif-frames", type=int, default=420)
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)

    rom = object_rom()
    write_vh(os.path.join(args.out, "tt_game_map.vh"), rom)

    # net clearance of the arcs (straight line + bump), metres
    net_top = Y_TABLE + G.NET_H + BALL_HALF
    def arc_y_at_net(z0, y0, z1, y1, kind):
        n, g, rn, gh = ARC_C[kind]
        k = n * z0 / (z0 - z1)
        return (y0 + (y1 - y0) * k / n) + (g * k * (n - k) / 2) / (U * Q16)
    print("net clearance (ball centre above net top + half ball), m:")
    print(f"  rally : {arc_y_at_net(Z_HIT, Y_HIT, -Z_RALLY_B, Y_BOUNCE, A_RALLY) - net_top:+.3f}")
    print(f"  serve : {arc_y_at_net(Z_SERVE_B1, Y_BOUNCE, -Z_SERVE_B2, Y_BOUNCE, A_SERVE2) - net_top:+.3f}")

    mq, lines = load_scene(args.scene)
    vh = open(os.path.join(args.scene, "tt_scene_map.vh")).read()
    n_static = int(vh.split("TT_RACKET_P_BASE = ")[1].split(";")[0])
    stat = static_tris(lines, n_static)

    runs = {
        "auto": (run(args.frames, auto=(1, 1)), True),
        "human": (run(args.frames, script=human_script, auto=(0, 1)), True),
        # long computer-vs-computer run: full games, deuce, game over, auto
        # restart, plus a new-game button press at frame 700 (log only)
        "long": (run(args.long_frames, auto=(1, 1), newgame_at=(700,)), False),
        "deuce": (run(6000, script=deuce_script, auto=(0, 0)), False),
    }
    ok = True
    for name, ((g, out), check) in runs.items():
        with open(os.path.join(args.out, f"golden_{name}.log"), "w") as fh:
            fh.write("\n".join(l for l, _, _, _ in out) + "\n")
        overs = sum(1 for a, b in zip(out, out[1:]) if a[2] != S_OVER and b[2] == S_OVER)
        if not check:
            print(f"{name}: {len(out)} frames, final score {g.score}, games finished {overs} (log only)")
            continue
        worst_m, worst_c, seen = 1e9, 0, set()
        for line, poses, st, sc in out:
            key = tuple(poses[0]) + tuple(poses[1]) + tuple(poses[2])
            if key in seen:
                continue
            seen.add(key)
            m, c = check_frame(mq, stat, rom, poses)
            worst_m = min(worst_m, m)
            worst_c = max(worst_c, c)
        t_ms = (worst_c + G.CLEAR_CYCLES) / G.F_CLK * 1e3
        print(f"{name}: {len(out)} frames, final score {g.score}, unique poses {len(seen)}, "
              f"min on-screen margin {worst_m:.1f} px, worst raster cost {worst_c} cyc "
              f"-> ~{t_ms:.2f} ms with clear (gap 3.97 ms)")
        if worst_m < 1.0 or t_ms > 3.6:
            ok = False
    if args.gif:
        (g, out), _ = runs["auto"]
        sel = [(p, s) for _, p, _, s in out[:args.gif_frames]]
        horizon = int(open(os.path.join(args.scene, "tt_scene_map.vh")).read().split("TT_HORIZON_Y = ")[1].split(";")[0])
        gif(mq, stat, rom, sel, os.path.join(args.out, "rally_preview.gif"), horizon)
        print("rally_preview.gif written")
    print("CHECKS", "PASSED" if ok else "FAILED")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()

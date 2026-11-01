#!/usr/bin/env python3
"""Checks the output of tb_cpu_top (frames.log + cpu_frame_<n>.ppm).

  * frames with the original matrix.mem (power-up frame 0 and the CPU's
    frame 1) are pixel-identical to cube_top's frame 0
  * every other matrix is exactly what the firmware's fixed-point code gives
    for some (yaw, pitch) pair - recomputed here bit-exactly in Python - and
    within 1 LSB (Q24.8) of the ideal floating-point matrix
  * VGA-synchronised frames start when the scan leaves the framebuffer and
    finish inside the off-screen window (tear-free)
  * every frame draws something, and the animation actually moves

usage: check_frames.py frames.log [golden.ppm]
  golden.ppm = cube_000.ppm from the team's tb_cube (cube_top power-up frame,
  matrix.mem); without it the image check of the original-matrix frames is
  skipped.
"""
import math, os, re, sys

ORIG = [0x1e4, 0, 0x153, 0, 0x6b, 0x192, -0x9a, 0, 0x137, -0xfc, -0x1bb, 0xab,
        0x85, -0x6c, -0xbe, 0x600]

# ---- the firmware's fixed-point model (must match main.c exactly) ----
SIN = [round(65536 * math.sin(i * 2 * math.pi / 1024)) for i in range(257)]
P00, P11, P22, T2, DIST = 151349, 113512, -152917, 43691, 393216

def isin(a):
    a &= 1023; q, i = a >> 8, a & 255
    return [SIN[i], SIN[256 - i], -SIN[i], -SIN[256 - i]][q]

def icos(a): return isin(a + 256)
def mul16(a, b): return (a * b + 32768) >> 16
def q16_to_q8(v): return (v + 128) >> 8
def mul_to_q8(p, r): return (p * r + (1 << 23)) >> 24

def build(yaw, pitch):
    sy, cy, sp, cp = isin(yaw), icos(yaw), isin(pitch), icos(pitch)
    r0 = [cy, 0, sy]
    r1 = [mul16(sp, sy), cp, -mul16(sp, cy)]
    r2 = [-mul16(cp, sy), sp, mul16(cp, cy)]
    m = [0] * 16
    for c in range(3):
        m[c] = mul_to_q8(P00, r0[c]); m[4 + c] = mul_to_q8(P11, r1[c])
        m[8 + c] = mul_to_q8(P22, r2[c]); m[12 + c] = q16_to_q8(-r2[c])
    m[11] = q16_to_q8(T2); m[15] = q16_to_q8(DIST)
    return m

def ideal(yaw, pitch):
    """float reference: P * T(0,0,-6) * Rx(pitch) * Ry(yaw)"""
    y, p = yaw * 2 * math.pi / 1024, pitch * 2 * math.pi / 1024
    f = 1 / math.tan(math.radians(30)); a = 300 / 400; n, fa, d = 4.0, 10.0, 6.0
    r0 = [math.cos(y), 0, math.sin(y)]
    r1 = [math.sin(p) * math.sin(y), math.cos(p), -math.sin(p) * math.cos(y)]
    r2 = [-math.cos(p) * math.sin(y), math.sin(p), math.cos(p) * math.cos(y)]
    A, Bc = (fa + n) / (n - fa), 2 * fa * n / (n - fa)
    m = [f / a * v for v in r0] + [0] + [f * v for v in r1] + [0] + \
        [A * v for v in r2] + [A * -d + Bc] + [-v for v in r2] + [d]
    return [v * 256 for v in m]

def s32(h):
    v = int(h, 16)
    return v - (1 << 32) if v & 0x80000000 else v

def load_ppm(path):
    t = open(path).read().split()
    return t[4:]

def main():
    log = sys.argv[1]
    golden = sys.argv[2] if len(sys.argv) > 2 else None
    base = os.path.dirname(os.path.abspath(log))
    errors = 0
    def fail(msg):
        nonlocal errors; errors += 1; print("FAIL ", msg)

    golden_px = load_ppm(golden) if golden and os.path.exists(golden) else None
    WINDOW = 397_200          # 125 lines off screen at 25.175 MHz, in 100 MHz cycles
    prev_px, prev_pose = None, None
    frames = [l for l in open(log) if l.startswith("frame")]
    for line in frames:
        mt = re.match(r"frame (\d+) sync=(\d) start_line=(\d+) draw_cycles=(\d+) M=(.*)", line)
        n, sync, sline, cyc = map(int, mt.groups()[:4])
        M = [s32(h) for h in mt.group(5).split()]
        px = load_ppm(os.path.join(base, f"cpu_frame_{n}.ppm"))
        lit = sum(1 for i in range(0, len(px), 3) if px[i:i+3] != ["0", "0", "0"])
        note = []

        if M == ORIG:
            note.append("original matrix")
            if prev_pose == "orig": note.append("(same pose)")
            prev_pose = "orig"
            if golden_px is not None:
                if px == golden_px: note.append("image == golden")
                else: fail(f"frame {n}: original matrix but image differs from golden")
        else:
            yaw = round(math.atan2(M[2], M[0]) * 1024 / (2 * math.pi)) & 1023
            # row 3 = (cos p sin y, -sin p, -cos p cos y) * 256: |cos p| only,
            # so try p and 180 deg - p
            p0 = round(math.atan2(-M[13], math.hypot(M[12], M[14])) * 1024 / (2 * math.pi))
            hit = None
            for pitch in (p0 & 1023, (512 - p0) & 1023):
                for dy in range(-2, 3):
                    for dp in range(-2, 3):
                        if build((yaw + dy) & 1023, (pitch + dp) & 1023) == M:
                            hit = ((yaw + dy) & 1023, (pitch + dp) & 1023)
            if hit is None:
                fail(f"frame {n}: matrix is not a firmware rotation matrix: {M}")
            else:
                err = max(abs(a - b) for a, b in zip(M, ideal(*hit)))
                note.append(f"yaw {hit[0]*360/1024:6.1f} pitch {hit[1]*360/1024:5.1f} deg, max err {err:.2f} LSB")
                if err > 1.0: fail(f"frame {n}: fixed-point error {err:.2f} LSB > 1")
                if prev_pose == hit: note.append("(same pose)")
                prev_pose = hit

        if sync:
            if sline != 440: fail(f"frame {n}: sync frame started at line {sline}, expected 440")
            if cyc >= WINDOW: fail(f"frame {n}: {cyc} cycles does not fit the {WINDOW}-cycle off-screen window")
        if lit == 0: fail(f"frame {n}: empty image")
        if prev_px is not None and px == prev_px and "(same pose)" not in note:
            fail(f"frame {n}: image identical to previous frame although the pose changed")
        prev_px = px
        print(f"frame {n:2d} sync={sync} line={sline:3d} draw={cyc/100:7.1f} us lit={lit:6d}  " + ", ".join(note))

    if not frames: fail("no frames in log")
    print("ALL CHECKS PASS" if errors == 0 else f"{errors} CHECK(S) FAILED")
    sys.exit(1 if errors else 0)

if __name__ == "__main__":
    main()

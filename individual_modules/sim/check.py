#!/usr/bin/env python3
"""Check triangle setup outputs against exact rational math.

usage: check.py <out.txt> [--max-depth-err UNITS]
Exits non-zero if any edge/bbox/flag result is wrong, or if the worst depth
error at the vertices of ordinary triangles exceeds --max-depth-err.
"""
import sys
from fractions import Fraction as F

def s(v, bits):
    v &= (1 << bits) - 1
    return v - (1 << bits) if v >> (bits - 1) else v

args = sys.argv[1:]
outfile = args[0]
max_err = float(args[args.index("--max-depth-err") + 1]) if "--max-depth-err" in args else None

words = [int(l, 16) for l in open("stim.hex")]
outs = [l.split() for l in open(outfile)]
XM, YM = 76800, 102400
cnt = dict(tri=0, degenerate=0, orient_neg=0, ovA=0, ovB=0, ovC=0, xy_range=0, bbox_clamped=0)
fails, ord_err = [], []

for i, o in enumerate(outs):
    raw = [s(w, 32) for w in words[i*10:i*10+9]]
    xs, ys, zs = raw[0::3], raw[1::3], raw[2::3]
    bbox = tuple(int(o[k], 16) for k in range(4))
    E = [(s(int(o[4+3*k], 16), 32), s(int(o[5+3*k], 16), 32), s(int(o[6+3*k], 16), 34)) for k in range(3)]
    zA, zB, zC = (s(int(o[k], 16), 32) for k in (13, 14, 15))
    ov, ovxy, ovec, ova, ovb, ovc, ovd, bcl, oneg, deg = (c == '1' for c in o[17])
    for k, b in (("tri", 1), ("degenerate", deg), ("orient_neg", oneg), ("ovA", ova), ("ovB", ovb),
                 ("ovC", ovc), ("xy_range", ovxy), ("bbox_clamped", bcl)):
        cnt[k] += b
    cx = [min(max(v, 0), XM) for v in xs]
    cy = [min(max(v, 0), YM) for v in ys]
    if ovxy != (cx + cy != xs + ys): fails.append((i, "xy flag"))
    D = (cx[1]-cx[0])*(cy[2]-cy[0]) - (cx[2]-cx[0])*(cy[1]-cy[0])
    if deg != (D == 0): fails.append((i, "degenerate"))
    if oneg != (D < 0): fails.append((i, "orient"))
    for k in range(3):
        a, b = k, (k + 1) % 3
        if E[k] != (cy[a]-cy[b], cx[b]-cx[a], cx[a]*cy[b]-cx[b]*cy[a]): fails.append((i, f"edge{k}"))
    want = (max(0, min(299, min(cx) >> 8)), max(0, min(299, -((-max(cx)) >> 8))),
            max(0, min(399, min(cy) >> 8)), max(0, min(399, -((-max(cy)) >> 8))))
    if bbox != want: fails.append((i, "bbox"))
    if ov != (ovxy or ovec or ova or ovb or ovc) or ovd != (ova or ovb or ovc): fails.append((i, "ov OR"))
    if deg and (zA, zB, zC) != (0, 0, 0): fails.append((i, "degenerate z"))
    # depth accuracy on ordinary in-range triangles (stimulus categories 0-3)
    if i % 10 <= 3 and not deg and not ovd:
        for k in range(3):
            ord_err.append(abs(float(F(zA*cx[k] + zB*cy[k], 256) + zC - zs[k])) / 256)

print("coverage:", cnt)
print("functional mismatches:", len(fails), fails[:5])
ord_err.sort()
worst = ord_err[-1]
print(f"ordinary-triangle depth error at vertices (depth units): "
      f"median {ord_err[len(ord_err)//2]:.3f}  max {worst:.3f}")
ok = not fails and (max_err is None or worst <= max_err)
if max_err is not None:
    print(f"depth error limit {max_err}: {'PASS' if worst <= max_err else 'FAIL'}")
sys.exit(0 if ok else 1)

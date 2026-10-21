#!/usr/bin/env python3
"""Stimulus for tb_triangle_setup.v: N triangles, 10 x 32-bit hex words each."""
import random, sys

N = int(sys.argv[1]) if len(sys.argv) > 1 else 4000
random.seed(12345)
XM, YM = 76800, 102400           # 300.0, 400.0 in Q24.8
tris = []

def z_any():  return random.randint(-(1 << 31), (1 << 31) - 1)
def z_norm(): return random.randint(0, 65535 << 8)

def add(x0,y0,z0,x1,y1,z1,x2,y2,z2,c=None):
    tris.append([x0,y0,z0,x1,y1,z1,x2,y2,z2, random.getrandbits(16) if c is None else c])

while len(tris) < N:
    k = len(tris) % 10
    if k <= 3:                                  # ordinary in-range triangles
        add(*(v for _ in range(3) for v in (random.randint(0,XM), random.randint(0,YM), z_norm())))
    elif k == 4:                                # small triangles near a vertex
        cx, cy = random.randint(0,XM), random.randint(0,YM)
        p = [(min(XM,max(0,cx+random.randint(-2000,2000))),
              min(YM,max(0,cy+random.randint(-2000,2000))), z_norm()) for _ in range(3)]
        add(*(v for t in p for v in t))
    elif k == 5:                                # exact viewport corners / edges
        pts = [(0,0),(XM,0),(0,YM),(XM,YM),(XM,YM//2),(XM//2,0)]
        p = random.sample(pts,3)
        add(*(v for t in p for v in (t[0],t[1],z_norm())))
    elif k == 6:                                # degenerate: collinear / repeated
        x0,y0 = random.randint(0,XM//2), random.randint(0,YM//2)
        dx,dy = random.randint(0,200), random.randint(0,200)
        add(x0,y0,z_norm(), x0+dx,y0+dy,z_norm(), x0+2*dx,y0+2*dy,z_norm())
    elif k == 7:                                # slivers -> tiny |D|, huge zA/zB
        x0,y0 = random.randint(0,XM-300), random.randint(0,YM-300)
        add(x0,y0,z_any(), x0+random.randint(1,255),y0+random.randint(0,2),z_any(),
            x0+random.randint(0,3),y0+random.randint(1,255),z_any())
    elif k == 8:                                # extreme z values
        add(*(v for _ in range(3) for v in (random.randint(0,XM), random.randint(0,YM), z_any())))
    else:                                       # out-of-contract x/y (debug flags)
        def bad(m): return random.choice([random.randint(-(1<<31), -1),
                                          random.randint(m+1, (1<<31)-1),
                                          random.randint(0, m)])
        add(bad(XM),bad(YM),z_norm(), bad(XM),bad(YM),z_norm(), bad(XM),bad(YM),z_norm())

with open("stim.hex","w") as f:
    for t in tris[:N]:
        for v in t: f.write(f"{v & 0xffffffff:08x}\n")

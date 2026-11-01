#!/usr/bin/env python3
"""poses.mem for tb_pose_sweep: N poses spread over the full yaw/pitch range,
computed with the firmware's fixed-point code (model in check_frames.py),
written in matrix_memory layout (word k = row-major element 15-k)."""
import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from check_frames import build
n = int(sys.argv[1]) if len(sys.argv) > 1 else 24
with open("poses.mem", "w") as f, open("poses.txt", "w") as t:
    for i in range(n):
        # golden-ratio steps: no two poses repeat and every face / face
        # combination shows up (a cube is symmetric under 90 deg turns, so
        # steps that are simple fractions of a turn would repeat poses)
        yaw = (37 + i * 633) & 1023                 # 633/1024 ~ 0.618 turn
        pitch = (11 + i * 391) & 1023               # 391/1024 ~ 0.382 turn
        m = build(yaw, pitch)
        for k in range(16):
            f.write(f"{m[15 - k] & 0xffffffff:08x}\n")
        t.write(f"pose {i} yaw {yaw*360/1024:.1f} pitch {pitch*360/1024:.1f}\n")
print(f"{n} poses")

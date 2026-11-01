#!/bin/sh
# Pipeline-only pose sweep (no CPU), Icarus Verilog >= 11:
#   POSES (default 24) poses over the full rotation range, matrices from the
#   firmware's fixed-point code, loaded through the pipeline's re-render ports
#   -> unit/pose_<n>.ppm, unit/poses.log
set -e
HERE=$(cd "$(dirname "$0")" && pwd)
ROOT=$HERE/../..
PIPE="$ROOT/hardware_pipeline"
POSES=${POSES:-24}
mkdir -p "$HERE/unit"
cd "$HERE/unit"
cp "$ROOT/mem_files/vertices.mem" "$ROOT/mem_files/matrix.mem" "$ROOT/mem_files/recip_lut_q1_17.mem" .
python3 "$HERE/gen_poses.py" "$POSES"
iverilog -g2005 -o tb_pose_sweep.vvp "$HERE/tb_pose_sweep.v" \
    "$PIPE/hardware_pipeline.v" "$PIPE/matrix_memory.v" "$PIPE/vertex_memory.v" \
    "$PIPE/transform_engine.v" "$PIPE/projected_engine.v" "$PIPE/triangle_setup_unit_bbox.v" \
    "$PIPE/rasterizer.v" "$PIPE/z_f_buffer.v" "$PIPE/vga_controller.v"
vvp -n tb_pose_sweep.vvp +POSES="$POSES" | grep -E "pose|SWEEP|FAIL"

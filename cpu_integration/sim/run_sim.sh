#!/bin/sh
# Full-system simulation with Icarus Verilog (>= 11).
#   ./run_sim.sh [frames]      -> build/  (uart.log, frames.log, cpu_frame_*.ppm)
# Needs firmware/firmware.hex (cd ../firmware && make).
set -e
HERE=$(cd "$(dirname "$0")" && pwd)
ROOT=$HERE/../..
FRAMES=${1:-12}
mkdir -p "$HERE/build"
cd "$HERE/build"
cp "$ROOT/mem_files/vertices.mem" "$ROOT/mem_files/matrix.mem" "$ROOT/mem_files/recip_lut_q1_17.mem" .
cp "$HERE/../firmware/firmware.hex" .
PIPE="$ROOT/hardware_pipeline"

# 1) reference: the team's rotating cube (cube_top + tb_cube), power-up frame
CUBE="$PIPE/rotating_cube_top_files"
cp "$ROOT/mem_files/rotating_cube/matrices.mem" .
iverilog -g2005 -Ptb_cube.N_DUMP=1 -o tb_cube.vvp "$HERE/xilinx_sim_models.v" \
    "$CUBE/tb_cube.v" "$CUBE/cube_top.v" \
    "$PIPE/hardware_pipeline.v" "$PIPE/matrix_memory.v" "$PIPE/vertex_memory.v" \
    "$PIPE/transform_engine.v" "$PIPE/projected_engine.v" "$PIPE/triangle_setup_unit_bbox.v" \
    "$PIPE/rasterizer.v" "$PIPE/z_f_buffer.v" "$PIPE/vga_controller.v"
vvp -n tb_cube.vvp > tb_cube.out
cp cube_000.ppm golden.ppm
echo "reference frame: build/golden.ppm (cube_top frame 0)"

# 2) CPU + pipeline
iverilog -g2005 -o tb_cpu_top.vvp \
    "$HERE/xilinx_sim_models.v" "$HERE/tb_cpu_top.v" \
    "$HERE/../rtl/nexys_a7_cpu_top.v" "$HERE/../rtl/cpu_subsystem.v" \
    "$HERE/../rtl/gfx_ctrl.v" "$HERE/../rtl/uart.v" "$HERE/../rtl/picorv32.v" \
    "$PIPE/hardware_pipeline.v" "$PIPE/matrix_memory.v" "$PIPE/vertex_memory.v" \
    "$PIPE/transform_engine.v" "$PIPE/projected_engine.v" "$PIPE/triangle_setup_unit_bbox.v" \
    "$PIPE/rasterizer.v" "$PIPE/z_f_buffer.v" "$PIPE/vga_controller.v"
vvp -n tb_cpu_top.vvp +FRAMES="$FRAMES"
python3 "$HERE/check_frames.py" frames.log golden.ppm

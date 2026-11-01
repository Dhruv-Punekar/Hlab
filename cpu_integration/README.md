# CPU integration: PicoRV32 driving the HLab 3D pipeline

A small RISC-V CPU (PicoRV32, RV32IM) now runs the graphics pipeline. Every frame it computes a new
rotation matrix in fixed point and starts one render, so the cube rotates live, tear-free, at 60 fps
on VGA. Buttons, switches and the USB-UART control it.

This is the rotating cube from `hardware_pipeline/rotating_cube_top_files/cube_top.v`, with the
120-entry matrix ROM replaced by a CPU. **`hardware_pipeline/` is not modified.** The CPU uses the
same re-render ports that `cube_top` uses: `rst_render_n`, the `matrix_memory` write port, and
`render_done`.

```
              +----------------------- cpu_subsystem (100 MHz) ------------------------+
 SW / BTN --->|  PicoRV32 RV32IM <--> bus <--> 16 KiB RAM (firmware.hex)                 |
 LED     <----|                       |   --> uart, gpio, timer                          |
 UART   <---->|                       |   --> gfx_ctrl: MATRIX[16], CTRL, STATUS, stats  |
              |                                   frame sequencer (as in cube_top)       |
              +-------------------------------------|---------------------------------- +
   VGA window detector (line 440, from cube_top) ---+  rst_render_n, mat_we/addr/data,
                                                    v  render_done
   hardware_pipeline (unchanged): vertex fetch -> transform -> projection ->
   triangle setup -> rasterizer -> z/frame buffer (RGB444) -> VGA
```

## How a frame works

1. The firmware writes the 16 matrix words to `gfx_ctrl` and sets `CTRL = GO | SYNC`.
2. `gfx_ctrl` waits for the VGA window. The framebuffer is shown only on lines 40–439, so from line
   440 to line 40 of the next frame (125 lines, about 3.97 ms) nobody reads it.
3. It holds the render domain in reset, writes the 16 words into `matrix_memory`, then releases the
   reset. The pipeline clears the z and frame buffers (1.2 ms) and draws the cube (about 0.7 ms).
4. When `render_done` rises, `STATUS.BUSY` drops and the firmware prepares the next frame.

A frame takes about 1.9 ms, so it is finished long before the scan returns to the framebuffer.

## Files

```
cpu_integration/
  rtl/nexys_a7_cpu_top.v    board top: MMCM, reset, VGA window, CPU, pipeline
  rtl/cpu_subsystem.v       PicoRV32 + RAM + bus decode + GPIO + timer
  rtl/gfx_ctrl.v            CPU registers + frame sequencer -> pipeline re-render ports
  rtl/uart.v                115200 8N1 UART
  rtl/picorv32.v            YosysHQ PicoRV32, unmodified (ISC licence, commit ef203c2)
  constraints/nexys_a7_cpu_top.xdc   pins from Digilent's Nexys-A7-100T-Master.xdc
  firmware/                 C firmware (main.c), startup, linker script, Makefile
  firmware/firmware.hex     prebuilt image (so Vivado works without a RISC-V compiler)
  sim/run_sim.sh            full system: CPU + firmware + pipeline, checked by check_frames.py
  sim/run_unit.sh           pipeline only: 24-pose sweep over the whole rotation range
  vivado/build.tcl          batch build: synth, place, route, reports, bitstream
```

## Memory map (see `firmware/hlab.h`)

| Address | Register | |
|---|---|---|
| `0x0000_0000` | RAM 16 KiB | code, data, stack (top of RAM) |
| `0x1000_0000` | `CTRL` | bit0 GO (draw a frame), bit1 SYNC (wait for the VGA window, default 1) |
| `0x1000_0004` | `STATUS` | bit0 BUSY (poll until 0) |
| `0x1000_0008` | `FRAME_COUNT` | frames finished (the power-up render is frame 0) |
| `0x1000_0010` | `FRAME_CYCLES` | last frame, GO accepted → done (includes the VGA wait) |
| `0x1000_0014` | `DRAW_CYCLES` | last frame, clear + draw only |
| `0x1000_0040`–`0x7C` | `MATRIX[16]` | 4×4, **row-major**, signed Q24.8 |
| `0x2000_0000` | UART `DATA` / `+4 STATUS` | |
| `0x3000_0000` | `LED` / `+4 SW` / `+8 BTN` | BTN = {D,R,L,U,C} |
| `0x4000_0000` | `TIMER` lo / `+4` hi | 100 MHz cycle counter |

`matrix.mem` and `matrices.mem` keep their reversed order (M33 first … M00 last). `gfx_ctrl` handles
the conversion, so firmware always uses normal row-major order.

## Firmware

Each frame, the firmware builds `M = P · T(0,0,−6) · Rx(pitch) · Ry(yaw)` in fixed point. The
camera is the same as `gen_rotating_cube.py`: 60° FOV, aspect 0.75, near 4, far 10, distance 6. The
start pose is yaw 35°, pitch 25°, which is `matrix.mem`.

Frame 0 is the pipeline's own power-up render. Frame 1 is the same `matrix.mem` sent through the CPU
path, and it must be pixel-identical. After that the cube rotates.

| Control | Action |
|---|---|
| BTNL / BTNR | yaw speed − / + |
| BTNU / BTND | pitch speed + / − (tumble) |
| BTNC | back to the start pose and speeds |
| SW0 | pause |
| SW1 | free-running, no VGA sync (raw GPU frame rate; may tear) |
| SW15 | hold the original `matrix.mem` frame |
| LED[7:0] | frame counter / 4 |
| LED8 / LED9 | paused / free-running |
| LED14 | GPU busy for more than half a frame |
| LED15 | CPU trap (should never light) |
| UART 115200 | `h` help, `p` pause, `r` reset, `s` stats, `a`/`d` yaw, `w`/`x` pitch; prints fps and draw time once per second |

Rebuild the firmware with `make` (clang + ld.lld) or `make TOOLCHAIN=gcc CROSS=riscv64-unknown-elf-`.

## Simulation (Icarus Verilog ≥ 11, Python 3)

```
cd cpu_integration/sim
./run_sim.sh 12      # ~13 min: full system, 12 frames, ends with "ALL CHECKS PASS"
./run_unit.sh        # ~2 min: 24-pose sweep, pipeline only
```

`run_sim.sh` first runs the team's `tb_cube` to get `cube_top`'s power-up frame as a reference. It
then simulates the CPU system and checks the following:

- frames drawn with `matrix.mem` (the power-up frame and the CPU's first frame) match the reference
  pixel for pixel
- every later matrix is exactly the firmware's fixed-point result for some (yaw, pitch), recomputed
  in Python, and within 1 LSB of the floating-point matrix (measured: ≤ 0.5 LSB)
- synchronised frames start at line 440 and finish inside the off-screen window
- the CPU never traps and the GPU never stalls
- buttons, switches and UART RX all take effect

The Vivado simulator works too: add the same sources and `tb_cpu_top.v`. Leave out
`sim/xilinx_sim_models.v`, because Vivado has the real MMCM and BUFG models.

## Building the bitstream (needs Vivado on Windows or Linux)

```
cd cpu_integration/firmware && make          # only if main.c changed
cd ../vivado && vivado -mode batch -source build.tcl   # -> out/nexys_a7_cpu_top.bit + reports
```

This has **not been through Vivado yet**. It was verified in simulation, plus one open-source synthesis
pass (Yosys `synth_xilinx`, no place and route or timing). That pass gives about 10.6 k LUTs (of 63.4 k),
7.6 k flip-flops, 138 DSP48 (of 240), and 109 RAMB36 + 2 RAMB18 (of 135 RAMB36), so it fits the
XC7A100T with room to spare. Vivado's numbers will differ somewhat. Things to check on the first
build:

1. **Timing at 100 MHz.** PicoRV32 normally closes far above 100 MHz on Artix-7, and the CPU bus is
   fully registered. `build.tcl` stops with an error if WNS or WHS is negative.
2. **Block RAM.** The CPU adds 4 RAMB36. Since the 12-bit framebuffer there is plenty of room. If not,
   `CPU_RAM_BLOCK = 0` moves the CPU RAM into LUTs.
3. `$readmemh` needs `firmware.hex`, `vertices.mem`, `matrix.mem` and `recip_lut_q1_17.mem` in
   Vivado's working directory. `build.tcl` copies them; in a GUI project, add them as design sources.

## Ideas for next steps

- Let the CPU upload new meshes. `vertex_memory` already has a write port, and `NUM_VERTICES` would
  need to become a register.
- Clear only the previous frame's bounding box (the 1.2 ms clear is two thirds of the frame time).
- Have a PC script send matrices or commands over the UART (the design doc's Phase 2).

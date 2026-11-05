# F3GA — Fixed-Function 3D Graphics Accelerator

F3GA is a fixed-function 3D graphics pipeline written entirely in Verilog and implemented on the **Digilent Nexys A7-100T** (Xilinx Artix-7 `xc7a100tcsg324-1`). It takes a 3D model stored as vertices plus a 4×4 transformation matrix, and turns it into shaded, depth-correct pixels on a VGA monitor — **with no CPU and no software in the loop**. Every stage, from vertex fetch to the VGA signal, is dedicated hardware.

The pipeline renders into a **300 × 400** framebuffer, centred in a standard 640 × 480 @ 60 Hz VGA output (RGB444).

---

## Highlights

- **Pure hardware rendering** — vertex fetch, transformation, perspective projection, triangle setup, rasterization and depth testing all run as pipelined RTL.
- **Fixed-point arithmetic throughout** (Q24.8 coordinates, Q16.16 / Q14.18 / Q6.9 depth), mapped onto DSP48 slices instead of costly floating-point units.
- **Valid/ready handshaking** between stages, with backpressure, so every stage runs at 100 MHz.
- **Z-buffering** with signed depth, fast clear and data forwarding.
- **Back-face culling** and discarding of degenerate / out-of-range triangles.
- **Optional double buffering** (two 4-bit palette-index framebuffers) for tear-free animation.
- **Live re-render ports** — the matrix and vertices can be rewritten at runtime, so the scene can be driven from buttons, switches and the on-board accelerometer.
- **One pixel per clock** rasterizer throughput.

---

## Pipeline Architecture

Each stage passes its output to the next:

```
 vertex_memory ─┐
                ├─► transform_engine ─► projection_engine ─► triangle_setup_unit
 matrix_memory ─┘                                                   │
                                                                    ▼
             VGA monitor ◄── vga_controller ◄── z_f_buffer ◄── rasterizer
```

| # | Module | File | Role |
|---|--------|------|------|
| 1 | Vertex Memory | `vertex_memory.v` | Holds model vertices `{X, Y, Z, W, RGB565}` (144-bit words), loaded from `vertices.mem` |
| 1 | Matrix Memory | `matrix_memory.v` | Holds the 4×4 Q24.8 model-view-projection matrix, loaded from `matrix.mem` |
| 2 | Transformation Engine | `transform_engine.v` | Multiplies each vertex by the matrix (vertex → clip space) |
| 3 | Projection Engine | `projected_engine.v` | Perspective divide using a reciprocal ROM, viewport mapping to 300 × 400, groups vertices into triangles |
| 4 | Triangle Setup Unit | `triangle_setup_unit_bbox.v` | Computes edge equations, depth plane and bounding box; culls back-facing/degenerate triangles; uses `recip_lut_q1_17.mem` |
| 5 | Rasterizer | `rasterizer.v` | Walks the bounding box and emits every covered pixel with its depth and colour |
| 6 | Z / Frame Buffer | `z_f_buffer.v` | Depth test + framebuffer write (render clock), dual-ported read for the display (VGA clock) |
| 7 | VGA Controller | `vga_controller.v` | 640 × 480 @ 60 Hz timing, centres the 300 × 400 image, drives the RGB444 DAC |

`hardware_pipeline.v` is the top-level that wires all seven stages together.

### Clocks and reset

| Signal | Frequency / type | Used by |
|--------|------------------|---------|
| `clk` | 100 MHz | Render domain (vertex fetch → z/f buffer write) |
| `clk_vga` | 25 MHz (25.175 MHz from MMCM) | Display domain (z/f buffer read, VGA timing) |
| `rst_n` | Active-low | Resets both domains (mapped to `CPU_RESETN`) |

### Memory initialisation files

| File | Loaded into | Contents |
|------|-------------|----------|
| `vertices.mem` | `vertex_memory` | One 144-bit vertex per line, counter-clockwise triangles |
| `matrix.mem` | `matrix_memory` | 16 × 32-bit Q24.8 words, `M33` first … `M00` last |
| `recip_lut_q1_17.mem` | Triangle setup / projection | 1024 × 18-bit Q1.17 reciprocal lookup table |
| `palette.mem` | `z_f_buffer` | 16-colour palette (only when `DOUBLE_BUF = 1`) |

---

## Demos

Several board-level top modules are built on the same pipeline:

| Demo | Top module | What it shows |
|------|------------|---------------|
| Rotating cube | `cube_top` | A cube animated through 120 precomputed matrices |
| Interactive cube game | `game_top` | Cube controlled live: rotate, move, zoom, auto-spin, speed, colour themes and tilt control via the accelerometer; double buffering, FPS and pixels-per-frame on the 7-segment display |
| City navigation | `nav_top` | Walk through a small 3D city at 60 fps with a mini-map and compass |
| Table tennis | `tt_top` | A playable table-tennis game with moving objects, multiple camera views, score overlay, sound and tilt aiming |
| Static image | `image_top` | A low-poly picture (~1,100 triangles) rendered from a converted image |

> `vertex_memory` and `matrix_memory` load the fixed names `vertices.mem` and `matrix.mem`, so build each demo in its own Vivado project with that demo's memory files.

---

## Repository Structure

```
Hlab/
├── hardware_pipeline/          # Final integrated design
│   ├── hardware_pipeline.v     # Top-level pipeline wiring all stages
│   ├── vertex_memory.v         # Stage 1
│   ├── matrix_memory.v         # Stage 1
│   ├── transform_engine.v      # Stage 2
│   ├── projected_engine.v      # Stage 3
│   ├── triangle_setup_unit_bbox.v  # Stage 4
│   ├── rasterizer.v            # Stage 5
│   ├── z_f_buffer.v            # Stage 6
│   ├── vga_controller.v        # Stage 7
│   ├── rotating_cube_top_files/   # cube_top, testbench, constraints
│   ├── game_files/                # game_top + controls, accelerometer, double buffering, 7-seg, bitstream
│   ├── navigation_top_files/      # nav_top + camera, mini-map overlay, constraints
│   ├── tt_game_top_files/         # tt_top + game logic, animator, score, sound, tilt, testbenches
│   └── image_top_files/           # image_top + constraints
│
├── individual_modules/         # Each stage developed and verified on its own
│   ├── memory/                 # Vertex/matrix memory, testbench, memory.py generator
│   ├── transform_engine/       # Transform engine + testbenches
│   ├── Projected engine/       # Projection engine, testbenches, standalone board top
│   ├── Triangle Unit/          # Triangle setup (normal + debug), testbench
│   ├── rasterizer/             # Rasterizer, triangle ROM, demo top, testbenches
│   ├── Z_Buffer/               # Z/frame buffer + testbench
│   └── vga_controller/         # VGA controller + testbench
│
├── mem_files/                  # Memory initialisation data and generators
│   ├── vertices.mem, matrix.mem, recip_lut_q1_17.mem   # Default cube scene
│   ├── rotating_cube/          # 120-frame matrix set + generator script
│   ├── navigation/             # City world, map, palette + nav_world.py
│   ├── tt_game/                # Table-tennis scene, views, previews + generators
│   └── face_image/             # Image-to-triangles data + image_to_mem.py
│
└── design_doc/                 # Technical specifications and helper scripts
    ├── initial_modules.txt, transformation_module.txt,
    │   design_doc_triangle_setup.txt, design_doc_raster.txt,
    │   design_doc_z_buffer.txt          # Per-stage specifications
    ├── design_doc_tt_scene.txt, design_doc_tt_game.txt   # Table-tennis notes
    ├── projection_engine_schematic.png
    └── img2mem.py, read_uart.py         # Projection-engine test helpers
```

---

## Getting Started

**Requirements:** Xilinx Vivado, a Nexys A7-100T board, a VGA monitor (or VGA-to-HDMI adapter), and Python 3 if you want to regenerate memory files.

1. Create a new Vivado project for the part `xc7a100tcsg324-1`.
2. Add `hardware_pipeline/hardware_pipeline.v`, the seven stage modules, and the files for the demo you want (e.g. `hardware_pipeline/game_files/`).
3. Add that demo's `.mem` files from `mem_files/` as design sources so `$readmemh` can find them.
4. Add the demo's `.xdc` constraints file and set its top module (e.g. `game_top`) as top.
5. Run synthesis, implementation and generate the bitstream, then program the board.
6. Press `CPU_RESETN` (the red button) to reset and re-render.

To simulate a single stage, use the testbenches under `individual_modules/` with the matching `.mem` files.

---

## Additional Contributions

This project is developed as a team. For the full list of contributors, commit history and the latest source, see the main repository:

**[Dhruv-Punekar/Hlab](https://github.com/Dhruv-Punekar/Hlab)**

Contributions, issues and suggestions are welcome through the repository.

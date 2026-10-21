# Triangle setup unit

Converts one screen-space triangle into a rasterizer payload: pixel bounding
box (10-bit), three edge equations, a Q24.8 depth plane, colour and status
flags. Target: Artix-7 100T.

| File | What it is |
|------|-----------|
| `triangle_setup_unit_bbox.v` | The setup unit (module `triangle_setup_unit_debug`) |
| `recip_lut_q1_17.mem` | 1024-entry reciprocal LUT the module loads with `$readmemh` |
| `sim/` | Self-checking regression: testbench, stimulus, exact-math checker |
| `timing_check.tcl` | Vivado out-of-context timing check |

`hardware_pipeline/` keeps its own copy of the RTL and LUT; update both when
the module changes.

## Key facts

- 12-stage pipeline (16 with `RECIP_NR=1`, the default); one triangle per
  clock. Latency is the `LATENCY` localparam: 11 cycles, or 15 with NR.
- Ready/valid on both sides; a stalled output freezes the whole pipeline.
- `RECIP_NR=1` refines the LUT reciprocal with one Newton-Raphson step
  (worst depth error ~1.2 units instead of ~25). `RECIP_NR=0` gives the
  original 3-stage design's numbers exactly.
- `rst_n` resets valid bits and flags only; drive it from a reset
  synchronizer.

## Simulation

Needs Icarus Verilog 11+ and Python 3. From any folder:

```sh
sh individual_modules/triangle_setup/sim/run.sh
```

Runs 4000 triangles (normal, slivers, collinear, viewport corners, extreme z,
out-of-range x/y) under random backpressure. Must end with `ALL PASS`.

## Timing

From this folder, in a Vivado Tcl shell:

```sh
vivado -mode batch -source timing_check.tcl                         # 100 MHz, xc7a100tcsg324-1
vivado -mode batch -source timing_check.tcl -tclargs triangle_setup_unit_bbox.v 8   # 125 MHz
```

Read the final `RESULT` line: `WNS >= 0` and `WHS >= 0` mean timing is met.
Reports land in `timing/` (ignored by git). Expect about 28 DSP48E1 and
1 RAMB18 in `utilization.rpt`.

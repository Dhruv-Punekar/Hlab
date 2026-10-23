# Projection Engine (FPGA)

Computes the **horizontal and vertical projection profiles** of a 64×64 8-bit
grayscale image stored in on-chip block RAM, and sends the results to a PC
over UART.

```
P_row[y] = Σx I(x, y)     64 values (one per row)
P_col[x] = Σy I(x, y)     64 values (one per column)
```

One pixel per clock → a full frame takes 4,098 cycles (≈41 µs at 100 MHz).

## Folder layout

| Folder | Contents |
|---|---|
| `rtl/` | Synthesizable Verilog, one module per file |
| `tb/` | Self-checking testbenches |
| `mem/` | `image.mem` (the image in ROM), golden results, preview PNG |
| `constraints/` | `basys3.xdc` pin constraints |
| `scripts/` | `img2mem.py` (image → .mem), `read_uart.py` (PC receiver) |

### RTL modules

| File | Role |
|---|---|
| `pixel_addr_gen.v` | Raster-scan address / (x, y) generator |
| `row_projection.v` | Row sums with a running accumulator |
| `col_projection.v` | Column sums with 64 accumulators in LUT RAM |
| `projection_engine.v` | Engine: address gen + pipeline alignment + both projections |
| `image_rom.v` | 4096×8 block RAM loaded from `image.mem` |
| `uart_tx.v` | 8N1 UART transmitter |
| `result_sender.v` | Streams the 128 results as a UART packet |
| `debounce.v` | Push-button synchroniser / debouncer |
| `top.v` | Board top level (Basys 3) |

### Testbenches

| File | Checks |
|---|---|
| `tb_pixel_addr_gen.v` | Scan order, `last`, start-while-busy ignored, two frames |
| `tb_projection_engine.v` | All 128 sums vs. Python golden model, latency, re-run |
| `tb_uart_tx.v` | Serial framing and bit timing with a behavioural receiver |
| `tb_top.v` | Whole system: decodes the UART packet, LED readout, btnU re-run |

## Simulate

Icarus Verilog, from this folder:

```
make all          # each testbench prints PASS or FAIL
make top          # just one
make wave T=top   # view waveform in GTKWave
```

**Vivado simulator:** add `rtl/*.v` as design sources, `tb/*.v` as simulation
sources, and the three `mem/*.mem` files as simulation sources. Then set the
Verilog defines `IMAGE_MEM="image.mem"`, `GOLDEN_ROW="golden_row.mem"`,
`GOLDEN_COL="golden_col.mem"` (Settings → Simulation → Verilog options), or
leave the defaults if the simulator runs from the repo root.

## Run on the FPGA (Basys 3)

1. Create a Vivado project for part **xc7a35tcpg236-1**.
2. Add `rtl/*.v` as design sources and set **`top`** as the top module.
3. Add `mem/image.mem` as a design source (Vivado lists it as a Memory
   Initialization File). Synthesis needs it to fill the BRAM.
4. Add `constraints/basys3.xdc`.
5. Run synthesis, implementation, generate the bitstream and program the board.

On the board:

- After programming (or pressing **btnC**), the frame is processed and the
  results are sent over USB-UART at **115200 8N1**. LED15 = done.
- **btnU** re-runs the engine and re-sends.
- **sw[5:0]** selects an index, **sw[15]** selects row (0) or column (1);
  **led[13:0]** shows that value in binary.
- On the PC: `pip install pyserial matplotlib` then
  `python3 scripts/read_uart.py --port COM5 --plot`. It checks the received
  values against `mem/golden_*.mem` and prints MATCH or MISMATCH.

For a different board, change the pins in the `.xdc` and the clock-dependent
parameters `CLKS_PER_BIT` (= f_clk / 115200) and `DEBOUNCE_CYCLES` in `top.v`.

## Using a different image

```
python3 scripts/img2mem.py path/to/image.png --out mem
```

The script converts to grayscale, centre-crops to square, resizes to 64×64
and regenerates `image.mem` and the golden files. Re-run synthesis afterwards.

## UART packet format

```
A5 5A | row[0]_hi row[0]_lo ... row[63] | col[0]_hi col[0]_lo ... col[63]
```

258 bytes, 16-bit big-endian values (max value 64 × 255 = 16320).

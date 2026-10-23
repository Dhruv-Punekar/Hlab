#!/usr/bin/env python3
"""
read_uart.py - receive projection results from the FPGA and check them.

Packet: 0xA5 0x5A, then 64 row sums and 64 column sums, each 16-bit big-endian.

Usage:
  pip install pyserial matplotlib
  python3 scripts/read_uart.py --port COM5          (Windows)
  python3 scripts/read_uart.py --port /dev/ttyUSB1  (Linux)
Then press btnC (reset) or btnU (re-run) on the board.
"""
import argparse
import os

import serial

W = H = 64


def read_mem(path):
    with open(path) as f:
        return [int(line, 16) for line in f if line.strip()]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True)
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--mem", default="mem")
    ap.add_argument("--plot", action="store_true")
    args = ap.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=10)
    print("Waiting for packet... (press btnC or btnU on the board)")

    # sync on header 0xA5 0x5A
    prev = None
    while True:
        b = ser.read(1)
        if not b:
            raise SystemExit("Timeout: no data from the board")
        if prev == 0xA5 and b[0] == 0x5A:
            break
        prev = b[0]

    payload = ser.read(2 * (H + W))
    if len(payload) != 2 * (H + W):
        raise SystemExit(f"Short packet: {len(payload)} bytes")
    vals = [(payload[2 * i] << 8) | payload[2 * i + 1] for i in range(H + W)]
    row, col = vals[:H], vals[H:]

    g_row = read_mem(os.path.join(args.mem, "golden_row.mem"))
    g_col = read_mem(os.path.join(args.mem, "golden_col.mem"))
    bad = [i for i in range(H) if row[i] != g_row[i]] + \
          [H + i for i in range(W) if col[i] != g_col[i]]

    print("Row projection:", row)
    print("Col projection:", col)
    print("MATCH with golden model" if not bad else f"MISMATCH at {len(bad)} values: {bad[:10]}")

    if args.plot:
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(1, 2, figsize=(10, 4))
        ax[0].plot(row); ax[0].set_title("Row projection (FPGA)"); ax[0].set_xlabel("y")
        ax[1].plot(col); ax[1].set_title("Column projection (FPGA)"); ax[1].set_xlabel("x")
        plt.tight_layout()
        plt.show()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""firmware.bin -> firmware.hex for $readmemh: one 32-bit little-endian word
per line, padded with zeros to the RAM size (default 4096 words = 16 KiB)."""
import sys
src, dst = sys.argv[1], sys.argv[2]
words = int(sys.argv[3]) if len(sys.argv) > 3 else 4096
data = open(src, "rb").read()
data += b"\0" * (-len(data) % 4)
n = len(data) // 4
if n > words:
    sys.exit(f"firmware is {n} words, RAM holds {words}")
with open(dst, "w") as f:
    for i in range(words):
        w = int.from_bytes(data[4*i:4*i+4], "little") if i < n else 0
        f.write(f"{w:08x}\n")
print(f"{dst}: {n} of {words} words used ({100*n//words}%)")

#!/usr/bin/env python3
"""Generate recip_lut_q1_17.mem for the triangle setup unit.

Address a = the 10 fraction bits after the leading one of |D|, so the bin
covers m in [1 + a/1024, 1 + (a+1)/1024). Entry a = round(2^17 / (1 + a/1024)),
i.e. 1/m at the START of the bin, as Q1.17 (18 bits). Matches the LUT used in
hardware_pipeline/ and the formula in tb_hardware_pipeline.v (round half up).
usage: gen_recip_lut.py [output.mem]
"""
import math
import sys

ADDR_BITS = 10
FRAC_BITS = 17


def entry(a: int) -> int:
    v = math.floor((1 << FRAC_BITS) / (1.0 + a / (1 << ADDR_BITS)) + 0.5)
    assert 0 < v < (1 << 18)
    return v


def main(path: str = "recip_lut_q1_17.mem") -> None:
    with open(path, "w", newline="\n") as f:
        f.write("// recip_lut_q1_17.mem - 1024 x 18-bit, "
                "entry a = round(2^17 / (1 + a/1024)), Q1.17\n")
        for a in range(1 << ADDR_BITS):
            f.write(f"{entry(a):05X}\n")


if __name__ == "__main__":
    main(*sys.argv[1:])

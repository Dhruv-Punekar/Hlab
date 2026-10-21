#!/usr/bin/env python3
"""Generate recip_lut_q1_17.mem for triangle_setup_unit.

Address = the 10 fraction bits that follow the leading one of |D|
(d_normalized[37:28]), so each entry covers m in [1 + a/1024, 1 + (a+1)/1024).
Each entry stores round(2^17 / m_mid) with m_mid the CENTRE of that bin, which
halves the worst-case error versus using the bin start (max rel. error ~2^-11).
Values are in (0.5, 1.0] -> Q1.17, 18 bits, one hex word per line.
"""
import sys

ADDR_BITS = 10
FRAC_BITS = 17


def entry(a: int) -> int:
    m_mid = 1.0 + (a + 0.5) / (1 << ADDR_BITS)
    v = int(round((1 << FRAC_BITS) / m_mid))
    assert 0 < v < (1 << 18)
    return v


def main(path: str = "recip_lut_q1_17.mem") -> None:
    with open(path, "w") as f:
        for a in range(1 << ADDR_BITS):
            f.write(f"{entry(a):05x}\n")


if __name__ == "__main__":
    main(*sys.argv[1:])

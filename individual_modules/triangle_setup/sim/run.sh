#!/bin/sh
# Regression for triangle_setup_unit_debug.v (needs iverilog >= 11, python3).
#   sh sim/run.sh
# 1. LUT-only numerics must be bit-identical to the original design
#    (sha256 of the output stream, latency-independent).
# 2. Edges, bbox, flags are checked against exact math.
# 3. If the RTL has RECIP_NR, the default (refined) build must also keep the
#    ordinary-triangle depth error at or below 2 depth units.
set -e
HERE=$(cd "$(dirname "$0")" && pwd)
RTL="$HERE/../triangle_setup_unit_bbox.v"
B="$HERE/build"
mkdir -p "$B"; cd "$B"
cp "$HERE/../recip_lut_q1_17.mem" .
python3 "$HERE/gen_stim.py" 4000

EXTRA=""
if grep -q "RECIP_NR" "$RTL"; then
    echo 'module nr0; defparam tb.dut.RECIP_NR = 0; endmodule' > nr0.v
    EXTRA=nr0.v
fi

echo "== LUT-only numerics =="
iverilog -g2005 -o sim_lut -DDUT=triangle_setup_unit_debug -DOUTFILE='"out_lut.txt"' \
    "$HERE/tb_triangle_setup.v" "$RTL" $EXTRA
vvp -n sim_lut | grep -E "sent=|TIMEOUT"
python3 "$HERE/check.py" out_lut.txt
if [ "$(sha256sum out_lut.txt | cut -d' ' -f1)" = "$(cat "$HERE/golden_lut_only.sha256")" ]; then
    echo "PASS: output stream bit-identical to the original design"
else
    echo "FAIL: output stream differs from the original design"; exit 1
fi

if [ -n "$EXTRA" ]; then
    echo "== default build (Newton-Raphson reciprocal) =="
    iverilog -g2005 -o sim_def -DDUT=triangle_setup_unit_debug -DOUTFILE='"out_def.txt"' \
        "$HERE/tb_triangle_setup.v" "$RTL"
    vvp -n sim_def | grep -E "sent=|TIMEOUT"
    python3 "$HERE/check.py" out_def.txt --max-depth-err 2.0
fi
echo "ALL PASS"

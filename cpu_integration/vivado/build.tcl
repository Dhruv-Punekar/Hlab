# ============================================================================
# build.tcl - non-project Vivado build of nexys_a7_cpu_top (Nexys A7-100T)
#
#   cd cpu_integration/firmware && make          (creates firmware.hex)
#   cd ../vivado && vivado -mode batch -source build.tcl
#
# Output in cpu_integration/vivado/out/:
#   nexys_a7_cpu_top.bit, utilization.rpt, timing_summary.rpt, drc.rpt
# The script stops with an error if timing is not met.
#
# hardware_pipeline/ is used as it is (same files as cube_top).
# In the Vivado GUI instead: create an RTL project for xc7a100tcsg324-1, add
# the same sources and memory files as below (the .hex/.mem files as
# "design sources" so $readmemh finds them), the XDC, and set
# nexys_a7_cpu_top as top.
# ============================================================================
set here [file normalize [file dirname [info script]]]
set root [file normalize "$here/../.."]
set out  "$here/out"
file mkdir $out

set part xc7a100tcsg324-1

# memory files must be in the working directory for $readmemh during synthesis
foreach f [list \
    "$root/mem_files/vertices.mem" \
    "$root/mem_files/recip_lut_q1_17.mem" \
    "$root/mem_files/matrix.mem" \
    "$root/cpu_integration/firmware/firmware.hex"] {
    if {![file exists $f]} { error "missing $f (run make in cpu_integration/firmware?)" }
    file copy -force $f [pwd]
}

read_verilog [list \
    "$root/hardware_pipeline/hardware_pipeline.v" \
    "$root/hardware_pipeline/matrix_memory.v" \
    "$root/hardware_pipeline/vertex_memory.v" \
    "$root/hardware_pipeline/transform_engine.v" \
    "$root/hardware_pipeline/projected_engine.v" \
    "$root/hardware_pipeline/triangle_setup_unit_bbox.v" \
    "$root/hardware_pipeline/rasterizer.v" \
    "$root/hardware_pipeline/z_f_buffer.v" \
    "$root/hardware_pipeline/vga_controller.v" \
    "$root/cpu_integration/rtl/picorv32.v" \
    "$root/cpu_integration/rtl/gfx_ctrl.v" \
    "$root/cpu_integration/rtl/uart.v" \
    "$root/cpu_integration/rtl/cpu_subsystem.v" \
    "$root/cpu_integration/rtl/nexys_a7_cpu_top.v" ]
read_xdc "$root/cpu_integration/constraints/nexys_a7_cpu_top.xdc"

synth_design -top nexys_a7_cpu_top -part $part
report_utilization -file "$out/post_synth_utilization.rpt"

opt_design
place_design
phys_opt_design
route_design

report_utilization    -file "$out/utilization.rpt"
report_timing_summary -file "$out/timing_summary.rpt" -max_paths 20
report_drc            -file "$out/drc.rpt"
report_methodology    -file "$out/methodology.rpt"

set wns [get_property SLACK [get_timing_paths -max_paths 1 -nworst 1 -setup]]
set whs [get_property SLACK [get_timing_paths -max_paths 1 -nworst 1 -hold]]
puts "WNS = $wns ns, WHS = $whs ns"
if {$wns < 0 || $whs < 0} {
    error "timing not met (WNS $wns, WHS $whs) - see $out/timing_summary.rpt"
}

write_bitstream -force "$out/nexys_a7_cpu_top.bit"
puts "bitstream: $out/nexys_a7_cpu_top.bit"

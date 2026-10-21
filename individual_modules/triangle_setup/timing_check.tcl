# Out-of-context timing check of the triangle setup unit on Artix-7.
#
# From this folder, in a Vivado Tcl shell:
#   vivado -mode batch -source timing_check.tcl
#   vivado -mode batch -source timing_check.tcl -tclargs <rtl.v> <period_ns> [part]
#
# Defaults: triangle_setup_unit_bbox.v, 10 ns (100 MHz), xc7a100tcsg324-1.
# Reports go to timing/<rtl name>_<period>ns/. The last line printed is
#   RESULT ... WNS=<setup slack> WHS=<hold slack> Fmax~<estimate>
# WNS >= 0 and WHS >= 0 mean timing is met at that clock.

set here   [file dirname [file normalize [info script]]]
set rtl    [expr {[llength $argv] > 0 ? [lindex $argv 0] : "$here/triangle_setup_unit_bbox.v"}]
set period [expr {[llength $argv] > 1 ? [lindex $argv 1] : 10}]
set part   [expr {[llength $argv] > 2 ? [lindex $argv 2] : "xc7a100tcsg324-1"}]
set top    triangle_setup_unit_debug
set out    "$here/timing/[file rootname [file tail $rtl]]_${period}ns"
file mkdir $out

set f [open "$out/clk.xdc" w]
puts $f "create_clock -name clk -period $period \[get_ports clk\]"
close $f

# The LUT file has gone missing after earlier runs. Keep its contents in
# memory, give Vivado a copy inside $out, and put the original back at the
# end if it disappeared.
set mem_src "$here/recip_lut_q1_17.mem"
set f [open $mem_src r]; fconfigure $f -translation binary
set mem_data [read $f]; close $f
file copy -force $mem_src "$out/recip_lut_q1_17.mem"

read_verilog $rtl
read_mem     "$out/recip_lut_q1_17.mem"
read_xdc     "$out/clk.xdc"

# Out-of-context: the module's ~1000 ports are not mapped to package pins.
synth_design -top $top -part $part -mode out_of_context
opt_design
place_design
phys_opt_design
route_design

report_timing_summary -max_paths 10 -file "$out/timing_summary.rpt"
report_timing -sort_by slack -max_paths 5 -file "$out/worst_paths.rpt"
report_utilization -file "$out/utilization.rpt"

set wns [get_property SLACK [get_timing_paths -setup -max_paths 1]]
set whs [get_property SLACK [get_timing_paths -hold  -max_paths 1]]
puts "RESULT [file tail $rtl] @ ${period} ns on $part : WNS=$wns ns  WHS=$whs ns  Fmax~[format %.1f [expr {1000.0 / ($period - $wns)}]] MHz"
puts "Reports: $out"

if {![file exists $mem_src]} {
    set f [open $mem_src w]; fconfigure $f -translation binary
    puts -nonewline $f $mem_data; close $f
    puts "WARNING: $mem_src was missing after the run and has been restored."
}

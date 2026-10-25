## ============================================================================
## nexys_a7_top.xdc  -  Nexys A7-100T (xc7a100tcsg324-1) constraints for the
## 3D rendering pipeline. Pins from Digilent's Nexys-A7-100T-Master.xdc.
## Top module: nexys_a7_top
## ============================================================================

## ---- 100 MHz board oscillator ----------------------------------------------
set_property -dict { PACKAGE_PIN E3  IOSTANDARD LVCMOS33 } [get_ports { CLK100MHZ }];
create_clock -add -name sys_clk_pin -period 10.00 -waveform {0 5} [get_ports { CLK100MHZ }];

## ---- Reset / re-render button (red CPU_RESETN, active low) -----------------
set_property -dict { PACKAGE_PIN C12 IOSTANDARD LVCMOS33 } [get_ports { CPU_RESETN }];

## ---- VGA connector (RGB444 + sync) -----------------------------------------
set_property -dict { PACKAGE_PIN A3  IOSTANDARD LVCMOS33 } [get_ports { VGA_R[0] }];
set_property -dict { PACKAGE_PIN B4  IOSTANDARD LVCMOS33 } [get_ports { VGA_R[1] }];
set_property -dict { PACKAGE_PIN C5  IOSTANDARD LVCMOS33 } [get_ports { VGA_R[2] }];
set_property -dict { PACKAGE_PIN A4  IOSTANDARD LVCMOS33 } [get_ports { VGA_R[3] }];
set_property -dict { PACKAGE_PIN C6  IOSTANDARD LVCMOS33 } [get_ports { VGA_G[0] }];
set_property -dict { PACKAGE_PIN A5  IOSTANDARD LVCMOS33 } [get_ports { VGA_G[1] }];
set_property -dict { PACKAGE_PIN B6  IOSTANDARD LVCMOS33 } [get_ports { VGA_G[2] }];
set_property -dict { PACKAGE_PIN A6  IOSTANDARD LVCMOS33 } [get_ports { VGA_G[3] }];
set_property -dict { PACKAGE_PIN B7  IOSTANDARD LVCMOS33 } [get_ports { VGA_B[0] }];
set_property -dict { PACKAGE_PIN C7  IOSTANDARD LVCMOS33 } [get_ports { VGA_B[1] }];
set_property -dict { PACKAGE_PIN D7  IOSTANDARD LVCMOS33 } [get_ports { VGA_B[2] }];
set_property -dict { PACKAGE_PIN D8  IOSTANDARD LVCMOS33 } [get_ports { VGA_B[3] }];
set_property -dict { PACKAGE_PIN B11 IOSTANDARD LVCMOS33 } [get_ports { VGA_HS }];
set_property -dict { PACKAGE_PIN B12 IOSTANDARD LVCMOS33 } [get_ports { VGA_VS }];

## ---- Clock domains ------------------------------------------------------------
## The MMCM output clocks (100 MHz render, 25.175 MHz VGA) are derived by Vivado
## automatically from sys_clk_pin. Their ratio is not an integer, so treat them
## as asynchronous; every crossing in the design is synchronised in RTL
## (2-FF reset/clear_done synchronisers, dual-clock framebuffer BRAM).
set_clock_groups -asynchronous \
    -group [get_clocks -of_objects [get_pins u_mmcm/CLKOUT1]] \
    -group [get_clocks -of_objects [get_pins u_mmcm/CLKOUT0]]

## The button is a slow asynchronous input; it goes through a 2-FF synchroniser.
set_false_path -from [get_ports { CPU_RESETN }]

## VGA outputs are registered in vga_controller; a monitor samples them with
## its own PLL, so no output delay constraint is needed.
set_false_path -to [get_ports { VGA_R[*] VGA_G[*] VGA_B[*] VGA_HS VGA_VS }]

## ---- Configuration bank voltage (removes the CFGBVS DRC warning) -------------
set_property CFGBVS VCCO        [current_design]
set_property CONFIG_VOLTAGE 3.3 [current_design]
set_property BITSTREAM.GENERAL.COMPRESS TRUE [current_design]

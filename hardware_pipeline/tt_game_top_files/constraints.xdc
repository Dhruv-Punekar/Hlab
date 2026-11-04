## ============================================================================
## constraints.xdc  -  Nexys A7-100T (xc7a100tcsg324-1), table-tennis game
## Top module: tt_top.  Pins from Digilent's Nexys-A7-100T-Master.xdc.
## ============================================================================

## ---- 100 MHz board oscillator ----------------------------------------------
set_property -dict { PACKAGE_PIN E3  IOSTANDARD LVCMOS33 } [get_ports { CLK100MHZ }];
create_clock -add -name sys_clk_pin -period 10.00 -waveform {0 5} [get_ports { CLK100MHZ }];

## ---- Reset (red CPU_RESETN, active low) --------------------------------------
set_property -dict { PACKAGE_PIN C12 IOSTANDARD LVCMOS33 } [get_ports { CPU_RESETN }];

## ---- Game buttons (active high) ---------------------------------------------
##   P1 (near, red):  BTNR forehand, BTNL backhand
##   P2 (far, black): BTNU forehand, BTND backhand
##   BTNC new game
set_property -dict { PACKAGE_PIN N17 IOSTANDARD LVCMOS33 } [get_ports { BTNC }];
set_property -dict { PACKAGE_PIN M18 IOSTANDARD LVCMOS33 } [get_ports { BTNU }];
set_property -dict { PACKAGE_PIN P17 IOSTANDARD LVCMOS33 } [get_ports { BTNL }];
set_property -dict { PACKAGE_PIN M17 IOSTANDARD LVCMOS33 } [get_ports { BTNR }];
set_property -dict { PACKAGE_PIN P18 IOSTANDARD LVCMOS33 } [get_ports { BTND }];

## ---- Switches: SW0 = P2 by computer, SW1 = P1 by computer --------------------
set_property -dict { PACKAGE_PIN J15 IOSTANDARD LVCMOS33 } [get_ports { SW[0] }];
set_property -dict { PACKAGE_PIN L16 IOSTANDARD LVCMOS33 } [get_ports { SW[1] }];

## ---- LEDs ----------------------------------------------------------------------
set_property -dict { PACKAGE_PIN H17 IOSTANDARD LVCMOS33 } [get_ports { LED[0] }];
set_property -dict { PACKAGE_PIN K15 IOSTANDARD LVCMOS33 } [get_ports { LED[1] }];
set_property -dict { PACKAGE_PIN J13 IOSTANDARD LVCMOS33 } [get_ports { LED[2] }];
set_property -dict { PACKAGE_PIN N14 IOSTANDARD LVCMOS33 } [get_ports { LED[3] }];
set_property -dict { PACKAGE_PIN R18 IOSTANDARD LVCMOS33 } [get_ports { LED[4] }];
set_property -dict { PACKAGE_PIN V17 IOSTANDARD LVCMOS33 } [get_ports { LED[5] }];
set_property -dict { PACKAGE_PIN U17 IOSTANDARD LVCMOS33 } [get_ports { LED[6] }];
set_property -dict { PACKAGE_PIN U16 IOSTANDARD LVCMOS33 } [get_ports { LED[7] }];
set_property -dict { PACKAGE_PIN V16 IOSTANDARD LVCMOS33 } [get_ports { LED[8] }];
set_property -dict { PACKAGE_PIN T15 IOSTANDARD LVCMOS33 } [get_ports { LED[9] }];
set_property -dict { PACKAGE_PIN U14 IOSTANDARD LVCMOS33 } [get_ports { LED[10] }];
set_property -dict { PACKAGE_PIN T16 IOSTANDARD LVCMOS33 } [get_ports { LED[11] }];
set_property -dict { PACKAGE_PIN V15 IOSTANDARD LVCMOS33 } [get_ports { LED[12] }];
set_property -dict { PACKAGE_PIN V14 IOSTANDARD LVCMOS33 } [get_ports { LED[13] }];
set_property -dict { PACKAGE_PIN V12 IOSTANDARD LVCMOS33 } [get_ports { LED[14] }];
set_property -dict { PACKAGE_PIN V11 IOSTANDARD LVCMOS33 } [get_ports { LED[15] }];

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
## 100 MHz render and 25.175 MHz pixel clock from the MMCM: unrelated ratio, so
## asynchronous. Every crossing is synchronised in RTL (see tt_game.v):
## 2-FF resets / buttons / switches / render-busy, toggle-synchronised pose
## bus (stable for a whole frame), dual-clock framebuffer BRAM.
set_clock_groups -asynchronous \
    -group [get_clocks -of_objects [get_pins u_mmcm/CLKOUT1]] \
    -group [get_clocks -of_objects [get_pins u_mmcm/CLKOUT0]]

## Slow asynchronous inputs, all synchronised (2-FF) before use.
set_false_path -from [get_ports { CPU_RESETN BTNC BTNU BTNL BTNR BTND SW[*] }]

## Outputs: VGA registered (monitor samples with its own PLL), LEDs static.
set_false_path -to [get_ports { VGA_R[*] VGA_G[*] VGA_B[*] VGA_HS VGA_VS LED[*] }]

## ---- Configuration bank voltage (removes the CFGBVS DRC warning) -------------
set_property CFGBVS VCCO        [current_design]
set_property CONFIG_VOLTAGE 3.3 [current_design]
set_property BITSTREAM.GENERAL.COMPRESS TRUE [current_design]

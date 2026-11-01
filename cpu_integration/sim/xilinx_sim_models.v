`timescale 1ps / 1ps
// Simulation-only stand-ins for the two Xilinx primitives (MMCME2_BASE, BUFG) the board tops use.
// Vivado uses its own UNISIM library; never add this file to synthesis.
module BUFG (input wire I, output wire O);
    assign O = I;
endmodule

module MMCME2_BASE #(
    parameter BANDWIDTH = "OPTIMIZED", parameter real CLKIN1_PERIOD = 10.0,
    parameter integer DIVCLK_DIVIDE = 1, parameter real CLKFBOUT_MULT_F = 9.0,
    parameter real CLKFBOUT_PHASE = 0.0,
    parameter real CLKOUT0_DIVIDE_F = 35.75, parameter real CLKOUT0_DUTY_CYCLE = 0.5,
    parameter real CLKOUT0_PHASE = 0.0,
    parameter integer CLKOUT1_DIVIDE = 9, parameter real CLKOUT1_DUTY_CYCLE = 0.5,
    parameter real CLKOUT1_PHASE = 0.0,
    parameter real REF_JITTER1 = 0.01, parameter STARTUP_WAIT = "FALSE"
)(
    input wire CLKIN1, input wire CLKFBIN, output wire CLKFBOUT, output wire CLKFBOUTB,
    output reg CLKOUT0 = 0, output wire CLKOUT0B, output reg CLKOUT1 = 0, output wire CLKOUT1B,
    output wire CLKOUT2, CLKOUT2B, CLKOUT3, CLKOUT3B, CLKOUT4, CLKOUT5, CLKOUT6,
    output reg LOCKED = 0, input wire PWRDWN, input wire RST
);
    // VCO period (ps) = CLKIN1_PERIOD(ns)*1000*DIVCLK/MULT
    localparam real VCO_PS = CLKIN1_PERIOD * 1000.0 * DIVCLK_DIVIDE / CLKFBOUT_MULT_F;
    localparam real H0 = VCO_PS * CLKOUT0_DIVIDE_F / 2.0;
    localparam real H1 = VCO_PS * CLKOUT1_DIVIDE   / 2.0;
    assign CLKFBOUT = CLKIN1;
    assign CLKFBOUTB = ~CLKIN1; assign CLKOUT0B = ~CLKOUT0; assign CLKOUT1B = ~CLKOUT1;
    assign {CLKOUT2, CLKOUT2B, CLKOUT3, CLKOUT3B, CLKOUT4, CLKOUT5, CLKOUT6} = 7'b0;
    initial begin #(1_000_000); LOCKED = 1; end     // lock after 1 us
    always begin #(H0) CLKOUT0 = ~CLKOUT0; end
    always begin #(H1) CLKOUT1 = ~CLKOUT1; end
endmodule

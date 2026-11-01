`timescale 1ns / 1ps

// ============================================================================
// Module: tt_top   (board top, Nexys A7-100T, xc7a100tcsg324-1)
//         Table-tennis demo game.
//
//   CLK100MHZ -> MMCME2_BASE -> clk_render 100 MHz, clk_vga 25.175 MHz
//   CPU_RESETN (red button) -> full reset (score 0-0, P1 serves)
//
//   Controls (two players on one board):
//     P1, near end, red racket  : BTNR forehand   BTNL backhand
//     P2, far end, black racket : BTNU forehand   BTND backhand
//     The server serves with either of his buttons; the receiver returns with
//     either. A return counts only if pressed in the timing window as the
//     ball comes to the racket (see tt_game_core).
//     BTNC: new game.   SW0 = P2 by computer, SW1 = P1 by computer.
//   LEDs: LD0..3 buttons (P1 fore/back, P2 fore/back), LD12/13 server P1/P2,
//         LD14 game over, LD15 render overrun (should never light).
//   Score: panel in the top-right corner of the picture.
//
// Files to add to the Vivado project (design sources):
//   RTL : hardware_pipeline/*.v (pipeline), tt_game_top_files/tt_top.v,
//         tt_game.v, tt_game_core.v, tt_obj_animator.v, tt_score_overlay.v,
//         tt_buttons.v
//   data: vertices.mem, matrix.mem, tt_scene_map.vh, tt_game_map.vh
//         (mem_files/tt_game), recip_lut_q1_17.mem (mem_files)
//   constraints: tt_game_top_files/constraints.xdc
//   vertex_memory / matrix_memory load the fixed names "vertices.mem" and
//   "matrix.mem": use a separate Vivado project for this demo.
// ============================================================================
module tt_top #(
    parameter RECIP_LUT_FILE = "recip_lut_q1_17.mem",
    parameter Z_FRAC_OUT     = 9
)(
    input  wire        CLK100MHZ,
    input  wire        CPU_RESETN,      // red button, active low

    input  wire        BTNC,
    input  wire        BTNU,
    input  wire        BTNL,
    input  wire        BTNR,
    input  wire        BTND,
    input  wire [1:0]  SW,
    output wire [15:0] LED,

    output wire [3:0]  VGA_R,
    output wire [3:0]  VGA_G,
    output wire [3:0]  VGA_B,
    output wire        VGA_HS,
    output wire        VGA_VS
);

    // ========================================================================
    // CLOCKING: one MMCM -> 100 MHz render clock + 25.175 MHz pixel clock
    //    VCO = 100 * 9 = 900 MHz; /35.75 = 25.1748 MHz; /9 = 100 MHz
    // ========================================================================
    wire clk_fb_out, clk_fb_in, clk_vga_unbuf, clk_render_unbuf;
    wire clk_vga, clk_render, mmcm_locked;

    MMCME2_BASE #(
        .BANDWIDTH          ("OPTIMIZED"),
        .CLKIN1_PERIOD      (10.000),
        .DIVCLK_DIVIDE      (1),
        .CLKFBOUT_MULT_F    (9.000),
        .CLKFBOUT_PHASE     (0.000),
        .CLKOUT0_DIVIDE_F   (35.750),
        .CLKOUT0_DUTY_CYCLE (0.500),
        .CLKOUT0_PHASE      (0.000),
        .CLKOUT1_DIVIDE     (9),
        .CLKOUT1_DUTY_CYCLE (0.500),
        .CLKOUT1_PHASE      (0.000),
        .REF_JITTER1        (0.010),
        .STARTUP_WAIT       ("FALSE")
    ) u_mmcm (
        .CLKIN1   (CLK100MHZ),
        .CLKFBIN  (clk_fb_in),
        .CLKFBOUT (clk_fb_out),
        .CLKOUT0  (clk_vga_unbuf),
        .CLKOUT1  (clk_render_unbuf),
        .LOCKED   (mmcm_locked),
        .PWRDWN   (1'b0),
        .RST      (1'b0)
    );

    BUFG u_bufg_fb     (.I(clk_fb_out),       .O(clk_fb_in));
    BUFG u_bufg_vga    (.I(clk_vga_unbuf),    .O(clk_vga));
    BUFG u_bufg_render (.I(clk_render_unbuf), .O(clk_render));

    // ========================================================================
    // THE GAME (reset synchronisers per clock domain are inside)
    // ========================================================================
    tt_game #(
        .RECIP_LUT_FILE (RECIP_LUT_FILE),
        .Z_FRAC_OUT     (Z_FRAC_OUT)
    ) u_game (
        .clk_render (clk_render),
        .clk_vga    (clk_vga),
        .arst_n     (CPU_RESETN & mmcm_locked),
        .btn        ({BTNC, BTNU, BTNL, BTNR, BTND}),
        .sw         (SW),
        .led        (LED),
        .vga_r      (VGA_R),
        .vga_g      (VGA_G),
        .vga_b      (VGA_B),
        .vga_hs     (VGA_HS),
        .vga_vs     (VGA_VS)
    );

endmodule

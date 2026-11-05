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
//     Shots: the server serves with either of his buttons; the receiver returns
//     with either. A return counts only if pressed in the timing window as the
//     ball comes to the racket AND the racket is within reach (0.16 m) of it.
//     BTNC: short press = next camera view (player 1 / side / player 2),
//           hold 1 s = new game.
//     SW0 = P2 by computer        SW1 = P1 by computer
//     SW2 = P1 tilt aiming: P1's racket follows the ball quickly and the board
//           tilt (accelerometer) decides where P1's shot lands.
//           SW1+SW2 = fully automatic P1, aimed by tilting the board.
//     SW3 = mute   SW4 = invert tilt   SW5 = use the accelerometer Y axis
//   Rules: first to 11 points wins the game, serve changes every 2 points.
//   Sound: hit / bounce / point on the audio jack (AUD_PWM).
//   LEDs: LD0..3 buttons (P1 fore/back, P2 fore/back), LD4..11 aim position
//         (SW2 on) or camera view (SW2 off), LD12/13 server P1/P2,
//         LD14 game over, LD15 render overrun (should never light).
//   Score: panel in the top-right corner of the picture.
//
// Files to add to the Vivado project (design sources):
//   RTL : hardware_pipeline/*.v (pipeline), tt_game_top_files/tt_top.v,
//         tt_game.v, tt_game_core.v, tt_obj_animator.v, tt_score_overlay.v,
//         tt_buttons.v, tt_view_loader.v, tt_sound.v, tt_tilt.v,
//         hardware_pipeline/game_files/adxl362_reader.v
//   data: vertices.mem, matrix.mem, tt_views.mem, tt_scene_map.vh, tt_game_map.vh
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
    input  wire [5:0]  SW,
    output wire [15:0] LED,

    output wire        ACL_SCLK,
    output wire        ACL_MOSI,
    input  wire        ACL_MISO,
    output wire        ACL_CSN,

    inout  wire        AUD_PWM,         // open drain: driven low or released
    output wire        AUD_SD,

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

    wire aud_pwm_o;
    assign AUD_PWM = aud_pwm_o ? 1'bz : 1'b0;

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
        .acl_sclk   (ACL_SCLK),
        .acl_mosi   (ACL_MOSI),
        .acl_miso   (ACL_MISO),
        .acl_csn    (ACL_CSN),
        .aud_pwm_o  (aud_pwm_o),
        .aud_sd     (AUD_SD),
        .vga_r      (VGA_R),
        .vga_g      (VGA_G),
        .vga_b      (VGA_B),
        .vga_hs     (VGA_HS),
        .vga_vs     (VGA_VS)
    );

endmodule

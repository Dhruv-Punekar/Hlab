`timescale 1ns / 1ps

// ============================================================================
// Module: tt_top   (board top, Nexys A7-100T, xc7a100tcsg324-1)
//         Table-tennis demo scene, step 1: static scene.
//
//   CLK100MHZ -> MMCME2_BASE -> clk_render 100 MHz, clk_vga 25.175 MHz
//   CPU_RESETN (red button) -> reset: background clear + one render
//
//   Scene (made by mem_files/tt_game/gen_tt_scene.py): sky + floor are the
//   two-colour framebuffer clear (no rasterizer time), floor grid lines, blue
//   table with white lines, net with posts, two rackets and an orange ball.
//   ~333 triangles; clear 1.2 ms + render ~1.8 ms = ~3.0 ms, which fits the
//   3.97 ms window in which the VGA does not read the framebuffer (lines 440
//   .. 39), so a later per-frame redraw can be tear-free (see cube_top).
//
// Files to add to the Vivado project (design sources):
//   vertices.mem          mem_files/tt_game  -> vertex_memory
//   matrix.mem            mem_files/tt_game  -> matrix_memory (camera)
//   recip_lut_q1_17.mem   mem_files          -> triangle_setup
//   tt_scene_map.vh       mem_files/tt_game  -> included below (Verilog header)
//   vertex_memory / matrix_memory load the fixed names "vertices.mem" and
//   "matrix.mem": use a separate Vivado project for this demo (as for the
//   cube and the picture).
//
// Motion (later steps): tt_scene_map.vh gives the vertex range and start
// centre of each moving object (TT_RACKET_P_*, TT_RACKET_O_*, TT_BALL_*).
// They are stored last in vertices.mem, so they can be moved by rewriting
// those vertices through vertex_memory's write port (hardware_pipeline ties
// it off today) or by a per-object matrix, then re-rendering in the VGA
// window with rst_render_n, as cube_top does. The ports below are kept
// identical to cube_top / image_top so the same XDC works unchanged.
//
// Constraints: the same XDC as cube_top (same ports, MMCM instance u_mmcm).
// Needs hardware_pipeline.v with the CLEAR_COLOR_* / CLEAR_SPLIT_Y parameters.
// ============================================================================
module tt_top #(
    parameter RECIP_LUT_FILE = "recip_lut_q1_17.mem",
    parameter Z_FRAC_OUT     = 9,
    parameter CULL_MODE      = 1        // gen_tt_scene.py winds triangles CCW
)(
    input  wire       CLK100MHZ,
    input  wire       CPU_RESETN,       // red button, active low

    output wire [3:0] VGA_R,
    output wire [3:0] VGA_G,
    output wire [3:0] VGA_B,
    output wire       VGA_HS,
    output wire       VGA_VS
);

`include "tt_scene_map.vh"

    // ========================================================================
    // 1) CLOCKING: one MMCM -> 100 MHz render clock + 25.175 MHz pixel clock
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
    // 2) RESET: button synchronised, MMCM lock as asynchronous reset
    // ========================================================================
    (* ASYNC_REG = "TRUE" *) reg [1:0] btn_sync = 2'b00;
    always @(posedge clk_render or negedge mmcm_locked)
        if (!mmcm_locked) btn_sync <= 2'b00;
        else              btn_sync <= {btn_sync[0], CPU_RESETN};
    wire rst_n = btn_sync[1];

    // ========================================================================
    // 3) THE PIPELINE (draws the scene once after every reset)
    // ========================================================================
    wire render_done;   // high once the scene is complete (LED / ILA / motion sequencer)

    hardware_pipeline #(
        .NUM_VERTICES       (TT_NUM_VERTICES),
        .VERTEX_DEPTH       (TT_VERTEX_DEPTH),
        .RECIP_LUT_FILE     (RECIP_LUT_FILE),
        .LOOP_RENDER        (0),
        .Z_FRAC_OUT         (Z_FRAC_OUT),
        .CULL_MODE          (CULL_MODE),
        .CLEAR_COLOR_TOP    (TT_SKY_RGB444),     // sky
        .CLEAR_COLOR_BOTTOM (TT_FLOOR_RGB444),   // floor
        .CLEAR_SPLIT_Y      (TT_HORIZON_Y)       // far edge of the floor grid
    ) u_pipeline (
        .clk          (clk_render),
        .clk_vga      (clk_vga),
        .rst_n        (rst_n),
        .rst_render_n (1'b1),           // step 1: no re-render yet (motion step)
        .mat_we       (1'b0),           // camera matrix = matrix.mem
        .mat_addr     (4'd0),
        .mat_data     (32'd0),
        .render_done  (render_done),
        .vga_r        (VGA_R),
        .vga_g        (VGA_G),
        .vga_b        (VGA_B),
        .vga_hs       (VGA_HS),
        .vga_vs       (VGA_VS)
    );

endmodule

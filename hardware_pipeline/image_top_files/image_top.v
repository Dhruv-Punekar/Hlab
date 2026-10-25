`timescale 1ns / 1ps

// ============================================================================
// Module: image_top   (board top, Nexys A7-100T, xc7a100tcsg324-1)
//         Static picture: the low-poly image made by image_to_mem.py
//         (default run: 1121 triangles = 3363 vertices, identity matrix).
//
//   CLK100MHZ -> MMCME2_BASE -> clk_render 100 MHz, clk_vga 25.175 MHz
//   CPU_RESETN (red button) -> reset: z/framebuffer clear + one render
//
//   After reset the picture is drawn once (~1.2 ms clear + ~2.8 ms render)
//   and then stays on screen; nothing else runs. No matrix updates, so the
//   matrix write port and the render-only reset of hardware_pipeline are
//   tied off.
//
// Files (from the image_to_mem.py output folder) -> add as design sources:
//   vertices.mem          3363 vertices, padded to 4096 lines -> vertex_memory
//   matrix.mem            identity                             -> matrix_memory
//   recip_lut_q1_17.mem   reciprocal ROM (same file as for the cube)
//   vertex_memory / matrix_memory load the fixed names "vertices.mem" and
//   "matrix.mem", so use a separate Vivado project (or folder) for the picture,
//   or swap the files when switching between cube and picture.
//
// NUM_VERTICES / VERTEX_DEPTH must match scene_info.txt of the run you use.
// image_to_mem.py writes CCW triangles -> CULL_MODE 1 (or 0) is correct.
//
// BRAM: vertex RAM 4096 x 144 ~ 16 RAMB36, framebuffer ~45, z-buffer ~59,
//       reciprocal ROMs ~1  -> ~121 of 135. (--triangles 680 -> VERTEX_DEPTH
//       2048, ~8 RAMB36 for the vertex RAM.)
//
// Constraints: the same XDC as cube_top (same ports, MMCM instance u_mmcm).
// Needs hardware_pipeline.v with the rst_render_n / mat_* / render_done ports.
// ============================================================================
module image_top #(
    parameter NUM_VERTICES   = 3363,    // scene_info.txt: NUM_VERTICES
    parameter VERTEX_DEPTH   = 4096,    // scene_info.txt: VERTEX_DEPTH
    parameter RECIP_LUT_FILE = "recip_lut_q1_17.mem",
    parameter Z_FRAC_OUT     = 9,
    parameter CULL_MODE      = 1        // image_to_mem.py default winding: CCW
)(
    input  wire       CLK100MHZ,
    input  wire       CPU_RESETN,       // red button, active low

    output wire [3:0] VGA_R,
    output wire [3:0] VGA_G,
    output wire [3:0] VGA_B,
    output wire       VGA_HS,
    output wire       VGA_VS
);

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
    // 3) THE PIPELINE (draws the picture once after every reset)
    // ========================================================================
    wire render_done;   // high once the picture is complete (handy for an LED / ILA)

    hardware_pipeline #(
        .NUM_VERTICES   (NUM_VERTICES),
        .VERTEX_DEPTH   (VERTEX_DEPTH),
        .RECIP_LUT_FILE (RECIP_LUT_FILE),
        .LOOP_RENDER    (0),
        .Z_FRAC_OUT     (Z_FRAC_OUT),
        .CULL_MODE      (CULL_MODE)
    ) u_pipeline (
        .clk          (clk_render),
        .clk_vga      (clk_vga),
        .rst_n        (rst_n),
        .rst_render_n (1'b1),           // no re-render: static picture
        .mat_we       (1'b0),           // matrix stays = matrix.mem (identity)
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

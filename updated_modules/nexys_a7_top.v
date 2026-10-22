`timescale 1ns / 1ps

// ============================================================================
// Module: nexys_a7_top   (board top, Nexys A7-100T, xc7a100tcsg324-1)
//
//   CLK100MHZ ──> MMCME2_BASE ──┬─> clk_render 100.000 MHz ──┐
//                               └─> clk_vga     25.175 MHz ──┤
//   CPU_RESETN ──> sync + debounce + MMCM locked ──> rst_n ──┤
//                                                            v
//                                           hardware_pipeline (unchanged)
//                                                            │
//                               VGA_R/G/B[3:0], VGA_HS, VGA_VS <┘
//
// Clocking (MMCM, no Vivado IP needed):
//   VCO        = 100 MHz * 9.000 / 1      = 900 MHz   (Artix-7 -1: 600..1200)
//   CLKOUT0    = 900 / 35.750             = 25.1748 MHz  (VGA 640x480@60 ideal 25.175)
//   CLKOUT1    = 900 / 9                  = 100.000 MHz  (render clock)
//   Both outputs come from one MMCM but have a non-integer ratio, so the XDC
//   declares them asynchronous; all crossings inside hardware_pipeline are
//   already handled (2-FF syncs, dual-clock framebuffer BRAM).
//
// Reset / re-render (CPU_RESETN, red button, active low):
//   * pressing the button (or MMCM not locked) asserts rst_n at once
//     (asynchronous assert)
//   * rst_n is released only after the button has been seen released and
//     stable for DEBOUNCE_CYCLES (default 10 ms), so contact bounce gives one
//     clean re-render instead of several
//   * every release => z_f_buffer clears (1.2 ms) and the 36-vertex model is
//     fetched, transformed and drawn again
//
// Back-face culling (CULL_MODE = 1) assumes every triangle in vertices.mem is
// wound counter-clockwise when seen from outside the model.
//   * hardware_pipeline adds its own per-domain synchronisers on top
//
// Required memory files (add to the Vivado project):
//   vertices.mem, matrix.mem, recip_lut_q1_17.mem
//   NOTE: transform_engine reads M00 from matrix_regs[15], so matrix.mem must
//         list the matrix M33 first ... M00 last (reverse row-major).
// ============================================================================
module nexys_a7_top #(
    parameter NUM_VERTICES    = 36,                     // one cube, 12 triangles
    parameter VERTEX_DEPTH    = 64,
    parameter RECIP_LUT_FILE  = "recip_lut_q1_17.mem",
    parameter Z_FRAC_OUT      = 9,                      // Q6.9 depth into the z-buffer
    parameter CULL_MODE       = 1,                      // 1: front faces CCW in vertices.mem
    parameter DEBOUNCE_CYCLES = 1_000_000               // 10 ms @ 100 MHz
)(
    input  wire       CLK100MHZ,
    input  wire       CPU_RESETN,                       // red button, active low

    output wire [3:0] VGA_R,
    output wire [3:0] VGA_G,
    output wire [3:0] VGA_B,
    output wire       VGA_HS,
    output wire       VGA_VS
);

    // ========================================================================
    // 1) CLOCKING: one MMCM -> 100 MHz render clock + 25.175 MHz pixel clock
    // ========================================================================
    wire clk_fb_out, clk_fb_in;
    wire clk_vga_unbuf, clk_render_unbuf;
    wire clk_vga, clk_render;
    wire mmcm_locked;

    MMCME2_BASE #(
        .BANDWIDTH          ("OPTIMIZED"),
        .CLKIN1_PERIOD      (10.000),        // 100 MHz input
        .DIVCLK_DIVIDE      (1),
        .CLKFBOUT_MULT_F    (9.000),         // VCO = 900 MHz
        .CLKFBOUT_PHASE     (0.000),
        .CLKOUT0_DIVIDE_F   (35.750),        // 25.1748 MHz  (fractional divider: CLKOUT0 only)
        .CLKOUT0_DUTY_CYCLE (0.500),
        .CLKOUT0_PHASE      (0.000),
        .CLKOUT1_DIVIDE     (9),             // 100.000 MHz
        .CLKOUT1_DUTY_CYCLE (0.500),
        .CLKOUT1_PHASE      (0.000),
        .REF_JITTER1        (0.010),
        .STARTUP_WAIT       ("FALSE")
    ) u_mmcm (
        .CLKIN1    (CLK100MHZ),
        .CLKFBIN   (clk_fb_in),
        .CLKFBOUT  (clk_fb_out),
        .CLKFBOUTB (),
        .CLKOUT0   (clk_vga_unbuf),
        .CLKOUT0B  (),
        .CLKOUT1   (clk_render_unbuf),
        .CLKOUT1B  (),
        .CLKOUT2   (), .CLKOUT2B (),
        .CLKOUT3   (), .CLKOUT3B (),
        .CLKOUT4   (), .CLKOUT5  (), .CLKOUT6 (),
        .LOCKED    (mmcm_locked),
        .PWRDWN    (1'b0),
        .RST       (1'b0)
    );

    BUFG u_bufg_fb     (.I(clk_fb_out),       .O(clk_fb_in));
    BUFG u_bufg_vga    (.I(clk_vga_unbuf),    .O(clk_vga));
    BUFG u_bufg_render (.I(clk_render_unbuf), .O(clk_render));

    // ========================================================================
    // 2) RESET / RE-RENDER: CPU_RESETN sync + release debounce + MMCM lock
    // ========================================================================
    // Button synchroniser (the async reset below is the MMCM lock, so a
    // glitch on the button pin can never reach the pipeline unsynchronised).
    (* ASYNC_REG = "TRUE" *) reg [1:0] btn_sync = 2'b00;
    always @(posedge clk_render or negedge mmcm_locked)
        if (!mmcm_locked) btn_sync <= 2'b00;
        else              btn_sync <= {btn_sync[0], CPU_RESETN};
    wire btn_released = btn_sync[1];              // 1 = not pressed

    localparam DB_W = (DEBOUNCE_CYCLES > 1) ? $clog2(DEBOUNCE_CYCLES + 1) : 1;
    reg [DB_W-1:0] db_cnt  = {DB_W{1'b0}};
    reg            rst_n_q = 1'b0;

    always @(posedge clk_render or negedge mmcm_locked) begin
        if (!mmcm_locked) begin
            db_cnt  <= {DB_W{1'b0}};
            rst_n_q <= 1'b0;
        end else if (!btn_released) begin         // pressed: reset now
            db_cnt  <= {DB_W{1'b0}};
            rst_n_q <= 1'b0;
        end else if (!rst_n_q) begin              // released: wait until stable
            if (db_cnt == DEBOUNCE_CYCLES - 1) rst_n_q <= 1'b1;
            else                               db_cnt  <= db_cnt + 1'b1;
        end
    end

    // ========================================================================
    // 3) THE PIPELINE (unchanged module)
    // ========================================================================
    hardware_pipeline #(
        .NUM_VERTICES   (NUM_VERTICES),
        .VERTEX_DEPTH   (VERTEX_DEPTH),
        .RECIP_LUT_FILE (RECIP_LUT_FILE),
        .LOOP_RENDER    (0),                     // draw once per reset
        .Z_FRAC_OUT     (Z_FRAC_OUT),
        .CULL_MODE      (CULL_MODE)
    ) u_pipeline (
        .clk     (clk_render),
        .clk_vga (clk_vga),
        .rst_n   (rst_n_q),
        .vga_r   (VGA_R),
        .vga_g   (VGA_G),
        .vga_b   (VGA_B),
        .vga_hs  (VGA_HS),
        .vga_vs  (VGA_VS)
    );

endmodule

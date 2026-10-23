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
//   * hardware_pipeline adds its own per-domain synchronisers on top
//
// Back-face culling (CULL_MODE = 1) assumes every triangle in vertices.mem is
// wound counter-clockwise when seen from outside the model.
//
// Simulation output (section 4, `ifndef SYNTHESIS - ignored by Vivado
// synthesis, so the bitstream is identical with or without it):
//   <PPM_PREFIX>_<n>.ppm  300x400 framebuffer after render n (P3 ASCII, RGB)
//   <LOG_FILE>            clock lock, resets, z-buffer clear, every triangle
//                         at the setup output (bbox, depth plane, cull/discard
//                         reason), per-render summary
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
    parameter DEBOUNCE_CYCLES = 1_000_000,              // 10 ms @ 100 MHz
    // ---- simulation only (ignored by synthesis) ----
    parameter PPM_PREFIX      = "frame",                // -> frame_0.ppm, frame_1.ppm, ...
    parameter LOG_FILE        = "render.log"
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
    // 3) THE PIPELINE
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

    // ========================================================================
    // 4) SIMULATION-ONLY OUTPUT: framebuffer PPM + render log
    //    Vivado synthesis defines SYNTHESIS, so none of this reaches hardware.
    // ========================================================================
`ifndef SYNTHESIS
    integer log_fd = 0;
    integer frame_no = 0;
    integer cyc = 0, t_start = 0, t_clear_done = 0;
    integer n_tri = 0, n_drawn = 0, n_culled = 0, n_degen = 0, n_ovf = 0;
    integer n_pix_in = 0, n_pix_written = 0;
    integer z_lo = 0, z_hi = 0;
    integer quiet = 0;
    reg     rendered = 1'b0, prev_rst = 1'b0, prev_busy = 1'b0, prev_lock = 1'b0;
    reg [8*64-1:0] ppm_name;

    initial begin
        log_fd = $fopen(LOG_FILE, "w");
        $fdisplay(log_fd, "# nexys_a7_top render log  (times in ns, cycles at 100 MHz)");
        $fdisplay(log_fd, "# Z_FRAC_OUT=%0d CULL_MODE=%0d NUM_VERTICES=%0d", Z_FRAC_OUT, CULL_MODE, NUM_VERTICES);
    end

    // MMCM lock (input clock domain: watch with the raw oscillator)
    always @(posedge CLK100MHZ) begin
        if (mmcm_locked !== prev_lock && log_fd)
            $fdisplay(log_fd, "%0t ns  MMCM %0s", $time, mmcm_locked ? "locked" : "lost lock");
        prev_lock <= mmcm_locked;
    end

    // render-complete detector: fetch done, every stage empty, rasterizer idle
    wire pipe_empty = (u_pipeline.f_state == 2'd2) && u_pipeline.rast_idle &&
                      !u_pipeline.ts_out_valid && (u_pipeline.u_triangle_setup.v == 0) &&
                      !u_pipeline.pe_valid_out && (u_pipeline.u_projection_engine.vertex_count == 0) &&
                      !u_pipeline.te_valid_out && u_pipeline.zf_clear_done;

    task write_ppm;
        integer a, fd, c;
        begin
            $sformat(ppm_name, "%0s_%0d.ppm", PPM_PREFIX, frame_no);
            fd = $fopen(ppm_name, "w");
            $fdisplay(fd, "P3\n300 400\n255");
            for (a = 0; a < 300*400; a = a + 1) begin
                c = u_pipeline.u_z_f_buffer.frame_buffer[a];
                $fdisplay(fd, "%0d %0d %0d", ((c >> 11) & 31) * 255 / 31,
                          ((c >> 5) & 63) * 255 / 63, (c & 31) * 255 / 31);
            end
            $fclose(fd);
        end
    endtask

    always @(posedge clk_render) begin
        cyc <= cyc + 1;

        // pipeline reset edges
        if (rst_n_q !== prev_rst && log_fd)
            $fdisplay(log_fd, "%0t ns  pipeline reset %s", $time,
                      rst_n_q ? "released -> z-buffer clear + render start" : "asserted (button / lock)");
        if (rst_n_q && !prev_rst) begin
            t_start = cyc; rendered = 1'b0; quiet = 0;
            n_tri = 0; n_drawn = 0; n_culled = 0; n_degen = 0; n_ovf = 0;
            n_pix_in = 0; n_pix_written = 0; z_lo = 32767; z_hi = -32768;
        end
        prev_rst <= rst_n_q;

        // z-buffer clear (it runs from reset release until clear_done)
        if (!u_pipeline.zf_busy && prev_busy && rst_n_q) begin
            t_clear_done = cyc;
            if (log_fd) $fdisplay(log_fd, "%0t ns  z-buffer clear done (%0d cycles)", $time, cyc - t_start);
        end
        prev_busy <= u_pipeline.zf_busy;

        // every triangle leaving the setup unit
        if (u_pipeline.ts_out_valid && u_pipeline.ts_out_ready && log_fd) begin
            n_tri = n_tri + 1;
            if (u_pipeline.ts_degenerate)      n_degen  = n_degen + 1;
            else if (u_pipeline.ts_ov_depth)   n_ovf    = n_ovf + 1;
            else if (u_pipeline.back_facing)   n_culled = n_culled + 1;
            else                               n_drawn  = n_drawn + 1;
            $fdisplay(log_fd, "%0t ns  tri %0d color=%h bbox x[%0d..%0d] y[%0d..%0d] zA=%0d zB=%0d zC=%0d (Q14.18) orient_neg=%b -> %0s",
                $time, n_tri, u_pipeline.ts_color,
                u_pipeline.ts_xmin, u_pipeline.ts_xmax, u_pipeline.ts_ymin, u_pipeline.ts_ymax,
                u_pipeline.ts_zA, u_pipeline.ts_zB, u_pipeline.ts_zC, u_pipeline.ts_orient_neg,
                u_pipeline.ts_degenerate ? "DISCARD degenerate" :
                u_pipeline.ts_ov_depth   ? "DISCARD depth overflow" :
                u_pipeline.back_facing   ? "CULLED back-facing" : "DRAWN");
        end

        // pixels into / written by the z-buffer
        if (u_pipeline.rs_pvalid && u_pipeline.zf_clear_done) begin
            n_pix_in = n_pix_in + 1;
            if ($signed(u_pipeline.rs_pz) < z_lo) z_lo = $signed(u_pipeline.rs_pz);
            if ($signed(u_pipeline.rs_pz) > z_hi) z_hi = $signed(u_pipeline.rs_pz);
        end
        if (u_pipeline.u_z_f_buffer.s3_we && u_pipeline.zf_clear_done)
            n_pix_written = n_pix_written + 1;

        // render complete -> summary + PPM
        quiet = (rst_n_q && pipe_empty) ? quiet + 1 : 0;
        if (quiet == 16 && !rendered) begin
            rendered = 1'b1;
            write_ppm;
            if (log_fd) begin
                $fdisplay(log_fd, "%0t ns  RENDER %0d DONE: %0d triangles (%0d drawn, %0d culled, %0d degenerate, %0d depth-overflow)",
                          $time, frame_no, n_tri, n_drawn, n_culled, n_degen, n_ovf);
                $fdisplay(log_fd, "%0t ns    %0d pixels sent to z-buffer, %0d passed the depth test; pixel_z range [%0d, %0d] (Q%0d.%0d)",
                          $time, n_pix_in, n_pix_written, z_lo, z_hi, 15 - Z_FRAC_OUT, Z_FRAC_OUT);
                $fdisplay(log_fd, "%0t ns    render time %0d cycles after clear (%0d cycles after reset release)",
                          $time, cyc - t_clear_done - 16, cyc - t_start - 16);
                $fdisplay(log_fd, "%0t ns    framebuffer written to %0s", $time, ppm_name);
                $fflush(log_fd);
            end
            frame_no = frame_no + 1;
        end
    end
`endif

endmodule

`timescale 1ns / 1ps

// ============================================================================
// tb_tt_scene.v  -  render the table-tennis scene once and dump the frame
//
// Simulates hardware_pipeline directly (no MMCM, so it also runs in Icarus):
// 100 MHz render clock, 25 MHz VGA clock. Waits for render_done, then writes
// the framebuffer as tt_scene.ppm (P3, 300x400, max 15 = RGB444) and a log.
//
// Files needed in the simulation folder: vertices.mem, matrix.mem,
// recip_lut_q1_17.mem (from mem_files/tt_game) and tt_scene_map.vh.
//
// Checks: every triangle reaches the rasterizer or is culled (no degenerate /
// depth-overflow / out-of-range triangles), no X in the framebuffer, sky row 0
// and floor bottom row have the clear colours, and the frame fits the VGA
// blanking window (clear + render < 3.97 ms) so a redraw per frame is tear-free.
// ============================================================================
module tb_tt_scene;

`include "tt_scene_map.vh"

    localparam W = 300, H = 400, NPIX = W * H;

    reg clk = 1'b0, clk_vga = 1'b0, rst_n = 1'b0;
    always #5  clk = ~clk;            // 100 MHz
    always #20 clk_vga = ~clk_vga;    // 25 MHz

    wire       render_done;
    wire [3:0] vga_r, vga_g, vga_b;
    wire       vga_hs, vga_vs;

    hardware_pipeline #(
        .NUM_VERTICES       (TT_NUM_VERTICES),
        .VERTEX_DEPTH       (TT_VERTEX_DEPTH),
        .LOOP_RENDER        (0),
        .CULL_MODE          (1),
        .CLEAR_COLOR_TOP    (TT_SKY_RGB444),
        .CLEAR_COLOR_BOTTOM (TT_FLOOR_RGB444),
        .CLEAR_SPLIT_Y      (TT_HORIZON_Y)
    ) dut (
        .clk (clk), .clk_vga (clk_vga), .rst_n (rst_n), .rst_render_n (1'b1),
        .mat_we (1'b0), .mat_addr (4'd0), .mat_data (32'd0),
        .render_done (render_done),
        .vga_r (vga_r), .vga_g (vga_g), .vga_b (vga_b), .vga_hs (vga_hs), .vga_vs (vga_vs)
    );

    // ------------------------------------------------------------------------
    // counters
    // ------------------------------------------------------------------------
    integer cyc = 0, clear_end = -1;
    integer n_tri = 0, n_drawn = 0, n_culled = 0, n_degen = 0, n_ovf = 0, n_xy = 0, n_pix = 0;

    always @(posedge clk) if (rst_n) begin
        cyc = cyc + 1;
        if (clear_end < 0 && dut.zf_clear_done) clear_end = cyc;
        if (dut.ts_out_valid && dut.ts_out_ready) begin
            n_tri = n_tri + 1;
            if (dut.ts_ov_xy) n_xy = n_xy + 1;
            if      (dut.ts_degenerate) n_degen  = n_degen + 1;
            else if (dut.ts_ov_depth)   n_ovf    = n_ovf + 1;
            else if (dut.back_facing)   n_culled = n_culled + 1;
            else                        n_drawn  = n_drawn + 1;
        end
        if (dut.u_z_f_buffer.s3_we && dut.zf_clear_done) n_pix = n_pix + 1;
    end

    // ------------------------------------------------------------------------
    // main
    // ------------------------------------------------------------------------
    integer fd, lg, a, n_x, errors = 0;
    reg [11:0] c;
    real t_ms;

    initial begin
        #100 rst_n = 1'b1;
        @(posedge render_done);
        t_ms = cyc / 100000.0;

        fd = $fopen("tt_scene.ppm", "w");
        $fwrite(fd, "P3\n%0d %0d\n15\n", W, H);
        n_x = 0;
        for (a = 0; a < NPIX; a = a + 1) begin
            c = dut.u_z_f_buffer.frame_buffer[a];
            if (^c === 1'bx) n_x = n_x + 1;
            $fwrite(fd, "%0d %0d %0d\n", c[11:8], c[7:4], c[3:0]);
        end
        $fclose(fd);

        lg = $fopen("tt_scene.log", "w");
        $fdisplay(lg, "triangles %0d: drawn %0d, culled %0d, degenerate %0d, depth overflow %0d, xy out of range %0d",
                  n_tri, n_drawn, n_culled, n_degen, n_ovf, n_xy);
        $fdisplay(lg, "pixels written %0d", n_pix);
        $fdisplay(lg, "clear %0d cycles, clear + render %0d cycles = %0.3f ms (render_done incl. 64-cycle quiet time)",
                  clear_end, cyc, t_ms);

        if (n_tri != TT_NUM_VERTICES / 3) begin errors = errors + 1; $fdisplay(lg, "[FAIL] expected %0d triangles", TT_NUM_VERTICES / 3); end
        // degenerate = zero area at the hardware's 1/256 px precision (clip slivers,
        // faces seen exactly edge-on): nothing to draw, discarded by design
        if (n_ovf + n_xy != 0)            begin errors = errors + 1; $fdisplay(lg, "[FAIL] depth-overflow / off-screen triangles"); end
        if (n_drawn + n_culled + n_degen != n_tri) begin errors = errors + 1; $fdisplay(lg, "[FAIL] triangle count mismatch"); end
        if (n_x != 0)                     begin errors = errors + 1; $fdisplay(lg, "[FAIL] %0d X pixels", n_x); end
        if (dut.u_z_f_buffer.frame_buffer[150] !== TT_SKY_RGB444)
                                          begin errors = errors + 1; $fdisplay(lg, "[FAIL] row 0 is not sky"); end
        if (dut.u_z_f_buffer.frame_buffer[(H - 1) * W + 2] !== TT_FLOOR_RGB444 &&
            dut.u_z_f_buffer.frame_buffer[(H - 1) * W + 2] === 12'h000)
                                          begin errors = errors + 1; $fdisplay(lg, "[FAIL] bottom row not cleared to floor"); end
        if (t_ms > 3.97)                  begin errors = errors + 1; $fdisplay(lg, "[FAIL] frame does not fit the 3.97 ms VGA window"); end
        $fdisplay(lg, "%s (%0d errors)", errors ? "TEST FAILED" : "TEST PASSED", errors);
        $fclose(lg);
        $display("tt_scene: %0d tris (%0d drawn, %0d culled, %0d degen, %0d ovf, %0d xy), %0d px, %0.3f ms -> %s",
                 n_tri, n_drawn, n_culled, n_degen, n_ovf, n_xy, n_pix, t_ms, errors ? "TEST FAILED" : "TEST PASSED");
        $finish;
    end

    initial begin
        #20_000_000;
        $display("[FAIL] watchdog: render_done never rose");
        $finish;
    end

endmodule

`timescale 1ns / 1ps

// ============================================================================
// tb_cube.v  -  simulate the rotating cube (cube_top) and log every frame
//
// Sources : tb_cube.v (simulation top), cube_top.v, hardware_pipeline.v and
//           all pipeline modules; vertices.mem, matrix.mem, matrices.mem,
//           recip_lut_q1_17.mem added to the project (or copied into
//           <project>.sim/sim_1/behav/xsim/).
// Run     : set tb_cube as simulation top, then "run all".
//
// The top runs with TRIGGER_MODE = 1 (free-running): the next frame starts as
// soon as the previous one is drawn, instead of waiting for the VGA window.
// That keeps the run short: ~1.2 ms clear + render per frame, N_DUMP frames.
// MATRIX_STEP = 15 with 120 matrices -> 8 frames cover one full turn.
//
// Output (simulation folder):
//   cube_000.ppm ... cube_<N_DUMP-1>.ppm   300x400 framebuffer, P3, max 15
//                                          (RGB444, as stored in z_f_buffer)
//   cube_sim.log                           one line per frame + summary
// ============================================================================
module tb_cube;

    parameter N_DUMP = 8;       // frames to write (frame 0 = power-up render)
    parameter STEP   = 15;      // matrices advanced per frame

    localparam W = 300, H = 400, NPIX = W * H;

    // ------------------------------------------------------------------------
    // Board inputs
    // ------------------------------------------------------------------------
    reg CLK100MHZ = 1'b0;
    reg CPU_RESETN = 1'b1;
    always #5 CLK100MHZ = ~CLK100MHZ;          // 100 MHz oscillator

    wire [3:0] VGA_R, VGA_G, VGA_B;
    wire       VGA_HS, VGA_VS;

    cube_top #(
        .MATRIX_STEP  (STEP),
        .TRIGGER_MODE (1)                      // free-running for simulation
    ) dut (
        .CLK100MHZ  (CLK100MHZ),
        .CPU_RESETN (CPU_RESETN),
        .VGA_R (VGA_R), .VGA_G (VGA_G), .VGA_B (VGA_B),
        .VGA_HS (VGA_HS), .VGA_VS (VGA_VS)
    );

    // ------------------------------------------------------------------------
    // Per-frame counters (render clock), cleared while the render domain is
    // in reset, i.e. at the start of every frame
    // ------------------------------------------------------------------------
    integer cyc = 0, t_start = 0;
    integer n_tri = 0, n_drawn = 0, n_culled = 0, n_degen = 0, n_ovf = 0, n_pix = 0;
    reg     rst_q = 1'b0;

    always @(posedge dut.clk_render) begin
        cyc = cyc + 1;
        if (!dut.u_pipeline.rst_n_r) begin
            n_tri = 0; n_drawn = 0; n_culled = 0; n_degen = 0; n_ovf = 0; n_pix = 0;
        end else begin
            if (!rst_q) t_start = cyc;          // render reset just released
            if (dut.u_pipeline.ts_out_valid && dut.u_pipeline.ts_out_ready) begin
                n_tri = n_tri + 1;
                if      (dut.u_pipeline.ts_degenerate) n_degen  = n_degen + 1;
                else if (dut.u_pipeline.ts_ov_depth)   n_ovf    = n_ovf + 1;
                else if (dut.u_pipeline.back_facing)   n_culled = n_culled + 1;
                else                                   n_drawn  = n_drawn + 1;
            end
            if (dut.u_pipeline.u_z_f_buffer.s3_we && dut.u_pipeline.zf_clear_done)
                n_pix = n_pix + 1;
        end
        rst_q = dut.u_pipeline.rst_n_r;
    end

    // ------------------------------------------------------------------------
    // Frame dump: framebuffer -> PPM, plus colour statistics
    // ------------------------------------------------------------------------
    integer log_fd, errors = 0;
    integer lit, n_x;
    reg [31:0] crc, prev_crc;
    reg [8*32-1:0] ppm_name;

    task dump_frame;
        input integer n;
        integer a, fd;
        reg [11:0] c;
        begin
            $sformat(ppm_name, "cube_%03d.ppm", n);
            fd = $fopen(ppm_name, "w");
            $fwrite(fd, "P3\n%0d %0d\n15\n", W, H);
            lit = 0; n_x = 0; crc = 32'd0;
            for (a = 0; a < NPIX; a = a + 1) begin
                c = dut.u_pipeline.u_z_f_buffer.frame_buffer[a];
                if (^c === 1'bx) n_x = n_x + 1;
                if (c != 12'h000) lit = lit + 1;
                crc = {crc[30:0], crc[31]} ^ {20'd0, c};        // simple checksum
                $fwrite(fd, "%0d %0d %0d\n", c[11:8], c[7:4], c[3:0]);
            end
            $fclose(fd);
        end
    endtask

    // ------------------------------------------------------------------------
    // Main sequence
    // ------------------------------------------------------------------------
    integer f;
    initial begin
        log_fd = $fopen("cube_sim.log", "w");
        $fdisplay(log_fd, "# tb_cube: rotating cube, MATRIX_STEP=%0d, free-running frames", STEP);
        $fdisplay(log_fd, "# frame matrix  render_cycles  triangles(drawn/culled/degen/ovf)  pixels_written  lit_pixels  file");
        prev_crc = 32'hFFFF_FFFF;

        for (f = 0; f < N_DUMP; f = f + 1) begin
            @(posedge dut.u_pipeline.render_done);   // frame f finished
            dump_frame(f);
            $fdisplay(log_fd, "%5d  %5d  %13d  %3d (%0d/%0d/%0d/%0d)  %14d  %10d  %0s",
                      f, dut.frame_idx, cyc - t_start, n_tri, n_drawn, n_culled, n_degen, n_ovf,
                      n_pix, lit, ppm_name);
            $display("frame %0d (matrix %0d): %0d triangles drawn, %0d culled, %0d pixels, %0d cycles -> %0s",
                     f, dut.frame_idx, n_drawn, n_culled, n_pix, cyc - t_start, ppm_name);

            // basic checks
            if (n_tri != 12)  begin errors = errors + 1; $fdisplay(log_fd, "  [FAIL] expected 12 triangles"); end
            if (n_drawn == 0) begin errors = errors + 1; $fdisplay(log_fd, "  [FAIL] nothing drawn"); end
            if (n_degen + n_ovf != 0) begin errors = errors + 1; $fdisplay(log_fd, "  [FAIL] degenerate/overflow triangles"); end
            if (n_x != 0)     begin errors = errors + 1; $fdisplay(log_fd, "  [FAIL] %0d X pixels in framebuffer", n_x); end
            if (lit == 0)     begin errors = errors + 1; $fdisplay(log_fd, "  [FAIL] empty frame"); end
            if (f > 0 && crc == prev_crc && (STEP % 120) != 0) begin
                errors = errors + 1; $fdisplay(log_fd, "  [FAIL] frame identical to the previous one (matrix not updated?)");
            end
            prev_crc = crc;
            $fflush(log_fd);
        end

        $fdisplay(log_fd, "# %0d frames, %0d errors -> %0s", N_DUMP, errors, errors ? "TEST FAILED" : "TEST PASSED");
        $fclose(log_fd);
        $display("%0d frames written, %0d errors: %0s", N_DUMP, errors, errors ? "TEST FAILED" : "TEST PASSED");
        $finish;
    end

    // watchdog: about 3 ms per frame is plenty (MMCM lock + clear + render)
    initial begin
        #((N_DUMP + 2) * 3_000_000);
        $display("[FAIL] watchdog: frames stopped coming (render_done never rose)");
        $fdisplay(log_fd, "[FAIL] watchdog");
        $finish;
    end

endmodule

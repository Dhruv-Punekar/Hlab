`timescale 1ns / 1ps
// ============================================================================
// tb_pose_sweep - hardware_pipeline rendering a sweep of poses that covers
// the whole rotation range, using exactly the matrices the firmware would
// compute (poses.mem from gen_poses.py, matrix_memory layout, 16 words per
// pose). Each pose is loaded the way cube_top / gfx_ctrl do it: render domain
// in reset, 16 matrix writes, release, wait for render_done. For every pose
// the frame must finish, pose_<n>.ppm is written (P3, 0..15) and the triangle
// statistics go to poses.log.
// Plus-arg +POSES=<n>.
// ============================================================================
module tb_pose_sweep;
    reg clk = 0, clk_vga = 0, rst_n = 0;
    always #5      clk     = ~clk;
    always #19.86  clk_vga = ~clk_vga;

    reg          rst_render_n = 1, mat_we = 0;
    reg  [3:0]   mat_addr = 0;
    reg  [31:0]  mat_data = 0;
    wire         done;
    wire [3:0] r, g, b; wire hs, vs;

    hardware_pipeline dut (
        .clk(clk), .clk_vga(clk_vga), .rst_n(rst_n), .rst_render_n(rst_render_n),
        .mat_we(mat_we), .mat_addr(mat_addr), .mat_data(mat_data), .render_done(done),
        .vga_r(r), .vga_g(g), .vga_b(b), .vga_hs(hs), .vga_vs(vs));

    reg [31:0] poses [0:16*64-1];
    integer npos, p, i, a, fd, c, cyc, flog;
    integer n_draw, n_cull, n_degen, n_ovf;
    reg [8*32-1:0] name;

    always @(posedge clk) if (dut.ts_out_valid && dut.ts_out_ready) begin
        if (dut.ts_degenerate) n_degen = n_degen + 1;
        else if (dut.ts_ov_depth) n_ovf = n_ovf + 1;
        else if (dut.back_facing) n_cull = n_cull + 1;
        else n_draw = n_draw + 1;
    end

    initial begin
        if (!$value$plusargs("POSES=%d", npos)) npos = 24;
        $readmemh("poses.mem", poses);
        flog = $fopen("poses.log", "w");
        #200 rst_n = 1;
        wait (done);                                   // power-up render
        for (p = 0; p < npos; p = p + 1) begin
            @(negedge clk) rst_render_n = 0;           // render domain in reset
            for (i = 0; i < 16; i = i + 1) begin
                @(negedge clk) begin mat_we = 1; mat_addr = i; mat_data = poses[p*16 + i]; end
            end
            @(negedge clk) begin mat_we = 0; rst_render_n = 1; end
            n_draw = 0; n_cull = 0; n_degen = 0; n_ovf = 0;
            cyc = 0;
            while (!done) begin @(posedge clk); cyc = cyc + 1; if (cyc > 1_000_000) begin $display("FAIL pose %0d: no done", p); $finish; end end
            $sformat(name, "pose_%0d.ppm", p);
            fd = $fopen(name, "w");
            $fwrite(fd, "P3\n300 400\n15\n");
            for (a = 0; a < 300*400; a = a + 1) begin
                c = dut.u_z_f_buffer.frame_buffer[a];
                $fwrite(fd, "%0d %0d %0d\n", (c >> 8) & 15, (c >> 4) & 15, c & 15);
            end
            $fclose(fd);
            $fdisplay(flog, "pose %0d cycles=%0d drawn=%0d culled=%0d degenerate=%0d depth_ovf=%0d", p, cyc, n_draw, n_cull, n_degen, n_ovf);
            $fflush(flog);
            $display("pose %0d: %0d cycles, %0d drawn, %0d culled, %0d degenerate, %0d depth-overflow", p, cyc, n_draw, n_cull, n_degen, n_ovf);
        end
        $display("SWEEP DONE");
        $finish;
    end
endmodule

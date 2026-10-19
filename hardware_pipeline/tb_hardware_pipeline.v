`timescale 1ns / 1ps

// ============================================================================
// tb_hardware_pipeline.v  -  self-checking testbench for hardware_pipeline
//
// HOW IT WORKS (kept deliberately simple)
//   1. Each test builds a small scene (2 triangles = 6 vertices + 1 matrix)
//      in TB arrays.
//   2. While reset is held, the scene is written straight into the DUT
//      memories (vertex RAM, matrix regs, reciprocal ROM) through
//      hierarchical paths. So the TB does NOT need the .mem files.
//   3. A plain-Verilog reference model draws the same scene into ref_fb[].
//   4. Reset is released, the DUT clears its buffers and renders.
//   5. The DUT framebuffer is compared pixel-by-pixel with ref_fb[].
//   6. One test also checks a full VGA frame (sync timing + every RGB value)
//      and writes it to frame_out.ppm (open with any image viewer / GIMP).
//
// ALWAYS-ON CHECKERS
//   * valid/ready protocol on every link (valid must hold, data stable)
//   * handshake counters (vertices, triangles, drops, pixels)
//   * no X on VGA outputs after reset
//   * rasterizer pixels must lie inside the triangle's bounding box
//   * global watchdog
//
// RESULT: "TEST PASSED" / "TEST FAILED" at the end. Tests marked KNOWN BUG
//         reproduce a reported module bug and are counted separately.
//
// Run (Icarus):
//   iverilog -g2012 -o sim tb_hardware_pipeline.v hardware_pipeline.v rtl/*.v
//   vvp sim                (add +DUMP for a waveform file tb.vcd)
// ============================================================================
module tb_hardware_pipeline;

    // ------------------------------------------------------------------------
    // Clocks and reset
    // ------------------------------------------------------------------------
    reg clk     = 1'b0;     // 100 MHz
    reg clk_vga = 1'b0;     // 25 MHz
    reg rst_n   = 1'b0;

    always #5  clk     = ~clk;
    always #20 clk_vga = ~clk_vga;

    // ------------------------------------------------------------------------
    // DUT
    // ------------------------------------------------------------------------
    wire [3:0] vga_r, vga_g, vga_b;
    wire       vga_hs, vga_vs;

    localparam NV     = 6;          // vertices per scene (2 triangles)
    localparam W      = 300;        // framebuffer width
    localparam H      = 400;        // framebuffer height
    localparam NPIX   = W * H;

    hardware_pipeline #(
        .NUM_VERTICES (NV)
    ) dut (
        .clk     (clk),
        .clk_vga (clk_vga),
        .rst_n   (rst_n),
        .vga_r   (vga_r),
        .vga_g   (vga_g),
        .vga_b   (vga_b),
        .vga_hs  (vga_hs),
        .vga_vs  (vga_vs)
    );

    // ------------------------------------------------------------------------
    // Score keeping
    // ------------------------------------------------------------------------
    integer errors     = 0;
    integer known_bugs = 0;
    integer tests_run  = 0;

    task fail;
        input [8*96-1:0] msg;
        begin
            errors = errors + 1;
            $display("  [FAIL] %0s  (t=%0t)", msg, $time);
        end
    endtask

    task check_int;     // compare two integers, print PASS/FAIL
        input [8*48-1:0] what;
        input integer got;
        input integer exp;
        begin
            if (got == exp)
                $display("  [ ok ] %0s = %0d", what, got);
            else begin
                errors = errors + 1;
                $display("  [FAIL] %0s = %0d, expected %0d", what, got, exp);
            end
        end
    endtask

    // ------------------------------------------------------------------------
    // Colours (RGB565)
    // ------------------------------------------------------------------------
    localparam [15:0] RED   = 16'hF800;
    localparam [15:0] GREEN = 16'h07E0;
    localparam [15:0] BLUE  = 16'h001F;
    localparam [15:0] WHITE = 16'hFFFF;
    localparam [15:0] CYAN  = 16'h07FF;

    localparam signed [31:0] ONE = 32'sd256;   // 1.0 in Q24.8

    // ------------------------------------------------------------------------
    // Scene storage (what gets loaded into the DUT)
    // ------------------------------------------------------------------------
    reg signed [31:0] sv_x [0:NV-1];
    reg signed [31:0] sv_y [0:NV-1];
    reg signed [31:0] sv_z [0:NV-1];
    reg signed [31:0] sv_w [0:NV-1];
    reg        [15:0] sv_c [0:NV-1];
    reg signed [31:0] mat  [0:15];      // row-major: mat[r*4+c] = M[r][c]

    task set_vertex;
        input integer i;
        input integer x, y, z, w;       // Q24.8 raw values
        input [15:0] c;
        begin
            sv_x[i] = x; sv_y[i] = y; sv_z[i] = z; sv_w[i] = w; sv_c[i] = c;
        end
    endtask

    task set_identity;
        integer k;
        begin
            for (k = 0; k < 16; k = k + 1) mat[k] = 0;
            mat[0] = ONE; mat[5] = ONE; mat[10] = ONE; mat[15] = ONE;
        end
    endtask

    // second triangle = all vertices at the same point -> degenerate, dropped
    task set_degenerate_tri;
        input integer first;
        begin
            set_vertex(first+0, 0, 0, 0, ONE, CYAN);
            set_vertex(first+1, 0, 0, 0, ONE, CYAN);
            set_vertex(first+2, 0, 0, 0, ONE, CYAN);
        end
    endtask

    // ------------------------------------------------------------------------
    // Load scene into the DUT (only while reset is held)
    // ------------------------------------------------------------------------
    task load_scene_into_dut;
        integer k;
        begin
            for (k = 0; k < NV; k = k + 1)
                dut.u_vertex_memory.ram[k] = {sv_x[k], sv_y[k], sv_z[k], sv_w[k], sv_c[k]};
            for (k = 0; k < 16; k = k + 1)
                dut.u_matrix_memory.matrix_regs[k] = mat[k];
        end
    endtask

    // Reciprocal ROM for the triangle setup unit: Q1.17 value of 1/m,
    // m = 1.addr (10-bit mantissa). Filled by the TB so no .mem file is needed.
    task load_recip_rom;
        integer a;
        real m;
        begin
            for (a = 0; a < 1024; a = a + 1) begin
                m = 1.0 + a / 1024.0;
                dut.u_triangle_setup.recip_rom[a] = $rtoi(131072.0 / m + 0.5);
            end
        end
    endtask

    // ------------------------------------------------------------------------
    // REFERENCE MODEL
    // ------------------------------------------------------------------------
    reg        [15:0] ref_fb [0:NPIX-1];   // expected colour
    integer           ref_z  [0:NPIX-1];   // expected depth (Q24.8 NDC)
    integer           ref_pixels;          // pixels inside all triangles
    integer           ref_tris;            // non-degenerate triangles

    task ref_clear;
        integer a;
        begin
            for (a = 0; a < NPIX; a = a + 1) begin
                ref_fb[a] = 16'h0000;
                ref_z[a]  = 32'h7FFFFFFF;  // "infinitely far"
            end
            ref_pixels = 0;
            ref_tris   = 0;
        end
    endtask

    // Draw one flat-depth triangle. Coordinates are screen space in Q24.8.
    // Sample at pixel centres, ties count as inside (same rule as the HW).
    // All maths is exact 64-bit integer.
    task ref_draw_tri;
        input integer x0, y0, x1, y1, x2, y2;   // Q24.8 screen coords
        input integer zval;                     // Q24.8 depth
        input [15:0]  col;
        reg signed [63:0] area, e0, e1, e2, px, py;
        integer xs, ys, xmn, xmx, ymn, ymx, a;
        begin
            area = (x1 - x0) * (y2 - y0) - (x2 - x0) * (y1 - y0);
            if (area != 0) begin
                ref_tris = ref_tris + 1;
                // loop window = bounding box +/- 1 pixel, clipped to screen
                xmn = x0; if (x1 < xmn) xmn = x1; if (x2 < xmn) xmn = x2;
                xmx = x0; if (x1 > xmx) xmx = x1; if (x2 > xmx) xmx = x2;
                ymn = y0; if (y1 < ymn) ymn = y1; if (y2 < ymn) ymn = y2;
                ymx = y0; if (y1 > ymx) ymx = y1; if (y2 > ymx) ymx = y2;
                xmn = (xmn >>> 8) - 1; xmx = (xmx >>> 8) + 1;
                ymn = (ymn >>> 8) - 1; ymx = (ymx >>> 8) + 1;
                if (xmn < 0) xmn = 0; if (xmx > W-1) xmx = W-1;
                if (ymn < 0) ymn = 0; if (ymx > H-1) ymx = H-1;

                for (ys = ymn; ys <= ymx; ys = ys + 1)
                    for (xs = xmn; xs <= xmx; xs = xs + 1) begin
                        px = xs * 256 + 128;               // pixel centre, Q24.8
                        py = ys * 256 + 128;
                        e0 = (y0 - y1) * px + (x1 - x0) * py + (x0 * y1 - x1 * y0);
                        e1 = (y1 - y2) * px + (x2 - x1) * py + (x1 * y2 - x2 * y1);
                        e2 = (y2 - y0) * px + (x0 - x2) * py + (x2 * y0 - x0 * y2);
                        if (area < 0) begin e0 = -e0; e1 = -e1; e2 = -e2; end
                        if (e0 >= 0 && e1 >= 0 && e2 >= 0) begin
                            ref_pixels = ref_pixels + 1;
                            a = ys * W + xs;
                            if (zval < ref_z[a]) begin     // strict: HW uses '<'
                                ref_z[a]  = zval;
                                ref_fb[a] = col;
                            end
                        end
                    end
            end
        end
    endtask

    // Run the whole scene through the reference: matrix -> /W -> viewport.
    // Matrix maths matches the HW (Q24.8, products floored by >>> 8).
    task ref_render_scene;
        integer t, k, r, c;
        reg signed [63:0] v [0:3];
        reg signed [63:0] o [0:3];
        integer sx [0:2];
        integer sy [0:2];
        integer sz [0:2];
        reg signed [63:0] nx, ny, nz;
        begin
            ref_clear;
            for (t = 0; t < NV/3; t = t + 1) begin
                for (k = 0; k < 3; k = k + 1) begin
                    v[0] = sv_x[t*3+k]; v[1] = sv_y[t*3+k];
                    v[2] = sv_z[t*3+k]; v[3] = sv_w[t*3+k];
                    for (r = 0; r < 4; r = r + 1) begin
                        o[r] = 0;
                        for (c = 0; c < 4; c = c + 1)
                            o[r] = o[r] + ((v[c] * mat[r*4+c]) >>> 8);
                    end
                    // ideal perspective divide (exact for the test values)
                    nx = (o[0] * 256) / o[3];
                    ny = (o[1] * 256) / o[3];
                    nz = (o[2] * 256) / o[3];
                    sx[k] = (nx + 256) * 150;
                    sy[k] = (256 - ny) * 200;
                    sz[k] = nz;
                end
                ref_draw_tri(sx[0], sy[0], sx[1], sy[1], sx[2], sy[2],
                             sz[0], sv_c[t*3+2]);   // HW: colour of vertex 2
            end
        end
    endtask

    // ------------------------------------------------------------------------
    // HANDSHAKE COUNTERS (cleared by reset)
    // ------------------------------------------------------------------------
    integer cnt_vtx_in, cnt_vtx_proj, cnt_tri_setup, cnt_tri_rast,
            cnt_tri_drop, cnt_pix;

    always @(posedge clk) begin
        if (!rst_n) begin
            cnt_vtx_in = 0; cnt_vtx_proj = 0; cnt_tri_setup = 0;
            cnt_tri_rast = 0; cnt_tri_drop = 0; cnt_pix = 0;
        end else begin
            if (dut.te_valid_in && dut.te_ready_out)          cnt_vtx_in    = cnt_vtx_in + 1;
            if (dut.te_valid_out && dut.pe_ready_out)         cnt_vtx_proj  = cnt_vtx_proj + 1;
            if (dut.pe_valid_out && dut.ts_in_ready)          cnt_tri_setup = cnt_tri_setup + 1;
            if (dut.rast_tri_valid && dut.rast_idle)          cnt_tri_rast  = cnt_tri_rast + 1;
            if (dut.ts_out_valid && dut.pipe_discard)         cnt_tri_drop  = cnt_tri_drop + 1;
            if (dut.rs_pvalid && !dut.zf_busy)                cnt_pix       = cnt_pix + 1;
        end
    end

    // ------------------------------------------------------------------------
    // PROTOCOL CHECKERS: once valid is high and not accepted, the same data
    // must still be there on the next clock.
    // ------------------------------------------------------------------------
    reg         p_te_v,  p_te_r;   reg [143:0] p_te_d;
    reg         p_pe_v,  p_pe_r;   reg [143:0] p_pe_d;
    reg         p_ts_v,  p_ts_r;   reg [287:0] p_ts_d;
    reg         p_rs_v,  p_rs_r;   reg [133:0] p_rs_d;

    wire [287:0] ts_in_bus  = {dut.pe_x0, dut.pe_y0, dut.pe_z0, dut.pe_x1, dut.pe_y1,
                               dut.pe_z1, dut.pe_x2, dut.pe_y2, dut.pe_z2};
    wire [133:0] rast_in_bus = {dut.ts_e0A, dut.ts_e0B, dut.ts_e0C,
                                dut.ts_xmin, dut.ts_xmax, dut.ts_ymin, dut.ts_ymax,
                                dut.ts_color, 16'd0};

    always @(posedge clk) begin
        if (!rst_n) begin
            p_te_v <= 0; p_pe_v <= 0; p_ts_v <= 0; p_rs_v <= 0;
        end else begin
            // FSM -> transform
            if (p_te_v && !p_te_r && (!dut.te_valid_in || dut.vm_data !== p_te_d))
                fail("protocol: vertex FSM dropped/changed data while stalled");
            // transform -> projection
            if (p_pe_v && !p_pe_r && (!dut.te_valid_out || dut.te_vertex_out !== p_pe_d))
                fail("protocol: transform_engine dropped/changed data while stalled");
            // projection -> setup
            if (p_ts_v && !p_ts_r && (!dut.pe_valid_out || ts_in_bus !== p_ts_d))
                fail("protocol: projection_engine dropped/changed data while stalled");
            // setup -> rasterizer
            if (p_rs_v && !p_rs_r && (!dut.ts_out_valid || rast_in_bus !== p_rs_d))
                fail("protocol: triangle_setup dropped/changed data while stalled");

            p_te_v <= dut.te_valid_in;   p_te_r <= dut.te_ready_out; p_te_d <= dut.vm_data;
            p_pe_v <= dut.te_valid_out;  p_pe_r <= dut.pe_ready_out; p_pe_d <= dut.te_vertex_out;
            p_ts_v <= dut.pe_valid_out;  p_ts_r <= dut.ts_in_ready;  p_ts_d <= ts_in_bus;
            p_rs_v <= dut.ts_out_valid;  p_rs_r <= dut.ts_out_ready; p_rs_d <= rast_in_bus;
        end
    end

    // Rasterizer output pixel must be inside its captured bounding box
    always @(posedge clk) begin
        if (rst_n && dut.rs_pvalid && !dut.zf_busy) begin
            if (dut.rs_px < dut.u_rasterizer.xmin_q || dut.rs_px > dut.u_rasterizer.xmax_q ||
                dut.rs_py < dut.u_rasterizer.ymin_q || dut.rs_py > dut.u_rasterizer.ymax_q)
                fail("rasterizer emitted a pixel outside its bounding box");
            if (dut.rs_px >= W || dut.rs_py >= H)
                fail("rasterizer emitted a pixel outside 300x400");
        end
    end

    // Per-triangle debug flags from the setup unit (printed, not failed)
    always @(posedge clk) begin
        if (rst_n && dut.ts_out_valid && dut.ts_out_ready)
            $display("        setup -> tri bbox x[%0d..%0d] y[%0d..%0d] col=%h  degen=%0d neg=%0d ovf=%0d clamp=%0d",
                     dut.ts_xmin, dut.ts_xmax, dut.ts_ymin, dut.ts_ymax, dut.ts_color,
                     dut.ts_degenerate, dut.ts_orient_neg, dut.ts_overflow, dut.ts_bbox_clamped);
    end

    // ------------------------------------------------------------------------
    // VGA checker (negedge sampling = stable values).
    // RGB/sync at the outputs belong to the scan position 2 pixel clocks
    // earlier, so a 2-deep history of (h,v) is kept.
    // ------------------------------------------------------------------------
    reg        vga_chk_on = 1'b0;
    integer    h1 = 0, v1 = 0, h2 = 0, v2 = 0;
    integer    vga_err, vga_cnt, hs_low_cnt, vs_low_cnt;
    reg [11:0] cap [0:NPIX-1];          // captured window, for the PPM file
    reg [11:0] exp_rgb;
    reg        exp_hs, exp_vs;
    integer    idx;

    always @(negedge clk_vga) begin
        // X check on outputs (any time after reset released)
        if (dut.rst_n_v && ^{vga_r, vga_g, vga_b, vga_hs, vga_vs} === 1'bx)
            fail("X on VGA outputs");

        if (vga_chk_on) begin
            exp_hs = !(h2 >= 656 && h2 < 752);
            exp_vs = !(v2 >= 490 && v2 < 492);
            if (h2 >= 170 && h2 < 470 && v2 >= 40 && v2 < 440) begin
                idx     = (v2 - 40) * W + (h2 - 170);
                exp_rgb = {ref_fb[idx][15:12], ref_fb[idx][10:7], ref_fb[idx][4:1]};
                cap[idx] = {vga_r, vga_g, vga_b};
            end else
                exp_rgb = 12'h000;

            if ({vga_r, vga_g, vga_b} !== exp_rgb || vga_hs !== exp_hs || vga_vs !== exp_vs) begin
                if (vga_err < 5)
                    $display("  [FAIL] VGA h=%0d v=%0d got rgb=%h hs=%b vs=%b  exp rgb=%h hs=%b vs=%b",
                             h2, v2, {vga_r, vga_g, vga_b}, vga_hs, vga_vs, exp_rgb, exp_hs, exp_vs);
                vga_err = vga_err + 1;
            end
            if (!vga_hs) hs_low_cnt = hs_low_cnt + 1;
            if (!vga_vs) vs_low_cnt = vs_low_cnt + 1;
            vga_cnt = vga_cnt + 1;
        end
        h2 = h1; v2 = v1;
        h1 = dut.u_vga_controller.h_cnt;
        v1 = dut.u_vga_controller.v_cnt;
    end

    task check_vga_frame;
        input [8*32-1:0] ppm_name;
        integer f, k;
        begin
            $display("  checking one full VGA frame (800x525 clocks) ...");
            vga_err = 0; vga_cnt = 0; hs_low_cnt = 0; vs_low_cnt = 0;
            // start exactly when the scan position at the outputs is (0,0)
            @(negedge clk_vga);
            while (!(h1 == 799 && v1 == 524)) @(negedge clk_vga);
            vga_chk_on = 1'b1;
            repeat (800 * 525) @(negedge clk_vga);
            vga_chk_on = 1'b0;

            check_int("VGA clocks checked", vga_cnt, 800*525);
            check_int("VGA pixel/sync mismatches", vga_err, 0);
            check_int("hsync low clocks per frame", hs_low_cnt, 96 * 525);
            check_int("vsync low clocks per frame", vs_low_cnt, 2 * 800);

            // dump the 300x400 window as a PPM image
            f = $fopen(ppm_name, "w");
            $fwrite(f, "P3\n%0d %0d\n15\n", W, H);
            for (k = 0; k < NPIX; k = k + 1)
                $fwrite(f, "%0d %0d %0d\n", cap[k][11:8], cap[k][7:4], cap[k][3:0]);
            $fclose(f);
            $display("  frame written to %0s", ppm_name);
        end
    endtask

    // ------------------------------------------------------------------------
    // Helpers: reset, wait for clear, wait for render
    // ------------------------------------------------------------------------
    integer clear_cycles;

    task apply_reset;
        begin
            rst_n = 1'b0;
            repeat (20) @(posedge clk);
        end
    endtask

    task release_and_wait_clear;
        begin
            @(negedge clk); rst_n = 1'b1;
            clear_cycles = 0;
            while (!dut.zf_clear_done) begin
                @(posedge clk); clear_cycles = clear_cycles + 1;
                if (clear_cycles > 200000) begin
                    fail("z_f_buffer clear never finished");
                    disable release_and_wait_clear;
                end
            end
        end
    endtask

    // render is finished when the vertex FSM is done and every link has been
    // quiet for 300 consecutive clocks
    task wait_render_done;
        integer quiet, guard;
        begin
            quiet = 0; guard = 0;
            while (quiet < 300) begin
                @(posedge clk); guard = guard + 1;
                if (dut.f_state == 2'd2 && !dut.te_valid_out && !dut.pe_valid_out &&
                    !dut.ts_out_valid && dut.rast_idle && !dut.rs_pvalid)
                    quiet = quiet + 1;
                else
                    quiet = 0;
                if (guard > 2000000) begin
                    fail("render timeout");
                    disable wait_render_done;
                end
            end
        end
    endtask

    // ------------------------------------------------------------------------
    // Compare DUT framebuffer with the reference
    // ------------------------------------------------------------------------
    integer fb_mism, fb_lit;

    task compare_framebuffer;
        integer a;
        reg [15:0] got;
        begin
            fb_mism = 0; fb_lit = 0;
            for (a = 0; a < NPIX; a = a + 1) begin
                got = dut.u_z_f_buffer.frame_buffer[a];
                if (got != 16'h0000) fb_lit = fb_lit + 1;
                if (got !== ref_fb[a]) begin
                    if (fb_mism < 5)
                        $display("        mismatch x=%0d y=%0d got=%h exp=%h",
                                 a % W, a / W, got, ref_fb[a]);
                    fb_mism = fb_mism + 1;
                end
            end
        end
    endtask

    // ------------------------------------------------------------------------
    // RUN ONE SCENE  (the core of every test)
    // ------------------------------------------------------------------------
    task run_scene;
        input [8*64-1:0] name;
        input            known_bug;     // 1 = expected to show a reported bug
        integer exp_lit, a;
        begin
            tests_run = tests_run + 1;
            $display("\n=== TEST %0d: %0s ===", tests_run, name);

            apply_reset;
            load_scene_into_dut;
            load_recip_rom;
            ref_render_scene;

            release_and_wait_clear;
            // 2 reset-synchroniser clocks + 120000 clear clocks + 1 sample edge
            check_int("clear cycles", clear_cycles, NPIX + 3);

            wait_render_done;

            exp_lit = 0;
            for (a = 0; a < NPIX; a = a + 1) if (ref_fb[a] != 0) exp_lit = exp_lit + 1;

            compare_framebuffer;

            if (known_bug) begin
                if (fb_mism != 0) begin
                    known_bugs = known_bugs + 1;
                    $display("  [KNOWN BUG] reproduced: %0d pixels differ from the ideal image", fb_mism);
                end else
                    $display("  [ ok ] known bug NOT observed for this input");
            end else begin
                check_int("vertices into transform",   cnt_vtx_in,    NV);
                check_int("vertices into projection",  cnt_vtx_proj,  NV);
                check_int("triangles into setup",      cnt_tri_setup, NV/3);
                check_int("triangles to rasterizer",   cnt_tri_rast,  ref_tris);
                check_int("triangles discarded (degen|ovf)", cnt_tri_drop, NV/3 - ref_tris);
                check_int("pixels sent to z-buffer",   cnt_pix,       ref_pixels);
                check_int("lit pixels in framebuffer", fb_lit,        exp_lit);
                check_int("framebuffer mismatches",    fb_mism,       0);
            end
        end
    endtask

    // ========================================================================
    // MAIN TEST SEQUENCE
    // ========================================================================
    integer k;

    initial begin
        if ($test$plusargs("DUMP")) begin
            $dumpfile("tb.vcd");
            $dumpvars(0, tb_hardware_pipeline);
        end

        $display("============================================================");
        $display(" hardware_pipeline testbench");
        $display("============================================================");

        // --------------------------------------------------------------------
        // TEST 0: reset behaviour of the VGA outputs
        // --------------------------------------------------------------------
        tests_run = tests_run + 1;
        $display("\n=== TEST %0d: outputs during reset ===", tests_run);
        rst_n = 1'b0;
        repeat (10) @(negedge clk_vga);
        if ({vga_r, vga_g, vga_b} !== 12'h000) fail("RGB not black during reset");
        else $display("  [ ok ] RGB black during reset");
        if (vga_hs !== 1'b1 || vga_vs !== 1'b1) fail("sync not idle-high during reset");
        else $display("  [ ok ] hsync/vsync idle high during reset");

        // --------------------------------------------------------------------
        // TEST 1: one triangle, identity matrix, counter-clockwise
        //   NDC (-0.5,0.5) (0.5,0.5) (0,-0.5) -> screen (75,100) (225,100) (150,300)
        // --------------------------------------------------------------------
        set_identity;
        set_vertex(0, -128,  128, 0, ONE, RED);
        set_vertex(1,  128,  128, 0, ONE, RED);
        set_vertex(2,    0, -128, 0, ONE, RED);
        set_degenerate_tri(3);
        run_scene("single triangle, identity matrix", 0);

        // --------------------------------------------------------------------
        // TEST 2: same triangle, opposite winding (orient_neg path)
        // --------------------------------------------------------------------
        set_identity;
        set_vertex(0, -128,  128, 0, ONE, GREEN);
        set_vertex(1,    0, -128, 0, ONE, GREEN);
        set_vertex(2,  128,  128, 0, ONE, GREEN);
        set_degenerate_tri(3);
        run_scene("single triangle, reversed winding", 0);

        // --------------------------------------------------------------------
        // TEST 3: depth test, far triangle drawn FIRST, near second
        //   red  = far  (z = +0.5)     green = near (z = -0.5, negative!)
        //   + full VGA frame check
        // --------------------------------------------------------------------
        set_identity;
        set_vertex(0, -192,  192,  128, ONE, RED);
        set_vertex(1,  192,  192,  128, ONE, RED);
        set_vertex(2,    0, -192,  128, ONE, RED);
        set_vertex(3,  -64,   64, -128, ONE, GREEN);
        set_vertex(4,  160,   64, -128, ONE, GREEN);
        set_vertex(5,   48, -160, -128, ONE, GREEN);
        run_scene("depth test: far first, near second", 0);
        check_vga_frame("frame_out.ppm");

        // --------------------------------------------------------------------
        // TEST 4: depth test, NEAR drawn first, far second (far must lose)
        // --------------------------------------------------------------------
        set_identity;
        set_vertex(0,  -64,   64, -128, ONE, GREEN);
        set_vertex(1,  160,   64, -128, ONE, GREEN);
        set_vertex(2,   48, -160, -128, ONE, GREEN);
        set_vertex(3, -192,  192,  128, ONE, RED);
        set_vertex(4,  192,  192,  128, ONE, RED);
        set_vertex(5,    0, -192,  128, ONE, RED);
        run_scene("depth test: near first, far second", 0);

        // --------------------------------------------------------------------
        // TEST 5: non-symmetric matrix (scale X by 0.5, translate +0.25,-0.125)
        //   catches matrix word-order mistakes between memory and transform
        // --------------------------------------------------------------------
        set_identity;
        mat[0]  = 128;      // M00 = 0.5
        mat[3]  = 64;       // M03 = +0.25  (x translation)
        mat[7]  = -32;      // M13 = -0.125 (y translation)
        set_vertex(0, -128,  128, 0, ONE, BLUE);
        set_vertex(1,  128,  128, 0, ONE, BLUE);
        set_vertex(2,    0, -128, 0, ONE, BLUE);
        set_degenerate_tri(3);
        run_scene("scale + translate matrix", 0);

        // --------------------------------------------------------------------
        // TEST 6: triangle touching the viewport borders (x=0..300, y=0..400)
        // --------------------------------------------------------------------
        set_identity;
        set_vertex(0, -256,  256, 0, ONE, WHITE);
        set_vertex(1,  256,  256, 0, ONE, WHITE);
        set_vertex(2, -256, -256, 0, ONE, WHITE);
        set_degenerate_tri(3);
        run_scene("half-screen triangle on viewport borders", 0);

        // --------------------------------------------------------------------
        // TEST 7: two triangles sharing an edge (a square) - no gaps, no overlap
        // --------------------------------------------------------------------
        set_identity;
        set_vertex(0, -128,  128, 0, ONE, RED);
        set_vertex(1,  128,  128, 0, ONE, RED);
        set_vertex(2,  128, -128, 0, ONE, RED);
        set_vertex(3, -128,  128, 64, ONE, BLUE);
        set_vertex(4,  128, -128, 64, ONE, BLUE);
        set_vertex(5, -128, -128, 64, ONE, BLUE);
        run_scene("square from two triangles (shared edge)", 0);

        // --------------------------------------------------------------------
        // TEST 8: reset in the middle of rendering, then render again
        // --------------------------------------------------------------------
        tests_run = tests_run + 1;
        $display("\n=== TEST %0d: reset during rendering ===", tests_run);
        apply_reset;
        load_scene_into_dut;            // same square as test 7
        release_and_wait_clear;
        repeat (3000) @(posedge clk);   // rasterizer is busy now
        if (dut.rast_idle) $display("  (note: rasterizer was already idle)");
        tests_run = tests_run - 1;      // run_scene counts the re-run
        run_scene("re-render after mid-frame reset", 0);

        // --------------------------------------------------------------------
        // TEST 9 (KNOWN BUG #5): perspective divide, W = 2.0
        //   X,Y doubled and W = 2 -> ideal image identical to test 1
        // --------------------------------------------------------------------
        set_identity;
        set_vertex(0, -256,  256, 0, 512, RED);
        set_vertex(1,  256,  256, 0, 512, RED);
        set_vertex(2,    0, -256, 0, 512, RED);
        set_degenerate_tri(3);
        run_scene("perspective divide W=2 (projection LUT bug)", 1);

        // --------------------------------------------------------------------
        // SUMMARY
        // --------------------------------------------------------------------
        $display("\n============================================================");
        $display(" tests run        : %0d", tests_run);
        $display(" errors           : %0d", errors);
        $display(" known bugs shown : %0d", known_bugs);
        if (errors == 0) $display(" TEST PASSED");
        else             $display(" TEST FAILED");
        $display("============================================================");
        $finish;
    end

    // global watchdog (simulation time)
    initial begin
        #400_000_000;
        $display("[FAIL] global watchdog expired");
        $display(" TEST FAILED");
        $finish;
    end

endmodule

`timescale 1ns/1ps
// ============================================================================
// tb_rasterizer.v -- self-checking testbench for rasterizer.v (DUT only)
//
// The testbench is the ONLY driver of every DUT input and acts as the
// Z-buffer/framebuffer consumer:
//   * a pixel on the outputs is consumed at the next clk edge where
//     busy_fbuffer == 0 (that is how the DUT's DRAIN state is defined)
//   * every consumed pixel_valid pixel is compared (x, y, z, color, order)
//     against an independent reference model (direct formula, not the DUT's
//     incremental scan):
//         2E = 2*c + A*(2x+1) + B*(2y+1),  c = Q18.16 -> Q24.8, half away from 0
//         inside <=> 2E0,2E1,2E2 >= 0     z = floor(2Z / 32), saturated to int16
//
// Also checked: pixel count, one-cycle done pulse, idle_raster low while busy,
// inputs captured at the handshake (they are scrambled right after it), a
// spurious triangle_valid during SCAN is ignored, async reset mid-triangle,
// back-to-back triangles, and four busy_fbuffer stall patterns.
// ============================================================================
module tb_rasterizer;
    reg clk = 1'b0;  always #5 clk = ~clk;
    reg rst_n = 1'b0;

    // ---- DUT inputs ----
    reg         triangle_valid = 1'b0;
    reg  [9:0]  bbox_min_x, bbox_max_x, bbox_min_y, bbox_max_y;
    reg  signed [31:0] e0a, e0b, e1a, e1b, e2a, e2b, za, zb, zc;
    reg  signed [33:0] e0c, e1c, e2c;
    reg  [15:0] color_in;
    reg         busy_fbuffer = 1'b0;
    // ---- DUT outputs ----
    wire [9:0]  pixel_x, pixel_y, dbg_x_cur, dbg_y_cur;
    wire [15:0] pixel_z, pixel_color;
    wire        pixel_valid, idle_raster, done_raster;
    wire [2:0]  dbg_state;

    rasterizer dut (
        .clk(clk), .rst_n(rst_n), .triangle_valid(triangle_valid),
        .bbox_min_x(bbox_min_x), .bbox_max_x(bbox_max_x),
        .bbox_min_y(bbox_min_y), .bbox_max_y(bbox_max_y),
        .edge0_A(e0a), .edge0_B(e0b), .edge0_C(e0c),
        .edge1_A(e1a), .edge1_B(e1b), .edge1_C(e1c),
        .edge2_A(e2a), .edge2_B(e2b), .edge2_C(e2c),
        .z_A(za), .z_B(zb), .z_C(zc), .color_in(color_in),
        .busy_fbuffer(busy_fbuffer),
        .pixel_x(pixel_x), .pixel_y(pixel_y), .pixel_z(pixel_z),
        .pixel_color(pixel_color), .pixel_valid(pixel_valid),
        .idle_raster(idle_raster), .done_raster(done_raster),
        .dbg_state(dbg_state), .dbg_x_cur(dbg_x_cur), .dbg_y_cur(dbg_y_cur)
    );

    // ------------------------------------------------------------------
    // Reference model
    // ------------------------------------------------------------------
    reg [9:0]  exp_x [0:1023];
    reg [9:0]  exp_y [0:1023];
    reg [15:0] exp_z [0:1023];
    reg [15:0] exp_c;
    integer n_exp, got_idx;
    integer errors = 0, tests = 0, total_px = 0;

    function signed [63:0] conv_c;           // Q18.16 -> Q24.8, half away from zero
        input signed [33:0] c;
        reg signed [63:0] e;
        begin
            e = c;
            if (e < 0) e = e + 127; else e = e + 128;
            conv_c = e >>> 8;
        end
    endfunction

    task compute_expected;
        integer x, y;
        reg signed [63:0] xx, yy, c0, c1, c2, E0, E1, E2, Z, zs;
        begin
            n_exp = 0;
            c0 = conv_c(e0c); c1 = conv_c(e1c); c2 = conv_c(e2c);
            exp_c = color_in;
            for (y = bbox_min_y; y <= bbox_max_y; y = y + 1)
                for (x = bbox_min_x; x <= bbox_max_x; x = x + 1) begin
                    xx = 2*x + 1;  yy = 2*y + 1;
                    E0 = 2*c0 + e0a*xx + e0b*yy;
                    E1 = 2*c1 + e1a*xx + e1b*yy;
                    E2 = 2*c2 + e2a*xx + e2b*yy;
                    if (E0 >= 0 && E1 >= 0 && E2 >= 0) begin
                        Z  = 2*zc + za*xx + zb*yy;
                        zs = Z >>> 5;
                        if (zs >  32767) zs =  32767;
                        if (zs < -32768) zs = -32768;
                        exp_x[n_exp] = x;  exp_y[n_exp] = y;
                        exp_z[n_exp] = zs[15:0];
                        n_exp = n_exp + 1;
                    end
                end
        end
    endtask

    // ------------------------------------------------------------------
    // Consumer / checker: sample outputs at every edge where en = ~busy
    // ------------------------------------------------------------------
    always @(posedge clk) if (rst_n && !busy_fbuffer && pixel_valid) begin
        if (got_idx >= n_exp) begin
            errors = errors + 1;
            $display("[%0t] ERR extra pixel (%0d,%0d)", $time, pixel_x, pixel_y);
        end else if (pixel_x !== exp_x[got_idx] || pixel_y !== exp_y[got_idx] ||
                     pixel_z !== exp_z[got_idx] || pixel_color !== exp_c) begin
            errors = errors + 1;
            $display("[%0t] ERR pixel %0d: got (%0d,%0d) z=%0d c=%h  exp (%0d,%0d) z=%0d c=%h",
                     $time, got_idx, pixel_x, pixel_y, $signed(pixel_z), pixel_color,
                     exp_x[got_idx], exp_y[got_idx], $signed(exp_z[got_idx]), exp_c);
        end
        got_idx  = got_idx + 1;
        total_px = total_px + 1;
    end

    // done must be a single-cycle pulse; idle must be low while a triangle is active
    reg active = 1'b0, done_q = 1'b0;
    integer done_cnt = 0;
    always @(posedge clk) begin
        done_q <= done_raster;
        if (done_raster) done_cnt = done_cnt + 1;
        if (done_raster && done_q) begin
            errors = errors + 1; $display("[%0t] ERR done_raster longer than 1 cycle", $time);
        end
        if (active && idle_raster) begin
            errors = errors + 1; $display("[%0t] ERR idle_raster high while busy", $time);
        end
    end

    // ------------------------------------------------------------------
    // Stall (busy_fbuffer) patterns + spurious triangle_valid during SCAN
    // ------------------------------------------------------------------
    integer stall_mode = 0, burst = 0;
    always @(negedge clk) begin
        case (stall_mode)
            0: busy_fbuffer = 1'b0;                              // never stalls
            1: busy_fbuffer = (($urandom % 2) == 0);             // 50 % random
            2: begin busy_fbuffer = (burst < 20); burst = (burst + 1) % 24; end  // long bursts
            3: busy_fbuffer = (($urandom % 10) != 0);            // 90 % stalled
            default: busy_fbuffer = 1'b0;
        endcase
        if (active) triangle_valid = (dbg_state == 3'd3) && (($urandom % 4) == 0);
    end

    // ------------------------------------------------------------------
    // Triangle construction helpers
    // ------------------------------------------------------------------
    task set_edge;                           // edge p -> q ; inside is E >= 0
        input integer idx, px, py, qx, qy;
        reg signed [63:0] dx, dy, A, B, C;
        begin
            dx = qx - px;  dy = qy - py;
            A = -dy * 256;  B = dx * 256;  C = (dy*px - dx*py) * 65536;
            case (idx)
                0: begin e0a = A; e0b = B; e0c = C; end
                1: begin e1a = A; e1b = B; e1c = C; end
                default: begin e2a = A; e2b = B; e2c = C; end
            endcase
        end
    endtask

    task set_verts;                          // integer vertices + per-vertex z
        input integer x0, y0, z0, x1, y1, z1, x2, y2, z2;
        integer det, tx, ty, tz, lo, hi;
        real r_det, ra, rb, rc;
        begin
            det = (x1-x0)*(y2-y0) - (x2-x0)*(y1-y0);
            if (det < 0) begin               // make orientation positive
                tx = x1; ty = y1; tz = z1;  x1 = x2; y1 = y2; z1 = z2;  x2 = tx; y2 = ty; z2 = tz;
                det = -det;
            end
            set_edge(0, x0, y0, x1, y1);
            set_edge(1, x1, y1, x2, y2);
            set_edge(2, x2, y2, x0, y0);
            if (det != 0) begin
                r_det = det;
                ra = ((z1-z0)*(y2-y0) - (z2-z0)*(y1-y0)) / r_det;
                rb = ((x1-x0)*(z2-z0) - (x2-x0)*(z1-z0)) / r_det;
                rc = z0 - ra*x0 - rb*y0;
                za = $rtoi($floor(ra*256.0 + 0.5));
                zb = $rtoi($floor(rb*256.0 + 0.5));
                zc = $rtoi($floor(rc*256.0 + 0.5));
            end else begin za = 0; zb = 0; zc = z0 * 256; end
            lo = x0; if (x1 < lo) lo = x1; if (x2 < lo) lo = x2;
            hi = x0; if (x1 > hi) hi = x1; if (x2 > hi) hi = x2;
            bbox_min_x = lo; bbox_max_x = hi;
            lo = y0; if (y1 < lo) lo = y1; if (y2 < lo) lo = y2;
            hi = y0; if (y1 > hi) hi = y1; if (y2 > hi) hi = y2;
            bbox_min_y = lo; bbox_max_y = hi;
        end
    endtask

    // ------------------------------------------------------------------
    // Run one triangle that is already loaded in the input regs
    // ------------------------------------------------------------------
    reg [9:0]  s_bx0, s_bx1, s_by0, s_by1;
    reg signed [31:0] s_e0a, s_e0b, s_e1a, s_e1b, s_e2a, s_e2b, s_za, s_zb, s_zc;
    reg signed [33:0] s_e0c, s_e1c, s_e2c;
    reg [15:0] s_col;
    task save_inputs;
        begin
            s_bx0 = bbox_min_x; s_bx1 = bbox_max_x; s_by0 = bbox_min_y; s_by1 = bbox_max_y;
            s_e0a = e0a; s_e0b = e0b; s_e0c = e0c; s_e1a = e1a; s_e1b = e1b; s_e1c = e1c;
            s_e2a = e2a; s_e2b = e2b; s_e2c = e2c; s_za = za; s_zb = zb; s_zc = zc; s_col = color_in;
        end
    endtask
    task restore_inputs;
        begin
            bbox_min_x = s_bx0; bbox_max_x = s_bx1; bbox_min_y = s_by0; bbox_max_y = s_by1;
            e0a = s_e0a; e0b = s_e0b; e0c = s_e0c; e1a = s_e1a; e1b = s_e1b; e1c = s_e1c;
            e2a = s_e2a; e2b = s_e2b; e2c = s_e2c; za = s_za; zb = s_zb; zc = s_zc; color_in = s_col;
        end
    endtask

    task scramble_inputs;
        begin
            bbox_min_x = $urandom; bbox_max_x = $urandom; bbox_min_y = $urandom; bbox_max_y = $urandom;
            e0a = $urandom; e0b = $urandom; e1a = $urandom; e1b = $urandom; e2a = $urandom; e2b = $urandom;
            za = $urandom; zb = $urandom; zc = $urandom;
            e0c = {$urandom, $urandom}; e1c = {$urandom, $urandom}; e2c = {$urandom, $urandom};
            color_in = $urandom;
        end
    endtask

    task wait_idle;
        integer t;
        begin
            t = 0;
            while (idle_raster !== 1'b1 && t < 400000) begin @(posedge clk); t = t + 1; end
            if (t >= 400000) begin errors = errors + 1; $display("ERR timeout waiting for idle"); end
        end
    endtask

    task do_triangle;
        input integer mode;
        input [8*30-1:0] name;
        integer t, e_before, dc_before;
        reg done_seen;
        begin
            stall_mode = mode;
            compute_expected;
            save_inputs;
            got_idx = 0;
            e_before = errors;
            wait_idle;
            dc_before = done_cnt;
            @(negedge clk);
            triangle_valid = 1'b1;                    // handshake: valid && idle
            @(posedge clk);                           // DUT captures here
            #1 triangle_valid = 1'b0;
            scramble_inputs;                          // DUT must have captured by now
            active = 1'b1;
            if (idle_raster !== 1'b0) begin errors = errors + 1; $display("ERR idle_raster not cleared after handshake"); end
            t = 0; done_seen = 1'b0;
            while (!done_seen && t < 400000) begin
                @(negedge clk); t = t + 1;
                if (done_raster) done_seen = 1'b1;
            end
            active = 1'b0;  triangle_valid = 1'b0;
            restore_inputs;
            if (!done_seen) begin
                errors = errors + 1; $display("ERR timeout, no done_raster (%0s)", name);
                rst_n = 1'b0; repeat (3) @(posedge clk); #1 rst_n = 1'b1;
            end else begin
                if (got_idx !== n_exp) begin
                    errors = errors + 1;
                    $display("ERR pixel count: got %0d expected %0d (%0s)", got_idx, n_exp, name);
                end
                if (pixel_valid !== 1'b0) begin errors = errors + 1; $display("ERR pixel_valid still high at done"); end
                @(negedge clk); @(negedge clk);
                if (idle_raster !== 1'b1) begin errors = errors + 1; $display("ERR idle_raster not high after done"); end
                if (done_cnt - dc_before !== 1) begin errors = errors + 1; $display("ERR done pulse count %0d", done_cnt - dc_before); end
            end
            tests = tests + 1;
            $display("%-30s mode=%0d  pixels=%3d  %0s", name, mode, n_exp, (errors == e_before) ? "ok" : "FAIL");
        end
    endtask

    // ------------------------------------------------------------------
    // Async reset in the middle of a triangle
    // ------------------------------------------------------------------
    task reset_midway;
        integer e_before;
        begin
            e_before = errors;
            set_verts(2,2,100, 40,3,100, 5,30,100);  color_in = 16'hABCD;
            stall_mode = 0;
            compute_expected;  got_idx = 0;
            wait_idle;
            @(negedge clk); triangle_valid = 1'b1; @(posedge clk); #1 triangle_valid = 1'b0;
            active = 1'b1;
            repeat (60) @(posedge clk);
            active = 1'b0;  triangle_valid = 1'b0;
            #2 rst_n = 1'b0;                                  // async, away from a clock edge
            #1;
            if (dbg_state !== 3'd0 || idle_raster !== 1'b1 || pixel_valid !== 1'b0 || done_raster !== 1'b0 ||
                pixel_x !== 10'd0 || pixel_y !== 10'd0 || pixel_z !== 16'd0 || pixel_color !== 16'd0) begin
                errors = errors + 1; $display("ERR async reset did not clear outputs/state");
            end
            repeat (3) @(posedge clk);
            #3 rst_n = 1'b1;
            repeat (20) @(posedge clk);
            if (idle_raster !== 1'b1 || dbg_state !== 3'd0 || pixel_valid !== 1'b0) begin
                errors = errors + 1; $display("ERR not idle after reset release");
            end
            n_exp = 0; got_idx = 0;                          // aborted triangle: nothing more expected
            tests = tests + 1;
            $display("%-30s                         %0s", "async reset mid-triangle", (errors == e_before) ? "ok" : "FAIL");
        end
    endtask

    // ------------------------------------------------------------------
    // Test sequence
    // ------------------------------------------------------------------
    integer k, m, n, w, h, cx, cy;
    reg signed [63:0] jit, ct;
    initial begin
        n_exp = 0; got_idx = 0;
        scramble_inputs;
        #37 rst_n = 1'b1;
        repeat (4) @(posedge clk);

        // ---- directed ----
        color_in = 16'hF800;  set_verts(2,2,100,  9,2,100,  2,9,100);   do_triangle(0, "right tri (tie on hypotenuse)");
        color_in = 16'hF800;  set_verts(2,2,100,  9,2,100,  2,9,100);   do_triangle(1, "same, 50% stall");
        color_in = 16'h07E0;  set_verts(3,3,100, 10,5,500,  5,10,900);  do_triangle(2, "depth ramp, burst stall");
        color_in = 16'h001F;  set_verts(508,6,300, 515,6,300, 511,13,300); do_triangle(3, "straddles x=511/512, 90% stall");
        color_in = 16'hFFE0;  set_verts(20,20,50, 30,22,50, 20,21,50);  do_triangle(0, "thin sliver");
        color_in = 16'h1234;  set_verts(5,5,10, 5,5,10, 5,5,10);        do_triangle(0, "degenerate point (1 pixel)");
        color_in = 16'h4321;  set_verts(1016,1010,7, 1023,1010,7, 1016,1023,7); do_triangle(1, "max coordinates");
        color_in = 16'h0F0F;  set_verts(0,0,1, 12,0,2, 0,12,3);         do_triangle(0, "origin corner");

        // single-pixel bounding box: inside and outside
        color_in = 16'h5555;
        bbox_min_x = 7; bbox_max_x = 7; bbox_min_y = 9; bbox_max_y = 9;
        e0a = 0; e0b = 0; e0c = 34'sd65536; e1a = 0; e1b = 0; e1c = 34'sd65536; e2a = 0; e2b = 0; e2c = 34'sd65536;
        za = 0; zb = 0; zc = 32'sd2560;                                  do_triangle(0, "1x1 bbox inside");
        e1c = -34'sd65536;                                               do_triangle(0, "1x1 bbox outside");

        // z saturation and Q12.4 edges
        color_in = 16'hAAAA;
        set_verts(2,2,0, 9,2,0, 2,9,0);  za = 0; zb = 0;
        zc = 32'sd1280000;   do_triangle(0, "z saturate high (+5000)");
        zc = -32'sd1280000;  do_triangle(0, "z saturate low (-5000)");
        zc = 32'sd524272;    do_triangle(0, "z = 2047.9375 (max exact)");
        zc = 32'sd524288;    do_triangle(0, "z = 2048 (clamps)");
        zc = -32'sd524288;   do_triangle(0, "z = -2048 (min exact)");
        zc = -32'sd524289;   do_triangle(0, "z just below min");

        // rounding of the Q18.16 edge constant exactly at the inside boundary
        // edge0: E = y + 0.5 - 2.5  -> boundary at y = 2 ; offsets around the round-half point
        color_in = 16'h7777;
        bbox_min_x = 0; bbox_max_x = 3; bbox_min_y = 0; bbox_max_y = 4;
        e0a = 0; e0b = 32'sd256;
        e1a = 0; e1b = 0; e1c = 34'sd1073741824;
        e2a = 0; e2b = 0; e2c = 34'sd1073741824;
        za = 0; zb = 0; zc = 32'sd256;
        for (k = -130; k <= 130; k = k + 1) begin
            if (k == -130 || k == -129 || k == -128 || k == -127 || k == -126 || k == -2 || k == -1 || k == 0 ||
                k == 1 || k == 2 || k == 126 || k == 127 || k == 128 || k == 129 || k == 130) begin
                e0c = -34'sd163840 + k;
                za = 0; zb = 0; zc = 32'sd256;
                do_triangle(0, "C rounding boundary");
            end
        end

        // back-to-back (no gap between done and next valid) is exercised by every run above;
        // abort test:
        reset_midway;
        color_in = 16'hC0DE;  set_verts(2,2,100,  9,2,100,  2,9,100);  do_triangle(1, "after reset");

        // ---- randomized ----
        for (m = 0; m < 400; m = m + 1) begin
            w = 1 + ($urandom % 14);  h = 1 + ($urandom % 14);
            bbox_min_x = $urandom % (1024 - w);  bbox_max_x = bbox_min_x + w - 1;
            bbox_min_y = $urandom % (1024 - h);  bbox_max_y = bbox_min_y + h - 1;
            if (($urandom % 8) == 0) begin bbox_min_x = 1024 - w; bbox_max_x = 1023; end   // right edge
            cx = bbox_min_x + ($urandom % w);  cy = bbox_min_y + ($urandom % h);
            za = $signed($urandom % 4001) - 2000;  zb = $signed($urandom % 4001) - 2000;
            zc = ($signed($urandom % 6001) - 3000) * 256;
            color_in = $urandom;
            for (n = 0; n < 3; n = n + 1) begin
                // edge through (cx,cy)+jitter, random orientation of A,B, random low C bits
                case (n)
                    0: begin e0a = $signed($urandom % 20481) - 10240; e0b = $signed($urandom % 20481) - 10240; end
                    1: begin e1a = $signed($urandom % 20481) - 10240; e1b = $signed($urandom % 20481) - 10240; end
                    default: begin e2a = $signed($urandom % 20481) - 10240; e2b = $signed($urandom % 20481) - 10240; end
                endcase
            end
            jit = $signed($urandom % 4001) - 2000;
            ct  = -(e0a * cx + e0b * cy) + jit;   e0c = ct * 256 + ($urandom % 256);
            jit = $signed($urandom % 4001) - 2000;
            ct  = -(e1a * cx + e1b * cy) + jit;   e1c = ct * 256 + ($urandom % 256);
            jit = $signed($urandom % 4001) - 2000;
            ct  = -(e2a * cx + e2b * cy) + jit;   e2c = ct * 256 + ($urandom % 256);
            do_triangle($urandom % 4, "random");
        end

        $display("--------------------------------------------------------------");
        $display("tests=%0d  pixels checked=%0d  errors=%0d", tests, total_px, errors);
        if (errors == 0) $display("PASS"); else $display("FAIL");
        $finish;
    end

    initial begin #500000000; $display("GLOBAL TIMEOUT"); $display("FAIL"); $finish; end
endmodule

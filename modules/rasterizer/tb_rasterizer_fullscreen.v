`timescale 1ns/1ps
// ============================================================================
// tb_rasterizer_fullscreen.v -- full 300 x 400 screen scan test for rasterizer.v
//
// Screen: 300 pixels wide (x = 0..299) x 400 pixels tall (y = 0..399)
//         = 120,000 pixels.  bbox = {min_x 0, max_x 299, min_y 0, max_y 399}.
//         Pixel (x,y) covers [x,x+1] x [y,y+1] and is sampled at its centre.
//
// Tests (every one scans all 120,000 positions):
//   1  full-cover triangle (x>=-10, y>=-10, x+y<=700), no stall : all 120,000 pixels valid, exact row-major
//                                      order (x fastest), every x/y, z and colour
//   2  same triangle, 50 % random busy_fbuffer stalls
//   3  same triangle, long-burst stalls
//   4  throughput: scan takes <= pixels + a few cycles (1 pixel / clock)
//   5  two triangles forming the screen quad: every pixel covered EXACTLY once
//      (no gaps, no overlap along the diagonal)
//   6  big triangle clipped by the screen, hypotenuse x+y=399 passes through
//      pixel centres (tie rule): expects exactly 75,150 pixels
//   7  z plane across the whole screen incl. saturation at the top end
// Reference model is the same direct formula used in tb_rasterizer.v.
// ============================================================================
module tb_rasterizer_fullscreen;
    localparam W = 300, H = 400, NPIX = W*H;
    localparam TIMEOUT = 6000000;                         // cycles per triangle

    reg clk = 1'b0;  always #5 clk = ~clk;
    reg rst_n = 1'b0;

    reg         triangle_valid = 1'b0;
    reg  [9:0]  bbox_min_x, bbox_max_x, bbox_min_y, bbox_max_y;
    reg  signed [31:0] e0a, e0b, e1a, e1b, e2a, e2b, za, zb, zc;
    reg  signed [33:0] e0c, e1c, e2c;
    reg  [15:0] color_in;
    reg         busy_fbuffer = 1'b0;
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

    // ---------------- reference model ----------------
    reg [9:0]  exp_x [0:NPIX-1];
    reg [9:0]  exp_y [0:NPIX-1];
    reg [15:0] exp_z [0:NPIX-1];
    reg [15:0] exp_c;
    integer n_exp, got_idx;
    integer errors = 0, tests = 0, total_px = 0, shown = 0;

    
    function signed [63:0] conv_c;
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

    // ---------------- consumer / checker ----------------
    reg [1:0] cov [0:NPIX-1];            // how many times each screen pixel was drawn
    reg       cov_on = 1'b0;
    integer   i;
    always @(posedge clk) if (rst_n && !busy_fbuffer && pixel_valid) begin
        if (got_idx >= n_exp) begin
            errors = errors + 1;
            if (shown < 10) begin shown = shown + 1; $display("[%0t] ERR extra pixel (%0d,%0d)", $time, pixel_x, pixel_y); end
        end else if (pixel_x !== exp_x[got_idx] || pixel_y !== exp_y[got_idx] ||
                     pixel_z !== exp_z[got_idx] || pixel_color !== exp_c) begin
            errors = errors + 1;
            if (shown < 10) begin
                shown = shown + 1;
                $display("[%0t] ERR pixel %0d: got (%0d,%0d) z=%0d c=%h  exp (%0d,%0d) z=%0d c=%h",
                         $time, got_idx, pixel_x, pixel_y, $signed(pixel_z), pixel_color,
                         exp_x[got_idx], exp_y[got_idx], $signed(exp_z[got_idx]), exp_c);
            end
        end
        if (cov_on && pixel_x < W && pixel_y < H && cov[pixel_y*W + pixel_x] != 2'd3)
            cov[pixel_y*W + pixel_x] = cov[pixel_y*W + pixel_x] + 2'd1;
        got_idx  = got_idx + 1;
        total_px = total_px + 1;
    end

    reg active = 1'b0, done_q = 1'b0;
    integer done_cnt = 0;
    always @(posedge clk) begin
        done_q <= done_raster;
        if (done_raster) done_cnt = done_cnt + 1;
        if (done_raster && done_q) begin errors = errors + 1; $display("ERR done_raster longer than 1 cycle"); end
        if (active && idle_raster)  begin errors = errors + 1; $display("ERR idle_raster high while busy"); end
    end

    integer stall_mode = 0, burst = 0;
    always @(negedge clk) begin
        case (stall_mode)
            0: busy_fbuffer = 1'b0;
            1: busy_fbuffer = (($urandom % 2) == 0);
            2: begin busy_fbuffer = (burst < 20); burst = (burst + 1) % 24; end
            default: busy_fbuffer = 1'b0;
        endcase
        if (active) triangle_valid = (dbg_state == 3'd3) && (($urandom % 64) == 0);
    end

    // ---------------- triangle helpers ----------------
    task set_edge;
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

    task set_edge_raw;                    // E = a*x + b*y + c, integer a,b,c ; inside E >= 0
        input integer idx, a, b, c;
        reg signed [63:0] A, B, C;
        begin
            A = a * 256;  B = b * 256;  C = c * 65536;
            case (idx)
                0: begin e0a = A; e0b = B; e0c = C; end
                1: begin e1a = A; e1b = B; e1c = C; end
                default: begin e2a = A; e2b = B; e2c = C; end
            endcase
        end
    endtask

    task set_verts;                       // integer vertices (may lie outside the screen); z flat
        input integer x0, y0, x1, y1, x2, y2;
        integer det, tx, ty;
        begin
            det = (x1-x0)*(y2-y0) - (x2-x0)*(y1-y0);
            if (det < 0) begin tx = x1; ty = y1; x1 = x2; y1 = y2; x2 = tx; y2 = ty; end
            set_edge(0, x0, y0, x1, y1);
            set_edge(1, x1, y1, x2, y2);
            set_edge(2, x2, y2, x0, y0);
        end
    endtask

    task set_screen_bbox;
        begin bbox_min_x = 0; bbox_max_x = W-1; bbox_min_y = 0; bbox_max_y = H-1; end
    endtask

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

    integer last_cycles;
    task do_triangle;
        input integer mode;
        input [8*34-1:0] name;
        integer t, e_before, dc_before;
        reg done_seen;
        begin
            stall_mode = mode;
            compute_expected;
            save_inputs;
            got_idx = 0;  shown = 0;
            e_before = errors;
            t = 0; while (idle_raster !== 1'b1 && t < TIMEOUT) begin @(posedge clk); t = t + 1; end
            dc_before = done_cnt;
            @(negedge clk);
            triangle_valid = 1'b1;
            @(posedge clk);
            #1 triangle_valid = 1'b0;
            scramble_inputs;
            active = 1'b1;
            t = 0; done_seen = 1'b0;
            while (!done_seen && t < TIMEOUT) begin
                @(negedge clk); t = t + 1;
                if (done_raster) done_seen = 1'b1;
            end
            last_cycles = t;
            active = 1'b0;  triangle_valid = 1'b0;
            restore_inputs;
            if (!done_seen) begin
                errors = errors + 1; $display("ERR timeout, no done_raster (%0s)", name);
                rst_n = 1'b0; repeat (3) @(posedge clk); #1 rst_n = 1'b1;
            end else begin
                if (got_idx !== n_exp) begin
                    errors = errors + 1; $display("ERR pixel count: got %0d expected %0d", got_idx, n_exp);
                end
                if (pixel_valid !== 1'b0) begin errors = errors + 1; $display("ERR pixel_valid high at done"); end
                @(negedge clk); @(negedge clk);
                if (idle_raster !== 1'b1)           begin errors = errors + 1; $display("ERR idle_raster not high after done"); end
                if (done_cnt - dc_before !== 1)     begin errors = errors + 1; $display("ERR done pulse count"); end
            end
            tests = tests + 1;
            $display("%-34s stall=%0d  valid px=%6d / %0d scanned  cycles=%0d  %0s",
                     name, mode, n_exp, (bbox_max_x-bbox_min_x+1)*(bbox_max_y-bbox_min_y+1), last_cycles,
                     (errors == e_before) ? "ok" : "FAIL");
        end
    endtask

    // ---------------- sequence ----------------
    integer holes, overlaps, e_q;
    initial begin
        $display("Running full screen test");
            
        n_exp = 0; got_idx = 0;
        for (i = 0; i < NPIX; i = i + 1) cov[i] = 2'd0;
        scramble_inputs;
        #37 rst_n = 1'b1;
        repeat (4) @(posedge clk);

        // ---- 1-4: triangle that contains the whole screen -> every pixel valid ----
        color_in = 16'hF800;
        set_edge_raw(0, 1,0,10);  set_edge_raw(1, 0,1,10);  set_edge_raw(2, -1,-1,700);  set_screen_bbox;
        za = 0; zb = 0; zc = 32'sd25600;
        do_triangle(0, "full-cover tri, no stall");
        if (n_exp !== NPIX) begin errors = errors + 1; $display("ERR full-cover triangle should cover all %0d pixels, model says %0d", NPIX, n_exp); end
        if (last_cycles > NPIX + 16) begin errors = errors + 1; $display("ERR scan not 1 pixel/clock: %0d cycles for %0d pixels", last_cycles, NPIX); end
        else $display("   throughput: %0d cycles for %0d pixels (%0d extra) = 1 pixel/clock", last_cycles, NPIX, last_cycles - NPIX);
        do_triangle(1, "full-cover tri, 50% stall");
        do_triangle(2, "full-cover tri, burst stall");

        // ---- 5: screen quad as two triangles: each pixel exactly once ----
        for (i = 0; i < NPIX; i = i + 1) cov[i] = 2'd0;
        cov_on = 1'b1;  e_q = errors;
        color_in = 16'h07E0;  set_verts(0,0, 300,0, 0,400);     set_screen_bbox; za = 0; zb = 0; zc = 32'sd1280;
        do_triangle(0, "quad: triangle A");
        color_in = 16'h001F;  set_verts(300,0, 300,400, 0,400); set_screen_bbox; za = 0; zb = 0; zc = 32'sd2560;
        do_triangle(0, "quad: triangle B");
        cov_on = 1'b0;
        holes = 0; overlaps = 0;
        for (i = 0; i < NPIX; i = i + 1) begin
            if (cov[i] == 2'd0) holes = holes + 1;
            if (cov[i] >= 2'd2) overlaps = overlaps + 1;
        end
        if (holes != 0 || overlaps != 0) begin
            errors = errors + 1; $display("ERR quad coverage: %0d uncovered pixels, %0d drawn more than once", holes, overlaps);
        end
        $display("%-34s uncovered=%0d  double-drawn=%0d  %0s", "quad coverage (A + B)", holes, overlaps,
                 (holes == 0 && overlaps == 0 && errors == e_q) ? "ok" : "FAIL");
        tests = tests + 1;

        // ---- 6: tie rule: hypotenuse x+y = 400 passes through centres of x+y = 399 ----
        color_in = 16'hFFE0;
        set_edge_raw(0, 1,0,0);  set_edge_raw(1, 0,1,0);  set_edge_raw(2, -1,-1,400);  set_screen_bbox;  za = 0; zb = 0; zc = 32'sd256;
        do_triangle(0, "tie rule, clipped triangle");
        if (n_exp !== 75150) begin errors = errors + 1; $display("ERR tie test: expected 75150 pixels, got %0d", n_exp); end

        // ---- 7: z plane over the whole screen (6.0/px in x, 4.0/px in y -> saturates) ----
        color_in = 16'h1357;
        set_edge_raw(0, 1,0,10);  set_edge_raw(1, 0,1,10);  set_edge_raw(2, -1,-1,700);  set_screen_bbox;
        za = 32'sd1536; zb = 32'sd1024; zc = 32'sd256 * 5;
        do_triangle(1, "z plane + saturation, 50% stall");

        $display("--------------------------------------------------------------");
        $display("tests=%0d  pixels checked=%0d  errors=%0d", tests, total_px, errors);
        if (errors == 0) $display("PASS"); else $display("FAIL");
        $finish;
    end

    initial begin #2000000000; $display("GLOBAL TIMEOUT"); $display("FAIL"); $finish; end
endmodule

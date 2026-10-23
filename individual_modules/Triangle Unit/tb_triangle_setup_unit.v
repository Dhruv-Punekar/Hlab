// ============================================================================
// tb_triangle_setup_unit.v
// Self-checking testbench for triangle_setup_unit_debug
// (file triangle_setup_unit_bbox.v, rev B, 15/19-stage pipeline).
//
// Needs recip_lut_q1_17.mem in the simulation directory
// (mem/recip_lut_q1_17.mem, or regenerate with scripts/gen_recip_lut.py).
//
// Reference model, computed from the inputs with 64-bit integers / reals:
//   S0 clamp      x -> [0, 76800], y -> [0, 102400]   (Q24.8)
//   bbox          floor(min)/ceil(max) in pixels, clamped to 299 / 399
//   edges         A = yi - yj, B = xj - xi, C = xi*yj - xj*yi  (exact)
//   D             (x1-x0)(y2-y0) - (x2-x0)(y1-y0)
//   orient_neg    D < 0          degenerate  D == 0
//   zA, zB        ideal plane slope in Q14.18: Nx*1024/D, Ny*1024/D
//                 checked to a tolerance (reciprocal is approximate):
//                 2 LSB + |ideal| * 2^-15   (RECIP_NR = 1)
//                 2 LSB + |ideal| * 2^-10   (RECIP_NR = 0)
//                 saturation flag/value checked when clearly out of range
//   zC            EXACT, from the zA/zB the unit output:
//                 round_half_even((z0<<10 - zA*x0 - zB*y0) / 256), saturated
//   flags         overflow_* / overflow / bbox_clamped exact
//
// Tests
//   1. reset: in_ready low while rst_n low, no out_valid
//   2. directed triangles with latency check (LATENCY edges)
//   3. 3000 random triangles (small/large/off-screen/degenerate/huge depth
//      slopes) with random in_valid and random out_ready backpressure
//   4. outputs must hold while out_valid && !out_ready
//
// Compile with +define+NR0 to test RECIP_NR = 0.
// ============================================================================
`timescale 1ns / 1ps

`ifndef RECIP_LUT
 `define RECIP_LUT "mem/recip_lut_q1_17.mem"
`endif

module tb_triangle_setup_unit;

`ifdef NR0
    localparam NR = 0;
`else
    localparam NR = 1;
`endif
    localparam LATENCY = 14 + (NR ? 4 : 0);
    localparam real TOL_REL = NR ? (1.0 / 32768.0) : (1.0 / 1024.0);

    reg clk = 0;
    always #5 clk = ~clk;

    reg rst_n = 0;
    reg in_valid = 0, out_ready = 1;
    reg signed [31:0] x0 = 0, y0 = 0, z0 = 0, x1 = 0, y1 = 0, z1 = 0, x2 = 0, y2 = 0, z2 = 0;
    reg        [15:0] color = 0;
    wire in_ready, out_valid;

    wire [9:0] bbox_xmin, bbox_xmax, bbox_ymin, bbox_ymax;
    wire signed [31:0] edge0_A, edge0_B, edge1_A, edge1_B, edge2_A, edge2_B;
    wire signed [33:0] edge0_C, edge1_C, edge2_C;
    wire signed [31:0] zA, zB, zC;
    wire [15:0] face_color;
    wire overflow, overflow_xy_range, overflow_edge_c, overflow_zA, overflow_zB,
         overflow_zC, overflow_depth, bbox_clamped, orient_neg, degenerate;

    triangle_setup_unit_debug #(
        .RECIP_LUT_FILE (`RECIP_LUT),
        .RECIP_NR       (NR)
    ) dut (
        .clk(clk), .rst_n(rst_n),
        .in_valid(in_valid), .in_ready(in_ready),
        .in_x0(x0), .in_y0(y0), .in_z0(z0),
        .in_x1(x1), .in_y1(y1), .in_z1(z1),
        .in_x2(x2), .in_y2(y2), .in_z2(z2),
        .in_face_color(color),
        .out_valid(out_valid), .out_ready(out_ready),
        .bbox_xmin(bbox_xmin), .bbox_xmax(bbox_xmax),
        .bbox_ymin(bbox_ymin), .bbox_ymax(bbox_ymax),
        .edge0_A(edge0_A), .edge0_B(edge0_B), .edge0_C(edge0_C),
        .edge1_A(edge1_A), .edge1_B(edge1_B), .edge1_C(edge1_C),
        .edge2_A(edge2_A), .edge2_B(edge2_B), .edge2_C(edge2_C),
        .zA(zA), .zB(zB), .zC(zC), .face_color(face_color),
        .overflow(overflow), .overflow_xy_range(overflow_xy_range),
        .overflow_edge_c(overflow_edge_c), .overflow_zA(overflow_zA),
        .overflow_zB(overflow_zB), .overflow_zC(overflow_zC),
        .overflow_depth(overflow_depth), .bbox_clamped(bbox_clamped),
        .orient_neg(orient_neg), .degenerate(degenerate)
    );

    // Q24.8 pixel helper
    function signed [31:0] px(input integer p); px = p * 256; endfunction

    integer errors = 0, checked = 0, skipped = 0, n_ov = 0, n_degen = 0, n_neg = 0;
    integer seed = 99;

    // ------------------------------------------------------------------------
    // Scoreboard of accepted inputs
    // ------------------------------------------------------------------------
    localparam QD = 4096;
    reg signed [31:0] qx0 [0:QD-1], qy0 [0:QD-1], qz0 [0:QD-1];
    reg signed [31:0] qx1 [0:QD-1], qy1 [0:QD-1], qz1 [0:QD-1];
    reg signed [31:0] qx2 [0:QD-1], qy2 [0:QD-1], qz2 [0:QD-1];
    reg        [15:0] qc  [0:QD-1];
    integer wp = 0, rp = 0;

    task push;
        begin
            qx0[wp % QD] = x0; qy0[wp % QD] = y0; qz0[wp % QD] = z0;
            qx1[wp % QD] = x1; qy1[wp % QD] = y1; qz1[wp % QD] = z1;
            qx2[wp % QD] = x2; qy2[wp % QD] = y2; qz2[wp % QD] = z2;
            qc [wp % QD] = color;
            wp = wp + 1;
        end
    endtask

    // ------------------------------------------------------------------------
    // Reference helpers
    // ------------------------------------------------------------------------
    function integer clampc(input signed [31:0] v, input integer vmax);
        clampc = (v < 0) ? 0 : (v > vmax) ? vmax : v;
    endfunction

    function integer min3i(input integer a, input integer b, input integer c);
        min3i = (a < b) ? ((a < c) ? a : c) : ((b < c) ? b : c);
    endfunction

    function integer max3i(input integer a, input integer b, input integer c);
        max3i = (a > b) ? ((a > c) ? a : c) : ((b > c) ? b : c);
    endfunction

    function real rabs(input real a); rabs = (a < 0.0) ? -a : a; endfunction

    task err(input [8*24-1:0] what, input integer idx);
        begin
            if (errors < 20)
                $display("  ERROR triangle %0d: %0s  (v0=(%0d,%0d,%0d) v1=(%0d,%0d,%0d) v2=(%0d,%0d,%0d))",
                         idx, what, qx0[idx % QD], qy0[idx % QD], qz0[idx % QD],
                         qx1[idx % QD], qy1[idx % QD], qz1[idx % QD],
                         qx2[idx % QD], qy2[idx % QD], qz2[idx % QD]);
            errors = errors + 1;
        end
    endtask

    // check zA or zB against the ideal value
    task check_slope(input [8*2-1:0] name, input real ideal, input signed [31:0] got,
                     input ov, input integer idx);
        real tol, hi, lo;
        begin
            tol = 2.0 + rabs(ideal) * TOL_REL;
            hi  = 2147483647.0;
            lo  = -2147483648.0;
            if (ideal > hi + tol) begin
                if (!ov || got !== 32'sh7FFF_FFFF) err({name, " not saturated +"}, idx);
            end else if (ideal < lo - tol) begin
                if (!ov || got !== 32'sh8000_0000) err({name, " not saturated -"}, idx);
            end else if (ideal < hi - tol && ideal > lo + tol) begin
                if (ov) err({name, " false overflow"}, idx);
                else if (rabs(got - ideal) > tol) begin
                    if (errors < 20)
                        $display("    %0s = %0d, ideal %f, tolerance %f", name, got, ideal, tol);
                    err({name, " out of tolerance"}, idx);
                end
            end else
                skipped = skipped + 1;      // within tolerance of the limit
        end
    endtask

    // ------------------------------------------------------------------------
    // Full check of one output against input number idx
    // ------------------------------------------------------------------------
    task check_triangle(input integer idx);
        integer cx0, cy0, cx1, cy1, cx2, cy2, xlo, xhi, ylo, yhi;
        reg exp_xy_bad, exp_clamped, exp_degen, exp_neg, exp_edge_ov;
        reg [9:0] exb_xmin, exb_xmax, exb_ymin, exb_ymax;
        reg signed [63:0] C0, C1, C2, D, Nx, Ny, dz1, dz2, Z0, Z1, Z2, X0, Y0, X1, Y1, X2, Y2,
                          ZA, ZB, num, mag, q, r, lim;
        reg exp_ovc, neg, g, st;
        reg signed [31:0] exp_zc;
        real ideal_a, ideal_b, rn, rd;
        integer k;
        begin
            k = idx % QD;
            exp_xy_bad = (qx0[k] < 0) || (qx0[k] > 76800) || (qx1[k] < 0) || (qx1[k] > 76800) ||
                         (qx2[k] < 0) || (qx2[k] > 76800) || (qy0[k] < 0) || (qy0[k] > 102400) ||
                         (qy1[k] < 0) || (qy1[k] > 102400) || (qy2[k] < 0) || (qy2[k] > 102400);
            cx0 = clampc(qx0[k], 76800); cy0 = clampc(qy0[k], 102400);
            cx1 = clampc(qx1[k], 76800); cy1 = clampc(qy1[k], 102400);
            cx2 = clampc(qx2[k], 76800); cy2 = clampc(qy2[k], 102400);

            // bounding box
            xlo = min3i(cx0, cx1, cx2) / 256;
            xhi = (max3i(cx0, cx1, cx2) + 255) / 256;
            ylo = min3i(cy0, cy1, cy2) / 256;
            yhi = (max3i(cy0, cy1, cy2) + 255) / 256;
            exp_clamped = (xlo > 299) || (xhi > 299) || (ylo > 399) || (yhi > 399);
            exb_xmin = (xlo > 299) ? 299 : xlo;  exb_xmax = (xhi > 299) ? 299 : xhi;
            exb_ymin = (ylo > 399) ? 399 : ylo;  exb_ymax = (yhi > 399) ? 399 : yhi;

            if (bbox_xmin !== exb_xmin || bbox_xmax !== exb_xmax ||
                bbox_ymin !== exb_ymin || bbox_ymax !== exb_ymax) begin
                if (errors < 20)
                    $display("    bbox got x %0d..%0d y %0d..%0d, expected x %0d..%0d y %0d..%0d",
                             bbox_xmin, bbox_xmax, bbox_ymin, bbox_ymax,
                             exb_xmin, exb_xmax, exb_ymin, exb_ymax);
                err("bbox", idx);
            end
            if (bbox_clamped !== exp_clamped)       err("bbox_clamped", idx);
            if (overflow_xy_range !== exp_xy_bad)   err("overflow_xy_range", idx);

            // edge equations (all products in 64-bit)
            X0 = cx0; Y0 = cy0; X1 = cx1; Y1 = cy1; X2 = cx2; Y2 = cy2;
            C0 = X0 * Y1 - X1 * Y0;
            C1 = X1 * Y2 - X2 * Y1;
            C2 = X2 * Y0 - X0 * Y2;
            exp_edge_ov = (C0 > 64'sd8589934591) || (C0 < -64'sd8589934592) ||
                          (C1 > 64'sd8589934591) || (C1 < -64'sd8589934592) ||
                          (C2 > 64'sd8589934591) || (C2 < -64'sd8589934592);
            if (edge0_A !== cy0 - cy1 || edge0_B !== cx1 - cx0 || (!exp_edge_ov && edge0_C !== C0)) err("edge0", idx);
            if (edge1_A !== cy1 - cy2 || edge1_B !== cx2 - cx1 || (!exp_edge_ov && edge1_C !== C1)) err("edge1", idx);
            if (edge2_A !== cy2 - cy0 || edge2_B !== cx0 - cx2 || (!exp_edge_ov && edge2_C !== C2)) err("edge2", idx);
            if (overflow_edge_c !== exp_edge_ov) err("overflow_edge_c", idx);

            // determinant and plane numerators
            D   = (X1 - X0) * (Y2 - Y0) - (X2 - X0) * (Y1 - Y0);
            Z0  = qz0[k];  Z1 = qz1[k];  Z2 = qz2[k];
            dz1 = Z1 - Z0;
            dz2 = Z2 - Z0;
            Nx  = dz1 * (Y2 - Y0) - dz2 * (Y1 - Y0);
            Ny  = (X1 - X0) * dz2 - (X2 - X0) * dz1;
            exp_degen = (D == 0);
            exp_neg   = (D < 0);
            if (degenerate !== exp_degen) err("degenerate", idx);
            if (orient_neg !== exp_neg)   err("orient_neg", idx);
            if (exp_degen) n_degen = n_degen + 1;
            if (exp_neg)   n_neg   = n_neg + 1;

            if (exp_degen) begin
                if (zA !== 0 || zB !== 0 || zC !== 0) err("degenerate z not 0", idx);
                if (overflow_zA || overflow_zB || overflow_zC) err("degenerate z overflow", idx);
            end else begin
                rn = Nx; rd = D; ideal_a = rn * 1024.0 / rd;
                rn = Ny;         ideal_b = rn * 1024.0 / rd;
                check_slope("zA", ideal_a, zA, overflow_zA, idx);
                check_slope("zB", ideal_b, zB, overflow_zB, idx);

                // zC exact from the unit's own zA / zB
                ZA = zA;  ZB = zB;
                num = (Z0 <<< 10) - ZA * X0 - ZB * Y0;
                neg = num < 0;
                mag = neg ? -num : num;
                q   = mag >>> 8;
                g   = mag[7];
                st  = |mag[6:0];
                lim = neg ? 64'sd2147483648 : 64'sd2147483647;
                if (q > lim || (q == lim && (g || st))) begin
                    exp_ovc = 1;
                    exp_zc  = neg ? 32'sh8000_0000 : 32'sh7FFF_FFFF;
                end else begin
                    exp_ovc = 0;
                    r = q + (g && (st || q[0]));
                    exp_zc = neg ? -r : r;
                end
                if (zC !== exp_zc || overflow_zC !== exp_ovc) begin
                    if (errors < 20)
                        $display("    zC = %0d (ov %b), expected %0d (ov %b)", zC, overflow_zC, exp_zc, exp_ovc);
                    err("zC", idx);
                end
            end

            if (overflow_depth !== (overflow_zA | overflow_zB | overflow_zC)) err("overflow_depth", idx);
            if (overflow !== (exp_xy_bad | exp_edge_ov | overflow_zA | overflow_zB | overflow_zC))
                err("overflow", idx);
            if (overflow_depth) n_ov = n_ov + 1;
            if (face_color !== qc[k]) err("face_color", idx);
            checked = checked + 1;
        end
    endtask

    // ------------------------------------------------------------------------
    // Directed: one triangle into an empty pipeline, check latency
    // ------------------------------------------------------------------------
    task directed(input [8*32-1:0] name,
                  input signed [31:0] ax, input signed [31:0] ay, input signed [31:0] az,
                  input signed [31:0] bx, input signed [31:0] by, input signed [31:0] bz,
                  input signed [31:0] cx, input signed [31:0] cy, input signed [31:0] cz);
        integer n, e0;
        begin
            @(negedge clk);
            out_ready = 1;
            x0 = ax; y0 = ay; z0 = az; x1 = bx; y1 = by; z1 = bz; x2 = cx; y2 = cy; z2 = cz;
            color = $random(seed);
            in_valid = 1;
            #1 push;
            @(negedge clk);
            in_valid = 0;
            n = 1;
            while (!out_valid && n < LATENCY + 10) begin @(negedge clk); n = n + 1; end
            e0 = errors;
            if (n != LATENCY + 1) begin
                $display("  ERROR %0s: out_valid after %0d edges, expected %0d", name, n, LATENCY + 1);
                errors = errors + 1;
            end
            check_triangle(rp);
            rp = rp + 1;
            $display("  %0s %0s: bbox x %0d..%0d y %0d..%0d  zA %0d zB %0d zC %0d  neg %b degen %b ov %b",
                     (errors == e0) ? "ok " : "BAD", name, bbox_xmin, bbox_xmax, bbox_ymin, bbox_ymax,
                     zA, zB, zC, orient_neg, degenerate, overflow);
            @(negedge clk);
            if (out_valid) begin $display("  ERROR %0s: out_valid stuck", name); errors = errors + 1; end
        end
    endtask

    // ------------------------------------------------------------------------
    // Random triangle generator
    // ------------------------------------------------------------------------
    task random_triangle;
        integer kind, cxp, cyp, sz, zr;
        begin
            kind = {$random(seed)} % 100;
            cxp  = {$random(seed)} % 76801;
            cyp  = {$random(seed)} % 102401;
            sz   = 1 << ({$random(seed)} % 18);          // 1 .. 131072 (Q8)
            x0 = cxp + $random(seed) % sz;  y0 = cyp + $random(seed) % sz;
            x1 = cxp + $random(seed) % sz;  y1 = cyp + $random(seed) % sz;
            x2 = cxp + $random(seed) % sz;  y2 = cyp + $random(seed) % sz;
            zr = (kind < 10) ? 0 : (kind < 85) ? (1 << 17) : (1 << 30);
            if (zr == 0) begin
                z0 = $random(seed); z1 = $random(seed); z2 = $random(seed);   // full range
            end else begin
                z0 = $random(seed) % zr; z1 = $random(seed) % zr; z2 = $random(seed) % zr;
            end
            if (kind >= 95) begin           // degenerate: repeated vertex
                x2 = x0; y2 = y0;
            end else if (kind >= 90) begin  // far off-screen vertex
                x1 = $random(seed); y1 = $random(seed);
            end
            color = $random(seed);
        end
    endtask

    task random_stream(input integer count, input integer valid_pct, input integer ready_pct);
        integer sent, guard;
        reg accepted, hold_valid, hold_ready;
        reg [9:0] h_bx;
        reg signed [31:0] h_za, h_zc;
        reg signed [33:0] h_c0;
        begin
            sent = 0; guard = 0; accepted = 1; hold_valid = 0; hold_ready = 1;
            while ((sent < count || rp != wp) && guard < count * 60) begin
                @(negedge clk);
                guard = guard + 1;
                if (!hold_ready && hold_valid) begin
                    if (!out_valid || bbox_xmin !== h_bx || zA !== h_za || zC !== h_zc || edge0_C !== h_c0) begin
                        $display("  ERROR: output changed while stalled"); errors = errors + 1;
                    end
                end
                out_ready = (({$random(seed)} % 100) < ready_pct);
                if (!in_valid || accepted) begin
                    if (sent < count && (({$random(seed)} % 100) < valid_pct)) begin
                        random_triangle;
                        in_valid = 1;
                    end else
                        in_valid = 0;
                end
                #1;
                if (out_valid && out_ready) begin
                    if (rp == wp) begin $display("  ERROR: unexpected output"); errors = errors + 1; end
                    else begin check_triangle(rp); rp = rp + 1; end
                end
                accepted = in_valid && in_ready;
                if (accepted) begin push; sent = sent + 1; end
                hold_valid = out_valid; hold_ready = out_ready;
                h_bx = bbox_xmin; h_za = zA; h_zc = zC; h_c0 = edge0_C;
            end
            @(negedge clk);
            in_valid = 0; out_ready = 1;
            if (rp != wp) begin $display("  ERROR: %0d outputs missing", wp - rp); errors = errors + 1; end
        end
    endtask

    // ------------------------------------------------------------------------
    initial begin
        $dumpfile("tb_triangle_setup_unit.vcd");
        $dumpvars(1, tb_triangle_setup_unit);
        $display("RECIP_NR = %0d, LATENCY = %0d", NR, LATENCY);

        // ---- 1. reset ----
        repeat (3) @(negedge clk);
        if (in_ready !== 1'b0) begin $display("  ERROR: in_ready high during reset"); errors = errors + 1; end
        if (out_valid !== 1'b0) begin $display("  ERROR: out_valid high during reset"); errors = errors + 1; end
        rst_n = 1;
        @(negedge clk);
        if (in_ready !== 1'b1) begin $display("  ERROR: in_ready low after reset"); errors = errors + 1; end
        repeat (LATENCY + 3) @(negedge clk);
        if (out_valid !== 1'b0) begin $display("  ERROR: out_valid with no input"); errors = errors + 1; end
        $display("Test 1: reset done");

        // ---- 2. directed ----
        $display("Test 2: directed triangles");
        // z = 0.1 per pixel in x -> zA ~ 26214.4 (Q14.18), zB = 0
        directed("right triangle, dz/dx=0.1",
                 px(0), px(0), 0,   px(10), px(0), 32'sd65536,   px(0), px(10), 0);
        directed("same, opposite winding",
                 px(0), px(0), 0,   px(0), px(10), 0,   px(10), px(0), 32'sd65536);
        directed("z = 0.5 + 0.01x - 0.02y",
                 px(50), px(60), 32'sd32768 + 655*50/1 - 1311*60/1,
                 px(150), px(80), 32'sd32768 + 655*150 - 1311*80,
                 px(90), px(200), 32'sd32768 + 655*90 - 1311*200);
        directed("fractional coords",
                 32'sd12345, 32'sd23456, 32'sd1000,  32'sd34567, 32'sd25000, -32'sd5000,
                 32'sd20000, 32'sd60000, 32'sd250000);
        directed("full screen (bbox clamp)",
                 px(0), px(0), 0,   px(300), px(0), 0,   px(0), px(400), 0);
        directed("off-screen vertices",
                 -px(5), px(20), 0,   px(320), px(500), 32'sd100000,   px(40), -32'sd77, -32'sd1000);
        directed("degenerate (repeated vertex)",
                 px(10), px(10), 32'sd5000,   px(20), px(30), 32'sd9000,   px(10), px(10), 32'sd5000);
        directed("degenerate (collinear)",
                 px(10), px(10), 0,   px(20), px(20), 32'sd70000,   px(40), px(40), 32'sd3);
        directed("zA saturates +",
                 px(0), px(0), 0,   32'sd256, px(0), 32'sh7FFF_0000,   px(0), 32'sd256, 0);
        directed("zA saturates -",
                 px(0), px(0), 0,   32'sd256, px(0), -32'sh7FFF_0000,  px(0), 32'sd256, 0);
        directed("zC saturates (big z0, steep)",
                 px(299), px(399), 32'sh7FFF_FFFF,   px(299)+1, px(399), 32'sh7FFF_FFFF,
                 px(298), px(399)+1, 32'sh8000_0000);
        directed("tiny (1/256 px)",
                 32'sd1000, 32'sd1000, 0,   32'sd1001, 32'sd1000, 32'sd1,   32'sd1000, 32'sd1001, 32'sd2);

        // ---- 3/4. random with backpressure ----
        $display("Test 3: random, no backpressure");
        random_stream(1000, 100, 100);
        $display("Test 3: random, 70%% valid / 50%% ready (stall hold checked)");
        random_stream(2000, 70, 50);

        $display("%0d triangles checked (%0d degenerate, %0d negative winding, %0d depth overflow, %0d slope checks at the limit skipped)",
                 checked, n_degen, n_neg, n_ov, skipped);
        if (errors == 0) $display("PASS: tb_triangle_setup_unit");
        else             $display("FAIL: tb_triangle_setup_unit, %0d errors", errors);
        $finish;
    end

    initial begin
        #20000000 $display("FAIL: timeout"); $finish;
    end

endmodule

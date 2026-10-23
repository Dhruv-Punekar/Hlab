// ============================================================================
// tb_transform_engine.v
// Self-checking testbench for transform_engine.v (4x4 matrix x vertex,
// Q24.8, 3-stage pipeline, saturation, ready/valid with global stall).
//
// Reference model (independent of the RTL):
//   term  = clamp32( floor( (a * m) / 256 ) )          a, m: signed Q24.8
//   out_i = sadd( sadd(term0, term1), sadd(term2, term3) )
//   sadd  = exact 33-bit sum clamped to signed 32 bit
//   color passes through unchanged
//
// Tests
//   1. reset: valid_out low, ready_out follows ready_in
//   2. directed vectors, one at a time: identity, scale, translation,
//      90-degree rotation, fractional floor, multiply saturation (+/-),
//      adder saturation (+/-), (-2^31)*(-2^31); latency must be 3 edges
//   3. 3000 random vertices/matrices with random valid_in and random
//      ready_in stalls; in-order scoreboard; outputs must hold while stalled
//   4. async reset in the middle of traffic flushes the pipeline
// ============================================================================
`timescale 1ns / 1ps

module tb_transform_engine;

    reg clk = 0;
    always #5 clk = ~clk;

    reg          rst_n     = 0;
    reg          valid_in  = 0;
    reg          ready_in  = 1;
    reg  [143:0] vertex_in = 0;
    reg  [511:0] matrix_in = 0;
    wire         ready_out, valid_out;
    wire [143:0] vertex_out;

    transform_engine dut (
        .clk(clk), .rst_n(rst_n),
        .valid_in(valid_in), .ready_out(ready_out),
        .ready_in(ready_in), .valid_out(valid_out),
        .vertex_in(vertex_in), .matrix_in(matrix_in), .vertex_out(vertex_out)
    );

    // Q24.8 helpers
    localparam signed [31:0] ONE = 32'sd256;

    integer errors  = 0;
    integer checked = 0;
    integer seed    = 1234;

    // ------------------------------------------------------------------------
    // Reference model
    // ------------------------------------------------------------------------
    function signed [31:0] r_trunc(input signed [63:0] p);
        reg signed [63:0] q;
        begin
            q = p >>> 8;                                   // floor(p / 256)
            if (q > 64'sh0000_0000_7FFF_FFFF)      r_trunc = 32'sh7FFF_FFFF;
            else if (q < 64'shFFFF_FFFF_8000_0000) r_trunc = 32'sh8000_0000;
            else                                   r_trunc = q[31:0];
        end
    endfunction

    function signed [31:0] r_add(input signed [31:0] a, input signed [31:0] b);
        reg signed [63:0] s;
        begin
            s = a;
            s = s + b;                                     // exact
            if (s > 64'sh0000_0000_7FFF_FFFF)      r_add = 32'sh7FFF_FFFF;
            else if (s < 64'shFFFF_FFFF_8000_0000) r_add = 32'sh8000_0000;
            else                                   r_add = s[31:0];
        end
    endfunction

    function signed [31:0] r_term(input signed [31:0] a, input signed [31:0] m);
        reg signed [63:0] A, B;
        begin
            A = a; B = m;                                  // sign-extend
            r_term = r_trunc(A * B);
        end
    endfunction

    // one output component: row r of the matrix times the vertex
    function signed [31:0] ref_row(input [143:0] v, input [511:0] m, input integer r);
        reg signed [31:0] x, y, z, w;
        begin
            x = v[143:112]; y = v[111:80]; z = v[79:48]; w = v[47:16];
            ref_row = r_add(r_add(r_term(x, m[511-128*r    -: 32]),
                                  r_term(y, m[511-128*r-32 -: 32])),
                            r_add(r_term(z, m[511-128*r-64 -: 32]),
                                  r_term(w, m[511-128*r-96 -: 32])));
        end
    endfunction

    function [143:0] ref_transform(input [143:0] v, input [511:0] m);
        ref_transform = {ref_row(v, m, 0), ref_row(v, m, 1),
                         ref_row(v, m, 2), ref_row(v, m, 3), v[15:0]};
    endfunction

    // ------------------------------------------------------------------------
    // Packing helpers
    // ------------------------------------------------------------------------
    function [143:0] vtx(input signed [31:0] x, input signed [31:0] y,
                         input signed [31:0] z, input signed [31:0] w,
                         input [15:0] c);
        vtx = {x, y, z, w, c};
    endfunction

    reg signed [31:0] M [0:15];                // row-major, M[r*4+c]
    function [511:0] pack_m(input dummy);
        integer i;
        begin
            for (i = 0; i < 16; i = i + 1) pack_m[511-32*i -: 32] = M[i];
        end
    endfunction

    task set_identity;
        integer i;
        begin
            for (i = 0; i < 16; i = i + 1) M[i] = (i % 5 == 0) ? ONE : 32'sd0;
        end
    endtask

    task show(input [143:0] v);
        $display("      (%0d, %0d, %0d, %0d) color %04h  [Q24.8]",
                 $signed(v[143:112]), $signed(v[111:80]), $signed(v[79:48]),
                 $signed(v[47:16]), v[15:0]);
    endtask

    // ------------------------------------------------------------------------
    // Directed: send one vertex into an empty pipeline, check value + latency
    // ------------------------------------------------------------------------
    task directed(input [8*40-1:0] name, input [143:0] v, input [511:0] m);
        reg [143:0] exp;
        integer n;
        begin
            exp = ref_transform(v, m);
            @(negedge clk);
            ready_in = 1; valid_in = 1; vertex_in = v; matrix_in = m;
            @(negedge clk);
            valid_in = 0;
            n = 1;
            while (!valid_out && n < 10) begin @(negedge clk); n = n + 1; end
            checked = checked + 1;
            if (n != 3) begin
                $display("  ERROR %0s: valid_out after %0d edges, expected 3", name, n);
                errors = errors + 1;
            end
            if (vertex_out !== exp) begin
                $display("  ERROR %0s:", name);
                $display("    got"); show(vertex_out);
                $display("    expected"); show(exp);
                errors = errors + 1;
            end else
                $display("  ok  %0s", name);
            @(negedge clk);
            if (valid_out) begin
                $display("  ERROR %0s: valid_out longer than one cycle", name);
                errors = errors + 1;
            end
        end
    endtask

    // ------------------------------------------------------------------------
    // Random stream with backpressure and scoreboard
    // ------------------------------------------------------------------------
    localparam QD = 4096;
    reg [143:0] q_exp [0:QD-1];
    integer wp, rp;

    function signed [31:0] rnd_val(input integer kind);
        begin
            case (kind)
                0: rnd_val = $random(seed) % (256 * 1000);   // +/- 1000.0
                1: rnd_val = $random(seed) % (256 * 4);      // +/- 4.0
                default: rnd_val = $random(seed);            // anything
            endcase
        end
    endfunction

    task random_stream(input integer count, input integer valid_pct,
                       input integer ready_pct);
        integer sent, i, kind, guard;
        reg         accepted, hold_valid, hold_ready;
        reg [143:0] hold_out;
        begin
            sent = 0; accepted = 1; guard = 0;
            hold_valid = 0; hold_ready = 1; hold_out = 0;
            while ((sent < count || rp != wp) && guard < count * 50) begin
                @(negedge clk);
                guard = guard + 1;

                // outputs must not move while the previous cycle was stalled
                if (!hold_ready) begin
                    if (valid_out !== hold_valid || (hold_valid && vertex_out !== hold_out)) begin
                        $display("  ERROR: output changed while ready_in was low");
                        errors = errors + 1;
                    end
                end

                ready_in = (({$random(seed)} % 100) < ready_pct);

                // new input only if the last one was taken (hold otherwise)
                if (!valid_in || accepted) begin
                    if (sent < count && (({$random(seed)} % 100) < valid_pct)) begin
                        kind = ({$random(seed)} % 10 < 7) ? 0 : 2;
                        vertex_in = vtx(rnd_val(kind), rnd_val(kind), rnd_val(kind),
                                        (kind == 0) ? ONE + rnd_val(1) : rnd_val(2),
                                        $random(seed));
                        kind = ({$random(seed)} % 10 < 7) ? 1 : 2;
                        for (i = 0; i < 16; i = i + 1) M[i] = rnd_val(kind);
                        matrix_in = pack_m(0);
                        valid_in  = 1;
                    end else
                        valid_in = 0;
                end
                #1;
                // this coming edge: output taken? input taken?
                if (valid_out && ready_in) begin
                    if (rp == wp) begin
                        $display("  ERROR: unexpected output"); errors = errors + 1;
                    end else begin
                        if (vertex_out !== q_exp[rp % QD]) begin
                            if (errors < 10) begin
                                $display("  ERROR random output %0d:", rp);
                                $display("    got"); show(vertex_out);
                                $display("    expected"); show(q_exp[rp % QD]);
                            end
                            errors = errors + 1;
                        end
                        rp = rp + 1;
                        checked = checked + 1;
                    end
                end
                accepted = valid_in && ready_out;
                if (accepted) begin
                    q_exp[wp % QD] = ref_transform(vertex_in, matrix_in);
                    wp   = wp + 1;
                    sent = sent + 1;
                end
                hold_valid = valid_out;
                hold_out   = vertex_out;
                hold_ready = ready_in;
            end
            @(negedge clk);
            valid_in = 0; ready_in = 1;
            if (rp != wp) begin
                $display("  ERROR: %0d outputs missing", wp - rp);
                errors = errors + 1;
            end
        end
    endtask

    // ------------------------------------------------------------------------
    initial begin
        $dumpfile("tb_transform_engine.vcd");
        $dumpvars(0, tb_transform_engine);
        wp = 0; rp = 0;

        // ---- 1. reset ----
        repeat (3) @(negedge clk);
        if (valid_out !== 1'b0) begin $display("  ERROR: valid_out not 0 in reset"); errors = errors + 1; end
        rst_n = 1;
        ready_in = 0; #1;
        if (ready_out !== 1'b0) begin $display("  ERROR: ready_out does not follow ready_in"); errors = errors + 1; end
        ready_in = 1; #1;
        if (ready_out !== 1'b1) begin $display("  ERROR: ready_out does not follow ready_in"); errors = errors + 1; end
        repeat (5) @(negedge clk);
        if (valid_out !== 1'b0) begin $display("  ERROR: valid_out high with no input"); errors = errors + 1; end
        $display("Test 1: reset / ready passthrough done");

        // ---- 2. directed ----
        $display("Test 2: directed vectors");
        set_identity;
        directed("identity", vtx(384, -576, 768, ONE, 16'hF800), pack_m(0));

        set_identity; M[0] = 2*ONE; M[5] = 2*ONE; M[10] = 2*ONE;
        directed("scale x2", vtx(384, -576, 768, ONE, 16'h07E0), pack_m(0));

        set_identity; M[3] = 10*ONE; M[7] = -5*ONE; M[11] = ONE/2;
        directed("translate (10,-5,0.5)", vtx(384, -576, 768, ONE, 16'h001F), pack_m(0));

        set_identity; M[0] = 0; M[1] = -ONE; M[4] = ONE; M[5] = 0;
        directed("rotate 90 about Z", vtx(3*ONE, 7*ONE, ONE, ONE, 16'hFFFF), pack_m(0));

        set_identity; M[0] = ONE/2;
        directed("fraction floor +", vtx(1, 0, 0, 0, 16'h1234), pack_m(0));
        directed("fraction floor -", vtx(-1, 0, 0, 0, 16'h1234), pack_m(0));

        set_identity; M[0] = 32'sd1 << 20;
        directed("multiply saturate +", vtx(32'sd1 << 30, 0, 0, ONE, 16'hAAAA), pack_m(0));
        directed("multiply saturate -", vtx(-(32'sd1 << 30), 0, 0, ONE, 16'h5555), pack_m(0));

        set_identity; M[0] = ONE; M[1] = ONE; M[2] = ONE; M[3] = ONE;
        directed("adder saturate +", vtx(32'sh4000_0000, 32'sh4000_0000, 32'sh4000_0000, 32'sh4000_0000, 16'h0F0F), pack_m(0));
        directed("adder saturate -", vtx(32'shC000_0000, 32'shC000_0000, 32'shC000_0000, 32'shC000_0000, 16'hF0F0), pack_m(0));

        set_identity; M[0] = 32'sh8000_0000;
        directed("(-2^31)*(-2^31)", vtx(32'sh8000_0000, 0, 0, 0, 16'h0001), pack_m(0));

        // ---- 3. random with backpressure ----
        $display("Test 3: random stream, 90%% valid / 100%% ready");
        random_stream(500, 90, 100);
        $display("Test 3: random stream, 60%% valid / 60%% ready");
        random_stream(2500, 60, 60);

        // ---- 4. reset during traffic ----
        $display("Test 4: reset during traffic");
        @(negedge clk);
        valid_in = 1; ready_in = 1;
        repeat (2) @(negedge clk);
        #2 rst_n = 0;                      // asynchronous, mid-cycle
        #1;
        if (valid_out !== 1'b0) begin $display("  ERROR: valid_out not cleared by reset"); errors = errors + 1; end
        valid_in = 0;
        @(negedge clk) rst_n = 1;
        repeat (5) @(negedge clk);
        if (valid_out !== 1'b0) begin $display("  ERROR: stale data after reset"); errors = errors + 1; end
        wp = 0; rp = 0;
        random_stream(200, 70, 80);

        $display("%0d outputs checked", checked);
        if (errors == 0) $display("PASS: tb_transform_engine");
        else             $display("FAIL: tb_transform_engine, %0d errors", errors);
        $finish;
    end

    initial begin
        #5000000 $display("FAIL: timeout"); $finish;
    end

endmodule

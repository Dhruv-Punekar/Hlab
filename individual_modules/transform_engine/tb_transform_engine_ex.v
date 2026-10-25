`timescale 1ns / 1ps

// Additional testbench for transform_engine.v
// Compile with:
//   iverilog -g2012 -s tb_transform_engine_extra -o simv transform_engine.v tb_transform_engine_extra.v
// Run with:
//   vvp simv

module tb_transform_engine_extra;

    reg clk = 0;
    always #5 clk = ~clk;

    reg rst_n = 0;
    reg valid_in = 0;
    reg ready_in = 1;
    reg [143:0] vertex_in = 0;
    reg [511:0] matrix_in = 0;

    wire ready_out;
    wire valid_out;
    wire [143:0] vertex_out;

    transform_engine dut (
        .clk(clk),
        .rst_n(rst_n),
        .valid_in(valid_in),
        .ready_out(ready_out),
        .ready_in(ready_in),
        .valid_out(valid_out),
        .vertex_in(vertex_in),
        .matrix_in(matrix_in),
        .vertex_out(vertex_out)
    );

    localparam signed [31:0] ONE = 32'sd256;

    integer errors = 0;
    integer tests = 0;
    integer i;

    reg signed [31:0] M [0:15];

    function [143:0] make_vertex;
        input signed [31:0] x, y, z, w;
        input [15:0] color;
        begin
            make_vertex = {x, y, z, w, color};
        end
    endfunction

    function [511:0] pack_matrix;
        input dummy;
        integer j;
        begin
            for (j = 0; j < 16; j = j + 1)
                pack_matrix[511 - 32*j -: 32] = M[j];
        end
    endfunction

    task identity_matrix;
        integer j;
        begin
            for (j = 0; j < 16; j = j + 1)
                M[j] = ((j % 5) == 0) ? ONE : 32'sd0;
        end
    endtask

    // Reference fixed-point multiply: (a * b) >> 8, with signed saturation.
    function signed [31:0] ref_mul;
        input signed [31:0] a, b;
        reg signed [63:0] p;
        reg signed [63:0] q;
        begin
            p = a * b;
            q = p >>> 8;
            if (q > 64'sh000000007fffffff)
                ref_mul = 32'sh7fffffff;
            else if (q < -64'sd2147483648)
                ref_mul = 32'sh80000000;
            else
                ref_mul = q[31:0];
        end
    endfunction

    function signed [31:0] ref_add;
        input signed [31:0] a, b;
        reg signed [63:0] sum;
        begin
            sum = a;
            sum = sum + b;
            if (sum > 64'sh000000007fffffff)
                ref_add = 32'sh7fffffff;
            else if (sum < -64'sd2147483648)
                ref_add = 32'sh80000000;
            else
                ref_add = sum[31:0];
        end
    endfunction

    function signed [31:0] row_result;
        input [143:0] v;
        input [511:0] m;
        input integer r;
        reg signed [31:0] x, y, z, w;
        reg signed [31:0] t0, t1, t2, t3;
        begin
            x = v[143:112];
            y = v[111:80];
            z = v[79:48];
            w = v[47:16];

            t0 = ref_mul(x, $signed(m[511 - 128*r      -: 32]));
            t1 = ref_mul(y, $signed(m[511 - 128*r - 32 -: 32]));
            t2 = ref_mul(z, $signed(m[511 - 128*r - 64 -: 32]));
            t3 = ref_mul(w, $signed(m[511 - 128*r - 96 -: 32]));

            row_result = ref_add(ref_add(t0, t1), ref_add(t2, t3));
        end
    endfunction

    function [143:0] reference_transform;
        input [143:0] v;
        input [511:0] m;
        begin
            reference_transform = {
                row_result(v, m, 0),
                row_result(v, m, 1),
                row_result(v, m, 2),
                row_result(v, m, 3),
                v[15:0]
            };
        end
    endfunction

    task check_case;
        input [8*32-1:0] test_name;
        input [143:0] v;
        input [511:0] m;
        reg [143:0] expected;
        integer cycles;
        begin
            expected = reference_transform(v, m);

            // Drive away from the active clock edge.
            @(negedge clk);
            while (ready_out !== 1'b1) @(negedge clk);
            vertex_in = v;
            matrix_in = m;
            valid_in = 1'b1;

            // Input is accepted at the next rising edge.
            @(negedge clk);
            valid_in = 1'b0;

            cycles = 1;
            while ((valid_out !== 1'b1) && cycles < 8) begin
                @(negedge clk);
                cycles = cycles + 1;
            end

            tests = tests + 1;
            if (valid_out !== 1'b1) begin
                $display("FAIL %-32s: valid_out timeout", test_name);
                errors = errors + 1;
            end else if (vertex_out !== expected) begin
                $display("FAIL %-32s", test_name);
                $display("  got      X=%0d Y=%0d Z=%0d W=%0d C=%h",
                    $signed(vertex_out[143:112]), $signed(vertex_out[111:80]),
                    $signed(vertex_out[79:48]), $signed(vertex_out[47:16]),
                    vertex_out[15:0]);
                $display("  expected X=%0d Y=%0d Z=%0d W=%0d C=%h",
                    $signed(expected[143:112]), $signed(expected[111:80]),
                    $signed(expected[79:48]), $signed(expected[47:16]),
                    expected[15:0]);
                errors = errors + 1;
            end else begin
                $display("PASS %-32s", test_name);
            end

            // Let the pipeline become empty before the next isolated case.
            @(negedge clk);
        end
    endtask

    reg [143:0] v;
    reg [511:0] m;
    reg [143:0] expected;

    initial begin
        $dumpfile("tb_transform_engine_extra.vcd");
        $dumpvars(0, tb_transform_engine_extra);

        // Reset
        repeat (3) @(negedge clk);
        rst_n = 1'b1;
        repeat (4) @(negedge clk);

        // Test 1: identity matrix and negative coordinates.
        identity_matrix();
        check_case("identity negative coords",
                   make_vertex(-3*ONE, 5*ONE, -7*ONE, ONE, 16'hA15C),
                   pack_matrix(0));

        // Test 2: non-uniform scale.
        identity_matrix();
        M[0] = 2*ONE;
        M[5] = 3*ONE;
        M[10] = 4*ONE;
        check_case("non-uniform scale",
                   make_vertex(2*ONE, -3*ONE, ONE, ONE, 16'h07E0),
                   pack_matrix(0));

        // Test 3: custom mixed-sign matrix.
        identity_matrix();
        M[0] = 2*ONE;   M[1] = -ONE;   M[2] = ONE/2; M[3] = 3*ONE;
        M[4] = -ONE;    M[5] = ONE;    M[6] = 2*ONE; M[7] = -2*ONE;
        M[8] = ONE/2;   M[9] = 3*ONE;  M[10] = -ONE; M[11] = ONE;
        M[12] = ONE;    M[13] = -ONE;  M[14] = ONE;  M[15] = ONE;
        check_case("mixed-sign matrix",
                   make_vertex(4*ONE, -2*ONE, 3*ONE, ONE, 16'hC39A),
                   pack_matrix(0));

        // Test 4: fractional coefficients and small signed values.
        identity_matrix();
        M[0] = ONE/2;
        M[5] = ONE/4;
        M[10] = -ONE/2;
        check_case("fractional coefficients",
                   make_vertex(3, -5, 7, ONE, 16'h1234),
                   pack_matrix(0));

        // Test 5: translation.
        identity_matrix();
        M[3] = 8*ONE;
        M[7] = -6*ONE;
        M[11] = 2*ONE;
        check_case("translation",
                   make_vertex(ONE, 2*ONE, -ONE, ONE, 16'h55AA),
                   pack_matrix(0));

        // Test 6: color must pass through unchanged.
        identity_matrix();
        check_case("color passthrough",
                   make_vertex(11*ONE, -9*ONE, 4*ONE, ONE, 16'hF81F),
                   pack_matrix(0));

        // Test 7: stall while output is valid; output and valid must hold.
        identity_matrix();
        v = make_vertex(2*ONE, 3*ONE, 4*ONE, ONE, 16'h0ACE);
        m = pack_matrix(0);
        expected = reference_transform(v, m);

        @(negedge clk);
        vertex_in = v;
        matrix_in = m;
        valid_in = 1'b1;
        ready_in = 1'b1;
        @(negedge clk);
        valid_in = 1'b0;
        wait (valid_out === 1'b1);
        #1;
        if (vertex_out !== expected) begin
            $display("FAIL stall setup: output mismatch");
            errors = errors + 1;
        end

        // Since ready_in is low, the whole pipeline must freeze.
        ready_in = 1'b0;
        repeat (3) begin
            @(negedge clk);
            #1;
            tests = tests + 1;
            if (valid_out !== 1'b1 || vertex_out !== expected) begin
                $display("FAIL output did not hold during stall");
                errors = errors + 1;
            end
        end
        ready_in = 1'b1;
        @(negedge clk);
        if (ready_out !== 1'b1) begin
            $display("FAIL ready_out did not recover");
            errors = errors + 1;
        end else begin
            $display("PASS stall and resume");
            tests = tests + 1;
        end
        @(negedge clk);

        // Test 8: asynchronous reset clears valid pipeline flags.
        valid_in = 1'b1;
        ready_in = 1'b1;
        @(negedge clk);
        #2 rst_n = 1'b0;
        #1;
        tests = tests + 1;
        if (valid_out !== 1'b0) begin
            $display("FAIL asynchronous reset did not clear valid_out");
            errors = errors + 1;
        end else begin
            $display("PASS asynchronous reset");
        end
        valid_in = 1'b0;
        @(negedge clk);
        rst_n = 1'b1;
        repeat (4) @(negedge clk);
        if (valid_out !== 1'b0) begin
            $display("FAIL stale valid_out after reset");
            errors = errors + 1;
        end

        $display("----------------------------------------");
        $display("Tests/checks: %0d", tests);
        if (errors == 0)
            $display("ALL EXTRA TESTS PASSED");
        else
            $display("EXTRA TESTS FAILED: %0d error(s)", errors);
        $finish;
    end

    initial begin
        #100000;
        $display("FAIL: simulation timeout");
        $finish;
    end

endmodule

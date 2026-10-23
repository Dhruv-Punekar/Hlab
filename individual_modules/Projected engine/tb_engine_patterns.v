// ============================================================================
// tb_engine_patterns.v
// Runs projection_engine on synthetic 64x64 images whose projections are
// easy to reason about, plus random images. A reference model in the
// testbench computes the expected sums for each frame.
//
// Patterns:
//   0 all zeros            -> every sum 0
//   1 all 255              -> every sum 16320 (largest value, checks width)
//   2 checkerboard 0/255   -> every sum 8160
//   3 horizontal gradient  I = 4x        -> rows equal, columns grow
//   4 vertical gradient    I = 4y        -> columns equal, rows grow
//   5 single hot pixel     I(37,11)=200  -> one non-zero row and column
//   6 diagonal line        I(i,i)=255    -> every sum 255
//   7..9 random images
// The frames run back to back with no reset in between, so leftover state
// from one frame would show up as errors in the next.
// ============================================================================
`timescale 1ns / 1ps

module tb_engine_patterns;

    localparam W  = 64;
    localparam H  = 64;
    localparam DW = 8;
    localparam AW = 12;
    localparam IW = 6;
    localparam SW = 14;
    localparam NPAT = 10;

    reg clk = 0, rst = 1, start = 0;
    always #5 clk = ~clk;

    // behavioural image memory, 1-cycle read latency like a block RAM
    reg  [DW-1:0] img [0:W*H-1];
    reg  [DW-1:0] pix_q;
    wire [AW-1:0] pix_addr;
    always @(posedge clk) pix_q <= img[pix_addr];

    wire          busy, done;
    reg           rd_sel = 0;
    reg  [IW-1:0] rd_idx = 0;
    wire [SW-1:0] rd_data;

    projection_engine #(.W(W), .H(H), .DW(DW)) dut (
        .clk(clk), .rst(rst), .start(start),
        .pix_addr(pix_addr), .pix_data(pix_q),
        .busy(busy), .done(done),
        .rd_sel(rd_sel), .rd_idx(rd_idx), .rd_data(rd_data)
    );

    integer ref_row [0:H-1];
    integer ref_col [0:W-1];
    integer errors = 0;
    integer seed   = 2026;
    integer p, x, y, i, v;

    task fill_pattern(input integer pat);
        begin
            for (y = 0; y < H; y = y + 1)
                for (x = 0; x < W; x = x + 1) begin
                    case (pat)
                        0: v = 0;
                        1: v = 255;
                        2: v = ((x + y) % 2) ? 255 : 0;
                        3: v = 4 * x;
                        4: v = 4 * y;
                        5: v = (x == 37 && y == 11) ? 200 : 0;
                        6: v = (x == y) ? 255 : 0;
                        default: v = $random(seed) & 8'hFF;
                    endcase
                    img[y*W + x] = v;
                end
        end
    endtask

    task compute_reference;
        begin
            for (y = 0; y < H; y = y + 1) ref_row[y] = 0;
            for (x = 0; x < W; x = x + 1) ref_col[x] = 0;
            for (y = 0; y < H; y = y + 1)
                for (x = 0; x < W; x = x + 1) begin
                    ref_row[y] = ref_row[y] + img[y*W + x];
                    ref_col[x] = ref_col[x] + img[y*W + x];
                end
        end
    endtask

    task run_frame;
        integer guard;
        begin
            @(negedge clk) start = 1;
            @(negedge clk) start = 0;
            guard = 0;
            while (!done) begin
                @(negedge clk);
                guard = guard + 1;
                if (guard > W*H + 50) begin
                    $display("FAIL: done never asserted"); $finish;
                end
            end
        end
    endtask

    task check_results(input integer pat);
        integer e0;
        begin
            e0 = errors;
            rd_sel = 0;
            for (i = 0; i < H; i = i + 1) begin
                rd_idx = i; #1;
                if (rd_data !== ref_row[i]) begin
                    if (errors - e0 < 5)
                        $display("  ERROR pattern %0d row[%0d] = %0d expected %0d", pat, i, rd_data, ref_row[i]);
                    errors = errors + 1;
                end
            end
            rd_sel = 1;
            for (i = 0; i < W; i = i + 1) begin
                rd_idx = i; #1;
                if (rd_data !== ref_col[i]) begin
                    if (errors - e0 < 5)
                        $display("  ERROR pattern %0d col[%0d] = %0d expected %0d", pat, i, rd_data, ref_col[i]);
                    errors = errors + 1;
                end
            end
            $display("Pattern %0d: %s  (row[0]=%0d col[0]=%0d)", pat,
                     (errors == e0) ? "ok" : "MISMATCH", ref_row[0], ref_col[0]);
        end
    endtask

    initial begin
        $dumpfile("tb_engine_patterns.vcd");
        $dumpvars(0, tb_engine_patterns);
        repeat (3) @(negedge clk);
        rst = 0;
        repeat (2) @(negedge clk);

        for (p = 0; p < NPAT; p = p + 1) begin
            fill_pattern(p);          // memory is only read while busy
            compute_reference;
            run_frame;
            check_results(p);
        end

        if (errors == 0) $display("PASS: tb_engine_patterns (%0d images)", NPAT);
        else             $display("FAIL: tb_engine_patterns, %0d errors", errors);
        $finish;
    end

endmodule

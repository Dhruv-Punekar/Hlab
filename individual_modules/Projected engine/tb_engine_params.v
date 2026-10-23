// ============================================================================
// tb_engine_params.v
// Checks that projection_engine is correct for other image sizes and pixel
// widths, not only 64x64x8. Four engines run in parallel, each inside its
// own self-checking harness with random images (3 frames each):
//
//   32 x 16, 8-bit    (wide, non-power-of-two aspect)
//   16 x 32, 8-bit    (tall)
//    8 x  8, 4-bit    (small, narrow pixels)
//  128 x 64, 8-bit    (row sums need DW + 7 bits)
//
// Every harness also checks the frame latency (W*H + 1 edges after start).
// ============================================================================
`timescale 1ns / 1ps

module engine_harness #(
    parameter W    = 32,
    parameter H    = 16,
    parameter DW   = 8,
    parameter SEED = 1,
    parameter NFRAMES = 3
)(
    input  wire clk,
    output reg  finished,
    output reg  [31:0] errors
);
    localparam N  = W * H;
    localparam XW = $clog2(W);
    localparam YW = $clog2(H);
    localparam AW = $clog2(N);
    localparam IW = (XW > YW) ? XW : YW;
    localparam SW = DW + IW;

    reg rst = 1, start = 0;

    reg  [DW-1:0] img [0:N-1];
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
    integer seed;
    integer f, i, x, y, edges;

    initial begin
        finished = 0;
        errors   = 0;
        seed     = SEED;
        repeat (3) @(negedge clk);
        rst = 0;
        repeat (2) @(negedge clk);

        for (f = 0; f < NFRAMES; f = f + 1) begin
            // random image (all-max on the last frame to check sum width)
            for (i = 0; i < N; i = i + 1)
                img[i] = (f == NFRAMES-1) ? {DW{1'b1}} : ($random(seed) & ((1 << DW) - 1));
            for (y = 0; y < H; y = y + 1) ref_row[y] = 0;
            for (x = 0; x < W; x = x + 1) ref_col[x] = 0;
            for (y = 0; y < H; y = y + 1)
                for (x = 0; x < W; x = x + 1) begin
                    ref_row[y] = ref_row[y] + img[y*W + x];
                    ref_col[x] = ref_col[x] + img[y*W + x];
                end

            @(negedge clk) start = 1;
            @(posedge clk);
            #1 start = 0;
            edges = 0;
            while (!done) begin
                @(posedge clk); #1;
                edges = edges + 1;
                if (edges > N + 50) begin
                    $display("FAIL: %0dx%0d done never asserted", W, H); $finish;
                end
            end
            if (edges != N + 1) begin
                $display("  ERROR %0dx%0d: latency %0d, expected %0d", W, H, edges, N + 1);
                errors = errors + 1;
            end

            rd_sel = 0;
            for (i = 0; i < H; i = i + 1) begin
                rd_idx = i; #1;
                if (rd_data !== ref_row[i]) begin
                    if (errors < 5)
                        $display("  ERROR %0dx%0d frame %0d row[%0d] = %0d expected %0d",
                                 W, H, f, i, rd_data, ref_row[i]);
                    errors = errors + 1;
                end
            end
            rd_sel = 1;
            for (i = 0; i < W; i = i + 1) begin
                rd_idx = i; #1;
                if (rd_data !== ref_col[i]) begin
                    if (errors < 5)
                        $display("  ERROR %0dx%0d frame %0d col[%0d] = %0d expected %0d",
                                 W, H, f, i, rd_data, ref_col[i]);
                    errors = errors + 1;
                end
            end
            @(negedge clk);
        end

        $display("%0dx%0d, %0d-bit: %0d errors", W, H, DW, errors);
        finished = 1;
    end
endmodule


module tb_engine_params;

    reg clk = 0;
    always #5 clk = ~clk;

    wire        f0, f1, f2, f3;
    wire [31:0] e0, e1, e2, e3;

    engine_harness #(.W(32),  .H(16), .DW(8), .SEED(11)) h0 (.clk(clk), .finished(f0), .errors(e0));
    engine_harness #(.W(16),  .H(32), .DW(8), .SEED(22)) h1 (.clk(clk), .finished(f1), .errors(e1));
    engine_harness #(.W(8),   .H(8),  .DW(4), .SEED(33)) h2 (.clk(clk), .finished(f2), .errors(e2));
    engine_harness #(.W(128), .H(64), .DW(8), .SEED(44)) h3 (.clk(clk), .finished(f3), .errors(e3));

    initial begin
        $dumpfile("tb_engine_params.vcd");
        $dumpvars(1, tb_engine_params);
        wait (f0 && f1 && f2 && f3);
        if (e0 + e1 + e2 + e3 == 0) $display("PASS: tb_engine_params (4 configurations)");
        else                        $display("FAIL: tb_engine_params, %0d errors", e0 + e1 + e2 + e3);
        $finish;
    end

    initial begin
        #10000000 $display("FAIL: timeout"); $finish;
    end

endmodule

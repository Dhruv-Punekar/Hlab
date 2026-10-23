// ============================================================================
// tb_projection_engine.v
// Runs the projection engine on mem/image.mem (the photo, 64x64 grayscale)
// and compares every row and column sum against the Python golden model
// (mem/golden_row.mem, mem/golden_col.mem from scripts/img2mem.py).
// Also checks the frame latency and that a second run gives the same result.
//
// Override file paths with +define+IMAGE_MEM="..." etc. if running elsewhere.
// ============================================================================
`timescale 1ns / 1ps

`ifndef IMAGE_MEM
 `define IMAGE_MEM  "mem/image.mem"
`endif
`ifndef GOLDEN_ROW
 `define GOLDEN_ROW "mem/golden_row.mem"
`endif
`ifndef GOLDEN_COL
 `define GOLDEN_COL "mem/golden_col.mem"
`endif

module tb_projection_engine;

    localparam W  = 64;
    localparam H  = 64;
    localparam DW = 8;
    localparam AW = 12;
    localparam IW = 6;
    localparam SW = 14;

    reg clk = 0, rst = 1, start = 0;
    always #5 clk = ~clk;   // 100 MHz

    wire [AW-1:0] pix_addr;
    wire [DW-1:0] pix_data;
    wire          busy, done;
    reg           rd_sel = 0;
    reg  [IW-1:0] rd_idx = 0;
    wire [SW-1:0] rd_data;

    image_rom #(.DW(DW), .DEPTH(W*H), .MEM_FILE(`IMAGE_MEM)) u_rom (
        .clk(clk), .addr(pix_addr), .dout(pix_data)
    );

    projection_engine #(.W(W), .H(H), .DW(DW)) dut (
        .clk(clk), .rst(rst), .start(start),
        .pix_addr(pix_addr), .pix_data(pix_data),
        .busy(busy), .done(done),
        .rd_sel(rd_sel), .rd_idx(rd_idx), .rd_data(rd_data)
    );

    reg [15:0] golden_row [0:H-1];
    reg [15:0] golden_col [0:W-1];

    integer errors = 0;
    integer i, run, cycles;

    task run_and_check;
        begin
            @(negedge clk) start = 1;
            @(negedge clk) start = 0;
            cycles = 1;
            while (!done) begin
                @(negedge clk);
                cycles = cycles + 1;
                if (cycles > W*H + 100) begin
                    $display("FAIL: done never asserted");
                    $finish;
                end
            end
            $display("Run %0d: frame finished in %0d cycles (%0d pixels)", run, cycles, W*H);
            if (busy) begin
                $display("ERROR: busy still high with done");
                errors = errors + 1;
            end

            // row projection
            rd_sel = 0;
            for (i = 0; i < H; i = i + 1) begin
                rd_idx = i;
                #1;
                if (rd_data !== golden_row[i][SW-1:0]) begin
                    $display("ERROR row[%0d] = %0d, expected %0d", i, rd_data, golden_row[i]);
                    errors = errors + 1;
                end
            end
            // column projection
            rd_sel = 1;
            for (i = 0; i < W; i = i + 1) begin
                rd_idx = i;
                #1;
                if (rd_data !== golden_col[i][SW-1:0]) begin
                    $display("ERROR col[%0d] = %0d, expected %0d", i, rd_data, golden_col[i]);
                    errors = errors + 1;
                end
            end
        end
    endtask

    initial begin
        $dumpfile("tb_projection_engine.vcd");
        $dumpvars(0, tb_projection_engine);
        $readmemh(`GOLDEN_ROW, golden_row);
        $readmemh(`GOLDEN_COL, golden_col);

        repeat (3) @(negedge clk);
        rst = 0;
        repeat (2) @(negedge clk);

        for (run = 0; run < 2; run = run + 1) begin
            run_and_check;
            repeat (10) @(negedge clk);
        end

        $display("row[0]=%0d row[32]=%0d col[0]=%0d col[32]=%0d",
                 golden_row[0], golden_row[32], golden_col[0], golden_col[32]);
        if (errors == 0) $display("PASS: tb_projection_engine (%0d row + %0d column sums, 2 runs)", H, W);
        else             $display("FAIL: tb_projection_engine, %0d errors", errors);
        $finish;
    end

endmodule

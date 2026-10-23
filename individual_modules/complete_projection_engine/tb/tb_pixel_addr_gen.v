// ============================================================================
// tb_pixel_addr_gen.v
// Checks the raster scan: W*H valid cycles, addr == y*W + x, x fastest,
// `last` only on the final pixel, busy drops afterwards, start ignored while
// busy, and a second frame runs correctly.
// ============================================================================
`timescale 1ns / 1ps

module tb_pixel_addr_gen;

    localparam W  = 64;
    localparam H  = 64;
    localparam XW = $clog2(W);
    localparam YW = $clog2(H);
    localparam AW = $clog2(W*H);

    reg clk = 0, rst = 1, start = 0;
    always #5 clk = ~clk;

    wire [AW-1:0] addr;
    wire [XW-1:0] x;
    wire [YW-1:0] y;
    wire valid, last, busy;

    pixel_addr_gen #(.W(W), .H(H)) dut (
        .clk(clk), .rst(rst), .start(start),
        .addr(addr), .x(x), .y(y), .valid(valid), .last(last), .busy(busy)
    );

    integer errors = 0;
    integer count, last_count, frame;
    integer ex, ey;

    task run_frame;
        begin
            count = 0; last_count = 0; ex = 0; ey = 0;
            @(negedge clk) start = 1;
            @(negedge clk) start = 0;
            while (valid) begin
                if (addr !== ey*W + ex || x !== ex || y !== ey) begin
                    if (errors < 10)
                        $display("ERROR pixel %0d: addr=%0d x=%0d y=%0d (exp %0d,%0d,%0d)",
                                 count, addr, x, y, ey*W+ex, ex, ey);
                    errors = errors + 1;
                end
                if (last) begin
                    last_count = last_count + 1;
                    if (count != W*H-1) begin
                        $display("ERROR: last asserted at pixel %0d", count);
                        errors = errors + 1;
                    end
                end
                // a start pulse while busy must be ignored
                if (count == 100) start = 1;
                if (count == 101) start = 0;
                count = count + 1;
                ex = ex + 1;
                if (ex == W) begin ex = 0; ey = ey + 1; end
                @(negedge clk);
            end
            if (count != W*H) begin
                $display("ERROR: %0d valid pixels, expected %0d", count, W*H);
                errors = errors + 1;
            end
            if (last_count != 1) begin
                $display("ERROR: last asserted %0d times", last_count);
                errors = errors + 1;
            end
            if (busy) begin
                $display("ERROR: still busy after frame");
                errors = errors + 1;
            end
        end
    endtask

    initial begin
        $dumpfile("tb_pixel_addr_gen.vcd");
        $dumpvars(0, tb_pixel_addr_gen);
        repeat (3) @(negedge clk);
        rst = 0;
        repeat (2) @(negedge clk);
        for (frame = 0; frame < 2; frame = frame + 1) begin
            run_frame;
            repeat (5) @(negedge clk);
        end
        if (errors == 0) $display("PASS: tb_pixel_addr_gen (%0d pixels x 2 frames)", W*H);
        else             $display("FAIL: tb_pixel_addr_gen, %0d errors", errors);
        $finish;
    end

    initial begin
        #2000000 $display("FAIL: timeout"); $finish;
    end

endmodule

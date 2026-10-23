// ============================================================================
// tb_engine_control.v
// Control and timing tests for projection_engine (random 64x64 images):
//   1. busy rises on the clock edge that samples start; done is low during
//      the frame; done rises exactly W*H+1 edges after start is sampled and
//      busy falls on that same edge.
//   2. done stays high while idle and clears on the next start.
//   3. start pulses during a frame are ignored (same latency, same results).
//   4. reset in the middle of a frame stops it (busy/done low); a new frame
//      after reset gives correct results.
//   5. start held high for a long time: frames repeat back to back and the
//      results after start drops are correct.
//   6. results stay stable while idle even if the image memory changes.
// ============================================================================
`timescale 1ns / 1ps

module tb_engine_control;

    localparam W  = 64;
    localparam H  = 64;
    localparam N  = W * H;
    localparam DW = 8;
    localparam AW = 12;
    localparam IW = 6;
    localparam SW = 14;

    reg clk = 0, rst = 1, start = 0;
    always #5 clk = ~clk;

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
    integer errors = 0;
    integer seed   = 7;
    integer i, x, y, n;

    task err(input [8*64-1:0] msg);
        begin
            $display("  ERROR: %0s", msg);
            errors = errors + 1;
        end
    endtask

    task new_image;
        begin
            for (i = 0; i < N; i = i + 1) img[i] = $random(seed) & 8'hFF;
            for (y = 0; y < H; y = y + 1) ref_row[y] = 0;
            for (x = 0; x < W; x = x + 1) ref_col[x] = 0;
            for (y = 0; y < H; y = y + 1)
                for (x = 0; x < W; x = x + 1) begin
                    ref_row[y] = ref_row[y] + img[y*W + x];
                    ref_col[x] = ref_col[x] + img[y*W + x];
                end
        end
    endtask

    task check_results(input [8*32-1:0] name);
        integer e0;
        begin
            e0 = errors;
            rd_sel = 0;
            for (i = 0; i < H; i = i + 1) begin
                rd_idx = i; #1;
                if (rd_data !== ref_row[i]) errors = errors + 1;
            end
            rd_sel = 1;
            for (i = 0; i < W; i = i + 1) begin
                rd_idx = i; #1;
                if (rd_data !== ref_col[i]) errors = errors + 1;
            end
            if (errors != e0)
                $display("  ERROR: %0s - %0d wrong sums", name, errors - e0);
        end
    endtask

    // Pulse start, then count clock edges until done.
    // `glitch` = 1 sends extra start pulses in the middle of the frame.
    task timed_frame(input glitch, output integer edges);
        begin
            @(negedge clk) start = 1;
            @(posedge clk);                  // edge that samples start
            #1 start = 0;
            if (!busy) err("busy not high after start");
            if (done)  err("done not cleared by start");
            edges = 0;
            while (!done) begin
                @(posedge clk); #1;
                edges = edges + 1;
                if (glitch && (edges == 100 || edges == 2000)) begin
                    start = 1;
                    @(posedge clk); #1;
                    start = 0;
                    edges = edges + 1;
                end
                if (!done && !busy) err("busy dropped before done");
                if (edges > N + 50) begin
                    $display("FAIL: done never asserted"); $finish;
                end
            end
            if (busy) err("busy still high when done rises");
        end
    endtask

    initial begin
        $dumpfile("tb_engine_control.vcd");
        $dumpvars(0, tb_engine_control);
        new_image;
        repeat (3) @(negedge clk);
        rst = 0;
        repeat (2) @(negedge clk);

        if (busy || done) err("busy/done not low after reset");

        // ---- 1. latency ----
        timed_frame(0, n);
        $display("Test 1: done after %0d edges (expected %0d)", n, N + 1);
        if (n != N + 1) err("wrong frame latency");
        check_results("test 1");

        // ---- 2. done holds while idle ----
        repeat (50) @(negedge clk);
        if (!done) err("done dropped while idle");
        $display("Test 2: done held while idle");

        // ---- 3. start pulses mid-frame ignored ----
        new_image;
        timed_frame(1, n);
        $display("Test 3: frame with extra start pulses took %0d edges", n);
        if (n != N + 1) err("extra start pulses changed the frame");
        check_results("test 3");

        // ---- 4. reset mid-frame ----
        new_image;
        @(negedge clk) start = 1;
        @(negedge clk) start = 0;
        repeat (N / 2) @(negedge clk);
        rst = 1;
        @(negedge clk);
        if (busy || done) err("busy/done not cleared by reset mid-frame");
        rst = 0;
        repeat (5) @(negedge clk);
        if (busy) err("engine restarted by itself after reset");
        timed_frame(0, n);
        check_results("test 4 (after mid-frame reset)");
        $display("Test 4: reset mid-frame, re-run ok");

        // ---- 5. start held high ----
        new_image;
        @(negedge clk) start = 1;
        repeat (3 * N) @(negedge clk);
        start = 0;
        // let the running frame finish
        n = 0;
        while (!done || busy) begin
            @(negedge clk);
            n = n + 1;
            if (n > 2 * N) begin $display("FAIL: stuck with start held"); $finish; end
        end
        check_results("test 5 (start held high)");
        $display("Test 5: start held high, results ok");

        // ---- 6. results stable while idle ----
        for (i = 0; i < N; i = i + 1) img[i] = 8'hFF;
        repeat (100) @(negedge clk);
        check_results("test 6 (memory changed while idle)");
        $display("Test 6: results unchanged while idle");

        if (errors == 0) $display("PASS: tb_engine_control (6 tests)");
        else             $display("FAIL: tb_engine_control, %0d errors", errors);
        $finish;
    end

endmodule

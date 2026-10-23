// ============================================================================
// tb_z_f_buffer.v
// Self-checking testbench for z_f_buffer.v (depth test + framebuffer,
// 3-stage pipeline with distance-1/2 forwarding, fast clear, VGA read port).
//
// Uses a 40 x 30 buffer so the clear and full-frame readback are quick; the
// logic is identical for 300 x 400 (change W / H below to run full size).
//
// A behavioural model keeps the expected depth and color of every pixel:
//   pixel (x,y,z,c) accepted when x < W, y < H and z < zbuf (SIGNED, strict)
//   clear -> z = +32767 (0x7FFF), color = 0
// After each test the whole framebuffer is read back through the VGA port
// (separate, asynchronous clock) and compared with the model.
//
// Tests
//   1. reset / clear: busy, ready_out, clear_done timing (W*H cycles),
//      framebuffer black after clear
//   2. single pixels incl. corners
//   3. depth rules: nearer wins, farther / equal rejected, negative beats
//      positive, -32768 nearest, +32767 never beats the clear value
//   4. read-after-write hazards: same pixel at distance 1, 2, 3, 4 in both
//      depth orders, and runs of 2..6 same-pixel writes back to back
//   5. out-of-bounds pixels ignored (no address wrap into other pixels)
//   6. pixels sent during a clear are ignored; reset mid-render re-clears
//   7. random stress: 4000 pixels on a 4x3 patch (constant hazards) and
//      4000 pixels over the whole buffer, random valid gaps
//   8. VGA port: out-of-range coordinates read black
// ============================================================================
`timescale 1ns / 1ps

module tb_z_f_buffer;

    localparam W   = 40;
    localparam H   = 30;
    localparam PIX = W * H;

    reg clk_render = 0, clk_vga = 0;
    always #5   clk_render = ~clk_render;     // 100 MHz
    always #6.5 clk_vga    = ~clk_vga;        // ~77 MHz, unrelated phase

    reg         rst_n = 0;
    reg         valid_in = 0;
    reg  [9:0]  pixel_x = 0, pixel_y = 0;
    reg  [15:0] pixel_z = 0, pixel_color = 0;
    wire        ready_out, clear_done, busy;
    reg  [9:0]  vga_x = 0, vga_y = 0;
    wire [15:0] vga_color;

    z_f_buffer #(.WIDTH(W), .HEIGHT(H)) dut (
        .clk_render(clk_render), .rst_n(rst_n),
        .valid_in(valid_in), .pixel_x(pixel_x), .pixel_y(pixel_y),
        .pixel_z(pixel_z), .pixel_color(pixel_color),
        .ready_out(ready_out), .clear_done(clear_done), .busy(busy),
        .clk_vga(clk_vga), .vga_x(vga_x), .vga_y(vga_y), .vga_color(vga_color)
    );

    // ---------------- behavioural model ----------------
    reg signed [15:0] mz [0:PIX-1];
    reg        [15:0] mc [0:PIX-1];
    integer errors = 0, seed = 31, accepted_px = 0, sent_px = 0;
    reg [15:0] tag = 16'h0001;               // unique color per pixel write

    task model_clear;
        integer i;
        begin
            for (i = 0; i < PIX; i = i + 1) begin mz[i] = 16'sh7FFF; mc[i] = 16'h0000; end
        end
    endtask

    // drive one pixel for one render clock (model updated in order)
    task pix(input integer x, input integer y, input signed [15:0] z);
        integer a;
        begin
            pixel_x = x; pixel_y = y; pixel_z = z; pixel_color = tag;
            valid_in = 1;
            sent_px = sent_px + 1;
            if (x < W && y < H) begin
                a = y * W + x;
                if (z < mz[a]) begin
                    mz[a] = z; mc[a] = tag;
                    accepted_px = accepted_px + 1;
                end
            end
            tag = (tag == 16'hFFFF) ? 16'h0001 : tag + 1'b1;
            @(negedge clk_render);
        end
    endtask

    task idle(input integer n);
        begin
            valid_in = 0;
            pixel_x = $random(seed); pixel_y = $random(seed);      // junk on the bus
            pixel_z = 16'sh8000;                                    // would win if taken
            pixel_color = 16'hDEAD;
            repeat (n) @(negedge clk_render);
        end
    endtask

    // ---------------- VGA readback ----------------
    task vga_read(input integer x, input integer y, output [15:0] c);
        begin
            @(negedge clk_vga);
            vga_x = x; vga_y = y;
            @(posedge clk_vga);
            #1 c = vga_color;
        end
    endtask

    task check_frame(input [8*40-1:0] name);
        integer x, y, e0;
        reg [15:0] c;
        begin
            idle(6);                                  // let the pipeline drain
            e0 = errors;
            for (y = 0; y < H; y = y + 1)
                for (x = 0; x < W; x = x + 1) begin
                    vga_read(x, y, c);
                    if (c !== mc[y*W + x]) begin
                        if (errors - e0 < 5)
                            $display("  ERROR %0s: pixel (%0d,%0d) = %04h expected %04h (z %0d)",
                                     name, x, y, c, mc[y*W + x], mz[y*W + x]);
                        errors = errors + 1;
                    end
                end
            $display("%0s: %0s", name, (errors == e0) ? "ok" : "MISMATCH");
        end
    endtask

    task do_reset_and_clear(input check_timing);
        integer n;
        begin
            @(negedge clk_render);
            rst_n = 0;
            #1;
            if (!busy || ready_out || clear_done) begin
                $display("  ERROR: busy/ready_out/clear_done wrong in reset"); errors = errors + 1;
            end
            @(negedge clk_render);
            rst_n = 1;
            model_clear;
            n = 0;
            while (!clear_done && n < PIX + 10) begin
                if (ready_out) begin $display("  ERROR: ready_out during clear"); errors = errors + 1; end
                @(negedge clk_render);
                n = n + 1;
            end
            if (check_timing) begin
                if (n != PIX) begin
                    $display("  ERROR: clear took %0d cycles, expected %0d", n, PIX); errors = errors + 1;
                end else
                    $display("  clear took %0d cycles (= W*H)", n);
            end
            if (busy || !ready_out) begin $display("  ERROR: busy/ready after clear"); errors = errors + 1; end
        end
    endtask

    integer i, j, d, k, x, y, a, run;
    reg [15:0] c;
    reg signed [15:0] za, zb;

    initial begin
        $dumpfile("tb_z_f_buffer.vcd");
        $dumpvars(0, tb_z_f_buffer);

        // ---- 1. reset / clear ----
        $display("Test 1: reset and clear");
        repeat (3) @(negedge clk_render);
        do_reset_and_clear(1);
        check_frame("1 cleared frame is black");

        // ---- 2. single pixels ----
        pix(0, 0, 100);       idle(2);
        pix(W-1, 0, 100);     idle(2);
        pix(0, H-1, 100);     idle(2);
        pix(W-1, H-1, 100);   idle(2);
        pix(17, 11, 0);       idle(2);
        for (i = 0; i < 20; i = i + 1) begin pix(i, i % H, 500 + i); idle(1); end
        check_frame("2 single pixels");

        // ---- 3. depth rules (gaps of 5 cycles: no hazards) ----
        pix(5, 5, 100);     idle(5);
        pix(5, 5, 200);     idle(5);   // farther -> rejected
        pix(5, 5, 50);      idle(5);   // nearer  -> accepted
        pix(5, 5, 50);      idle(5);   // equal   -> rejected
        pix(5, 5, -10);     idle(5);   // negative beats positive
        pix(5, 5, -32768);  idle(5);   // nearest possible
        pix(5, 5, -32767);  idle(5);   // rejected
        pix(6, 5, 32767);   idle(5);   // equal to clear value -> rejected
        pix(7, 5, 32766);   idle(5);   // accepted
        pix(8, 5, 1);       idle(5);
        pix(8, 5, -1);      idle(5);   // signed: -1 < 1
        check_frame("3 depth rules (signed, strict)");

        // ---- 4. hazards ----
        for (d = 1; d <= 4; d = d + 1)
            for (k = 0; k < 2; k = k + 1) begin
                x = 10 + d; y = 10 + k;
                za = k ? 300 : 100;  zb = k ? 100 : 300;      // both orders
                pix(x, y, za);
                for (j = 1; j < d; j = j + 1) pix(0, 20 + j, 1000 + j);   // other pixels
                pix(x, y, zb);
                idle(4);
            end
        // distance 2 through a bubble
        pix(20, 3, 400); idle(1); pix(20, 3, 300); idle(4);
        // runs of same-pixel writes, random depths
        for (run = 2; run <= 6; run = run + 1) begin
            for (j = 0; j < run; j = j + 1) pix(25 + run, 7, $random(seed));
            idle(3);
        end
        // decreasing / increasing runs
        for (j = 0; j < 8; j = j + 1) pix(30, 9, 800 - 100 * j);
        for (j = 0; j < 8; j = j + 1) pix(31, 9, 100 * j - 300);
        // ping-pong two pixels sharing no address but adjacent in time
        for (j = 0; j < 12; j = j + 1) pix(32 + (j % 2), 9, 600 - 40 * j);
        check_frame("4 read-after-write hazards");

        // ---- 5. out of bounds ----
        pix(W, 0, -32768);          // would alias pixel (0,1) without the check
        pix(0, H, -32768);
        pix(1023, 1023, -32768);
        pix(W + 1, H - 1, -32768);
        pix(W - 1, H, -32768);
        idle(2);
        pix(0, 1, 10);              // in bounds, still behaves normally
        check_frame("5 out-of-bounds ignored");

        // ---- 6. pixels during clear are ignored; reset mid-render ----
        pix(3, 3, -5);
        pix(4, 3, -5);
        @(negedge clk_render);
        rst_n = 0;                  // reset while pixels are in the pipeline
        valid_in = 1; pixel_x = 2; pixel_y = 2; pixel_z = 16'sh8000; pixel_color = 16'hBAD0;
        @(negedge clk_render);
        rst_n = 1;
        model_clear;
        i = 0;
        while (!clear_done) begin    // keep sending during the clear
            pixel_x = {$random(seed)} % W; pixel_y = {$random(seed)} % H;
            pixel_z = 16'sh8000; pixel_color = 16'hBAD1;
            valid_in = 1;
            @(negedge clk_render);
            i = i + 1;
        end
        valid_in = 0;
        if (i != PIX) begin $display("  ERROR: re-clear took %0d cycles", i); errors = errors + 1; end
        check_frame("6 reset mid-render, pixels during clear ignored");

        // ---- 7. random stress ----
        for (i = 0; i < 4000; i = i + 1) begin
            if (({$random(seed)} % 100) < 30) idle(1 + {$random(seed)} % 3);
            pix({$random(seed)} % 4, {$random(seed)} % 3, $random(seed));
        end
        check_frame("7a random on a 4x3 patch (constant hazards)");
        for (i = 0; i < 4000; i = i + 1) begin
            if (({$random(seed)} % 100) < 20) idle(1);
            x = {$random(seed)} % (W + 3);              // a few out of bounds
            y = {$random(seed)} % (H + 2);
            pix(x, y, $random(seed) % 2000);
        end
        check_frame("7b random over the whole buffer");

        // ---- 8. VGA out of range ----
        k = errors;
        vga_read(W, 0, c);     if (c !== 0) errors = errors + 1;
        vga_read(0, H, c);     if (c !== 0) errors = errors + 1;
        vga_read(1023, 5, c);  if (c !== 0) errors = errors + 1;
        vga_read(W - 1, H - 1, c);
        if (c !== mc[PIX - 1]) errors = errors + 1;
        $display("8 VGA out-of-range reads black: %0s", (errors == k) ? "ok" : "MISMATCH");

        $display("%0d pixels sent, %0d passed the depth test in the model", sent_px, accepted_px);
        if (errors == 0) $display("PASS: tb_z_f_buffer");
        else             $display("FAIL: tb_z_f_buffer, %0d errors", errors);
        $finish;
    end

    initial begin
        #50000000 $display("FAIL: timeout"); $finish;
    end

endmodule

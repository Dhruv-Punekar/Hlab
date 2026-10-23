// ============================================================================
// tb_vga_controller.v
// Self-checking testbench for vga_controller.v (640x480@60, 300x400 window
// at (170,40), RGB565 -> RGB444, 2-cycle output pipeline, fb_ready CDC).
//
// The framebuffer read port is modelled like z_f_buffer: registered, one
// clock of latency, black for x >= 300 or y >= 400. Each pixel has a known
// color, so every output can be predicted.
//
// Every clock the testbench checks, against counters of its own:
//   fb_x / fb_y        window coordinates for the current scan position,
//                      (300, 400) outside the window
//   hsync / vsync      low exactly for h 656..751 / lines 490..491,
//                      delayed 2 clocks like the RGB
//   vga_r/g/b          color of the pixel scanned 2 clocks earlier,
//                      top bits of RGB565, black outside the window and while
//                      the synchronised fb_ready is low
//   video_active       window flag of the pixel scanned 1 clock earlier
//
// Tests
//   1. reset values (syncs high, RGB black)
//   2. fb_ready low for the first part of frame 0 -> black screen, then high
//      (2-FF synchroniser delay checked exactly)
//   3. two full frames: line = 800 clocks, frame = 525 lines (420000 clocks)
//   4. asynchronous reset in the middle of a line, then a third full frame
// Runs about 1.3 million clocks.
// ============================================================================
`timescale 1ns / 1ps

module tb_vga_controller;

    localparam H_TOTAL = 800, V_TOTAL = 525;
    localparam X_OFF = 170, Y_OFF = 40, FB_W = 300, FB_H = 400;

    reg clk_pix = 0;
    always #20 clk_pix = ~clk_pix;            // 25 MHz

    reg         rst_n = 0;
    reg         fb_ready = 0;
    reg  [15:0] fb_color = 0;
    wire [9:0]  fb_x, fb_y;
    wire [3:0]  vga_r, vga_g, vga_b;
    wire        hsync, vsync, video_active;

    vga_controller dut (
        .clk_pix(clk_pix), .rst_n(rst_n), .fb_ready(fb_ready), .fb_color(fb_color),
        .fb_x(fb_x), .fb_y(fb_y), .vga_r(vga_r), .vga_g(vga_g), .vga_b(vga_b),
        .hsync(hsync), .vsync(vsync), .video_active(video_active)
    );

    // ---------------- framebuffer model ----------------
    function [15:0] color_of(input integer x, input integer y);
        reg [9:0] xx, yy;
        begin
            xx = x; yy = y;
            color_of = {xx[7:0] ^ yy[7:0], yy[3:0], xx[3:0]};
        end
    endfunction

    always @(posedge clk_pix)
        fb_color <= (fb_x < FB_W && fb_y < FB_H) ? color_of(fb_x, fb_y) : 16'h0000;

    // ---------------- reference counters ----------------
    // cnt = number of clock edges since reset was released
    integer cnt = 0;
    reg     rdy_at [0:7];                    // fb_ready seen at edge cnt (mod 8)
    always @(posedge clk_pix or negedge rst_n) begin
        if (!rst_n) cnt = 0;
        else begin
            cnt = cnt + 1;
            rdy_at[cnt % 8] = fb_ready;
        end
    end

    function in_window(input integer k);     // k = scan index (clocks since reset)
        integer h, v;
        begin
            h = k % H_TOTAL;
            v = (k / H_TOTAL) % V_TOTAL;
            in_window = (h >= X_OFF) && (h < X_OFF + FB_W) && (v >= Y_OFF) && (v < Y_OFF + FB_H);
        end
    endfunction

    integer errors = 0, checks = 0, lit = 0, frames_seen = 0;

    task fail(input [8*20-1:0] what);
        begin
            if (errors < 15)
                $display("  ERROR at clock %0d (h=%0d v=%0d): %0s", cnt,
                         (cnt - 2) % H_TOTAL, ((cnt - 2) / H_TOTAL) % V_TOTAL, what);
            errors = errors + 1;
        end
    endtask

    // check all outputs after the edge number `cnt`
    task check_now;
        integer k, h, v, hn, vn;
        reg exp_hs, exp_vs, exp_show, exp_act, rdy;
        reg [15:0] c;
        reg [9:0] exp_fx, exp_fy;
        begin
            checks = checks + 1;
            // fb_x / fb_y from the current scan position (index cnt)
            hn = cnt % H_TOTAL; vn = (cnt / H_TOTAL) % V_TOTAL;
            exp_fx = in_window(cnt) ? hn - X_OFF : FB_W;
            exp_fy = in_window(cnt) ? vn - Y_OFF : FB_H;
            if (fb_x !== exp_fx || fb_y !== exp_fy) fail("fb_x/fb_y");

            // video_active = window flag of scan index cnt-1
            exp_act = (cnt >= 1) ? in_window(cnt - 1) : 1'b0;
            if (video_active !== exp_act) fail("video_active");

            if (cnt < 2) begin
                if (hsync !== 1'b1 || vsync !== 1'b1) fail("sync after reset");
                if ({vga_r, vga_g, vga_b} !== 12'h000) fail("rgb after reset");
            end else begin
                k = cnt - 2;
                h = k % H_TOTAL;
                v = (k / H_TOTAL) % V_TOTAL;
                exp_hs = !((h >= 656) && (h < 752));
                exp_vs = !((v >= 490) && (v < 492));
                if (hsync !== exp_hs) fail("hsync");
                if (vsync !== exp_vs) fail("vsync");
                rdy = (k >= 1) ? rdy_at[k % 8] : 1'b0;   // fb_ready at edge cnt-2
                exp_show = in_window(k) && rdy;
                if (exp_show) begin
                    c = color_of(h - X_OFF, v - Y_OFF);
                    if (vga_r !== c[15:12] || vga_g !== c[10:7] || vga_b !== c[4:1]) fail("rgb value");
                    if (c[15:12] | c[10:7] | c[4:1]) lit = lit + 1;
                end else if ({vga_r, vga_g, vga_b} !== 12'h000) fail("rgb not black");
            end
        end
    endtask

    // run n clocks, checking after every edge
    task run(input integer n);
        integer i;
        begin
            for (i = 0; i < n; i = i + 1) begin
                @(negedge clk_pix);
                check_now;
            end
        end
    endtask

    // frame / line timing measured on the outputs
    integer t_vs_fall = -1, vs_period = 0, hs_count = 0, hs_per_frame = 0, hs_low = 0, hs_width = 0;
    reg hs_prev = 1, vs_prev = 1;
    always @(posedge clk_pix) if (rst_n) begin
        if (hs_prev && !hsync) hs_count = hs_count + 1;
        if (!hsync) hs_low = hs_low + 1;
        if (!hs_prev && hsync) begin hs_width = hs_low; hs_low = 0; end
        if (vs_prev && !vsync) begin
            if (t_vs_fall >= 0) begin
                vs_period = cnt - t_vs_fall;
                hs_per_frame = hs_count;
                frames_seen = frames_seen + 1;
            end
            t_vs_fall = cnt;
            hs_count = 0;
        end
        hs_prev = hsync; vs_prev = vsync;
    end

    initial begin
        $dumpfile("tb_vga_controller.vcd");
        $dumpvars(1, tb_vga_controller);

        // ---- 1. reset ----
        repeat (4) @(negedge clk_pix);
        if (hsync !== 1'b1 || vsync !== 1'b1 || {vga_r, vga_g, vga_b} !== 12'h000) begin
            $display("  ERROR: outputs not idle during reset"); errors = errors + 1;
        end
        $display("Test 1: reset values checked");

        // ---- 2/3. fb_ready low, then high mid-frame; two full frames ----
        rst_n = 1;
        check_now;
        run(H_TOTAL * 100 + 333);                // window rows 40..99 stay black
        fb_ready = 1;                            // in the middle of a visible line
        run(2 * H_TOTAL * V_TOTAL);
        $display("Test 2/3: %0d frames, last frame %0d clocks, %0d hsync pulses, hsync %0d clocks wide",
                 frames_seen, vs_period, hs_per_frame, hs_width);
        if (vs_period != H_TOTAL * V_TOTAL || hs_per_frame != V_TOTAL || hs_width != 96) begin
            $display("  ERROR: frame timing"); errors = errors + 1;
        end
        $display("  frame rate at 25.175 MHz: %f Hz", 25175000.0 / (H_TOTAL * V_TOTAL));

        // ---- 4. async reset mid-line ----
        #7 rst_n = 0;
        #1;
        if (hsync !== 1'b1 || vsync !== 1'b1 || {vga_r, vga_g, vga_b} !== 12'h000 ||
            fb_x !== FB_W || fb_y !== FB_H) begin
            $display("  ERROR: outputs not reset asynchronously"); errors = errors + 1;
        end
        repeat (3) @(negedge clk_pix);
        rst_n = 1;
        t_vs_fall = -1;
        check_now;
        run(H_TOTAL * V_TOTAL + 2 * H_TOTAL);
        $display("Test 4: reset mid-line, then a full frame");

        $display("%0d clocks checked, %0d lit pixels compared", checks, lit);
        if (lit < 100000) begin $display("  ERROR: too few lit pixels, test not meaningful"); errors = errors + 1; end
        if (errors == 0) $display("PASS: tb_vga_controller");
        else             $display("FAIL: tb_vga_controller, %0d errors", errors);
        $finish;
    end

    initial begin
        #100000000 $display("FAIL: timeout"); $finish;
    end

endmodule

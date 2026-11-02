`timescale 1ns / 1ps
// ============================================================================
// Module: perf_monitor                       (NEW, numbers for the 7-segment)
//
// Measures, with plain hardware counters (render clock, 100 MHz):
//   FPS          frames that reached the screen in the last second
//   render time  clear + draw time of the last frame, in microseconds
//   pixels       pixels the rasterizer sent to the z-buffer in the last frame
// and turns them into 8 digit codes for seg7_driver.
//
// All counters count directly in BCD (one 4-bit decimal digit per group), so
// no binary-to-decimal converter is needed: "add 1" is done digit by digit
// with a carry, like counting by hand. Every counter stops at its maximum
// (999, 9999, 999999) instead of wrapping.
//
// Display (digit 7 = leftmost, AN7):
//   page 0 (SW10 down):  [d|S] [FPS 3 digits] [render time ms, X.XXX]
//                        e.g.  d  60 1.392   -> double buffer, 60 FPS, 1.392 ms
//                              S 712 1.388   -> single buffer, 712 FPS
//   page 1 (SW10 up):    [P] [blank] [pixels per frame, 6 digits]
//                        e.g.  P    23456
// Leading zeros are blanked. The numbers on the display are refreshed 4 times
// per second so they are readable (the counters themselves run every frame).
// ============================================================================
module perf_monitor #(
    parameter WINDOW_CLKS = 100_000_000,   // FPS window: 1 s at 100 MHz
    parameter DISP_CLKS   = 25_000_000,    // display refresh: 0.25 s
    parameter US_CLKS     = 100            // 1 microsecond at 100 MHz
)(
    input  wire        clk,
    input  wire        rst_n,

    input  wire        frame_shown,   // pulse: one new frame is on the screen
    input  wire        render_start,  // pulse: render of a frame starts
    input  wire        render_end,    // pulse: render finished (render_done)
    input  wire        pixel,         // 1 = one pixel sent to the z-buffer
    input  wire        mode_dbl,      // 1 = double buffering, 0 = single
    input  wire        page,          // 0 = FPS / render time, 1 = pixels

    output reg  [39:0] digits,        // 8 x 5-bit codes, digit 7 in [39:35]
    output reg  [7:0]  dp             // decimal points, 1 = on
);
    // digit codes understood by seg7_driver
    localparam [4:0] C_BLANK = 5'd10, C_d = 5'd11, C_S = 5'd12, C_P = 5'd13;

    // ------------------------------------------------------------------------
    // saturating BCD increment of the lowest nd digits of v
    // ------------------------------------------------------------------------
    function [23:0] bcd_inc;
        input [23:0] v;
        input integer nd;
        integer i;
        reg carry, all9;
        begin
            bcd_inc = v;
            all9 = 1'b1;
            for (i = 0; i < 6; i = i + 1)
                if (i < nd && v[i*4 +: 4] != 4'd9) all9 = 1'b0;
            carry = !all9;                         // at the maximum: stay
            for (i = 0; i < 6; i = i + 1)
                if (carry && i < nd) begin
                    if (v[i*4 +: 4] == 4'd9)
                        bcd_inc[i*4 +: 4] = 4'd0;
                    else begin
                        bcd_inc[i*4 +: 4] = v[i*4 +: 4] + 4'd1;
                        carry = 1'b0;
                    end
                end
        end
    endfunction

    // ------------------------------------------------------------------------
    // FPS: count shown frames inside a 1-second window
    // ------------------------------------------------------------------------
    reg [31:0] win_cnt;
    reg [11:0] fps_cnt;      // 3 BCD digits
    reg [11:0] fps_last;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            win_cnt <= 32'd0; fps_cnt <= 12'd0; fps_last <= 12'd0;
        end else if (win_cnt == WINDOW_CLKS - 1) begin
            win_cnt  <= 32'd0;
            fps_last <= frame_shown ? bcd_inc({12'd0, fps_cnt}, 3) : fps_cnt;
            fps_cnt  <= 12'd0;
        end else begin
            win_cnt <= win_cnt + 32'd1;
            if (frame_shown) fps_cnt <= bcd_inc({12'd0, fps_cnt}, 3);
        end
    end

    // ------------------------------------------------------------------------
    // render time in microseconds (4 BCD digits, max 9999 us)
    // ------------------------------------------------------------------------
    reg        rt_run;
    reg [7:0]  us_pre;
    reg [15:0] rt_cnt;
    reg [15:0] rt_last;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rt_run <= 1'b0; us_pre <= 8'd0; rt_cnt <= 16'd0; rt_last <= 16'd0;
        end else if (render_start) begin
            rt_run <= 1'b1; us_pre <= 8'd0; rt_cnt <= 16'd0;
        end else if (rt_run) begin
            if (render_end) begin
                rt_run  <= 1'b0;
                rt_last <= rt_cnt;
            end else if (us_pre == US_CLKS - 1) begin
                us_pre <= 8'd0;
                rt_cnt <= bcd_inc({8'd0, rt_cnt}, 4);
            end else
                us_pre <= us_pre + 8'd1;
        end
    end

    // ------------------------------------------------------------------------
    // pixels per frame (6 BCD digits)
    //   This counter can step on EVERY clock, so (timing) the carry from one
    //   digit to the next goes through a register (px_c) instead of rippling
    //   through all 6 digits in one clock: each digit only checks "am I 9?".
    //   A digit wraps at most once every 10 steps, so a 1-clock late carry is
    //   never lost; render_end comes 64+ quiet clocks after the last pixel,
    //   so all carries have arrived by then. (Above 999999 it wraps; a frame
    //   has at most 120000 pixels without overdraw.)
    // ------------------------------------------------------------------------
    reg [23:0] px_cnt, px_last;
    reg [4:0]  px_c;          // px_c[i-1] = "add 1 to digit i" (digits 1..5)
    integer    d;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            px_cnt <= 24'd0; px_last <= 24'd0; px_c <= 5'd0;
        end else if (render_start) begin
            px_cnt <= 24'd0; px_c <= 5'd0;
        end else begin
            // digit 0: counts the pixels
            px_c[0] <= 1'b0;
            if (pixel) begin
                if (px_cnt[3:0] == 4'd9) begin px_cnt[3:0] <= 4'd0; px_c[0] <= 1'b1; end
                else                            px_cnt[3:0] <= px_cnt[3:0] + 4'd1;
            end
            // digits 1..5: count the (registered) carries of the digit below
            for (d = 1; d < 6; d = d + 1) begin
                if (d < 5) px_c[d] <= 1'b0;
                if (px_c[d-1]) begin
                    if (px_cnt[d*4 +: 4] == 4'd9) begin
                        px_cnt[d*4 +: 4] <= 4'd0;
                        if (d < 5) px_c[d] <= 1'b1;
                    end else
                        px_cnt[d*4 +: 4] <= px_cnt[d*4 +: 4] + 4'd1;
                end
            end
            if (render_end) px_last <= px_cnt;
        end
    end

    // ------------------------------------------------------------------------
    // display copy, refreshed 4 times per second
    // ------------------------------------------------------------------------
    reg [31:0] disp_cnt;
    reg [11:0] d_fps;
    reg [15:0] d_rt;
    reg [23:0] d_px;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            disp_cnt <= 32'd0; d_fps <= 12'd0; d_rt <= 16'd0; d_px <= 24'd0;
        end else if (disp_cnt == DISP_CLKS - 1) begin
            disp_cnt <= 32'd0;
            d_fps <= fps_last; d_rt <= rt_last; d_px <= px_last;
        end else
            disp_cnt <= disp_cnt + 32'd1;
    end

    // ------------------------------------------------------------------------
    // digit codes
    // ------------------------------------------------------------------------
    // leading-zero blanking helpers
    wire f2z = (d_fps[11:8] == 4'd0);
    wire f1z = f2z && (d_fps[7:4] == 4'd0);

    wire p5z = (d_px[23:20] == 4'd0);
    wire p4z = p5z && (d_px[19:16] == 4'd0);
    wire p3z = p4z && (d_px[15:12] == 4'd0);
    wire p2z = p3z && (d_px[11:8]  == 4'd0);
    wire p1z = p2z && (d_px[7:4]   == 4'd0);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            digits <= {8{C_BLANK}};
            dp     <= 8'd0;
        end else if (!page) begin
            digits <= { (mode_dbl ? C_d : C_S),
                        (f2z ? C_BLANK : {1'b0, d_fps[11:8]}),
                        (f1z ? C_BLANK : {1'b0, d_fps[7:4]}),
                        {1'b0, d_fps[3:0]},
                        {1'b0, d_rt[15:12]},
                        {1'b0, d_rt[11:8]},
                        {1'b0, d_rt[7:4]},
                        {1'b0, d_rt[3:0]} };
            dp     <= 8'b0000_1000;               // X.XXX ms
        end else begin
            digits <= { C_P, C_BLANK,
                        (p5z ? C_BLANK : {1'b0, d_px[23:20]}),
                        (p4z ? C_BLANK : {1'b0, d_px[19:16]}),
                        (p3z ? C_BLANK : {1'b0, d_px[15:12]}),
                        (p2z ? C_BLANK : {1'b0, d_px[11:8]}),
                        (p1z ? C_BLANK : {1'b0, d_px[7:4]}),
                        {1'b0, d_px[3:0]} };
            dp     <= 8'b0000_0000;
        end
    end
endmodule

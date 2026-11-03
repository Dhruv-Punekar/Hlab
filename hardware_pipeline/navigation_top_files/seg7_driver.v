`timescale 1ns / 1ps
// ============================================================================
// Module: seg7_driver                         (NEW, Nexys A7 8-digit display)
//
// The 8 digits share the same 7 segment wires (CA..CG, DP); only one digit is
// switched on at a time (its AN pin low) and the driver steps through all 8
// quickly. 2^REFRESH_BITS clocks per digit: 2^17 / 100 MHz = 1.3 ms, so the
// whole display is redrawn 95 times per second -> the eye sees all 8 at once.
// All Nexys A7 display signals are active LOW.
//
// Digit codes: 0..9 = number, 10 = blank, 11 = 'd', 12 = 'S', 13 = 'P', 14 = '-'
// seg[0] = CA (a) ... seg[6] = CG (g)
//        a
//      f   b
//        g
//      e   c
//        d   .dp
// ============================================================================
module seg7_driver #(
    parameter REFRESH_BITS = 17
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire [39:0] digits,     // 8 x 5-bit codes, digit 7 (leftmost) in [39:35]
    input  wire [7:0]  dp,         // decimal point per digit, 1 = on
    output reg  [7:0]  an,         // digit enable, active low
    output reg  [6:0]  seg,        // CA..CG, active low
    output reg         dp_n        // decimal point, active low
);
    reg [REFRESH_BITS+2:0] cnt;
    wire [2:0] sel = cnt[REFRESH_BITS+2:REFRESH_BITS];

    always @(posedge clk or negedge rst_n)
        if (!rst_n) cnt <= 0;
        else        cnt <= cnt + 1'b1;

    // segment pattern, active high, bit order {g,f,e,d,c,b,a}
    function [6:0] pattern;
        input [4:0] c;
        begin
            case (c)
                5'd0:  pattern = 7'b0111111;
                5'd1:  pattern = 7'b0000110;
                5'd2:  pattern = 7'b1011011;
                5'd3:  pattern = 7'b1001111;
                5'd4:  pattern = 7'b1100110;
                5'd5:  pattern = 7'b1101101;
                5'd6:  pattern = 7'b1111101;
                5'd7:  pattern = 7'b0000111;
                5'd8:  pattern = 7'b1111111;
                5'd9:  pattern = 7'b1101111;
                5'd11: pattern = 7'b1011110;   // d
                5'd12: pattern = 7'b1101101;   // S
                5'd13: pattern = 7'b1110011;   // P
                5'd14: pattern = 7'b1000000;   // -
                default: pattern = 7'b0000000; // blank
            endcase
        end
    endfunction

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            an   <= 8'hFF;
            seg  <= 7'h7F;
            dp_n <= 1'b1;
        end else begin
            an   <= ~(8'd1 << sel);
            seg  <= ~pattern(digits[sel*5 +: 5]);
            dp_n <= ~dp[sel];
        end
    end
endmodule

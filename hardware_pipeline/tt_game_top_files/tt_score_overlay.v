`timescale 1ns / 1ps

// ============================================================================
// Module: tt_score_overlay   (pixel clock domain)
//   Draws the score panel on top of the pipeline's VGA output and gives the
//   game its frame timing.
//
//   Panel (screen pixels, top-right corner of the 300x400 picture by default):
//       [ 0 7 - 0 5 ]     P1 score (red) - P2 score (white), 5x7 font x2
//        ‾‾‾              yellow bar under the server's score
//   At game over the winner's score blinks. Nothing is rendered by the
//   rasterizer, so the panel costs no render time and never flickers.
//
//   Position tracking: the pipeline's VGA outputs are registered and follow
//   640x480@60 timing, so the screen position of the pixel on the inputs is
//   recovered from the sync edges: hsync falls at column 656, vsync falls at
//   column 0 of line 490. (hx, vy) = position of the input pixel.
//   Ticks (one clock): line 440 (framebuffer no longer read -> game step and
//   re-render) and line 40 (framebuffer read starts -> render must be done).
//
//   Output: 2 register stages (decode, mux); RGB and syncs are delayed alike,
//   so the picture just moves 2 pixels right, as with the pipeline's own
//   2-clock latency.
// ============================================================================
module tt_score_overlay #(
    parameter PX0 = 398,               // panel left column (screen)
    parameter PY0 = 46                 // panel top line
)(
    input  wire       clk,
    input  wire       rst_n,

    input  wire [3:0] in_r, in_g, in_b,
    input  wire       in_hs, in_vs,

    input  wire [6:0] score0,
    input  wire [6:0] score1,
    input  wire       server,
    input  wire       game_over,
    input  wire       winner,

    output reg  [3:0] vga_r, vga_g, vga_b,
    output reg        vga_hs, vga_vs,

    output reg        tick_440,        // start of line 440
    output reg        tick_40          // start of line 40
);

    localparam TX0 = PX0 + 4;          // text origin
    localparam TY0 = PY0 + 4;
    localparam PW  = 66, PH = 26;      // panel size

    localparam [11:0] C_PANEL = 12'h124;
    localparam [11:0] C_P1    = 12'hF54;
    localparam [11:0] C_P2    = 12'hFFF;
    localparam [11:0] C_DASH  = 12'hAAA;
    localparam [11:0] C_SERVE = 12'hFD0;

    // ------------------------------------------------------------------------
    // screen position of the input pixel
    // ------------------------------------------------------------------------
    reg       hs_q, vs_q;
    reg [9:0] hx, vy;
    wire hs_fall = hs_q & ~in_hs;
    wire vs_fall = vs_q & ~in_vs;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            hs_q <= 1'b1;  vs_q <= 1'b1;
            hx <= 10'd0;  vy <= 10'd0;
            tick_440 <= 1'b0;  tick_40 <= 1'b0;
        end else begin
            hs_q <= in_hs;
            vs_q <= in_vs;
            if (hs_fall)            hx <= 10'd657;
            else if (hx == 10'd799) hx <= 10'd0;
            else                    hx <= hx + 10'd1;
            if (vs_fall)            vy <= 10'd490;
            else if (hx == 10'd799) vy <= (vy == 10'd524) ? 10'd0 : vy + 10'd1;
            tick_440 <= (hx == 10'd799) && (vy == 10'd439) && !vs_fall;
            tick_40  <= (hx == 10'd799) && (vy == 10'd39)  && !vs_fall;
        end
    end

    // ------------------------------------------------------------------------
    // digits (registered every clock; scores change once per frame)
    // ------------------------------------------------------------------------
    function [7:0] bcd;  input [6:0] v;  begin
        if      (v >= 7'd90) bcd = {4'd9, v - 7'd90};
        else if (v >= 7'd80) bcd = {4'd8, v - 7'd80};
        else if (v >= 7'd70) bcd = {4'd7, v - 7'd70};
        else if (v >= 7'd60) bcd = {4'd6, v - 7'd60};
        else if (v >= 7'd50) bcd = {4'd5, v - 7'd50};
        else if (v >= 7'd40) bcd = {4'd4, v - 7'd40};
        else if (v >= 7'd30) bcd = {4'd3, v - 7'd30};
        else if (v >= 7'd20) bcd = {4'd2, v - 7'd20};
        else if (v >= 7'd10) bcd = {4'd1, v - 7'd10};
        else                 bcd = {4'd0, v[3:0]};
    end endfunction

    reg [7:0] d0, d1;                  // {tens, ones}
    reg [5:0] fcnt;                    // frames, for blinking
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin d0 <= 8'd0; d1 <= 8'd0; fcnt <= 6'd0; end
        else begin
            d0 <= bcd(score0 > 7'd99 ? 7'd99 : score0);
            d1 <= bcd(score1 > 7'd99 ? 7'd99 : score1);
            if (tick_440) fcnt <= fcnt + 6'd1;
        end
    end

    // 5x7 font: glyph 0..9 digits, 10 dash; row 0 = top, bit 4 = left column
    function [4:0] font;  input [3:0] gl;  input [2:0] row;  begin
        case ({gl, row})
            {4'd0,3'd0}: font = 5'b01110; {4'd0,3'd1}: font = 5'b10001; {4'd0,3'd2}: font = 5'b10011;
            {4'd0,3'd3}: font = 5'b10101; {4'd0,3'd4}: font = 5'b11001; {4'd0,3'd5}: font = 5'b10001;
            {4'd0,3'd6}: font = 5'b01110;
            {4'd1,3'd0}: font = 5'b00100; {4'd1,3'd1}: font = 5'b01100; {4'd1,3'd2}: font = 5'b00100;
            {4'd1,3'd3}: font = 5'b00100; {4'd1,3'd4}: font = 5'b00100; {4'd1,3'd5}: font = 5'b00100;
            {4'd1,3'd6}: font = 5'b01110;
            {4'd2,3'd0}: font = 5'b01110; {4'd2,3'd1}: font = 5'b10001; {4'd2,3'd2}: font = 5'b00001;
            {4'd2,3'd3}: font = 5'b00010; {4'd2,3'd4}: font = 5'b00100; {4'd2,3'd5}: font = 5'b01000;
            {4'd2,3'd6}: font = 5'b11111;
            {4'd3,3'd0}: font = 5'b11111; {4'd3,3'd1}: font = 5'b00010; {4'd3,3'd2}: font = 5'b00100;
            {4'd3,3'd3}: font = 5'b00010; {4'd3,3'd4}: font = 5'b00001; {4'd3,3'd5}: font = 5'b10001;
            {4'd3,3'd6}: font = 5'b01110;
            {4'd4,3'd0}: font = 5'b00010; {4'd4,3'd1}: font = 5'b00110; {4'd4,3'd2}: font = 5'b01010;
            {4'd4,3'd3}: font = 5'b10010; {4'd4,3'd4}: font = 5'b11111; {4'd4,3'd5}: font = 5'b00010;
            {4'd4,3'd6}: font = 5'b00010;
            {4'd5,3'd0}: font = 5'b11111; {4'd5,3'd1}: font = 5'b10000; {4'd5,3'd2}: font = 5'b11110;
            {4'd5,3'd3}: font = 5'b00001; {4'd5,3'd4}: font = 5'b00001; {4'd5,3'd5}: font = 5'b10001;
            {4'd5,3'd6}: font = 5'b01110;
            {4'd6,3'd0}: font = 5'b00110; {4'd6,3'd1}: font = 5'b01000; {4'd6,3'd2}: font = 5'b10000;
            {4'd6,3'd3}: font = 5'b11110; {4'd6,3'd4}: font = 5'b10001; {4'd6,3'd5}: font = 5'b10001;
            {4'd6,3'd6}: font = 5'b01110;
            {4'd7,3'd0}: font = 5'b11111; {4'd7,3'd1}: font = 5'b00001; {4'd7,3'd2}: font = 5'b00010;
            {4'd7,3'd3}: font = 5'b00100; {4'd7,3'd4}: font = 5'b01000; {4'd7,3'd5}: font = 5'b01000;
            {4'd7,3'd6}: font = 5'b01000;
            {4'd8,3'd0}: font = 5'b01110; {4'd8,3'd1}: font = 5'b10001; {4'd8,3'd2}: font = 5'b10001;
            {4'd8,3'd3}: font = 5'b01110; {4'd8,3'd4}: font = 5'b10001; {4'd8,3'd5}: font = 5'b10001;
            {4'd8,3'd6}: font = 5'b01110;
            {4'd9,3'd0}: font = 5'b01110; {4'd9,3'd1}: font = 5'b10001; {4'd9,3'd2}: font = 5'b10001;
            {4'd9,3'd3}: font = 5'b01111; {4'd9,3'd4}: font = 5'b00001; {4'd9,3'd5}: font = 5'b00010;
            {4'd9,3'd6}: font = 5'b01100;
            {4'd10,3'd3}: font = 5'b11111;
            default: font = 5'b00000;
        endcase
    end endfunction

    // ------------------------------------------------------------------------
    // stage A: decode the input pixel position
    // ------------------------------------------------------------------------
    wire signed [10:0] rx = $signed({1'b0, hx}) - TX0;
    wire signed [10:0] ry = $signed({1'b0, vy}) - TY0;
    wire in_panel = (hx >= PX0) && (hx < PX0 + PW) && (vy >= PY0) && (vy < PY0 + PH);
    wire in_text  = (rx >= 0) && (rx < 58) && (ry >= 0) && (ry < 14);
    wire [2:0] cel = (rx >= 48) ? 3'd4 : (rx >= 36) ? 3'd3 : (rx >= 24) ? 3'd2 : (rx >= 12) ? 3'd1 : 3'd0;
    wire [5:0] cx   = rx[5:0] - {cel, 3'b000} - {cel, 2'b00};       // rx - 12*cel
    wire in_bar   = (ry >= 16) && (ry < 18) &&
                    (server ? (rx >= 36 && rx < 58) : (rx >= 0 && rx < 22));

    reg        a_panel, a_text, a_bar;
    reg  [2:0] a_cell, a_row, a_col;
    reg  [3:0] a_r, a_g, a_b;
    reg        a_hs, a_vs;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            a_panel <= 1'b0;  a_text <= 1'b0;  a_bar <= 1'b0;
            a_cell <= 3'd0;  a_row <= 3'd0;  a_col <= 3'd0;
            a_r <= 4'd0;  a_g <= 4'd0;  a_b <= 4'd0;  a_hs <= 1'b1;  a_vs <= 1'b1;
        end else begin
            a_panel <= in_panel;
            a_text  <= in_text && (cx < 6'd10);
            a_bar   <= in_bar;
            a_cell  <= cel;
            a_row   <= ry[3:1];
            a_col   <= cx[3:1];
            a_r <= in_r;  a_g <= in_g;  a_b <= in_b;  a_hs <= in_hs;  a_vs <= in_vs;
        end
    end

    // ------------------------------------------------------------------------
    // stage B: glyph pixel, colour, output registers
    // ------------------------------------------------------------------------
    wire [3:0] glyph = (a_cell == 3'd0) ? d0[7:4] : (a_cell == 3'd1) ? d0[3:0] :
                       (a_cell == 3'd2) ? 4'd10   : (a_cell == 3'd3) ? d1[7:4] : d1[3:0];
    wire [4:0] frow  = font(glyph, a_row);
    wire       fpix  = frow[3'd4 - a_col];
    wire       blink_off = game_over && fcnt[4] &&
                           (winner ? (a_cell >= 3'd3) : (a_cell <= 3'd1));
    wire [11:0] tcol = (a_cell <= 3'd1) ? C_P1 : (a_cell == 3'd2) ? C_DASH : C_P2;
    wire [11:0] pcol = (a_text && fpix && !blink_off) ? tcol : a_bar ? C_SERVE : C_PANEL;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            vga_r <= 4'd0;  vga_g <= 4'd0;  vga_b <= 4'd0;  vga_hs <= 1'b1;  vga_vs <= 1'b1;
        end else begin
            vga_hs <= a_hs;
            vga_vs <= a_vs;
            if (a_panel) begin
                vga_r <= pcol[11:8];  vga_g <= pcol[7:4];  vga_b <= pcol[3:0];
            end else begin
                vga_r <= a_r;  vga_g <= a_g;  vga_b <= a_b;
            end
        end
    end

endmodule

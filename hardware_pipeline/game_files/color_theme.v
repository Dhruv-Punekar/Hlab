`timescale 1ns / 1ps
// ============================================================================
// Module: color_theme                                     (NEW, game feature)
//   Re-colours the finished picture on its way to the VGA pins, so no
//   pipeline module (vertex colours, framebuffer) has to change.
//   Black (background / blanking) always stays black.
//   Registered: R, G, B, HSYNC and VSYNC are all delayed by the same one
//   pixel clock, so the picture stays aligned with the sync pulses.
//
//   theme (SW7 SW6 SW5)
//     0  000  original colours
//     1  001  channels rotated  (red faces -> blue, green -> red, blue -> green)
//     2  010  channels rotated the other way
//     3  011  inverted colours (photo negative)
//     4  100  grey scale  (6 different grey levels, one per face colour)
//     5  101  amber monitor
//     6  110  green monitor (retro CRT)
//     7  111  ice blue
// ============================================================================
module color_theme (
    input  wire       clk_pix,
    input  wire       rst_n,
    input  wire [2:0] theme,          // already synchronised to clk_pix
    input  wire [3:0] in_r, in_g, in_b,
    input  wire       in_hs, in_vs,
    output reg  [3:0] out_r, out_g, out_b,
    output reg        out_hs, out_vs
);
    // grey level: weights 3:5:1 give 6 different levels for the 6 cube colours,
    // then lifted into 4..14 so even the darkest face stays visible
    wire [7:0] luma0 = in_r * 3 + in_g * 5 + in_b;      // 0..135
    wire [3:0] luma1 = (luma0 >> 3 > 15) ? 4'd15 : luma0[6:3];
    wire [7:0] lift  = luma1 * 11;
    wire [3:0] y     = 4'd4 + lift[7:4];                 // 4..14
    wire       black = (in_r == 4'd0) && (in_g == 4'd0) && (in_b == 4'd0);

    reg [3:0] r, g, b;
    always @* begin
        case (theme)
            3'd1:    begin r = in_g;          g = in_b;          b = in_r;          end
            3'd2:    begin r = in_b;          g = in_r;          b = in_g;          end
            3'd3:    begin r = 4'd15 - in_r;  g = 4'd15 - in_g;  b = 4'd15 - in_b;  end
            3'd4:    begin r = y;             g = y;             b = y;             end
            3'd5:    begin r = y;             g = y >> 1;        b = 4'd0;          end
            3'd6:    begin r = 4'd0;          g = y;             b = 4'd0;          end
            3'd7:    begin r = 4'd0;          g = y >> 1;        b = y;             end
            default: begin r = in_r;          g = in_g;          b = in_b;          end
        endcase
        if (black) begin r = 4'd0; g = 4'd0; b = 4'd0; end
    end

    always @(posedge clk_pix or negedge rst_n) begin
        if (!rst_n) begin
            out_r <= 4'd0; out_g <= 4'd0; out_b <= 4'd0; out_hs <= 1'b1; out_vs <= 1'b1;
        end else begin
            out_r <= r; out_g <= g; out_b <= b; out_hs <= in_hs; out_vs <= in_vs;
        end
    end
endmodule

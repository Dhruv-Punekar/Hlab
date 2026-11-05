`timescale 1ns / 1ps

// ============================================================================
// Module: tt_tilt   (pixel clock domain)
//   Board tilt (ADXL362, mg) -> P1 aim: the x (Q8.8 m) where P1's shot lands.
//     a      = swap_axes ? acc_y : acc_x ;  invert -> -a
//     f     += (a - f) / 4                  (light low-pass, per sample)
//     d      = dead zone +-50 mg, clamp +-400 mg
//     tilt_x = d * 23 / 8   ->  +-350 mg -> +-1006 (~ +-0.39 m), clamped +-TT_TILT_MAX
//   aim_leds: one-hot position of tilt_x over 8 LEDs (bit 7 = far left).
// ============================================================================
module tt_tilt (
    input  wire               clk,
    input  wire               rst_n,
    input  wire signed [11:0] acc_x,
    input  wire signed [11:0] acc_y,
    input  wire               acc_valid,
    input  wire               swap_axes,
    input  wire               invert,
    output reg         [15:0] tilt_x,      // signed Q8.8
    output reg         [7:0]  aim_leds
);

`include "tt_game_map.vh"

    localparam integer DEAD = 50, LIM = 400;

    wire signed [12:0] a_raw = swap_axes ? acc_y : acc_x;
    wire signed [12:0] a     = invert ? -a_raw : a_raw;
    reg  signed [12:0] f;
    wire signed [13:0] f_nx  = f + ((a - f + 14'sd2) >>> 2);

    // dead zone + clamp
    wire signed [13:0] d = (f >  DEAD) ? ((f >  LIM) ?  (LIM - DEAD) : f - DEAD) :
                           (f < -DEAD) ? ((f < -LIM) ? -(LIM - DEAD) : f + DEAD) : 14'sd0;
    wire signed [18:0] t   = (d * 23) >>> 3;
    wire signed [18:0] t_c = (t >  TT_TILT_MAX) ?  TT_TILT_MAX :
                             (t < -TT_TILT_MAX) ? -TT_TILT_MAX : t;
    // LED index 0..7
    wire        [18:0] t_u = t_c + TT_TILT_MAX;              // 0 .. 2*MAX
    wire        [29:0] li  = (t_u * 455) >> 17;              // t_u * 8 / 2304 -> 0..7

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            f <= 13'sd0;  tilt_x <= 16'd0;  aim_leds <= 8'b0001_1000;
        end else begin
            if (acc_valid) f <= f_nx[12:0];
            tilt_x   <= t_c[15:0];
            aim_leds <= 8'b1000_0000 >> li[2:0];              // left LED = far left aim
        end
    end

endmodule

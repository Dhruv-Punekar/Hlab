`timescale 1ns / 1ps

// ============================================================================
// Module: tt_obj_animator   (render clock domain, 100 MHz)
//   Rewrites the vertices of the moving objects (two rackets, ball) in
//   vertex_memory from their poses, before each render.
//
//   For object vertex i (local coordinates l from the object ROM in
//   tt_game_map.vh, Q8.8 units, relative to the object's pivot):
//       X = tx + (cos*lx + sin*lz) >>> 14
//       Y = ty + ly
//       Z = tz + (cos*lz - sin*lx) >>> 14          (yaw about the vertical axis)
//   cos/sin Q1.14 from the 64-step table. A hidden object (vis = 0) gets all its
//   vertices on the pivot -> zero-area triangles, discarded by the setup unit.
//   Output word: X|Y|Z|W (Q24.8, W = 1.0) | RGB565, written to address
//   BASE + i (the objects are stored last and contiguous in vertices.mem).
//
//   Pipeline (one vertex per clock, every stage registered, the four 16x16
//   multiplies map to DSP48E1 with input + product registers):
//     P0 ROM read          P1 pose select       P2 multiply
//     P3 sums              P4 shift + translate P5 write port registers
//   156 vertices -> 156 + 6 clocks = 1.6 us. Use only while the render domain
//   is held in reset (no vertex fetch in flight).
//
//   Bit-exact reference: tt_game_model.py, animate().
// ============================================================================
module tt_obj_animator #(
    parameter BASE = 843                // first object vertex in vertices.mem
)(
    input  wire         clk,
    input  wire         rst_n,
    input  wire         start,          // 1-clock pulse; poses must be stable until done

    input  wire [15:0]  p0_x, p0_y, p0_z,   // object 0: near racket
    input  wire [5:0]   p0_yaw,
    input  wire [15:0]  p1_x, p1_y, p1_z,   // object 1: far racket
    input  wire [5:0]   p1_yaw,
    input  wire [15:0]  p2_x, p2_y, p2_z,   // object 2: ball
    input  wire [5:0]   p2_yaw,
    input  wire         p2_vis,

    output reg          vtx_we,
    output reg  [15:0]  vtx_addr,
    output reg  [143:0] vtx_data,
    output reg          busy,
    output reg          done            // 1-clock pulse after the last write
);

`include "tt_game_map.vh"

    // ------------------------------------------------------------------------
    // cos/sin of the three yaws, looked up once at start
    // ------------------------------------------------------------------------
    reg [31:0] cs0, cs1, cs2;
    always @(posedge clk) if (start) begin
        cs0 <= tt_sincos(p0_yaw);
        cs1 <= tt_sincos(p1_yaw);
        cs2 <= tt_sincos(p2_yaw);
    end

    // ------------------------------------------------------------------------
    // P0: index counter + ROM read
    // ------------------------------------------------------------------------
    reg  [7:0]  idx;
    reg         run;
    (* rom_style = "distributed" *) reg [65:0] rom_q;
    reg  [7:0]  i0;
    reg         v0;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            idx <= 8'd0;  run <= 1'b0;  v0 <= 1'b0;  i0 <= 8'd0;
        end else begin
            v0 <= 1'b0;
            if (start) begin
                idx <= 8'd0;  run <= 1'b1;
            end else if (run) begin
                v0 <= 1'b1;
                i0 <= idx;
                if (idx == TT_OBJ_VERTS - 1) run <= 1'b0;
                idx <= idx + 8'd1;
            end
        end
    end
    always @(posedge clk) if (run) rom_q <= tt_obj_rom(idx);

    // ------------------------------------------------------------------------
    // P1: pose of this vertex's object
    // ------------------------------------------------------------------------
    reg                v1, vis1;
    reg  [7:0]         i1;
    reg  signed [15:0] lx1, ly1, lz1, c1, s1, tx1, ty1, tz1;
    reg  [15:0]        col1;

    always @(posedge clk) begin
        v1   <= v0;
        i1   <= i0;
        lx1  <= rom_q[63:48];
        ly1  <= rom_q[47:32];
        lz1  <= rom_q[31:16];
        col1 <= rom_q[15:0];
        case (rom_q[65:64])
            2'd0:    begin tx1 <= p0_x; ty1 <= p0_y; tz1 <= p0_z; c1 <= cs0[31:16]; s1 <= cs0[15:0]; vis1 <= 1'b1;   end
            2'd1:    begin tx1 <= p1_x; ty1 <= p1_y; tz1 <= p1_z; c1 <= cs1[31:16]; s1 <= cs1[15:0]; vis1 <= 1'b1;   end
            default: begin tx1 <= p2_x; ty1 <= p2_y; tz1 <= p2_z; c1 <= cs2[31:16]; s1 <= cs2[15:0]; vis1 <= p2_vis; end
        endcase
    end

    // ------------------------------------------------------------------------
    // P2: products (DSP)
    // ------------------------------------------------------------------------
    reg                v2, vis2;
    reg  [7:0]         i2;
    reg  signed [31:0] m_cx, m_sz, m_sx, m_cz;
    reg  signed [15:0] ly2, tx2, ty2, tz2;
    reg  [15:0]        col2;

    always @(posedge clk) begin
        v2 <= v1;  i2 <= i1;  vis2 <= vis1;  col2 <= col1;
        ly2 <= ly1;  tx2 <= tx1;  ty2 <= ty1;  tz2 <= tz1;
        m_cx <= c1 * lx1;
        m_sz <= s1 * lz1;
        m_sx <= s1 * lx1;
        m_cz <= c1 * lz1;
    end

    // ------------------------------------------------------------------------
    // P3: rotated sums
    // ------------------------------------------------------------------------
    reg                v3, vis3;
    reg  [7:0]         i3;
    reg  signed [32:0] sum_x, sum_z;
    reg  signed [15:0] ly3, tx3, ty3, tz3;
    reg  [15:0]        col3;

    always @(posedge clk) begin
        v3 <= v2;  i3 <= i2;  vis3 <= vis2;  col3 <= col2;
        ly3 <= ly2;  tx3 <= tx2;  ty3 <= ty2;  tz3 <= tz2;
        sum_x <= m_cx + m_sz;
        sum_z <= m_cz - m_sx;
    end

    // ------------------------------------------------------------------------
    // P4: scale back (>>> 14) and translate; hidden -> pivot
    // ------------------------------------------------------------------------
    reg                v4;
    reg  [7:0]         i4;
    reg  signed [19:0] X4, Y4, Z4;
    reg  [15:0]        col4;
    wire signed [18:0] rx = sum_x[32:14];
    wire signed [18:0] rz = sum_z[32:14];

    always @(posedge clk) begin
        v4 <= v3;  i4 <= i3;  col4 <= col3;
        if (vis3) begin
            X4 <= tx3 + rx;
            Y4 <= ty3 + ly3;
            Z4 <= tz3 + rz;
        end else begin
            X4 <= tx3;
            Y4 <= ty3;
            Z4 <= tz3;
        end
    end

    // ------------------------------------------------------------------------
    // P5: write port registers
    // ------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            vtx_we   <= 1'b0;
            vtx_addr <= 16'd0;
            vtx_data <= 144'd0;
            busy     <= 1'b0;
            done     <= 1'b0;
        end else begin
            vtx_we   <= v4;
            vtx_addr <= BASE + i4;
            vtx_data <= {{12{X4[19]}}, X4, {12{Y4[19]}}, Y4, {12{Z4[19]}}, Z4, 32'h0000_0100, col4};
            done     <= v4 && (i4 == TT_OBJ_VERTS - 1);
            if (start)                                busy <= 1'b1;
            else if (v4 && (i4 == TT_OBJ_VERTS - 1))  busy <= 1'b0;
        end
    end

endmodule

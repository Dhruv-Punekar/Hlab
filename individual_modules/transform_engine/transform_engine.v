`timescale 1ns / 1ps

// ============================================================================
// Module: transform_engine
// Purpose: Applies a 4x4 Transformation Matrix to a 3D vertex using a
//          fully pipelined DSP datapath. Uses Q24.8 Fixed-Point math.
// Features: Handshaking (CE Stalls), Color Passthrough, Saturation logic.
//
// Timing fix (2026-10-27): 5 pipeline stages (was 3).
//   The old stage 1 did "input pins -> 32x32 multiply -> register" in one
//   cycle (~8.9 ns of DSP + carry logic). Anything in front of it (matrix
//   register fan-out, vertex BRAM clock-to-out + route) broke 100 MHz:
//   +0.185 ns for the cube, -3.1 ns for the 4096-vertex picture.
//     S0  register the vertex and the matrix      (-> DSP48 A/B input regs)
//     S1  32x32 multiply, first register          (-> DSP48 M/P regs, cascade)
//     S2  multiply, second register               (lets Vivado finish the
//                                                   multi-DSP product inside
//                                                   the DSP pipeline)
//     S3  saturating truncation + first add level  (unchanged maths)
//     S4  second add level -> output               (unchanged maths)
//   Results are bit-identical to the 3-stage version; only latency grows
//   from 3 to 5 cycles. Throughput stays 1 vertex per clock.
//
//   Data registers sit in a block WITHOUT reset, so Vivado can pack them into
//   the DSP48E1 registers (those only have a synchronous reset). Only the
//   valid bits use the asynchronous reset.
//   Every register advances only when ready_in = 1 (global stall), so data,
//   colour and valid always stay aligned.
//
// Matrix word order: M00 = matrix_in[511:480] ... M33 = matrix_in[31:0]
// (matrix_memory word 15 = M00, i.e. matrix.mem written M33 first).
// ============================================================================
module transform_engine (
    input  wire         clk,
    input  wire         rst_n,      // Active-low reset

    // Handshake Signals
    input  wire         valid_in,   // High when Master FSM provides valid data
    output wire         ready_out,  // High when this module can accept new data
    input  wire         ready_in,   // High when Projection Engine can accept data
    output wire         valid_out,  // High when output math is finished

    // Data Interfaces
    input  wire [143:0] vertex_in,  // Incoming Vertex {X, Y, Z, W, RGB565}
    input  wire [511:0] matrix_in,  // 4x4 Transformation Matrix (16x 32-bit)
    output wire [143:0] vertex_out  // Transformed Vertex {X', Y', Z', W', RGB565}
);

    localparam NST = 5;             // pipeline stages S0..S4

    // ========================================================================
    // COMBINATIONAL FUNCTIONS (Saturation Safety Nets) - unchanged
    // ========================================================================
    // 1. Saturation Truncation (64-bit to 32-bit Q24.8)
    function signed [31:0] sat_trunc;
        input signed [63:0] prod;
        begin
            if (prod[63] == 1'b0 && (|prod[62:39] == 1'b1))
                sat_trunc = 32'h7FFF_FFFF; // Clamp to Max Positive
            else if (prod[63] == 1'b1 && (~&prod[62:39] == 1'b1))
                sat_trunc = 32'h8000_0000; // Clamp to Max Negative
            else
                sat_trunc = prod[39:8];    // Normal Truncation
        end
    endfunction

    // 2. Saturation Addition (32-bit + 32-bit)
    function signed [31:0] sat_add;
        input signed [31:0] a;
        input signed [31:0] b;
        reg signed [31:0] sum;
        begin
            sum = a + b;
            if (a[31] == 1'b0 && b[31] == 1'b0 && sum[31] == 1'b1)
                sat_add = 32'h7FFF_FFFF; // Positive Overflow
            else if (a[31] == 1'b1 && b[31] == 1'b1 && sum[31] == 1'b0)
                sat_add = 32'h8000_0000; // Negative Overflow
            else
                sat_add = sum;           // Normal Addition
        end
    endfunction

    // ========================================================================
    // HANDSHAKE: global stall, same as before
    // ========================================================================
    assign ready_out = ready_in;

    // valid bits: the only registers with a reset
    reg [NST-1:0] val;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)        val <= {NST{1'b0}};
        else if (ready_in) val <= {val[NST-2:0], valid_in};
    end

    // ========================================================================
    // DATAPATH (no reset -> DSP48E1 register packing)
    //   v_r[c]          vertex component c (0=X 1=Y 2=Z 3=W)
    //   m_r[r*4+c]      matrix element M[r][c]
    //   pm/p[r*4+c]     product v[c] * M[r][c]
    // ========================================================================
    reg signed [31:0] v_r  [0:3];
    reg signed [31:0] m_r  [0:15];
    reg signed [63:0] pm   [0:15];
    reg signed [63:0] p    [0:15];
    reg signed [31:0] s_ab [0:3];   // row r: sat_trunc(p[r,0]) + sat_trunc(p[r,1])
    reg signed [31:0] s_cd [0:3];   // row r: sat_trunc(p[r,2]) + sat_trunc(p[r,3])
    reg signed [31:0] o    [0:3];   // X', Y', Z', W'
    reg        [15:0] col  [0:NST-1];

    integer i, r;
    always @(posedge clk) begin
        if (ready_in) begin
            // colour travels with the vertex, one register per stage
            col[0] <= vertex_in[15:0];
            for (i = 1; i < NST; i = i + 1) col[i] <= col[i-1];

            // S0: register inputs
            v_r[0] <= $signed(vertex_in[143:112]);   // X
            v_r[1] <= $signed(vertex_in[111:80]);    // Y
            v_r[2] <= $signed(vertex_in[79:48]);     // Z
            v_r[3] <= $signed(vertex_in[47:16]);     // W
            for (i = 0; i < 16; i = i + 1)
                m_r[i] <= $signed(matrix_in[(15 - i)*32 +: 32]);   // element 0 = M00 = [511:480]

            // S1: 16 multiplies (row r = i/4, column c = i%4)
            for (i = 0; i < 16; i = i + 1)
                pm[i] <= v_r[i % 4] * m_r[i];

            // S2: second product register (DSP pipeline)
            for (i = 0; i < 16; i = i + 1)
                p[i] <= pm[i];

            // S3: saturating truncation + first adder level
            for (r = 0; r < 4; r = r + 1) begin
                s_ab[r] <= sat_add(sat_trunc(p[r*4 + 0]), sat_trunc(p[r*4 + 1]));
                s_cd[r] <= sat_add(sat_trunc(p[r*4 + 2]), sat_trunc(p[r*4 + 3]));
            end

            // S4: final sum
            for (r = 0; r < 4; r = r + 1)
                o[r] <= sat_add(s_ab[r], s_cd[r]);
        end
    end

    // ========================================================================
    // OUTPUT ASSIGNMENT
    // ========================================================================
    assign vertex_out = {o[0], o[1], o[2], o[3], col[NST-1]};
    assign valid_out  = val[NST-1];

endmodule

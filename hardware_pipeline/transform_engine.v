`timescale 1ns / 1ps

// ============================================================================
// Module: transform_engine
// Purpose: Applies a 4x4 Transformation Matrix to a 3D vertex using a
//          fully pipelined 3-stage DSP datapath. Uses Q24.8 Fixed-Point math.
// Features: Handshaking (CE Stalls), Color Passthrough, Saturation logic.
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
    input  wire [511:0] matrix_in,  // 4x4 Matrix, element k=row*4+col at [k*32 +: 32]
    output wire [143:0] vertex_out  // Transformed Vertex {X', Y', Z', W', RGB565}
);

    // ========================================================================
    // COMBINATIONAL FUNCTIONS (Saturation Safety Nets)
    // ========================================================================
    // 1. Saturation Truncation (64-bit to 32-bit Q24.8)
    // Prevents extreme scaling overflow from wrapping around the screen.
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
    // Prevents adding two massive numbers from flipping the sign bit.
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
    // 1. DATA UNPACKING (Combinational)
    // ========================================================================
    // Extract Vertex Coordinates
    wire signed [31:0] in_X = $signed(vertex_in[143:112]);
    wire signed [31:0] in_Y = $signed(vertex_in[111:80]);
    wire signed [31:0] in_Z = $signed(vertex_in[79:48]);
    wire signed [31:0] in_W = $signed(vertex_in[47:16]);
    wire        [15:0] in_C = vertex_in[15:0];

    // Extract Matrix Elements (Rows 0 to 3)
    // FIX (matrix word order): matrix_memory flattens element k (row-major,
    // k = row*4 + col, i.e. line k of matrix.mem) into matrix_in[k*32 +: 32].
    // So M00 is in the LOWEST word [31:0] and M33 in the HIGHEST [511:480].
    // (The previous mapping read the bus back-to-front, rotating the matrix
    //  by 180 degrees: M[r][c] -> M[3-r][3-c].)
    wire signed [31:0] M00 = $signed(matrix_in[ 0*32 +: 32]);
    wire signed [31:0] M01 = $signed(matrix_in[ 1*32 +: 32]);
    wire signed [31:0] M02 = $signed(matrix_in[ 2*32 +: 32]);
    wire signed [31:0] M03 = $signed(matrix_in[ 3*32 +: 32]);
    
    wire signed [31:0] M10 = $signed(matrix_in[ 4*32 +: 32]);
    wire signed [31:0] M11 = $signed(matrix_in[ 5*32 +: 32]);
    wire signed [31:0] M12 = $signed(matrix_in[ 6*32 +: 32]);
    wire signed [31:0] M13 = $signed(matrix_in[ 7*32 +: 32]);
    
    wire signed [31:0] M20 = $signed(matrix_in[ 8*32 +: 32]);
    wire signed [31:0] M21 = $signed(matrix_in[ 9*32 +: 32]);
    wire signed [31:0] M22 = $signed(matrix_in[10*32 +: 32]);
    wire signed [31:0] M23 = $signed(matrix_in[11*32 +: 32]);
    
    wire signed [31:0] M30 = $signed(matrix_in[12*32 +: 32]);
    wire signed [31:0] M31 = $signed(matrix_in[13*32 +: 32]);
    wire signed [31:0] M32 = $signed(matrix_in[14*32 +: 32]);
    wire signed [31:0] M33 = $signed(matrix_in[15*32 +: 32]);

    // ========================================================================
    // PIPELINE REGISTERS
    // ========================================================================
    reg val_stg1, val_stg2, val_stg3;
    reg [15:0] color_stg1, color_stg2, color_stg3;

    // Stage 1: 64-bit Multiplication Results
    reg signed [63:0] pX0, pX1, pX2, pX3;
    reg signed [63:0] pY0, pY1, pY2, pY3;
    reg signed [63:0] pZ0, pZ1, pZ2, pZ3;
    reg signed [63:0] pW0, pW1, pW2, pW3;

    // Stage 2: 32-bit Partial Sums
    reg signed [31:0] sX_AB, sX_CD;
    reg signed [31:0] sY_AB, sY_CD;
    reg signed [31:0] sZ_AB, sZ_CD;
    reg signed [31:0] sW_AB, sW_CD;

    // Stage 3: Final 32-bit Output
    reg signed [31:0] out_X, out_Y, out_Z, out_W;

    // ========================================================================
    // CLOCKED PIPELINE LOGIC
    // ========================================================================
    assign ready_out = ready_in; // Back-pressure propagation

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            val_stg1 <= 0; val_stg2 <= 0; val_stg3 <= 0;
        end else if (ready_in) begin

            // Push Valid Flags & Color through pipeline
            val_stg1 <= valid_in;
            val_stg2 <= val_stg1;
            val_stg3 <= val_stg2;

            color_stg1 <= in_C;
            color_stg2 <= color_stg1;
            color_stg3 <= color_stg2;

            // ----------------------------------------------------------------
            // STAGE 1: 16 PARALLEL MULTIPLICATIONS (Infers DSP48E1 slices)
            // ----------------------------------------------------------------
            pX0 <= in_X * M00;  pX1 <= in_Y * M01;  pX2 <= in_Z * M02;  pX3 <= in_W * M03;
            pY0 <= in_X * M10;  pY1 <= in_Y * M11;  pY2 <= in_Z * M12;  pY3 <= in_W * M13;
            pZ0 <= in_X * M20;  pZ1 <= in_Y * M21;  pZ2 <= in_Z * M22;  pZ3 <= in_W * M23;
            pW0 <= in_X * M30;  pW1 <= in_Y * M31;  pW2 <= in_Z * M32;  pW3 <= in_W * M33;

            // ----------------------------------------------------------------
            // STAGE 2: SATURATION TRUNCATION & ADDER TREE STAGE 1
            // ----------------------------------------------------------------
            sX_AB <= sat_add(sat_trunc(pX0), sat_trunc(pX1));
            sX_CD <= sat_add(sat_trunc(pX2), sat_trunc(pX3));

            sY_AB <= sat_add(sat_trunc(pY0), sat_trunc(pY1));
            sY_CD <= sat_add(sat_trunc(pY2), sat_trunc(pY3));

            sZ_AB <= sat_add(sat_trunc(pZ0), sat_trunc(pZ1));
            sZ_CD <= sat_add(sat_trunc(pZ2), sat_trunc(pZ3));

            sW_AB <= sat_add(sat_trunc(pW0), sat_trunc(pW1));
            sW_CD <= sat_add(sat_trunc(pW2), sat_trunc(pW3));

            // ----------------------------------------------------------------
            // STAGE 3: SATURATION ADDER TREE STAGE 2 (Final Transform)
            // ----------------------------------------------------------------
            out_X <= sat_add(sX_AB, sX_CD);
            out_Y <= sat_add(sY_AB, sY_CD);
            out_Z <= sat_add(sZ_AB, sZ_CD);
            out_W <= sat_add(sW_AB, sW_CD);

        end
    end

    // ========================================================================
    // OUTPUT ASSIGNMENT
    // ========================================================================
    assign vertex_out = {out_X, out_Y, out_Z, out_W, color_stg3};
    assign valid_out  = val_stg3;

endmodule

`timescale 1ns / 1ps

// ============================================================================
// Module: transform_engine
// Purpose: Applies a 4x4 Transformation Matrix to a 3D vertex using a 
//          fully pipelined 3-stage DSP datapath. Uses Q24.8 Fixed-Point math.
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

    // ========================================================================
    // 1. DATA UNPACKING (Combinational)
    // ========================================================================
    // Extract Vertex Coordinates (Cast as signed for two's complement math)
    wire signed [31:0] in_X = $signed(vertex_in[143:112]);
    wire signed [31:0] in_Y = $signed(vertex_in[111:80]);
    wire signed [31:0] in_Z = $signed(vertex_in[79:48]);
    wire signed [31:0] in_W = $signed(vertex_in[47:16]);
    wire        [15:0] in_C = vertex_in[15:0];

    // Extract Matrix Elements (Row 0: Affects X')
    wire signed [31:0] M00 = $signed(matrix_in[511:480]);
    wire signed [31:0] M01 = $signed(matrix_in[479:448]);
    wire signed [31:0] M02 = $signed(matrix_in[447:416]);
    wire signed [31:0] M03 = $signed(matrix_in[415:384]);
    // Extract Matrix Elements (Row 1: Affects Y')
    wire signed [31:0] M10 = $signed(matrix_in[383:352]);
    wire signed [31:0] M11 = $signed(matrix_in[351:320]);
    wire signed [31:0] M12 = $signed(matrix_in[319:288]);
    wire signed [31:0] M13 = $signed(matrix_in[287:256]);
    // Extract Matrix Elements (Row 2: Affects Z')
    wire signed [31:0] M20 = $signed(matrix_in[255:224]);
    wire signed [31:0] M21 = $signed(matrix_in[223:192]);
    wire signed [31:0] M22 = $signed(matrix_in[191:160]);
    wire signed [31:0] M23 = $signed(matrix_in[159:128]);
    // Extract Matrix Elements (Row 3: Affects W')
    wire signed [31:0] M30 = $signed(matrix_in[127:96]);
    wire signed [31:0] M31 = $signed(matrix_in[95:64]);
    wire signed [31:0] M32 = $signed(matrix_in[63:32]);
    wire signed [31:0] M33 = $signed(matrix_in[31:0]);

    // ========================================================================
    // PIPELINE REGISTERS
    // ========================================================================
    // Handshaking delays
    reg val_stg1, val_stg2, val_stg3;
    // Color delay shift register
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
    // We only accept new data if the next module (Projection) is ready to 
    // receive data. This acts as our Clock Enable stall.
    assign ready_out = ready_in;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            val_stg1 <= 0; val_stg2 <= 0; val_stg3 <= 0;
        end else if (ready_in) begin
            
            // Push Valid Flags through pipeline
            val_stg1 <= valid_in;
            val_stg2 <= val_stg1;
            val_stg3 <= val_stg2;

            // Push Color through 3-cycle delay
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
            // STAGE 2: TRUNCATION & ADDER TREE STAGE 1
            // (Extracts [39:8] to restore Q24.8, then adds pairs)
            // ----------------------------------------------------------------
            sX_AB <= pX0[39:8] + pX1[39:8];   sX_CD <= pX2[39:8] + pX3[39:8];
            sY_AB <= pY0[39:8] + pY1[39:8];   sY_CD <= pY2[39:8] + pY3[39:8];
            sZ_AB <= pZ0[39:8] + pZ1[39:8];   sZ_CD <= pZ2[39:8] + pZ3[39:8];
            sW_AB <= pW0[39:8] + pW1[39:8];   sW_CD <= pW2[39:8] + pW3[39:8];

            // ----------------------------------------------------------------
            // STAGE 3: ADDER TREE STAGE 2 (Final Transform)
            // ----------------------------------------------------------------
            out_X <= sX_AB + sX_CD;
            out_Y <= sY_AB + sY_CD;
            out_Z <= sZ_AB + sZ_CD;
            out_W <= sW_AB + sW_CD;

        end
    end

    // ========================================================================
    // OUTPUT ASSIGNMENT
    // ========================================================================
    // Pack the final rotated coordinates and synced color back into 144 bits
    assign vertex_out = {out_X, out_Y, out_Z, out_W, color_stg3};
    assign valid_out  = val_stg3;

endmodule

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
    input  wire [511:0] matrix_in,  // 4x4 Transformation Matrix (16x 32-bit)
    output wire [143:0] vertex_out  // Transformed Vertex {X', Y', Z', W', RGB565}
);

    localparam NST = 3;             // pipeline stages

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
    wire signed [31:0] M00 = $signed(matrix_in[511:480]);
    wire signed [31:0] M01 = $signed(matrix_in[479:448]);
    wire signed [31:0] M02 = $signed(matrix_in[447:416]);
    wire signed [31:0] M03 = $signed(matrix_in[415:384]);
    
    wire signed [31:0] M10 = $signed(matrix_in[383:352]);
    wire signed [31:0] M11 = $signed(matrix_in[351:320]);
    wire signed [31:0] M12 = $signed(matrix_in[319:288]);
    wire signed [31:0] M13 = $signed(matrix_in[287:256]);
    
    wire signed [31:0] M20 = $signed(matrix_in[255:224]);
    wire signed [31:0] M21 = $signed(matrix_in[223:192]);
    wire signed [31:0] M22 = $signed(matrix_in[191:160]);
    wire signed [31:0] M23 = $signed(matrix_in[159:128]);
    
    wire signed [31:0] M30 = $signed(matrix_in[127:96]);
    wire signed [31:0] M31 = $signed(matrix_in[95:64]);
    wire signed [31:0] M32 = $signed(matrix_in[63:32]);
    wire signed [31:0] M33 = $signed(matrix_in[31:0]);

    // ========================================================================
    // HANDSHAKE: global stall, same as before
    // ========================================================================
    assign ready_out = ready_in; // Back-pressure propagation

    // valid bits: the only registers with a reset
    reg [NST-1:0] val;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)        val <= {NST{1'b0}};
        else if (ready_in) val <= {val[NST-2:0], valid_in};
    end

    // ========================================================================
    // DATAPATH (no reset -> DSP48E1 register packing)
    // ========================================================================
    reg [15:0]        col  [0:NST-1];
    reg signed [63:0] p    [0:15];
    reg signed [31:0] s_ab [0:3];   // row r: sat_trunc(p[r,0]) + sat_trunc(p[r,1])
    reg signed [31:0] s_cd [0:3];   // row r: sat_trunc(p[r,2]) + sat_trunc(p[r,3])
    reg signed [31:0] o    [0:3];   // X', Y', Z', W'

    integer i, r;
    always @(posedge clk) begin
        if (ready_in) begin
            // colour travels with the vertex, one register per stage
            col[0] <= in_C;
            for (i = 1; i < NST; i = i + 1) col[i] <= col[i-1];

            // ----------------------------------------------------------------
            // STAGE 1: 16 PARALLEL MULTIPLICATIONS (Infers DSP48E1 slices)
            // ----------------------------------------------------------------
            p[0]  <= in_X * M00; p[1]  <= in_Y * M01; p[2]  <= in_Z * M02; p[3]  <= in_W * M03;
            p[4]  <= in_X * M10; p[5]  <= in_Y * M11; p[6]  <= in_Z * M12; p[7]  <= in_W * M13;
            p[8]  <= in_X * M20; p[9]  <= in_Y * M21; p[10] <= in_Z * M22; p[11] <= in_W * M23;
            p[12] <= in_X * M30; p[13] <= in_Y * M31; p[14] <= in_Z * M32; p[15] <= in_W * M33;

            // ----------------------------------------------------------------
            // STAGE 2: SATURATION TRUNCATION & ADDER TREE STAGE 1
            // ----------------------------------------------------------------
            for (r = 0; r < 4; r = r + 1) begin
                s_ab[r] <= sat_add(sat_trunc(p[r*4 + 0]), sat_trunc(p[r*4 + 1]));
                s_cd[r] <= sat_add(sat_trunc(p[r*4 + 2]), sat_trunc(p[r*4 + 3]));
            end

            // ----------------------------------------------------------------
            // STAGE 3: SATURATION ADDER TREE STAGE 2 (Final Transform)
            // ----------------------------------------------------------------
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

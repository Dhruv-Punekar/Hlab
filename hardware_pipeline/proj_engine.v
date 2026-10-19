`timescale 1ns / 1ps

// ============================================================================
// Module: projection_engine
// Purpose: 1. 1/W using LUT
//          2. Parallel DSP multiplication (X*1/W, Y*1/W, Z*1/W)
//          3. Viewport Scaling (Maps to 300x400 screen coordinates)
//          4. Accumulates 3 vertices into discrete ports for Triangle Setup
// ============================================================================
module projection_engine (
    input  wire         clk,
    input  wire         rst_n,      // Active-low reset

    // Handshake Signals
    input  wire         valid_in,   // High when Transform Engine provides valid vertex
    output wire         ready_out,  // High when this module can accept new data
    input  wire         ready_in,   // High when Triangle Setup unit can accept data
    output reg          valid_out,  // High when a full 3-vertex triangle is ready

    // Data Interfaces (Input from Transform Engine)
    input  wire [143:0] vertex_in,  // Single incoming vertex {X', Y', Z', W', Color}

    // Data Interfaces (Output to Triangle Setup Unit)
    output reg signed [31:0] out_x0, out_y0, out_z0,
    output reg signed [31:0] out_x1, out_y1, out_z1,
    output reg signed [31:0] out_x2, out_y2, out_z2,
    output reg        [15:0] out_face_color
);

    // ========================================================================
    // SATURATION TRUNCATION FUNCTION (Q24.8 Fixed-Point Alignment)
    // ========================================================================
    function signed [31:0] sat_trunc;
        input signed [63:0] prod;
        begin
            if (prod[63] == 1'b0 && (|prod[62:39] == 1'b1))
                sat_trunc = 32'h7FFF_FFFF;
            else if (prod[63] == 1'b1 && (~&prod[62:39] == 1'b1))
                sat_trunc = 32'h8000_0000;
            else
                sat_trunc = prod[39:8];
        end
    endfunction

    // ========================================================================
    // 1. DATA UNPACKING
    // ========================================================================
    wire signed [31:0] in_X = $signed(vertex_in[143:112]);
    wire signed [31:0] in_Y = $signed(vertex_in[111:80]);
    wire signed [31:0] in_Z = $signed(vertex_in[79:48]);
    wire signed [31:0] in_W = $signed(vertex_in[47:16]);
    wire        [15:0] in_C = vertex_in[15:0];

    // ========================================================================
    // 2. LUT FOR 1/W RECIPROCAL
    // ========================================================================
    wire [7:0] lut_addr = in_W[23:16];
    reg  [31:0] inv_W_lut [0:255];
    integer i;   // loop variable (module level: plain Verilog does not allow
                 // declarations inside an unnamed begin/end block)

    initial begin
        for (i = 0; i < 256; i = i + 1) begin
            if (i == 0) inv_W_lut[i] = 32'h00000100; // Q24.8 representation of 1.0
            else        inv_W_lut[i] = 32'h00010000 / i;
        end
    end

    reg signed [31:0] r_invW;

    // ========================================================================
    // PIPELINE REGISTERS
    // ========================================================================
    reg val_stg1, val_stg2, val_stg3;
    reg [15:0] color_stg1, color_stg2, color_stg3;

    reg signed [31:0] reg_X, reg_Y, reg_Z;
    reg signed [63:0] mult_X, mult_Y, mult_Z;
    reg signed [31:0] proj_X, proj_Y, proj_Z;

    // GLOBAL BACKPRESSURE LOGIC: Freeze pipeline if output is valid but not accepted
    wire pipe_advance = !valid_out || ready_in;
    assign ready_out = pipe_advance;

    // ========================================================================
    // PIPELINED DATAPATH (Stages 1 to 3)
    // ========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            val_stg1 <= 1'b0; val_stg2 <= 1'b0; val_stg3 <= 1'b0;
        end else if (pipe_advance) begin

            // Valid and Metadata Propagation
            val_stg1 <= valid_in;
            val_stg2 <= val_stg1;
            val_stg3 <= val_stg2;

            color_stg1 <= in_C;
            color_stg2 <= color_stg1;
            color_stg3 <= color_stg2;

            // STAGE 1: LATCH COORDINATES & FETCH 1/W
            reg_X <= in_X;
            reg_Y <= in_Y;
            reg_Z <= in_Z;
            r_invW <= $signed(inv_W_lut[lut_addr]);

            // STAGE 2: PARALLEL DSP MULTIPLICATIONS (Division by W)
            mult_X <= reg_X * r_invW;
            mult_Y <= reg_Y * r_invW;
            mult_Z <= reg_Z * r_invW;

            // STAGE 3: TRUNCATION & VIEWPORT SCALING
            // X_screen = (X_ndc + 1.0) * 150 -> Maps -1..1 to 0..300
            // Y_screen = (1.0 - Y_ndc) * 200 -> Maps -1..1 to 400..0
            // Note: 1.0 in Q24.8 is 32'sd256
            proj_X <= (sat_trunc(mult_X) + 32'sd256) * 32'sd150;
            proj_Y <= (32'sd256 - sat_trunc(mult_Y)) * 32'sd200;
            proj_Z <= sat_trunc(mult_Z); // Z remains Q24.8 as per spec
        end
    end

    // ========================================================================
    // STAGE 4: TRIANGLE ACCUMULATION PACKER (Discrete Ports)
    // ========================================================================
    reg [1:0] vertex_count;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            vertex_count <= 2'd0;
            valid_out    <= 1'b0;
        end else begin
            // Handshake logic: clear valid_out when Triangle Setup consumes it
            if (valid_out && ready_in) begin
                valid_out <= 1'b0;
            end

            // Accumulate vertices when stage 3 produces new projected data
            if (val_stg3 && pipe_advance) begin
                if (vertex_count == 2'd2) begin
                    out_x2 <= proj_X; out_y2 <= proj_Y; out_z2 <= proj_Z;
                    out_face_color <= color_stg3; // Apply one color to the whole face
                    valid_out <= 1'b1;            // Trigger output
                    vertex_count <= 2'd0;         // Reset counter for next triangle
                end else if (vertex_count == 2'd1) begin
                    out_x1 <= proj_X; out_y1 <= proj_Y; out_z1 <= proj_Z;
                    vertex_count <= 2'd2;
                end else begin
                    out_x0 <= proj_X; out_y0 <= proj_Y; out_z0 <= proj_Z;
                    vertex_count <= 2'd1;
                end
            end
        end
    end

endmodule

`timescale 1ns / 1ps

// ============================================================================
// Module: projection_engine  (FIXED)
// Purpose: 1. 1/W via normalised reciprocal (leading-one + 1024-entry ROM)
//          2. Parallel DSP multiplication (X/W, Y/W, Z/W)
//          3. Viewport scaling to 300x400 screen space (Q24.8), saturated
//          4. Accumulates 3 vertices into discrete ports for Triangle Setup
//
// Number formats (unchanged contract with the neighbours):
//   in : X', Y', Z', W'  signed Q24.8 clip space   (from transform_engine)
//   out: x in [0,76800] (= 0..300 px), y in [0,102400] (= 0..400 px), Q24.8
//        z = Z'/W' signed Q16.16 NDC depth         (to triangle_setup)
//        (depth precision fix 2026-10-22: z was Q24.8; x/y unchanged)
//
// Reciprocal method:
//   W = 2^p * m, m in [1,2)            p = leading-one position of W (raw)
//   R = 2^17 / m  (Q1.17 ROM, indexed by 10 mantissa bits after the 1)
//   X/W (Q24.8 raw) = X * 256 / W = (X * R) >>> (p + 9)
//   Z/W (Q16.16 raw)= Z * 65536 / W = (Z * R) >>> (p + 1)
//
// W <= 0 (vertex at/behind the eye, no clipper in this pipeline): x/y are
// forced to +max so triangle_setup flags overflow_xy_range and the
// triangle is discarded instead of being drawn mirrored/garbled.
//
// Latency: 5 stages + packer (was 3). II = 1. Backpressure unchanged.
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
    // HELPERS
    // ========================================================================
    // (X * R) >>> sh with round-half-up, saturated to signed 32-bit
    function signed [31:0] shift_round_sat;
        input signed [50:0] prod;
        input        [5:0]  sh;            // 1..39  (x/y: p+9, z: p+1)
        reg   signed [51:0] rnd;
        begin
            rnd = ($signed({prod[50], prod}) + (52'sd1 <<< (sh - 6'd1))) >>> sh;
            if (rnd > 52'sd2147483647)       shift_round_sat = 32'sh7FFF_FFFF;
            else if (rnd < -52'sd2147483648) shift_round_sat = 32'sh8000_0000;
            else                             shift_round_sat = rnd[31:0];
        end
    endfunction

    function signed [31:0] sat64;
        input signed [63:0] v;
        begin
            if (v > 64'sd2147483647)       sat64 = 32'sh7FFF_FFFF;
            else if (v < -64'sd2147483648) sat64 = 32'sh8000_0000;
            else                           sat64 = v[31:0];
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
    // 2. W NORMALISATION (combinational, feeds stage 1)
    // ========================================================================
    reg  [4:0] w_lead;
    integer    b;
    always @* begin
        w_lead = 5'd0;
        for (b = 0; b < 31; b = b + 1)
            if (in_W[b]) w_lead = b[4:0];
    end

    wire [30:0] w_norm  = in_W[30:0] << (5'd30 - w_lead);  // leading 1 -> bit 30
    wire [9:0]  w_idx   = w_norm[29:20];                   // 10 mantissa bits
    wire [5:0]  w_shift = {1'b0, w_lead} + 6'd9;           // p + 9  (x, y -> Q24.8)
    wire [5:0]  w_shz   = {1'b0, w_lead} + 6'd1;           // p + 1  (z    -> Q16.16)
    wire        w_bad   = (in_W <= 32'sd0);

    // ========================================================================
    // 3. RECIPROCAL ROM  R[k] = round(2^17 / (1 + (k + 0.5)/1024)), Q1.17
    // ========================================================================
    reg [17:0] recip_rom [0:1023];
    integer    k;
    initial begin
        for (k = 0; k < 1024; k = k + 1)
            recip_rom[k] = (268435456 + (2049 + 2*k) / 2) / (2049 + 2*k);
    end

    // ========================================================================
    // PIPELINE REGISTERS
    // ========================================================================
    reg        val_s1, val_s2, val_s3, val_s4, val_s5;
    reg [15:0] col_s1, col_s2, col_s3, col_s4, col_s5;
    reg        bad_s1, bad_s2, bad_s3, bad_s4;

    reg signed [31:0] s1_X, s1_Y, s1_Z;   reg [9:0] s1_idx;  reg [5:0] s1_sh, s1_shz;
    reg signed [31:0] s2_X, s2_Y, s2_Z;   reg [17:0] s2_R;   reg [5:0] s2_sh, s2_shz;
    reg signed [50:0] s3_mX, s3_mY, s3_mZ;                   reg [5:0] s3_sh, s3_shz;
    reg signed [31:0] s4_nX, s4_nY, s4_nZ;                   // NDC: x,y Q24.8, z Q16.16
    reg signed [31:0] proj_X, proj_Y, proj_Z;                // screen, Q24.8

    // GLOBAL BACKPRESSURE: freeze pipeline if output is valid but not accepted
    wire pipe_advance = !valid_out || ready_in;
    assign ready_out = pipe_advance;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            val_s1 <= 1'b0; val_s2 <= 1'b0; val_s3 <= 1'b0;
            val_s4 <= 1'b0; val_s5 <= 1'b0;
        end else if (pipe_advance) begin
            val_s1 <= valid_in; val_s2 <= val_s1; val_s3 <= val_s2;
            val_s4 <= val_s3;   val_s5 <= val_s4;
        end
    end

    always @(posedge clk) begin
        if (pipe_advance) begin
            col_s1 <= in_C;   col_s2 <= col_s1; col_s3 <= col_s2;
            col_s4 <= col_s3; col_s5 <= col_s4;
            bad_s1 <= w_bad;  bad_s2 <= bad_s1; bad_s3 <= bad_s2; bad_s4 <= bad_s3;

            // STAGE 1: latch coordinates + normalised W
            s1_X <= in_X; s1_Y <= in_Y; s1_Z <= in_Z;
            s1_idx <= w_idx; s1_sh <= w_shift; s1_shz <= w_shz;

            // STAGE 2: synchronous ROM read (BRAM, CE = pipe_advance)
            s2_X <= s1_X; s2_Y <= s1_Y; s2_Z <= s1_Z;
            s2_R <= recip_rom[s1_idx]; s2_sh <= s1_sh; s2_shz <= s1_shz;

            // STAGE 3: DSP multiplies  (32 x 19 signed -> 51 bits)
            s3_mX <= s2_X * $signed({1'b0, s2_R});
            s3_mY <= s2_Y * $signed({1'b0, s2_R});
            s3_mZ <= s2_Z * $signed({1'b0, s2_R});
            s3_sh <= s2_sh; s3_shz <= s2_shz;

            // STAGE 4: exponent shift, rounding, saturation -> NDC (x,y Q24.8; z Q16.16)
            s4_nX <= shift_round_sat(s3_mX, s3_sh);
            s4_nY <= shift_round_sat(s3_mY, s3_sh);
            s4_nZ <= shift_round_sat(s3_mZ, s3_shz);

            // STAGE 5: viewport  x = (ndc_x + 1) * 150,  y = (1 - ndc_y) * 200
            //          64-bit intermediate + saturation: no wraparound
            if (bad_s4) begin
                proj_X <= 32'sh7FFF_FFFF;     // forces setup overflow -> discard
                proj_Y <= 32'sh7FFF_FFFF;
            end else begin
                proj_X <= sat64(($signed({{32{s4_nX[31]}}, s4_nX}) + 64'sd256) * 64'sd150);
                proj_Y <= sat64((64'sd256 - $signed({{32{s4_nY[31]}}, s4_nY})) * 64'sd200);
            end
            proj_Z <= s4_nZ;
        end
    end

    // ========================================================================
    // STAGE 6: TRIANGLE ACCUMULATION PACKER (Discrete Ports)  - unchanged
    // ========================================================================
    reg [1:0] vertex_count;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            vertex_count <= 2'd0;
            valid_out    <= 1'b0;
        end else begin
            if (valid_out && ready_in)
                valid_out <= 1'b0;

            if (val_s5 && pipe_advance) begin
                if (vertex_count == 2'd2) begin
                    out_x2 <= proj_X; out_y2 <= proj_Y; out_z2 <= proj_Z;
                    out_face_color <= col_s5;
                    valid_out      <= 1'b1;
                    vertex_count   <= 2'd0;
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

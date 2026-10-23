`timescale 1ns / 1ps

// ============================================================================
// Module: projection_engine  (FIXED)
// Purpose: 1. 1/W via normalised reciprocal (leading-one + 1024-entry ROM)
//          2. Parallel DSP multiplication (X/W, Y/W, Z/W)
//          3. Viewport scaling to 300x400 screen space (x,y Q24.8), saturated
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
// Latency: 8 stages + packer (was 5). II = 1. Backpressure unchanged.
//
// Timing fix (2026-10-23): at 100 MHz the old stage 4 (52-bit round-add +
// variable shift + saturation) and stage 5 (64-bit multiply by 150/200 +
// saturation) missed timing by up to 1.27 ns. Stage 4 is now 2 cycles
// (add | shift+saturate) and stage 5 is 3 cycles (offset + early range test |
// half products | final add + saturate). Every output value is bit-identical
// to the previous version; triangles simply arrive 3 cycles later.
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
    // HELPERS (stage 4 and stage 5 are split over several clock cycles; the
    // results are bit-identical to the former single-cycle versions:
    //   shift_round_sat(prod, sh)  = sat32((prod + 2^(sh-1)) >>> sh)
    //   sat64((n + 256) * 150)     and   sat64((256 - n) * 200)
    // ========================================================================

    // a 52-bit signed value fits signed 32 bits  <=>  bits [51:31] all equal
    function signed [31:0] sat32_from52;
        input signed [51:0] v;
        begin
            if (&v[51:31] || ~|v[51:31]) sat32_from52 = v[31:0];
            else if (v[51])              sat32_from52 = 32'sh8000_0000;
            else                         sat32_from52 = 32'sh7FFF_FFFF;
        end
    endfunction

    // Viewport saturation limits, decided BEFORE the multiply:
    //   t * 150 fits signed 32  <=>  -14316557 <= t <= 14316557
    //   u * 200 fits signed 32  <=>  -10737418 <= u <= 10737418
    // (2^31-1)/150 = 14316557.65, 2^31/150 = 14316557.65 ; same for 200: 10737418.24
    localparam signed [32:0] X_LIM =  33'sd14316557;
    localparam signed [32:0] Y_LIM =  33'sd10737418;

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
    //   S1 latch | S2 ROM | S3 multiply | S4A round-add | S4B shift + sat
    //   S5A +/-256 and range test | S5B half products | S5C sum + saturate
    // ========================================================================
    localparam NST = 8;                       // stages before the packer
    reg [NST:1] val;                          // val[i] = valid bit of stage i
    reg [15:0]  col [1:NST];                  // face colour, carried along
    reg [NST:1] bad;                          // W <= 0 flag, carried along

    reg signed [31:0] s1_X, s1_Y, s1_Z;   reg [9:0] s1_idx;  reg [5:0] s1_sh, s1_shz;
    reg signed [31:0] s2_X, s2_Y, s2_Z;   reg [17:0] s2_R;   reg [5:0] s2_sh, s2_shz;
    reg signed [50:0] s3_mX, s3_mY, s3_mZ;                   reg [5:0] s3_sh, s3_shz;
    reg signed [51:0] s4a_X, s4a_Y, s4a_Z;                   reg [5:0] s4a_sh, s4a_shz;
    reg signed [31:0] s4_nX, s4_nY, s4_nZ;                   // NDC: x,y Q24.8, z Q16.16
    reg signed [32:0] s5a_tX, s5a_uY;                        // nX+256, 256-nY
    reg               s5a_xhi, s5a_xlo, s5a_yhi, s5a_ylo;    // product will saturate
    reg signed [31:0] s5a_Z;
    reg        [31:0] s5b_xa, s5b_xb, s5b_ya, s5b_yb;        // partial products (mod 2^32)
    reg               s5b_xhi, s5b_xlo, s5b_yhi, s5b_ylo;
    reg signed [31:0] s5b_Z;
    reg signed [31:0] proj_X, proj_Y, proj_Z;                // screen x,y Q24.8; z = NDC depth Q16.16

    // GLOBAL BACKPRESSURE: freeze pipeline if output is valid but not accepted
    wire pipe_advance = !valid_out || ready_in;
    assign ready_out = pipe_advance;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)            val <= {NST{1'b0}};
        else if (pipe_advance) val <= {val[NST-1:1], valid_in};
    end

    integer j;
    always @(posedge clk) begin
        if (pipe_advance) begin
            col[1] <= in_C;   bad[1] <= w_bad;
            for (j = 2; j <= NST; j = j + 1) begin
                col[j] <= col[j-1];
                bad[j] <= bad[j-1];
            end

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

            // STAGE 4A: add the rounding constant 2^(sh-1)   (one 52-bit add)
            s4a_X <= $signed({s3_mX[50], s3_mX}) + (52'sd1 <<< (s3_sh  - 6'd1));
            s4a_Y <= $signed({s3_mY[50], s3_mY}) + (52'sd1 <<< (s3_sh  - 6'd1));
            s4a_Z <= $signed({s3_mZ[50], s3_mZ}) + (52'sd1 <<< (s3_shz - 6'd1));
            s4a_sh <= s3_sh; s4a_shz <= s3_shz;

            // STAGE 4B: arithmetic shift + saturate to 32 bits -> NDC
            s4_nX <= sat32_from52(s4a_X >>> s4a_sh);
            s4_nY <= sat32_from52(s4a_Y >>> s4a_sh);
            s4_nZ <= sat32_from52(s4a_Z >>> s4a_shz);

            // STAGE 5A: viewport offsets, and decide saturation up front
            //   x = (ndc_x + 1) * 150     y = (1 - ndc_y) * 200
            s5a_tX  <= $signed({s4_nX[31], s4_nX}) + 33'sd256;
            s5a_uY  <= 33'sd256 - $signed({s4_nY[31], s4_nY});
            s5a_xhi <= ($signed({s4_nX[31], s4_nX}) + 33'sd256) >  X_LIM;
            s5a_xlo <= ($signed({s4_nX[31], s4_nX}) + 33'sd256) < -X_LIM;
            s5a_yhi <= (33'sd256 - $signed({s4_nY[31], s4_nY})) >  Y_LIM;
            s5a_ylo <= (33'sd256 - $signed({s4_nY[31], s4_nY})) < -Y_LIM;
            s5a_Z   <= s4_nZ;

            // STAGE 5B: half of each constant multiply (mod 2^32 is exact
            //           whenever the result is not saturated)
            //   150 = 128 + 16 + 4 + 2        200 = 128 + 64 + 8
            s5b_xa <= (s5a_tX[31:0] << 7) + (s5a_tX[31:0] << 4);
            s5b_xb <= (s5a_tX[31:0] << 2) + (s5a_tX[31:0] << 1);
            s5b_ya <= (s5a_uY[31:0] << 7) + (s5a_uY[31:0] << 6);
            s5b_yb <= (s5a_uY[31:0] << 3);
            s5b_xhi <= s5a_xhi; s5b_xlo <= s5a_xlo;
            s5b_yhi <= s5a_yhi; s5b_ylo <= s5a_ylo;
         
// -----------------------------------------------------------------------------
// Triangle Setup Unit — debug/diagnostic variant  (rev B: Artix-7 timing fix)
//
// Converts one post-clipping, screen-space triangle into a rasterizer payload:
//   * inclusive integer pixel bounding box (10-bit unsigned indices)
//   * three signed edge equations  E(x,y) = A*x + B*y + C
//   * Q24.8 depth plane z(x,y) = zA*x + zB*y + zC
//   * shared RGB565 color and status flags
//
// Outputs are BIT-IDENTICAL to rev A (same LUT, same rounding/saturation,
// same flags; verified on 4000 random + corner-case triangles under random
// backpressure). What changed is the micro-architecture:
//
//   rev A: 4 register stages; the last stage chained a 53x18 multiply, an
//          80-bit negate, an 80-bit variable shifter, round/saturate, a 32x18
//          multiply, a 3-input 51-bit add and a second round/saturate in ONE
//          clock. On an Artix-7 (-1/-2) that path is several times longer than
//          a 10 ns (100 MHz) period.
//   rev B: 11 register stages (S0..S10), every stage holds at most one
//          multiply OR one shifter OR one carry chain. II is still 1.
//
//   S0  capture + range clamp            S6  Nx*inv, Ny*inv (sign folded in inv)
//   S1  differences, bbox min/max        S7  |p| >> (e+9), guard/sticky
//   S2  all products (DSP48 MREG)        S8  overflow test, round, saturate zA/zB
//   S3  D, Nx, Ny, edge C (DSP48 PREG)   S9  zA*x0, zB*y0
//   S4  |D| leading-one normalize        S10 zC round/saturate -> output regs
//   S5  reciprocal BRAM read (registered)
//
// Unstalled latency: input accepted on edge k -> out_valid after edge k+10.
// Backpressure: global stall (all stages + BRAM CE freeze together).
//
// Reset: rst_n is asynchronous active-low as before, but ONLY valid bits and
// status flags use it. Datapath registers have no reset so Vivado can pack
// them into DSP48E1 A/B/M/P registers and the RAMB18 output register (Xilinx
// primitives only support synchronous reset). Drive rst_n from a reset
// synchronizer (async assert, sync de-assert) at the top level.
// -----------------------------------------------------------------------------
module triangle_setup_unit_debug #(
    parameter RECIP_LUT_FILE = "recip_lut_q1_17.mem"
) (
    input wire                     clk,
    input wire                     rst_n,          // asynchronous active-low

    input wire                     in_valid,
    output wire                    in_ready,
    input wire signed [31:0]       in_x0, in_y0, in_z0,
    input wire signed [31:0]       in_x1, in_y1, in_z1,
    input wire signed [31:0]       in_x2, in_y2, in_z2,
    input wire        [15:0]       in_face_color,  // RGB565, one per face

    output wire                    out_valid,
    input wire                     out_ready,

    output reg        [9:0]        bbox_xmin,
    output reg        [9:0]        bbox_xmax,
    output reg        [9:0]        bbox_ymin,
    output reg        [9:0]        bbox_ymax,

    output reg signed [31:0]       edge0_A,
    output reg signed [31:0]       edge0_B,
    output reg signed [33:0]       edge0_C,
    output reg signed [31:0]       edge1_A,
    output reg signed [31:0]       edge1_B,
    output reg signed [33:0]       edge1_C,
    output reg signed [31:0]       edge2_A,
    output reg signed [31:0]       edge2_B,
    output reg signed [33:0]       edge2_C,

    output reg signed [31:0]       zA,
    output reg signed [31:0]       zB,
    output reg signed [31:0]       zC,
    output reg        [15:0]       face_color,
    output reg                     overflow,          // OR of range/edge/depth flags
    output reg                     overflow_xy_range, // any x/y input outside contract
    output reg                     overflow_edge_c,   // any edge C not representable in 34 bits
    output reg                     overflow_zA,       // zA saturated to signed Q24.8
    output reg                     overflow_zB,       // zB saturated to signed Q24.8
    output reg                     overflow_zC,       // zC saturated to signed Q24.8
    output reg                     overflow_depth,    // OR of zA/zB/zC flags
    output reg                     bbox_clamped,      // intentional viewport-bound clamp
    output reg                     orient_neg,
    output reg                     degenerate
);

    localparam integer LATENCY   = 10;            // edges from accept to out_valid
    localparam signed [31:0] X_MAX_Q8 = 32'sd76800;   // 300.0 in Q24.8
    localparam signed [31:0] Y_MAX_Q8 = 32'sd102400;  // 400.0 in Q24.8
    localparam integer X_PIX_MAX = 299;
    localparam integer Y_PIX_MAX = 399;

    // -------------------------------------------------------------------------
    // Global advance / backpressure
    // -------------------------------------------------------------------------
    // NOTE: in_ready is combinational from out_ready (same as rev A). If the
    // upstream/downstream ready path fails timing at system level, put a 2-entry
    // skid buffer on the output and drive pipe_advance from its "not full".
    wire pipe_advance = rst_n && (!out_valid || out_ready);
    assign in_ready   = pipe_advance;

    // Valid chain: v[i] is the valid bit of stage Si.
    reg [LATENCY:0] v;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)            v <= {(LATENCY+1){1'b0}};
        else if (pipe_advance) v <= {v[LATENCY-1:0], in_valid};
    end
    assign out_valid = v[LATENCY];

    // =========================================================================
    // S0: capture and coordinate range clamp (diagnostic only)
    // =========================================================================
    wire input_xy_range_bad =
        (in_x0 < 0) || (in_x0 > X_MAX_Q8) ||
        (in_x1 < 0) || (in_x1 > X_MAX_Q8) ||
        (in_x2 < 0) || (in_x2 > X_MAX_Q8) ||
        (in_y0 < 0) || (in_y0 > Y_MAX_Q8) ||
        (in_y1 < 0) || (in_y1 > Y_MAX_Q8) ||
        (in_y2 < 0) || (in_y2 > Y_MAX_Q8);

    function signed [17:0] clamp_coord;
        input signed [31:0] raw;
        input signed [31:0] max_q8;
        begin
            if (raw < 0)           clamp_coord = 18'sd0;
            else if (raw > max_q8) clamp_coord = max_q8[17:0];
            else                   clamp_coord = raw[17:0];
        end
    endfunction

    reg signed [17:0] s0_x0, s0_y0, s0_x1, s0_y1, s0_x2, s0_y2;
    reg signed [31:0] s0_z0, s0_z1, s0_z2;
    reg        [15:0] s0_color;
    reg               s0_xy_bad;

    always @(posedge clk) begin
        if (pipe_advance && in_valid) begin
            s0_x0 <= clamp_coord(in_x0, X_MAX_Q8);
            s0_y0 <= clamp_coord(in_y0, Y_MAX_Q8);
            s0_x1 <= clamp_coord(in_x1, X_MAX_Q8);
            s0_y1 <= clamp_coord(in_y1, Y_MAX_Q8);
            s0_x2 <= clamp_coord(in_x2, X_MAX_Q8);
            s0_y2 <= clamp_coord(in_y2, Y_MAX_Q8);
            s0_z0 <= in_z0;  s0_z1 <= in_z1;  s0_z2 <= in_z2;
            s0_color  <= in_face_color;
            s0_xy_bad <= input_xy_range_bad;
        end
    end

    // =========================================================================
    // S1: differences and bbox min/max
    // =========================================================================
    // Clamped coordinates are in [0, 102400], so every difference is within
    // +/-102400 and fits signed 18 bits -> each x*y product fits one DSP48E1
    // (25x18) instead of two.
    reg signed [17:0] s1_dx1, s1_dy1, s1_dx2, s1_dy2;
    reg signed [32:0] s1_dz1, s1_dz2;
    reg signed [17:0] s1_e0a, s1_e0b, s1_e1a, s1_e1b, s1_e2a, s1_e2b;
    reg signed [17:0] s1_x0, s1_y0, s1_x1, s1_y1, s1_x2, s1_y2;
    reg signed [17:0] s1_xmin, s1_xmax, s1_ymin, s1_ymax;
    reg signed [31:0] s1_z0;
    reg        [15:0] s1_color;
    reg               s1_xy_bad;

    function signed [17:0] min3;
        input signed [17:0] a, b, c;
        reg signed [17:0] t;
        begin
            t = (b < a) ? b : a;
            min3 = (c < t) ? c : t;
        end
    endfunction

    function signed [17:0] max3;
        input signed [17:0] a, b, c;
        reg signed [17:0] t;
        begin
            t = (b > a) ? b : a;
            max3 = (c > t) ? c : t;
        end
    endfunction

    always @(posedge clk) begin
        if (pipe_advance) begin
            s1_dx1 <= s0_x1 - s0_x0;
            s1_dy1 <= s0_y1 - s0_y0;
            s1_dx2 <= s0_x2 - s0_x0;
            s1_dy2 <= s0_y2 - s0_y0;
            s1_dz1 <= s0_z1 - s0_z0;      // 33-bit signed context: exact
            s1_dz2 <= s0_z2 - s0_z0;
            // Directed edges (0->1), (1->2), (2->0): A = yi - yj, B = xj - xi
            s1_e0a <= s0_y0 - s0_y1;  s1_e0b <= s0_x1 - s0_x0;
            s1_e1a <= s0_y1 - s0_y2;  s1_e1b <= s0_x2 - s0_x1;
            s1_e2a <= s0_y2 - s0_y0;  s1_e2b <= s0_x0 - s0_x2;
            s1_x0 <= s0_x0; s1_y0 <= s0_y0;
            s1_x1 <= s0_x1; s1_y1 <= s0_y1;
            s1_x2 <= s0_x2; s1_y2 <= s0_y2;
            s1_xmin <= min3(s0_x0, s0_x1, s0_x2);
            s1_xmax <= max3(s0_x0, s0_x1, s0_x2);
            s1_ymin <= min3(s0_y0, s0_y1, s0_y2);
            s1_ymax <= max3(s0_y0, s0_y1, s0_y2);
            s1_z0     <= s0_z0;
            s1_color  <= s0_color;
            s1_xy_bad <= s0_xy_bad;
        end
    end

    // =========================================================================
    // S2: products (maps to DSP48E1 with MREG) and bbox integer conversion
    // =========================================================================
    // floor/ceil of a Q.8 value. Fix vs rev A: the round-up bit is now a signed
    // {0,bit} operand. In rev A, "$signed(raw>>>8) + (|raw[7:0])" mixed a
    // signed and an unsigned operand, which makes the whole sum UNSIGNED and
    // zero-extends a negative quotient (latent bug, masked only because the
    // S0 clamp keeps raw >= 0).
    function signed [18:0] q8_floor;
        input signed [17:0] raw;
        begin
            q8_floor = raw >>> 8;
        end
    endfunction

    function signed [18:0] q8_ceil;
        input signed [17:0] raw;
        begin
            q8_ceil = (raw >>> 8) + $signed({1'b0, |raw[7:0]});
        end
    endfunction

    function [9:0] clamp_pix;
        input signed [18:0] v_in;
        input integer       vmax;
        begin
            if (v_in < 0)         clamp_pix = 10'd0;
            else if (v_in > vmax) clamp_pix = vmax;
            else                  clamp_pix = v_in[9:0];
        end
    endfunction

    function out_of_pix;
        input signed [18:0] v_in;
        input integer       vmax;
        begin
            out_of_pix = (v_in < 0) || (v_in > vmax);
        end
    endfunction

    wire signed [18:0] s1_xlo = q8_floor(s1_xmin);
    wire signed [18:0] s1_xhi = q8_ceil (s1_xmax);
    wire signed [18:0] s1_ylo = q8_floor(s1_ymin);
    wire signed [18:0] s1_yhi = q8_ceil (s1_ymax);

    reg signed [35:0] s2_p_d0, s2_p_d1;                 // dx1*dy2, dx2*dy1
    reg signed [50:0] s2_p_nx0, s2_p_nx1;               // dz1*dy2, dz2*dy1
    reg signed [50:0] s2_p_ny0, s2_p_ny1;               // dx1*dz2, dx2*dz1
    reg signed [35:0] s2_e0p0, s2_e0p1, s2_e1p0, s2_e1p1, s2_e2p0, s2_e2p1;
    reg signed [17:0] s2_e0a, s2_e0b, s2_e1a, s2_e1b, s2_e2a, s2_e2b;
    reg        [9:0]  s2_bxmin, s2_bxmax, s2_bymin, s2_bymax;
    reg               s2_bbox_clamped;
    reg signed [17:0] s2_x0, s2_y0;
    reg signed [31:0] s2_z0;
    reg        [15:0] s2_color;
    reg               s2_xy_bad;

    always @(posedge clk) begin
        if (pipe_advance) begin
            s2_p_d0  <= s1_dx1 * s1_dy2;
            s2_p_d1  <= s1_dx2 * s1_dy1;
            s2_p_nx0 <= s1_dz1 * s1_dy2;
            s2_p_nx1 <= s1_dz2 * s1_dy1;
            s2_p_ny0 <= s1_dx1 * s1_dz2;
            s2_p_ny1 <= s1_dx2 * s1_dz1;
            s2_e0p0 <= s1_x0 * s1_y1;  s2_e0p1 <= s1_x1 * s1_y0;
            s2_e1p0 <= s1_x1 * s1_y2;  s2_e1p1 <= s1_x2 * s1_y1;
            s2_e2p0 <= s1_x2 * s1_y0;  s2_e2p1 <= s1_x0 * s1_y2;
            s2_e0a <= s1_e0a; s2_e0b <= s1_e0b;
            s2_e1a <= s1_e1a; s2_e1b <= s1_e1b;
            s2_e2a <= s1_e2a; s2_e2b <= s1_e2b;
            s2_bxmin <= clamp_pix(s1_xlo, X_PIX_MAX);
            s2_bxmax <= clamp_pix(s1_xhi, X_PIX_MAX);
            s2_bymin <= clamp_pix(s1_ylo, Y_PIX_MAX);
            s2_bymax <= clamp_pix(s1_yhi, Y_PIX_MAX);
            s2_bbox_clamped <= out_of_pix(s1_xlo, X_PIX_MAX) | out_of_pix(s1_xhi, X_PIX_MAX) |
                               out_of_pix(s1_ylo, Y_PIX_MAX) | out_of_pix(s1_yhi, Y_PIX_MAX);
            s2_x0 <= s1_x0; s2_y0 <= s1_y0; s2_z0 <= s1_z0;
            s2_color  <= s1_color;
            s2_xy_bad <= s1_xy_bad;
        end
    end

    // =========================================================================
    // S3: determinant, plane numerators, edge C (DSP48E1 post-adder / PREG)
    // =========================================================================
    wire signed [36:0] s2_e0c = s2_e0p0 - s2_e0p1;
    wire signed [36:0] s2_e1c = s2_e1p0 - s2_e1p1;
    wire signed [36:0] s2_e2c = s2_e2p0 - s2_e2p1;

    reg signed [38:0] s3_d;
    reg signed [52:0] s3_nx, s3_ny;
    reg signed [33:0] s3_e0c, s3_e1c, s3_e2c;
    reg               s3_edge_c_ov;
    reg signed [17:0] s3_e0a, s3_e0b, s3_e1a, s3_e1b, s3_e2a, s3_e2b;
    reg        [9:0]  s3_bxmin, s3_bxmax, s3_bymin, s3_bymax;
    reg               s3_bbox_clamped;
    reg signed [17:0] s3_x0, s3_y0;
    reg signed [31:0] s3_z0;
    reg        [15:0] s3_color;
    reg               s3_xy_bad;

    always @(posedge clk) begin
        if (pipe_advance) begin
            s3_d  <= s2_p_d0  - s2_p_d1;
            s3_nx <= s2_p_nx0 - s2_p_nx1;
            s3_ny <= s2_p_ny0 - s2_p_ny1;
            s3_e0c <= s2_e0c[33:0];
            s3_e1c <= s2_e1c[33:0];
            s3_e2c <= s2_e2c[33:0];
            s3_edge_c_ov <= (s2_e0c[36:33] != {4{s2_e0c[33]}}) ||
                            (s2_e1c[36:33] != {4{s2_e1c[33]}}) ||
                            (s2_e2c[36:33] != {4{s2_e2c[33]}});
            s3_e0a <= s2_e0a; s3_e0b <= s2_e0b;
            s3_e1a <= s2_e1a; s3_e1b <= s2_e1b;
            s3_e2a <= s2_e2a; s3_e2b <= s2_e2b;
            s3_bxmin <= s2_bxmin; s3_bxmax <= s2_bxmax;
            s3_bymin <= s2_bymin; s3_bymax <= s2_bymax;
            s3_bbox_clamped <= s2_bbox_clamped;
            s3_x0 <= s2_x0; s3_y0 <= s2_y0; s3_z0 <= s2_z0;
            s3_color  <= s2_color;
            s3_xy_bad <= s2_xy_bad;
        end
    end

    // =========================================================================
    // S4: |D| leading-one detect and normalize -> ROM address
    // =========================================================================
    reg [38:0] s3_dmag;
    reg [5:0]  s3_dexp;
    reg [38:0] s3_dnorm;
    integer    nb;
    always @* begin
        s3_dmag = s3_d[38] ? (~s3_d + 39'd1) : s3_d;
        s3_dexp = 6'd0;
        for (nb = 0; nb < 39; nb = nb + 1)
            if (s3_dmag[nb]) s3_dexp = nb[5:0];
        s3_dnorm = s3_dmag << (6'd38 - s3_dexp);
    end

    reg [9:0]         s4_addr;
    reg [6:0]         s4_shift;            // e + 9
    reg               s4_dneg, s4_degen;
    reg signed [52:0] s4_nx, s4_ny;
    reg signed [33:0] s4_e0c, s4_e1c, s4_e2c;
    reg               s4_edge_c_ov;
    reg signed [17:0] s4_e0a, s4_e0b, s4_e1a, s4_e1b, s4_e2a, s4_e2b;
    reg        [9:0]  s4_bxmin, s4_bxmax, s4_bymin, s4_bymax;
    reg               s4_bbox_clamped;
    reg signed [17:0] s4_x0, s4_y0;
    reg signed [31:0] s4_z0;
    reg        [15:0] s4_color;
    reg               s4_xy_bad;

    always @(posedge clk) begin
        if (pipe_advance) begin
            s4_addr  <= s3_dnorm[37:28];
            s4_shift <= {1'b0, s3_dexp} + 7'd9;
            s4_dneg  <= s3_d[38];
            s4_degen <= (s3_d == 39'sd0);
            s4_nx <= s3_nx;  s4_ny <= s3_ny;
            s4_e0c <= s3_e0c; s4_e1c <= s3_e1c; s4_e2c <= s3_e2c;
            s4_edge_c_ov <= s3_edge_c_ov;
            s4_e0a <= s3_e0a; s4_e0b <= s3_e0b;
            s4_e1a <= s3_e1a; s4_e1b <= s3_e1b;
            s4_e2a <= s3_e2a; s4_e2b <= s3_e2b;
            s4_bxmin <= s3_bxmin; s4_bxmax <= s3_bxmax;
            s4_bymin <= s3_bymin; s4_bymax <= s3_bymax;
            s4_bbox_clamped <= s3_bbox_clamped;
            s4_x0 <= s3_x0; s4_y0 <= s3_y0; s4_z0 <= s3_z0;
            s4_color  <= s3_color;
            s4_xy_bad <= s3_xy_bad;
        end
    end

    // =========================================================================
    // S5: reciprocal ROM, 1024 x 18 -> one RAMB18E1, registered output.
    // Read is unconditional (degenerate triangles are zeroed later), so the
    // output register has no data-dependent reset and maps cleanly to BRAM.
    // =========================================================================
    (* rom_style = "block" *) reg [17:0] recip_rom [0:1023];
    initial $readmemh(RECIP_LUT_FILE, recip_rom);

    reg [17:0] s5_inv;
    always @(posedge clk)
        if (pipe_advance) s5_inv <= recip_rom[s4_addr];

    reg [6:0]         s5_shift;
    reg               s5_dneg, s5_degen;
    reg signed [52:0] s5_nx, s5_ny;
    reg signed [33:0] s5_e0c, s5_e1c, s5_e2c;
    reg               s5_edge_c_ov;
    reg signed [17:0] s5_e0a, s5_e0b, s5_e1a, s5_e1b, s5_e2a, s5_e2b;
    reg        [9:0]  s5_bxmin, s5_bxmax, s5_bymin, s5_bymax;
    reg               s5_bbox_clamped;
    reg signed [17:0] s5_x0, s5_y0;
    reg signed [31:0] s5_z0;
    reg        [15:0] s5_color;
    reg               s5_xy_bad;

    always @(posedge clk) begin
        if (pipe_advance) begin
            s5_shift <= s4_shift; s5_dneg <= s4_dneg; s5_degen <= s4_degen;
            s5_nx <= s4_nx; s5_ny <= s4_ny;
            s5_e0c <= s4_e0c; s5_e1c <= s4_e1c; s5_e2c <= s4_e2c;
            s5_edge_c_ov <= s4_edge_c_ov;
            s5_e0a <= s4_e0a; s5_e0b <= s4_e0b;
            s5_e1a <= s4_e1a; s5_e1b <= s4_e1b;
            s5_e2a <= s4_e2a; s5_e2b <= s4_e2b;
            s5_bxmin <= s4_bxmin; s5_bxmax <= s4_bxmax;
            s5_bymin <= s4_bymin; s5_bymax <= s4_bymax;
            s5_bbox_clamped <= s4_bbox_clamped;
            s5_x0 <= s4_x0; s5_y0 <= s4_y0; s5_z0 <= s4_z0;
            s5_color  <= s4_color;
            s5_xy_bad <= s4_xy_bad;
        end
    end

    // =========================================================================
    // S6: N * (+/-inv_m). Folding sign(D) into the 19-bit multiplier operand
    // replaces rev A's 80-bit post-multiply negation; result is identical
    // because Nx*(-inv) == -(Nx*inv) exactly.
    // =========================================================================
    wire signed [18:0] s5_inv_s = s5_dneg ? -$signed({1'b0, s5_inv})
                                          :  $signed({1'b0, s5_inv});

    reg signed [71:0] s6_pa, s6_pb;
    reg [6:0]         s6_shift;
    reg               s6_dneg, s6_degen;
    reg signed [33:0] s6_e0c, s6_e1c, s6_e2c;
    reg               s6_edge_c_ov;
    reg signed [17:0] s6_e0a, s6_e0b, s6_e1a, s6_e1b, s6_e2a, s6_e2b;
    reg        [9:0]  s6_bxmin, s6_bxmax, s6_bymin, s6_bymax;
    reg               s6_bbox_clamped;
    reg signed [17:0] s6_x0, s6_y0;
    reg signed [31:0] s6_z0;
    reg        [15:0] s6_color;
    reg               s6_xy_bad;

    always @(posedge clk) begin
        if (pipe_advance) begin
            s6_pa <= s5_nx * s5_inv_s;
            s6_pb <= s5_ny * s5_inv_s;
            s6_shift <= s5_shift; s6_dneg <= s5_dneg; s6_degen <= s5_degen;
            s6_e0c <= s5_e0c; s6_e1c <= s5_e1c; s6_e2c <= s5_e2c;
            s6_edge_c_ov <= s5_edge_c_ov;
            s6_e0a <= s5_e0a; s6_e0b <= s5_e0b;
            s6_e1a <= s5_e1a; s6_e1b <= s5_e1b;
            s6_e2a <= s5_e2a; s6_e2b <= s5_e2b;
            s6_bxmin <= s5_bxmin; s6_bxmax <= s5_bxmax;
            s6_bymin <= s5_bymin; s6_bymax <= s5_bymax;
            s6_bbox_clamped <= s5_bbox_clamped;
            s6_x0 <= s5_x0; s6_y0 <= s5_y0; s6_z0 <= s5_z0;
            s6_color  <= s5_color;
            s6_xy_bad <= s5_xy_bad;
        end
    end

    // =========================================================================
    // S7: magnitude and variable right shift; keep guard/sticky for rounding.
    // =========================================================================
    // Split form of rev A depth_round_saturate():
    //   q = |p| >> s, rem = |p| mod 2^s
    //   overflow  <=>  |p| > L*2^s  <=>  q > L || (q == L && rem != 0)
    //   with L = 2^31 (negative) or 2^31-1 (positive)
    //   rounding: half-to-even on q using guard = rem[s-1], sticky = |rem[s-2:0]
    function [35:0] shr_split;          // {q_hi_nz, guard, sticky, q[31:0], 1'b0}
        input [71:0] mag;
        input [6:0]  sh;                // 9..47
        reg [71:0] q, mask;
        begin
            q    = mag >> sh;
            mask = (72'd1 << (sh - 7'd1)) - 72'd1;
            shr_split = { |q[71:32],
                          mag[sh - 7'd1],
                          |(mag & mask),
                          q[31:0],
                          1'b0 };
        end
    endfunction

    wire [71:0] s6_maga = s6_pa[71] ? -s6_pa : s6_pa;
    wire [71:0] s6_magb = s6_pb[71] ? -s6_pb : s6_pb;
    wire [35:0] s6_sa   = shr_split(s6_maga, s6_shift);
    wire [35:0] s6_sb   = shr_split(s6_magb, s6_shift);

    reg [35:0]        s7_sa, s7_sb;
    reg               s7_nega, s7_negb;
    reg               s7_dneg, s7_degen;
    reg signed [33:0] s7_e0c, s7_e1c, s7_e2c;
    reg               s7_edge_c_ov;
    reg signed [17:0] s7_e0a, s7_e0b, s7_e1a, s7_e1b, s7_e2a, s7_e2b;
    reg        [9:0]  s7_bxmin, s7_bxmax, s7_bymin, s7_bymax;
    reg               s7_bbox_clamped;
    reg signed [17:0] s7_x0, s7_y0;
    reg signed [31:0] s7_z0;
    reg        [15:0] s7_color;
    reg               s7_xy_bad;

    always @(posedge clk) begin
        if (pipe_advance) begin
            s7_sa <= s6_sa;  s7_nega <= s6_pa[71];
            s7_sb <= s6_sb;  s7_negb <= s6_pb[71];
            s7_dneg <= s6_dneg; s7_degen <= s6_degen;
            s7_e0c <= s6_e0c; s7_e1c <= s6_e1c; s7_e2c <= s6_e2c;
            s7_edge_c_ov <= s6_edge_c_ov;
            s7_e0a <= s6_e0a; s7_e0b <= s6_e0b;
            s7_e1a <= s6_e1a; s7_e1b <= s6_e1b;
            s7_e2a <= s6_e2a; s7_e2b <= s6_e2b;
            s7_bxmin <= s6_bxmin; s7_bxmax <= s6_bxmax;
            s7_bymin <= s6_bymin; s7_bymax <= s6_bymax;
            s7_bbox_clamped <= s6_bbox_clamped;
            s7_x0 <= s6_x0; s7_y0 <= s6_y0; s7_z0 <= s6_z0;
            s7_color  <= s6_color;
            s7_xy_bad <= s6_xy_bad;
        end
    end

    // =========================================================================
    // S8: overflow test, round-half-even, saturate -> zA, zB (Q24.8)
    // =========================================================================
    function [32:0] round_sat;          // {overflow, value[31:0]}
        input [35:0] sp;                // from shr_split
        input        neg;
        reg        hi_nz, g, st;
        reg [31:0] q, lim;
        reg        ov;
        reg [32:0] r;
        begin
            hi_nz = sp[35]; g = sp[34]; st = sp[33]; q = sp[32:1];
            lim   = neg ? 32'h8000_0000 : 32'h7fff_ffff;
            ov    = hi_nz || (q > lim) || ((q == lim) && (g || st));
            if (ov)
                round_sat = {1'b1, neg ? 32'h8000_0000 : 32'h7fff_ffff};
            else begin
                r = {1'b0, q} + {32'd0, g && (st || q[0])};
                round_sat = {1'b0, neg ? (32'd0 - r[31:0]) : r[31:0]};
            end
        end
    endfunction

    wire [32:0] s7_za = s7_degen ? 33'd0 : round_sat(s7_sa, s7_nega);
    wire [32:0] s7_zb = s7_degen ? 33'd0 : round_sat(s7_sb, s7_negb);

    reg signed [31:0] s8_za, s8_zb;
    reg               s8_ova, s8_ovb;
    reg               s8_dneg, s8_degen;
    reg signed [33:0] s8_e0c, s8_e1c, s8_e2c;
    reg               s8_edge_c_ov;
    reg signed [17:0] s8_e0a, s8_e0b, s8_e1a, s8_e1b, s8_e2a, s8_e2b;
    reg        [9:0]  s8_bxmin, s8_bxmax, s8_bymin, s8_bymax;
    reg               s8_bbox_clamped;
    reg signed [17:0] s8_x0, s8_y0;
    reg signed [31:0] s8_z0;
    reg        [15:0] s8_color;
    reg               s8_xy_bad;

    always @(posedge clk) begin
        if (pipe_advance) begin
            s8_za <= s7_za[31:0]; s8_ova <= s7_za[32];
            s8_zb <= s7_zb[31:0]; s8_ovb <= s7_zb[32];
            s8_dneg <= s7_dneg; s8_degen <= s7_degen;
            s8_e0c <= s7_e0c; s8_e1c <= s7_e1c; s8_e2c <= s7_e2c;
            s8_edge_c_ov <= s7_edge_c_ov;
            s8_e0a <= s7_e0a; s8_e0b <= s7_e0b;
            s8_e1a <= s7_e1a; s8_e1b <= s7_e1b;
            s8_e2a <= s7_e2a; s8_e2b <= s7_e2b;
            s8_bxmin <= s7_bxmin; s8_bxmax <= s7_bxmax;
            s8_bymin <= s7_bymin; s8_bymax <= s7_bymax;
            s8_bbox_clamped <= s7_bbox_clamped;
            s8_x0 <= s7_x0; s8_y0 <= s7_y0; s8_z0 <= s7_z0;
            s8_color  <= s7_color;
            s8_xy_bad <= s7_xy_bad;
        end
    end

    // =========================================================================
    // S9: zA*x0, zB*y0 (Q16), from the exact zA/zB that will be output
    // =========================================================================
    reg signed [49:0] s9_pax, s9_pby;
    reg signed [31:0] s9_za, s9_zb;
    reg               s9_ova, s9_ovb;
    reg               s9_dneg, s9_degen;
    reg signed [33:0] s9_e0c, s9_e1c, s9_e2c;
    reg               s9_edge_c_ov;
    reg signed [17:0] s9_e0a, s9_e0b, s9_e1a, s9_e1b, s9_e2a, s9_e2b;
    reg        [9:0]  s9_bxmin, s9_bxmax, s9_bymin, s9_bymax;
    reg               s9_bbox_clamped;
    reg signed [31:0] s9_z0;
    reg        [15:0] s9_color;
    reg               s9_xy_bad;

    always @(posedge clk) begin
        if (pipe_advance) begin
            s9_pax <= s8_za * s8_x0;
            s9_pby <= s8_zb * s8_y0;
            s9_za <= s8_za; s9_zb <= s8_zb;
            s9_ova <= s8_ova; s9_ovb <= s8_ovb;
            s9_dneg <= s8_dneg; s9_degen <= s8_degen;
            s9_e0c <= s8_e0c; s9_e1c <= s8_e1c; s9_e2c <= s8_e2c;
            s9_edge_c_ov <= s8_edge_c_ov;
            s9_e0a <= s8_e0a; s9_e0b <= s8_e0b;
            s9_e1a <= s8_e1a; s9_e1b <= s8_e1b;
            s9_e2a <= s8_e2a; s9_e2b <= s8_e2b;
            s9_bxmin <= s8_bxmin; s9_bxmax <= s8_bxmax;
            s9_bymin <= s8_bymin; s9_bymax <= s8_bymax;
            s9_bbox_clamped <= s8_bbox_clamped;
            s9_z0 <= s8_z0;
            s9_color  <= s8_color;
            s9_xy_bad <= s8_xy_bad;
        end
    end

    // =========================================================================
    // S10: zC = round_sat((z0<<8 - zA*x0 - zB*y0) >> 8), output registers
    // =========================================================================
    wire signed [50:0] s9_zc_q16 = ($signed(s9_z0) <<< 8) - s9_pax - s9_pby;
    wire        [50:0] s9_zc_mag = s9_zc_q16[50] ? -s9_zc_q16 : s9_zc_q16;
    wire        [42:0] s9_zc_q   = s9_zc_mag[50:8];
    wire [35:0] s9_zc_sp = { |s9_zc_q[42:32], s9_zc_mag[7], |s9_zc_mag[6:0],
                             s9_zc_q[31:0], 1'b0 };
    wire [32:0] s9_zc = s9_degen ? 33'd0 : round_sat(s9_zc_sp, s9_zc_q16[50]);

    always @(posedge clk) begin
        if (pipe_advance && v[LATENCY-1]) begin
            bbox_xmin <= s9_bxmin;  bbox_xmax <= s9_bxmax;
            bbox_ymin <= s9_bymin;  bbox_ymax <= s9_bymax;
            edge0_A <= s9_e0a; edge0_B <= s9_e0b; edge0_C <= s9_e0c;
            edge1_A <= s9_e1a; edge1_B <= s9_e1b; edge1_C <= s9_e1c;
            edge2_A <= s9_e2a; edge2_B <= s9_e2b; edge2_C <= s9_e2c;
            zA <= s9_za;
            zB <= s9_zb;
            zC <= s9_zc[31:0];
            face_color <= s9_color;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            overflow          <= 1'b0;
            overflow_xy_range <= 1'b0;
            overflow_edge_c   <= 1'b0;
            overflow_zA       <= 1'b0;
            overflow_zB       <= 1'b0;
            overflow_zC       <= 1'b0;
            overflow_depth    <= 1'b0;
            bbox_clamped      <= 1'b0;
            orient_neg        <= 1'b0;
            degenerate        <= 1'b0;
        end else if (pipe_advance && v[LATENCY-1]) begin
            overflow_xy_range <= s9_xy_bad;
            overflow_edge_c   <= s9_edge_c_ov;
            overflow_zA       <= s9_ova;
            overflow_zB       <= s9_ovb;
            overflow_zC       <= s9_zc[32];
            overflow_depth    <= s9_ova | s9_ovb | s9_zc[32];
            overflow          <= s9_xy_bad | s9_edge_c_ov |
                                 s9_ova | s9_ovb | s9_zc[32];
            bbox_clamped      <= s9_bbox_clamped;
            orient_neg        <= s9_dneg && !s9_degen;
            degenerate        <= s9_degen;
        end
    end

endmodule

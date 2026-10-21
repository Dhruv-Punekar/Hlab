// -----------------------------------------------------------------------------
// Triangle Setup Unit — debug/diagnostic variant
//
// Converts one post-clipping, screen-space triangle into a rasterizer payload:
//   * inclusive integer pixel bounding box (10-bit unsigned indices)
//   * three signed edge equations
//   * Q24.8 depth plane z(x,y) = zA*x + zB*y + zC
//   * shared RGB565 color and status flags
//
// Architecture: input capture (R0), parallel first calculations (R1), bounds
// and synchronous reciprocal LUT (R2), final depth conversion/output (R3).
// Unstalled input-to-output latency is three clock intervals; II = 1.
// Backpressure freezes every stage and the LUT output register together.
//
// This is a separate diagnostic variant of triangle_setup_unit. The original
// module is unchanged. Detailed per-result flags identify input range,
// edge-C narrowing, and individual depth-output overflow cases.
// Numeric/interface contract is documented in triangle_setup_module_spec.txt.
// x/y range is guaranteed by the upstream screen-space clipping contract.
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

    // -------------------------------------------------------------------------
    // Global advance / backpressure
    // -------------------------------------------------------------------------
    // A blocked output freezes the complete pipeline. When the output is empty
    // or accepted this cycle, all stages may advance and a new input may enter.
    wire pipe_advance;
    assign pipe_advance = rst_n && (!out_valid || out_ready);
    assign in_ready     = pipe_advance;

    // -------------------------------------------------------------------------
    // Stage R0: capture and coordinate range reduction
    // -------------------------------------------------------------------------
    reg                r0_valid;
    reg signed [17:0]  r0_x0, r0_y0, r0_x1, r0_y1, r0_x2, r0_y2;
    reg signed [31:0]  r0_z0, r0_z1, r0_z2;
    reg        [15:0]  r0_color;
    reg                r0_xy_range_bad;

    // Range monitor runs on the full-width Q24.8 inputs, before narrowing.
    // Endpoints are included: x=300 and y=400 are legal viewport boundaries.
    reg input_xy_range_bad;
    always @* begin
        input_xy_range_bad =
            ($signed(in_x0) < 0 || $signed(in_x0) > 32'sd76800)  ||
            ($signed(in_x1) < 0 || $signed(in_x1) > 32'sd76800)  ||
            ($signed(in_x2) < 0 || $signed(in_x2) > 32'sd76800)  ||
            ($signed(in_y0) < 0 || $signed(in_y0) > 32'sd102400) ||
            ($signed(in_y1) < 0 || $signed(in_y1) > 32'sd102400) ||
            ($signed(in_y2) < 0 || $signed(in_y2) > 32'sd102400);
    end

    // The base contract guarantees x/y are in [0,300] and [0,400] in screen
    // space. This diagnostic variant checks that contract before narrowing.
    // Invalid inputs are clamped to the supported viewport to prevent low-bit
    // wraparound; overflow_xy_range tells downstream reg to reject/debug it.
    function signed [17:0] clamp_input_x;
        input signed [31:0] raw;

        begin
            if (raw < 0)               clamp_input_x = 18'sd0;
            else if (raw > 32'sd76800) clamp_input_x = 18'sd76800;
            else                       clamp_input_x = raw[17:0];
        end
    endfunction

    function signed [17:0] clamp_input_y;
        input signed [31:0] raw;

        begin
            if (raw < 0)                clamp_input_y = 18'sd0;
            else if (raw > 32'sd102400) clamp_input_y = 18'sd102400;
            else                        clamp_input_y = raw[17:0];
        end
    endfunction

    // Control (valid + flags) keeps the asynchronous reset. Datapath registers
    // have no reset so Vivado can pack them into DSP48E1 / RAMB18 registers,
    // which only support synchronous reset; outputs are qualified by valid.
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            r0_valid <= 1'b0;
            r0_xy_range_bad <= 1'b0;
        end else if (pipe_advance) begin
            r0_valid <= in_valid;
            if (in_valid)
                r0_xy_range_bad <= input_xy_range_bad;
        end
    end

    always @(posedge clk) begin
        if (pipe_advance) begin
            if (in_valid) begin
                r0_x0   <= clamp_input_x(in_x0);
                r0_y0   <= clamp_input_y(in_y0);
                r0_x1   <= clamp_input_x(in_x1);
                r0_y1   <= clamp_input_y(in_y1);
                r0_x2   <= clamp_input_x(in_x2);
                r0_y2   <= clamp_input_y(in_y2);
                r0_z0   <= in_z0;
                r0_z1   <= in_z1;
                r0_z2   <= in_z2;
                r0_color<= in_face_color;
            end
        end
    end

    // -------------------------------------------------------------------------
    // Stage R1 combinational calculations (all branches in parallel)
    // -------------------------------------------------------------------------
    reg signed [18:0] r0_dx1, r0_dy1, r0_dx2, r0_dy2;
    reg signed [32:0] r0_dz1, r0_dz2;
    reg signed [37:0] r0_p_dx1_dy2, r0_p_dx2_dy1;
    reg signed [38:0] r0_d_calc;
    reg signed [51:0] r0_p_dz1_dy2, r0_p_dz2_dy1;
    reg signed [51:0] r0_p_dx1_dz2, r0_p_dx2_dz1;
    reg signed [52:0] r0_nx_calc, r0_ny_calc;

    reg signed [18:0] r0_e0_a, r0_e0_b, r0_e1_a, r0_e1_b, r0_e2_a, r0_e2_b;
    reg signed [35:0] r0_e0_p0, r0_e0_p1, r0_e1_p0, r0_e1_p1, r0_e2_p0, r0_e2_p1;
    reg signed [36:0] r0_e0_c, r0_e1_c, r0_e2_c;

    reg signed [17:0] r0_x_min, r0_x_max, r0_y_min, r0_y_max;

    always @* begin
        r0_dx1 = $signed({r0_x1[17], r0_x1}) - $signed({r0_x0[17], r0_x0});
        r0_dy1 = $signed({r0_y1[17], r0_y1}) - $signed({r0_y0[17], r0_y0});
        r0_dx2 = $signed({r0_x2[17], r0_x2}) - $signed({r0_x0[17], r0_x0});
        r0_dy2 = $signed({r0_y2[17], r0_y2}) - $signed({r0_y0[17], r0_y0});
        r0_dz1 = $signed({r0_z1[31], r0_z1}) - $signed({r0_z0[31], r0_z0});
        r0_dz2 = $signed({r0_z2[31], r0_z2}) - $signed({r0_z0[31], r0_z0});

        r0_p_dx1_dy2 = $signed({{19{r0_dx1[18]}},r0_dx1}) *
                       $signed({{19{r0_dy2[18]}},r0_dy2});
        r0_p_dx2_dy1 = $signed({{19{r0_dx2[18]}},r0_dx2}) *
                       $signed({{19{r0_dy1[18]}},r0_dy1});
        r0_d_calc = $signed({r0_p_dx1_dy2[37], r0_p_dx1_dy2}) -
                    $signed({r0_p_dx2_dy1[37], r0_p_dx2_dy1});

        r0_p_dz1_dy2 = $signed({{19{r0_dz1[32]}},r0_dz1}) *
                       $signed({{33{r0_dy2[18]}},r0_dy2});
        r0_p_dz2_dy1 = $signed({{19{r0_dz2[32]}},r0_dz2}) *
                       $signed({{33{r0_dy1[18]}},r0_dy1});
        r0_p_dx1_dz2 = $signed({{33{r0_dx1[18]}},r0_dx1}) *
                       $signed({{19{r0_dz2[32]}},r0_dz2});
        r0_p_dx2_dz1 = $signed({{33{r0_dx2[18]}},r0_dx2}) *
                       $signed({{19{r0_dz1[32]}},r0_dz1});
        r0_nx_calc = $signed({r0_p_dz1_dy2[51], r0_p_dz1_dy2}) -
                     $signed({r0_p_dz2_dy1[51], r0_p_dz2_dy1});
        r0_ny_calc = $signed({r0_p_dx1_dz2[51], r0_p_dx1_dz2}) -
                     $signed({r0_p_dx2_dz1[51], r0_p_dx2_dz1});

        // Directed edges: (0->1), (1->2), (2->0).
        r0_e0_a = $signed({r0_y0[17],r0_y0}) - $signed({r0_y1[17],r0_y1});
        r0_e0_b = $signed({r0_x1[17],r0_x1}) - $signed({r0_x0[17],r0_x0});
        r0_e1_a = $signed({r0_y1[17],r0_y1}) - $signed({r0_y2[17],r0_y2});
        r0_e1_b = $signed({r0_x2[17],r0_x2}) - $signed({r0_x1[17],r0_x1});
        r0_e2_a = $signed({r0_y2[17],r0_y2}) - $signed({r0_y0[17],r0_y0});
        r0_e2_b = $signed({r0_x0[17],r0_x0}) - $signed({r0_x2[17],r0_x2});

        r0_e0_p0 = $signed({{18{r0_x0[17]}},r0_x0}) * $signed({{18{r0_y1[17]}},r0_y1});
        r0_e0_p1 = $signed({{18{r0_x1[17]}},r0_x1}) * $signed({{18{r0_y0[17]}},r0_y0});
        r0_e1_p0 = $signed({{18{r0_x1[17]}},r0_x1}) * $signed({{18{r0_y2[17]}},r0_y2});
        r0_e1_p1 = $signed({{18{r0_x2[17]}},r0_x2}) * $signed({{18{r0_y1[17]}},r0_y1});
        r0_e2_p0 = $signed({{18{r0_x2[17]}},r0_x2}) * $signed({{18{r0_y0[17]}},r0_y0});
        r0_e2_p1 = $signed({{18{r0_x0[17]}},r0_x0}) * $signed({{18{r0_y2[17]}},r0_y2});
        r0_e0_c = $signed({r0_e0_p0[35],r0_e0_p0}) - $signed({r0_e0_p1[35],r0_e0_p1});
        r0_e1_c = $signed({r0_e1_p0[35],r0_e1_p0}) - $signed({r0_e1_p1[35],r0_e1_p1});
        r0_e2_c = $signed({r0_e2_p0[35],r0_e2_p0}) - $signed({r0_e2_p1[35],r0_e2_p1});

        r0_x_min = r0_x0;
        r0_x_max = r0_x0;
        r0_y_min = r0_y0;
        r0_y_max = r0_y0;
        if (r0_x1 < r0_x_min) r0_x_min = r0_x1;
        if (r0_x2 < r0_x_min) r0_x_min = r0_x2;
        if (r0_x1 > r0_x_max) r0_x_max = r0_x1;
        if (r0_x2 > r0_x_max) r0_x_max = r0_x2;
        if (r0_y1 < r0_y_min) r0_y_min = r0_y1;
        if (r0_y2 < r0_y_min) r0_y_min = r0_y2;
        if (r0_y1 > r0_y_max) r0_y_max = r0_y1;
        if (r0_y2 > r0_y_max) r0_y_max = r0_y2;
    end

    // R1 registers hold the independent branch results and data needed later.
    reg                r1_valid;
    reg signed [38:0]  r1_d;
    reg signed [52:0]  r1_nx, r1_ny;
    reg signed [17:0]  r1_xmin_raw, r1_xmax_raw, r1_ymin_raw, r1_ymax_raw;
    reg signed [18:0]  r1_e0_a, r1_e0_b, r1_e1_a, r1_e1_b, r1_e2_a, r1_e2_b;
    reg signed [36:0]  r1_e0_c, r1_e1_c, r1_e2_c;
    reg signed [17:0]  r1_x0, r1_y0;
    reg signed [31:0]  r1_z0;
    reg        [15:0]  r1_color;
    reg                r1_xy_range_bad;
    reg                r1_edge_c_overflow;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            r1_valid <= 1'b0;
            r1_xy_range_bad <= 1'b0;
            r1_edge_c_overflow <= 1'b0;
        end else if (pipe_advance) begin
            r1_valid <= r0_valid;
            if (r0_valid) begin
                r1_xy_range_bad <= r0_xy_range_bad;
                // A signed 37-bit C is representable in signed 34 bits only
                // when its three discarded upper bits repeat bit 33.
                r1_edge_c_overflow <=
                    (r0_e0_c[36:34] != {3{r0_e0_c[33]}}) ||
                    (r0_e1_c[36:34] != {3{r0_e1_c[33]}}) ||
                    (r0_e2_c[36:34] != {3{r0_e2_c[33]}});
            end
        end
    end

    always @(posedge clk) begin
        if (pipe_advance) begin
            if (r0_valid) begin
                r1_d       <= r0_d_calc;
                r1_nx      <= r0_nx_calc;
                r1_ny      <= r0_ny_calc;
                r1_xmin_raw<= r0_x_min;
                r1_xmax_raw<= r0_x_max;
                r1_ymin_raw<= r0_y_min;
                r1_ymax_raw<= r0_y_max;
                r1_e0_a    <= r0_e0_a; r1_e0_b <= r0_e0_b; r1_e0_c <= r0_e0_c;
                r1_e1_a    <= r0_e1_a; r1_e1_b <= r0_e1_b; r1_e1_c <= r0_e1_c;
                r1_e2_a    <= r0_e2_a; r1_e2_b <= r0_e2_b; r1_e2_c <= r0_e2_c;
                r1_x0      <= r0_x0;   r1_y0   <= r0_y0;
                r1_z0      <= r0_z0;
                r1_color   <= r0_color;
            end
        end
    end

    // -------------------------------------------------------------------------
    // Reciprocal ROM address generation from R1 determinant
    // -------------------------------------------------------------------------
    reg [38:0] d_magnitude;
    reg [38:0] d_normalized;
    reg [5:0]  d_exponent;
    reg [9:0]  recip_address;
    integer      norm_bit;

    always @* begin
        if (r1_d[38])
            d_magnitude = (~r1_d) + 39'd1;
        else
            d_magnitude = r1_d;

        d_exponent = 6'd0;
        for (norm_bit = 0; norm_bit < 39; norm_bit = norm_bit + 1)
            if (d_magnitude[norm_bit]) d_exponent = norm_bit[5:0];

        d_normalized = 39'd0;
        if (d_magnitude != 0)
            d_normalized = d_magnitude << (38 - d_exponent);
        recip_address = d_normalized[37:28];
    end

    (* rom_style = "block" *) reg [17:0] recip_rom [0:1023];
    reg [17:0] recip_rom_q;
    initial begin
        $readmemh(RECIP_LUT_FILE, recip_rom);
    end

    // Synchronous ROM read; CE follows the global pipeline advance. The read
    // is unconditional: a data-dependent "else 0" acts like a reset on the
    // output register and can prevent RAMB18 inference. D == 0 is handled by
    // r2_degenerate, which forces zA/zB/zC to zero.
    always @(posedge clk) begin
        if (pipe_advance)
            recip_rom_q <= recip_rom[recip_address];
    end

    // -------------------------------------------------------------------------
    // Stage R2: bounding-box conversion, reciprocal capture, alignment
    // -------------------------------------------------------------------------
    function [9:0] bbox_floor_clamp_x;
        input signed [17:0] raw;

        reg signed [17:0] whole;
        begin
            whole = raw >>> 8;
            if (whole < 0)       bbox_floor_clamp_x = 10'd0;
            else if (whole > 299)bbox_floor_clamp_x = 10'd299;
            else                 bbox_floor_clamp_x = whole[9:0];
        end
    endfunction

    function [9:0] bbox_ceil_clamp_x;
        input signed [17:0] raw;

        reg signed [18:0] whole_ceil;
        begin
            whole_ceil = $signed(raw >>> 8) + $signed({1'b0, |raw[7:0]});
            if (whole_ceil < 0)       bbox_ceil_clamp_x = 10'd0;
            else if (whole_ceil > 299)bbox_ceil_clamp_x = 10'd299;
            else                     bbox_ceil_clamp_x = whole_ceil[9:0];
        end
    endfunction

    function [9:0] bbox_floor_clamp_y;
        input signed [17:0] raw;

        reg signed [17:0] whole;
        begin
            whole = raw >>> 8;
            if (whole < 0)       bbox_floor_clamp_y = 10'd0;
            else if (whole > 399)bbox_floor_clamp_y = 10'd399;
            else                 bbox_floor_clamp_y = whole[9:0];
        end
    endfunction

    function [9:0] bbox_ceil_clamp_y;
        input signed [17:0] raw;

        reg signed [18:0] whole_ceil;
        begin
            whole_ceil = $signed(raw >>> 8) + $signed({1'b0, |raw[7:0]});
            if (whole_ceil < 0)       bbox_ceil_clamp_y = 10'd0;
            else if (whole_ceil > 399)bbox_ceil_clamp_y = 10'd399;
            else                     bbox_ceil_clamp_y = whole_ceil[9:0];
        end
    endfunction

    // Separate diagnostic: reaching/clamping the viewport edge is expected
    // behavior and is not itself an arithmetic overflow. All debug flags are
    // per-output-triangle flags, aligned with out_valid (not sticky history).
    function  bbox_clamp_needed_x;
        input signed [17:0] min_raw;
        input signed [17:0] max_raw;

        reg signed [18:0] lo, hi;
        begin
            lo = min_raw >>> 8;
            hi = $signed(max_raw >>> 8) + $signed({1'b0, |max_raw[7:0]});
            bbox_clamp_needed_x = (lo < 0) || (lo > 299) || (hi < 0) || (hi > 299);
        end
    endfunction

    function  bbox_clamp_needed_y;
        input signed [17:0] min_raw;
        input signed [17:0] max_raw;

        reg signed [18:0] lo, hi;
        begin
            lo = min_raw >>> 8;
            hi = $signed(max_raw >>> 8) + $signed({1'b0, |max_raw[7:0]});
            bbox_clamp_needed_y = (lo < 0) || (lo > 399) || (hi < 0) || (hi > 399);
        end
    endfunction

    reg                r2_valid;
    reg        [9:0]   r2_bbox_xmin, r2_bbox_xmax, r2_bbox_ymin, r2_bbox_ymax;
    reg signed [31:0]  r2_e0_a, r2_e0_b, r2_e1_a, r2_e1_b, r2_e2_a, r2_e2_b;
    reg signed [33:0]  r2_e0_c, r2_e1_c, r2_e2_c;
    reg signed [52:0]  r2_nx, r2_ny;
    reg signed [17:0]  r2_x0, r2_y0;
    reg signed [31:0]  r2_z0;
    reg        [15:0]  r2_color;
    wire [17:0]         r2_inv_m;
    reg [5:0]          r2_exp;
    reg                r2_d_neg, r2_degenerate;
    reg                r2_xy_range_bad, r2_edge_c_overflow, r2_bbox_clamped;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            r2_valid <= 1'b0;
            r2_xy_range_bad <= 1'b0;
            r2_edge_c_overflow <= 1'b0;
            r2_bbox_clamped <= 1'b0;
        end else if (pipe_advance) begin
            r2_valid <= r1_valid;
            if (r1_valid) begin
                r2_xy_range_bad <= r1_xy_range_bad;
                r2_edge_c_overflow <= r1_edge_c_overflow;
                r2_bbox_clamped <= bbox_clamp_needed_x(r1_xmin_raw, r1_xmax_raw) |
                                   bbox_clamp_needed_y(r1_ymin_raw, r1_ymax_raw);
            end
        end
    end

    always @(posedge clk) begin
        if (pipe_advance) begin
            if (r1_valid) begin
                r2_bbox_xmin <= bbox_floor_clamp_x(r1_xmin_raw);
                r2_bbox_xmax <= bbox_ceil_clamp_x (r1_xmax_raw);
                r2_bbox_ymin <= bbox_floor_clamp_y(r1_ymin_raw);
                r2_bbox_ymax <= bbox_ceil_clamp_y (r1_ymax_raw);

                r2_e0_a <= {{13{r1_e0_a[18]}}, r1_e0_a};
                r2_e0_b <= {{13{r1_e0_b[18]}}, r1_e0_b};
                r2_e1_a <= {{13{r1_e1_a[18]}}, r1_e1_a};
                r2_e1_b <= {{13{r1_e1_b[18]}}, r1_e1_b};
                r2_e2_a <= {{13{r1_e2_a[18]}}, r1_e2_a};
                r2_e2_b <= {{13{r1_e2_b[18]}}, r1_e2_b};
                // Under the guaranteed screen-space range, each C fits signed
                // Q18.16 / 34 bits; retain the low 34 two's-complement bits.
                r2_e0_c <= r1_e0_c[33:0];
                r2_e1_c <= r1_e1_c[33:0];
                r2_e2_c <= r1_e2_c[33:0];

                r2_nx        <= r1_nx;
                r2_ny        <= r1_ny;
                r2_x0        <= r1_x0;
                r2_y0        <= r1_y0;
                r2_z0        <= r1_z0;
                r2_color     <= r1_color;
                r2_exp       <= d_exponent;
                r2_d_neg     <= r1_d[38];
                r2_degenerate<= (r1_d == 0);
            end
        end
    end

    // The synchronous ROM output itself is the R2 reciprocal register. It is
    // updated on the same edge as the R2 payload registers, so both are aligned.
    assign r2_inv_m = recip_rom_q;

    // -------------------------------------------------------------------------
    // Stage R3 combinational depth conversion and saturation
    // -------------------------------------------------------------------------
    function signed [79:0] round_shift_even;
        input signed [79:0] value;
        input [6:0] shift;

        reg [79:0] magnitude;
        reg [79:0] quotient;
        reg [79:0] low_mask;
        reg        guard_bit;
        reg        sticky_bit;
        begin
            if (shift == 0) begin
                round_shift_even = value;
            end else begin
                magnitude = value[79] ? (~value + 80'd1) : value;
                quotient  = magnitude >> shift;
                guard_bit = (magnitude >> (shift - 1'b1)) & 80'd1;
                if (shift > 1) begin
                    low_mask = (80'd1 << (shift - 1'b1)) - 80'd1;
                    sticky_bit = |(magnitude & low_mask);
                end else begin
                    sticky_bit = 1'b0;
                end
                if (guard_bit && (sticky_bit || quotient[0]))
                    quotient = quotient + 80'd1;
                round_shift_even = value[79] ? -$signed(quotient) : $signed(quotient);
            end
        end
    endfunction

    // Returns {overflow, saturated_signed_Q24.8}. Range is checked against the
    // unrounded value first; in-range values use round-to-nearest, ties-to-even.
    function [32:0] depth_round_saturate;
        input signed [79:0] numerator;
        input [6:0] shift;

        reg [79:0] magnitude;
        reg [79:0] limit;
        reg [79:0] rounded;
        reg        ov;
        begin
            magnitude = numerator[79] ? (~numerator + 80'd1) : numerator;
            if (numerator[79]) limit = 80'd2147483648;
            else              limit = 80'd2147483647;
            if (shift != 0) limit = limit << shift;
            ov = (magnitude > limit);
            if (ov) begin
                if (numerator[79])
                    depth_round_saturate = {1'b1, 32'h80000000};
                else
                    depth_round_saturate = {1'b1, 32'h7fffffff};
            end else begin
                rounded = round_shift_even(numerator, shift);
                depth_round_saturate = {1'b0, rounded[31:0]};
            end
        end
    endfunction

    reg signed [71:0] r2_prod_nx, r2_prod_ny;
    reg signed [79:0] r2_nx_scaled, r2_ny_scaled;
    reg        [6:0]  r2_depth_shift;
    reg        [32:0] r2_za_conv, r2_zb_conv;
    reg signed [49:0] r2_za_x0, r2_zb_y0;
    reg signed [50:0] r2_zc_q16;
    reg signed [79:0] r2_zc_numerator;
    reg        [32:0] r2_zc_conv;

    always @* begin
        // inv_m is Q1.17. Nx/D followed by Q8 output scaling gives a total
        // right shift of (e + 9) for the Nx_raw * inv_m_raw product.
        r2_prod_nx = $signed({{19{r2_nx[52]}},r2_nx}) *
                     $signed({{54{1'b0}},r2_inv_m});
        r2_prod_ny = $signed({{19{r2_ny[52]}},r2_ny}) *
                     $signed({{54{1'b0}},r2_inv_m});
        r2_nx_scaled = {{8{r2_prod_nx[71]}}, r2_prod_nx};
        r2_ny_scaled = {{8{r2_prod_ny[71]}}, r2_prod_ny};
        if (r2_d_neg) begin
            r2_nx_scaled = -r2_nx_scaled;
            r2_ny_scaled = -r2_ny_scaled;
        end
        r2_depth_shift = {1'b0, r2_exp} + 7'd9;

        if (r2_degenerate) begin
            r2_za_conv = {1'b0, 32'd0};
            r2_zb_conv = {1'b0, 32'd0};
        end else begin
            r2_za_conv = depth_round_saturate(r2_nx_scaled, r2_depth_shift);
            r2_zb_conv = depth_round_saturate(r2_ny_scaled, r2_depth_shift);
        end

        // zC is formed from the exact represented zA/zB values that will be
        // output. Products are Q16; z0 is aligned to Q16 before subtraction.
        r2_za_x0 = $signed({{18{r2_za_conv[31]}},r2_za_conv[31:0]}) *
                   $signed({{32{r2_x0[17]}},r2_x0});
        r2_zb_y0 = $signed({{18{r2_zb_conv[31]}},r2_zb_conv[31:0]}) *
                   $signed({{32{r2_y0[17]}},r2_y0});
        r2_zc_q16 = $signed({{11{r2_z0[31]}}, r2_z0, 8'b0}) -
                    $signed({r2_za_x0[49], r2_za_x0}) -
                    $signed({r2_zb_y0[49], r2_zb_y0});
        r2_zc_numerator = {{29{r2_zc_q16[50]}}, r2_zc_q16};
        if (r2_degenerate)
            r2_zc_conv = {1'b0, 32'd0};
        else
            r2_zc_conv = depth_round_saturate(r2_zc_numerator, 7'd8);
    end

    // -------------------------------------------------------------------------
    // Stage R3 output registers and status alignment
    // -------------------------------------------------------------------------
    reg r3_valid;
    assign out_valid = r3_valid;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            r3_valid <= 1'b0;
            overflow <= 1'b0;
            overflow_xy_range <= 1'b0;
            overflow_edge_c <= 1'b0;
            overflow_zA <= 1'b0;
            overflow_zB <= 1'b0;
            overflow_zC <= 1'b0;
            overflow_depth <= 1'b0;
            bbox_clamped <= 1'b0;
            orient_neg <= 1'b0;
            degenerate <= 1'b0;
        end else if (pipe_advance) begin
            r3_valid <= r2_valid;
            if (r2_valid) begin
                overflow_xy_range <= r2_xy_range_bad;
                overflow_edge_c <= r2_edge_c_overflow;
                overflow_zA <= r2_za_conv[32];
                overflow_zB <= r2_zb_conv[32];
                overflow_zC <= r2_zc_conv[32];
                overflow_depth <= r2_za_conv[32] | r2_zb_conv[32] | r2_zc_conv[32];
                overflow <= r2_xy_range_bad | r2_edge_c_overflow |
                            r2_za_conv[32] | r2_zb_conv[32] | r2_zc_conv[32];
                bbox_clamped <= r2_bbox_clamped;
                orient_neg <= r2_d_neg && !r2_degenerate;
                degenerate <= r2_degenerate;
            end
        end
    end

    always @(posedge clk) begin
        if (pipe_advance) begin
            if (r2_valid) begin
                bbox_xmin <= r2_bbox_xmin;
                bbox_xmax <= r2_bbox_xmax;
                bbox_ymin <= r2_bbox_ymin;
                bbox_ymax <= r2_bbox_ymax;

                edge0_A <= r2_e0_a; edge0_B <= r2_e0_b; edge0_C <= r2_e0_c;
                edge1_A <= r2_e1_a; edge1_B <= r2_e1_b; edge1_C <= r2_e1_c;
                edge2_A <= r2_e2_a; edge2_B <= r2_e2_b; edge2_C <= r2_e2_c;

                zA <= r2_za_conv[31:0];
                zB <= r2_zb_conv[31:0];
                zC <= r2_zc_conv[31:0];
                face_color <= r2_color;
            end
        end
    end

endmodule

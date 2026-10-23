// ============================================================================
// rasterizer.v
//
// Edge-function triangle rasterizer, 1 pixel evaluated per cycle.
//
//  * Samples at pixel centers: (x+0.5, y+0.5)
//  * Edge coefficients A, B are Q24.8 (COEF_W = 32, FRAC_IN = 8)
//  * Depth plane z_A, z_B, z_C has FRAC_Z fraction bits (default 8 = old Q24.8;
//    the pipeline uses 18 = Q14.18 since the 2026-10-22 depth precision fix)
//  * Edge constants edge0..2_C arrive as Q18.16 (EDGE_C_W = 34, EDGE_C_FRAC = 16) and are
//    converted to Q24.8 at capture: arithmetic >>> 8, rounded half away from zero
//    (C_ROUND = 1, keeps C(-edge) == -C(edge) so shared edges stay consistent) or
//    truncated toward -infinity (C_ROUND = 0)
//  * pixel_z output is signed, FRAC_OUT fraction bits (Q12.4 by default,
//    Q6.9 in the pipeline), saturating. Requires FRAC_OUT <= FRAC_Z + 1.
//  * Inside test: E0 >= 0 && E1 >= 0 && E2 >= 0 (ties count as inside)
//  * rst_n is asynchronous, active-low
//
// Internal scale: every plane accumulator holds 2x the Q24.8 raw value
// (1 LSB = 1/512), so the half-pixel offset (A+B)/2 is exact.
//     start value @ (xmin,ymin) = 2c + a + b + 2*a*xmin + 2*b*ymin
//     step in x = 2a, step in y = 2b   (real step is A and B)
//
// Plane index: 0,1,2 = edges, 3 = depth.
// ============================================================================
module rasterizer #(
    parameter COORD_W  = 10,   // pixel coordinate width
    parameter COEF_W   = 32,   // coefficient width (signed)
    parameter FRAC_IN  = 8,    // fractional bits of coefficients (Q24.8)
    parameter ACC_W    = 48,   // accumulator width (signed)
    parameter Z_OUT_W  = 16,   // pixel_z width
    parameter FRAC_OUT = 4,    // fractional bits of pixel_z (Q12.4)
    parameter FRAC_Z   = 8,    // fractional bits of z_A/z_B/z_C (depth plane)
    parameter COLOR_W  = 16,   // RGB565
    parameter Z_SIGNED = 1,    // 1: pixel_z signed [-2^(W-1),2^(W-1)-1], 0: unsigned
    parameter EDGE_C_W    = 34, // edge constant input width (signed)
    parameter EDGE_C_FRAC = 16, // fractional bits of the edge constant input (Q18.16)
    parameter C_ROUND     = 1   // 1: round half away from zero, 0: floor
)(
    input  wire                      clk,
    input  wire                      rst_n,           // async, active low

    // ---- from Triangle Setup ----
    input  wire                      triangle_valid,
    input  wire [COORD_W-1:0]        bbox_min_x,
    input  wire [COORD_W-1:0]        bbox_max_x,
    input  wire [COORD_W-1:0]        bbox_min_y,
    input  wire [COORD_W-1:0]        bbox_max_y,
    input  wire signed [COEF_W-1:0]  edge0_A, edge0_B,
    input  wire signed [EDGE_C_W-1:0] edge0_C,
    input  wire signed [COEF_W-1:0]  edge1_A, edge1_B,
    input  wire signed [EDGE_C_W-1:0] edge1_C,
    input  wire signed [COEF_W-1:0]  edge2_A, edge2_B,
    input  wire signed [EDGE_C_W-1:0] edge2_C,
    input  wire signed [COEF_W-1:0]  z_A, z_B, z_C,
    input  wire [COLOR_W-1:0]        color_in,
    input  wire                      orient_neg,

    // ---- from Z-buffer / framebuffer controller ----
    input  wire                      busy_fbuffer,

    // ---- to Z-buffer ----
    output reg  [COORD_W-1:0]        pixel_x,
    output reg  [COORD_W-1:0]        pixel_y,
    output reg  [Z_OUT_W-1:0]        pixel_z,
    output reg  [COLOR_W-1:0]        pixel_color,
    output reg                       pixel_valid,

    // ---- status ----
    output reg                       idle_raster,
    output reg                       done_raster,

    // ---- debug taps (added for top-level logging; no effect on behaviour) ----
    output wire [2:0]                dbg_state,
    output wire [COORD_W-1:0]        dbg_x_cur,
    output wire [COORD_W-1:0]        dbg_y_cur
);

    // ------------------------------------------------------------------
    // Derived constants
    // ------------------------------------------------------------------
    localparam PROD_W  = COEF_W + COORD_W + 1;        // exact signed product width
    localparam Z_SHIFT = FRAC_Z + 1 - FRAC_OUT;       // doubled depth plane -> pixel_z (18+1-9 = 10)
    localparam C_SHIFT = (EDGE_C_FRAC > FRAC_IN) ? (EDGE_C_FRAC - FRAC_IN) : 0;  // Q18.16 -> Q24.8 (= 8)

    // Edge constant: Q18.16 -> Q24.8.
    // One extra bit so adding the rounding offset cannot overflow; the integer part
    // (EDGE_C_W - EDGE_C_FRAC bits) always fits in COEF_W - FRAC_IN bits.
    function signed [COEF_W-1:0] conv_c;
        input signed [EDGE_C_W-1:0] c;
        reg signed [EDGE_C_W:0] e;
        begin
            e = {c[EDGE_C_W-1], c};
            if (C_SHIFT > 0) begin
                if (C_ROUND != 0)
                    e = e + (e[EDGE_C_W] ? ((1 << (C_SHIFT-1)) - 1) : (1 << (C_SHIFT-1)));
                e = e >>> C_SHIFT;
            end
            conv_c = e[COEF_W-1:0];
        end
    endfunction

    // FSM encoding
    localparam [2:0] S_IDLE   = 3'd0,
                     S_INIT_A = 3'd1,
                     S_INIT_B = 3'd2,
                     S_SCAN   = 3'd3,
                     S_DRAIN  = 3'd4,
                     S_FINISH = 3'd5;

    reg [2:0] state;

    // ------------------------------------------------------------------
    // Input coefficients packed as arrays (plane 0..3)
    // ------------------------------------------------------------------
    wire signed [COEF_W-1:0] in_a [0:3];
    wire signed [COEF_W-1:0] in_b [0:3];
    wire signed [COEF_W-1:0] in_c [0:3];

    wire signed [COEF_W-1:0]   n_a0 = orient_neg ? -edge0_A : edge0_A;
    wire signed [COEF_W-1:0]   n_b0 = orient_neg ? -edge0_B : edge0_B;
    wire signed [EDGE_C_W-1:0] n_c0 = orient_neg ? -edge0_C : edge0_C;
    wire signed [COEF_W-1:0]   n_a1 = orient_neg ? -edge1_A : edge1_A;
    wire signed [COEF_W-1:0]   n_b1 = orient_neg ? -edge1_B : edge1_B;
    wire signed [EDGE_C_W-1:0] n_c1 = orient_neg ? -edge1_C : edge1_C;
    wire signed [COEF_W-1:0]   n_a2 = orient_neg ? -edge2_A : edge2_A;
    wire signed [COEF_W-1:0]   n_b2 = orient_neg ? -edge2_B : edge2_B;
    wire signed [EDGE_C_W-1:0] n_c2 = orient_neg ? -edge2_C : edge2_C;

    assign in_a[0] = n_a0;     assign in_b[0] = n_b0;     assign in_c[0] = conv_c(n_c0);
    assign in_a[1] = n_a1;     assign in_b[1] = n_b1;     assign in_c[1] = conv_c(n_c1);
    assign in_a[2] = n_a2;     assign in_b[2] = n_b2;     assign in_c[2] = conv_c(n_c2);
    assign in_a[3] = z_A;      assign in_b[3] = z_B;      assign in_c[3] = z_C;

    // ------------------------------------------------------------------
    // Captured triangle (constant while rasterizing)
    // ------------------------------------------------------------------
    reg [COORD_W-1:0]        xmin_q, xmax_q, ymin_q, ymax_q;
    reg [COLOR_W-1:0]        color_q;
    reg signed [COEF_W-1:0]  cA [0:3];
    reg signed [COEF_W-1:0]  cB [0:3];
    reg signed [COEF_W-1:0]  cC [0:3];

    // ------------------------------------------------------------------
    // Scan state
    // ------------------------------------------------------------------
    reg [COORD_W-1:0]        x_cur, y_cur;
    reg signed [ACC_W-1:0]   row [0:3];   // plane value at (xmin, y_cur)
    reg signed [ACC_W-1:0]   cur [0:3];   // plane value at (x_cur, y_cur)
    reg signed [PROD_W-1:0]  mx  [0:3];   // INIT temp: a * xmin
    reg signed [PROD_W-1:0]  my  [0:3];   // INIT temp: b * ymin

    integer i;

    assign dbg_state = state;
    assign dbg_x_cur = x_cur;
    assign dbg_y_cur = y_cur;

    // ------------------------------------------------------------------
    // Combinational helpers
    // ------------------------------------------------------------------
    wire en       = ~busy_fbuffer;
    wire x_at_max = (x_cur == xmax_q);
    wire last     = x_at_max && (y_cur == ymax_q);

    // inside: sign bit of every edge accumulator is 0
    wire inside = ~(cur[0][ACC_W-1] | cur[1][ACC_W-1] | cur[2][ACC_W-1]);

    // depth: doubled Q24.8 -> Q12.4 (floor), then clamp to output range
    wire signed [ACC_W-1:0] zs    = cur[3] >>> Z_SHIFT;
    wire signed [ACC_W-1:0] z_max = Z_SIGNED ? ((1 <<< (Z_OUT_W-1)) - 1)
                                             : ((1 <<< Z_OUT_W) - 1);
    wire signed [ACC_W-1:0] z_min = Z_SIGNED ? -(1 <<< (Z_OUT_W-1)) : 0;
    wire [Z_OUT_W-1:0] z_out = (zs > z_max) ? z_max[Z_OUT_W-1:0] :
                               (zs < z_min) ? z_min[Z_OUT_W-1:0] :
                                              zs[Z_OUT_W-1:0];

    // ------------------------------------------------------------------
    // Control + registered outputs (async active-low reset)
    // ------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state       <= S_IDLE;
            idle_raster <= 1'b1;
            done_raster <= 1'b0;
            pixel_valid <= 1'b0;
            pixel_x     <= {COORD_W{1'b0}};
            pixel_y     <= {COORD_W{1'b0}};
            pixel_z     <= {Z_OUT_W{1'b0}};
            pixel_color <= {COLOR_W{1'b0}};
        end else begin
            case (state)
                S_IDLE: begin
                    // transfer happens when triangle_valid && idle_raster (=1 here)
                    if (triangle_valid) begin
                        idle_raster <= 1'b0;
                        state       <= S_INIT_A;
                    end
                end

                S_INIT_A: state <= S_INIT_B;
                S_INIT_B: state <= S_SCAN;

                S_SCAN: begin
                    if (en) begin
                        pixel_x     <= x_cur;
                        pixel_y     <= y_cur;
                        pixel_z     <= z_out;
                        pixel_color <= color_q;
                        pixel_valid <= inside;
                        if (last) state <= S_DRAIN;
                    end
                end

                S_DRAIN: begin
                    // last pixel is on the outputs; it is taken on this edge if en
                    if (en) begin
                        pixel_valid <= 1'b0;
                        done_raster <= 1'b1;
                        state       <= S_FINISH;
                    end
                end

                S_FINISH: begin
                    done_raster <= 1'b0;
                    idle_raster <= 1'b1;
                    state       <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

    // ------------------------------------------------------------------
    // Datapath (no reset needed)
    // ------------------------------------------------------------------
    always @(posedge clk) begin
        case (state)
            // capture inputs on the handshake edge
            S_IDLE: begin
                if (triangle_valid) begin
                    xmin_q  <= bbox_min_x;
                    xmax_q  <= bbox_max_x;
                    ymin_q  <= bbox_min_y;
                    ymax_q  <= bbox_max_y;
                    color_q <= color_in;
                    for (i = 0; i < 4; i = i + 1) begin
                        cA[i] <= in_a[i];
                        cB[i] <= in_b[i];
                        cC[i] <= in_c[i];
                    end
                end
            end

            // offset term and the two corner products
            S_INIT_A: begin
                x_cur <= xmin_q;
                y_cur <= ymin_q;
                for (i = 0; i < 4; i = i + 1) begin
                    row[i] <= cC[i] + cC[i] + cA[i] + cB[i];          // 2c + a + b
                    mx[i]  <= cA[i] * $signed({1'b0, xmin_q});        // a * xmin
                    my[i]  <= cB[i] * $signed({1'b0, ymin_q});        // b * ymin
                end
            end

            // start values at the center of (xmin, ymin)
            S_INIT_B: begin
                for (i = 0; i < 4; i = i + 1) begin
                    row[i] <= row[i] + mx[i] + mx[i] + my[i] + my[i];
                    cur[i] <= row[i] + mx[i] + mx[i] + my[i] + my[i];
                end
            end

            // walk the bounding box, row-major
            S_SCAN: begin
                if (en && !last) begin
                    if (x_at_max) begin
                        // wrap to the start of the next row
                        x_cur <= xmin_q;
                        y_cur <= y_cur + 1'b1;
                        for (i = 0; i < 4; i = i + 1) begin
                            row[i] <= row[i] + cB[i] + cB[i];
                            cur[i] <= row[i] + cB[i] + cB[i];
                        end
                    end else begin
                        // step right
                        x_cur <= x_cur + 1'b1;
                        for (i = 0; i < 4; i = i + 1)
                            cur[i] <= cur[i] + cA[i] + cA[i];
                    end
                end
            end

            default: ;
        endcase
    end

endmodule

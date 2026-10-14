// ============================================================================
// raster_demo.v -- helper for rasterizer_top.v
//   tri_rom (stand-in for Triangle Setup) + scene sequencer + slow-motion
//   busy_fbuffer generator + rasterizer instance. Exposes the 77-bit log bus.
// ============================================================================
module raster_demo #(
    parameter SLOW_DIV = 12_500_000,   // clk cycles per pixel step (1 = full speed)
    parameter GAP      = 100_000_000   // clk cycles between triangles
)(
    input  wire        clk,
    input  wire        rst_n,          // async, active low
    output wire [76:0] log
);
    // ------------------------------------------------------------------
    // Slow-motion: busy_fbuffer is high except for one cycle per SLOW_DIV,
    // so the rasterizer advances one pixel per tick and the LEDs are readable.
    // ------------------------------------------------------------------
    reg [31:0] slow_cnt;
    wire       tick = (slow_cnt == SLOW_DIV - 1);
    always @(posedge clk or negedge rst_n)
        if (!rst_n)    slow_cnt <= 32'd0;
        else if (tick) slow_cnt <= 32'd0;
        else           slow_cnt <= slow_cnt + 32'd1;

    wire busy_fbuffer = ~tick;

    // ------------------------------------------------------------------
    // Scene ROM (stand-in for Triangle Setup)
    // ------------------------------------------------------------------
    reg  [1:0] scene;
    wire [9:0]         bbox_min_x, bbox_max_x, bbox_min_y, bbox_max_y;
    wire signed [31:0] edge0_A, edge0_B, edge1_A, edge1_B, edge2_A, edge2_B;
    wire signed [33:0] edge0_C, edge1_C, edge2_C;
    wire signed [31:0] z_A, z_B, z_C;
    wire [15:0]        color_in;

    tri_rom u_rom (
        .scene(scene),
        .bbox_min_x(bbox_min_x), .bbox_max_x(bbox_max_x),
        .bbox_min_y(bbox_min_y), .bbox_max_y(bbox_max_y),
        .edge0_A(edge0_A), .edge0_B(edge0_B), .edge0_C(edge0_C),
        .edge1_A(edge1_A), .edge1_B(edge1_B), .edge1_C(edge1_C),
        .edge2_A(edge2_A), .edge2_B(edge2_B), .edge2_C(edge2_C),
        .z_A(z_A), .z_B(z_B), .z_C(z_C),
        .color_in(color_in)
    );

    // ------------------------------------------------------------------
    // Rasterizer
    // ------------------------------------------------------------------
    reg         triangle_valid;
    wire [9:0]  pixel_x, pixel_y, dbg_x_cur, dbg_y_cur;
    wire [15:0] pixel_z, pixel_color;
    wire        pixel_valid, idle_raster, done_raster;
    wire [2:0]  dbg_state;

    rasterizer u_rast (
        .clk(clk), .rst_n(rst_n),
        .triangle_valid(triangle_valid),
        .bbox_min_x(bbox_min_x), .bbox_max_x(bbox_max_x),
        .bbox_min_y(bbox_min_y), .bbox_max_y(bbox_max_y),
        .edge0_A(edge0_A), .edge0_B(edge0_B), .edge0_C(edge0_C),
        .edge1_A(edge1_A), .edge1_B(edge1_B), .edge1_C(edge1_C),
        .edge2_A(edge2_A), .edge2_B(edge2_B), .edge2_C(edge2_C),
        .z_A(z_A), .z_B(z_B), .z_C(z_C),
        .color_in(color_in),
        .busy_fbuffer(busy_fbuffer),
        .pixel_x(pixel_x), .pixel_y(pixel_y), .pixel_z(pixel_z),
        .pixel_color(pixel_color), .pixel_valid(pixel_valid),
        .idle_raster(idle_raster), .done_raster(done_raster),
        .dbg_state(dbg_state), .dbg_x_cur(dbg_x_cur), .dbg_y_cur(dbg_y_cur)
    );

    // ------------------------------------------------------------------
    // Sequencer: GAP -> present triangle -> wait for done -> next scene
    // ------------------------------------------------------------------
    localparam [1:0] SQ_GAP = 2'd0, SQ_SEND = 2'd1, SQ_WAIT = 2'd2;
    reg [1:0]  sq;
    reg [31:0] gap_cnt;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sq             <= SQ_GAP;
            gap_cnt        <= 32'd0;
            scene          <= 2'd0;
            triangle_valid <= 1'b0;
        end else begin
            case (sq)
                SQ_GAP: begin
                    if (gap_cnt >= GAP - 1) begin
                        gap_cnt        <= 32'd0;
                        triangle_valid <= 1'b1;
                        sq             <= SQ_SEND;
                    end else
                        gap_cnt <= gap_cnt + 32'd1;
                end
                SQ_SEND: begin                      // handshake: valid && idle
                    if (triangle_valid && idle_raster) begin
                        triangle_valid <= 1'b0;
                        sq             <= SQ_WAIT;
                    end
                end
                SQ_WAIT: begin
                    if (done_raster) begin
                        scene <= scene + 2'd1;      // wraps 3 -> 0
                        sq    <= SQ_GAP;
                    end
                end
                default: sq <= SQ_GAP;
            endcase
        end
    end

    // ------------------------------------------------------------------
    // Log bus
    // ------------------------------------------------------------------
    assign log = { pixel_x, pixel_y, pixel_z, pixel_valid, pixel_color,
                   idle_raster, dbg_state, dbg_x_cur, dbg_y_cur };
endmodule

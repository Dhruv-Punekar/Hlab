`timescale 1ns / 1ps

// ============================================================================
// Module: hardware_pipeline  (top level, Nexys A7-100T, xc7a100tcsg324-1)
//
//   vertex_memory ─┐
//                  ├─> transform_engine -> projection_engine
//   matrix_memory ─┘        -> triangle_setup_unit_debug -> rasterizer
//                           -> z_f_buffer -> vga_controller -> VGA (RGB444)
//
// Clocks : clk     = 100 MHz render clock (vertex fetch .. z_f_buffer write)
//          clk_vga = 25 MHz pixel clock   (z_f_buffer read port, VGA timing)
// Reset  : rst_n, active low (e.g. CPU_RESETN). Synchronised per clock domain
//          here (async assert, sync de-assert).
//
// Memory files (must be added to the Vivado project as design sources /
// memory-init files so $readmemh finds them):
//   vertices.mem            -> vertex_memory     (144-bit words)
//   matrix.mem              -> matrix_memory     (16 x 32-bit Q24.8)
//                              word order: M33 first ... M00 last, because
//                              transform_engine reads M00 from matrix_regs[15]
//   recip_lut_q1_17.mem     -> triangle_setup    (1024 x 18-bit Q1.17)
//   palette.mem             -> z_f_buffer        (only when DOUBLE_BUF = 1)
//
// Signed depth is handled inside z_f_buffer, so this level connects it directly.
// Setup -> rasterizer: degenerate, depth-overflow and (CULL_MODE) back-facing
// triangles are discarded.
//
// Re-render / animation ports (2026-10-24, used by cube_top):
//   rst_render_n  extra active-low reset for the RENDER domain only. Pulsing it
//                 low clears the z/framebuffer and re-draws the model, while the
//                 VGA timing keeps running (rst_n still resets both domains).
//                 Tie to 1'b1 if unused.
//   mat_we/addr/data  write port of matrix_memory (word k = matrix_regs[k],
//                 same order as matrix.mem: M33 first ... M00 last). Write only
//                 while rst_render_n = 0, so no vertex is in flight. Tie off if unused.
//   render_done   1 when the vertex fetch has finished, the z-buffer clear is
//                 done and every stage has been idle for 64 clocks.
//
// Double buffering (2026-11-02, used by game_top; see z_f_buffer.v):
//   DOUBLE_BUF    0 (default): exactly the old single RGB444 framebuffer. The
//                 new ports below are then unused and may be left unconnected,
//                 so cube_top / image_top / nexys_a7_top need no change.
//                 1: two 4-bit palette-index framebuffers.
//   draw_buf      (render clock) buffer the pipeline draws into. Change it only
//                 while rst_render_n = 0 or after render_done.
//   show_buf      (VGA clock) buffer the monitor shows. Change it only during
//                 vertical blanking (buffer_swap does this).
//   pal_we/addr/data  (render clock) write the draw buffer's palette. Tie to 0
//                 for now (prepared for lighting).
//   stat_pixel    (render clock) 1 for every pixel the rasterizer hands to the
//                 z-buffer (pixels inside visible triangles), for the
//                 "pixels per frame" counter on the 7-segment display.
//   With DOUBLE_BUF = 1 the VGA output stays black only until the FIRST frame
//   has been drawn after a full reset (fb_ready). In the old mode it is black
//   during every z-buffer clear, which is fine there because the clear happens
//   while the screen is not being read.
//
// Depth path (2026-10-22 precision fix): projection z Q16.16 -> setup plane
// Q14.18 (DEPTH_FRAC) -> rasterizer pixel_z Q6.9 (Z_FRAC_OUT) -> z_f_buffer.
//
// ---------------------------------------------------------------------------
// NAVIGATION COPY (hardware_pipeline_nav, 2026-11-03)
//   = hardware_pipeline.v of hardware_pipeline/ (double buffering, stat_pixel,
//     first_frame_done) with ONE addition, renamed so both files can sit in
//     the same Vivado project:
//   DISCARD_BEHIND = 1 drops a triangle between projection and setup when any
//   of its vertices had W' <= 0 (at or behind the eye). projection_engine
//   already marks such a vertex with x = y = 0x7FFFFFFF; without this the
//   setup unit clamps it to the screen corner and the triangle is drawn as a
//   big wrong wedge in front of everything. Only a camera that walks INSIDE
//   the scene ever has such vertices; with the default 0 this file behaves
//   exactly like hardware_pipeline.v.
// ============================================================================
module hardware_pipeline_nav #(
    parameter NUM_VERTICES    = 36,     // vertices in vertices.mem (multiple of 3)
    parameter VERTEX_DEPTH    = 64,     // vertex_memory DEPTH
    parameter RECIP_LUT_FILE  = "recip_lut_q1_17.mem",
    parameter LOOP_RENDER     = 0,      // 1: re-send the model forever
    // ---- depth precision ----
    parameter Z_FRAC_OUT       = 9,     // rasterizer pixel_z fraction bits (Q6.9); must be <= DEPTH_FRAC+1
    // ---- back-face culling ----
    // 0: off (draw both sides)
    // 1: cull clockwise-on-screen triangles = front faces wound COUNTER-CLOCKWISE
    //    seen from outside in vertices.mem (OpenGL convention)
    // 2: opposite (front faces wound CLOCKWISE in vertices.mem)
    // Requires every triangle in vertices.mem to use the same winding, and a
    // matrix without a mirror (negative determinant flips the winding).
    parameter CULL_MODE        = 1,
    // ---- double buffering ----
    parameter DOUBLE_BUF       = 0,     // 0: old single RGB444 buffer, 1: two palette-index buffers
    parameter INDEX_W          = 4,     // palette index bits (16 colours)
    parameter PALETTE_FILE     = "palette.mem",
    // ---- behind-camera triangles (walk-through scenes) ----
    parameter DISCARD_BEHIND   = 0      // 1: drop triangles with a vertex at/behind the eye
)(
    input  wire       clk,        // 100 MHz
    input  wire       clk_vga,    // 25 MHz
    input  wire       rst_n,      // active-low, resets render + VGA domains
    input  wire       rst_render_n, // active-low, resets the render domain only (re-render)

    // matrix write port (load a new matrix while rst_render_n = 0)
    input  wire        mat_we,
    input  wire [3:0]  mat_addr,
    input  wire [31:0] mat_data,

    // double buffering (DOUBLE_BUF = 1 only; may be left unconnected otherwise)
    input  wire               draw_buf,   // render clock
    input  wire               show_buf,   // VGA clock
    input  wire               pal_we,     // render clock
    input  wire [INDEX_W-1:0] pal_addr,
    input  wire [11:0]        pal_data,

    output wire       render_done,  // model drawn, pipeline idle
    output wire       stat_pixel,   // one pixel sent to the z-buffer this clock

    output wire [3:0] vga_r,
    output wire [3:0] vga_g,
    output wire [3:0] vga_b,
    output wire       vga_hs,
    output wire       vga_vs
);

    // ========================================================================
    // RESET SYNCHRONISERS (async assert, sync release) - one per clock domain
    // ========================================================================
    (* ASYNC_REG = "TRUE" *) reg [1:0] rst_sync_r = 2'b00;
    (* ASYNC_REG = "TRUE" *) reg [1:0] rst_sync_v = 2'b00;
    (* ASYNC_REG = "TRUE" *) reg [1:0] rst_sync_f = 2'b00;

    wire rst_n_render_src = rst_n & rst_render_n;

    always @(posedge clk or negedge rst_n_render_src)
        if (!rst_n_render_src) rst_sync_r <= 2'b00;
        else        rst_sync_r <= {rst_sync_r[0], 1'b1};

    always @(posedge clk_vga or negedge rst_n)
        if (!rst_n) rst_sync_v <= 2'b00;
        else        rst_sync_v <= {rst_sync_v[0], 1'b1};

    // render clock, FULL reset only (not the per-frame rst_render_n)
    always @(posedge clk or negedge rst_n)
        if (!rst_n) rst_sync_f <= 2'b00;
        else        rst_sync_f <= {rst_sync_f[0], 1'b1};

    wire rst_n_r = rst_sync_r[1];   // render domain
    wire rst_n_v = rst_sync_v[1];   // VGA domain
    wire rst_n_f = rst_sync_f[1];   // render domain, full reset only

    // ========================================================================
    // 1) VERTEX MEMORY + MATRIX MEMORY
    // ========================================================================
    localparam VADDR_W = $clog2(VERTEX_DEPTH);

    wire               vm_en;
    reg  [VADDR_W-1:0] vm_addr;
    wire [143:0]       vm_data;

    vertex_memory #(
        .DEPTH (VERTEX_DEPTH),
        .WIDTH (144)
    ) u_vertex_memory (
        .clk      (clk),
        .en       (vm_en),
        .we       (1'b0),
        .addr     (vm_addr),
        .data_in  (144'd0),
        .data_out (vm_data)
    );

    wire [511:0] mm_out;

    matrix_memory u_matrix_memory (
        .clk        (clk),
        .we         (mat_we),
        .write_addr (mat_addr),
        .write_data (mat_data),
        .matrix_out (mm_out)
    );

    // matrix_memory: word k at [k*32 +: 32]; transform_engine takes M00 from
    // [511:480] = word 15, so matrix.mem must be written M33 first ... M00 last.

    // ------------------------------------------------------------------------
    // Vertex fetch FSM (the "Master FSM" the transform engine expects).
    // vertex_memory has a 2-cycle registered read (BRAM output register used,
    // timing fix 2026-10-27), so: ISSUE (en=1) -> WAIT -> SEND (data valid,
    // held stable because en=0) until transform_engine accepts it.
    // ------------------------------------------------------------------------
    localparam F_ISSUE = 2'd0, F_SEND = 2'd1, F_DONE = 2'd2, F_WAIT = 2'd3;
    reg  [1:0] f_state;

    wire te_ready_out;
    wire te_valid_in = (f_state == F_SEND);
    assign vm_en     = (f_state == F_ISSUE);

    always @(posedge clk or negedge rst_n_r) begin
        if (!rst_n_r) begin
            f_state <= F_ISSUE;
            vm_addr <= {VADDR_W{1'b0}};
        end else begin
            case (f_state)
                F_ISSUE: f_state <= F_WAIT;
                F_WAIT:  f_state <= F_SEND;
                F_SEND: if (te_ready_out) begin
                    if (vm_addr == NUM_VERTICES - 1) begin
                        vm_addr <= {VADDR_W{1'b0}};
                        f_state <= LOOP_RENDER ? F_ISSUE : F_DONE;
                    end else begin
                        vm_addr <= vm_addr + 1'b1;
                        f_state <= F_ISSUE;
                    end
                end
                F_DONE:  f_state <= F_DONE;
                default: f_state <= F_ISSUE;
            endcase
        end
    end

    // ========================================================================
    // 2) TRANSFORM ENGINE
    // ========================================================================
    wire         te_valid_out;
    wire [143:0] te_vertex_out;
    wire         pe_ready_out;

    transform_engine u_transform_engine (
        .clk        (clk),
        .rst_n      (rst_n_r),
        .valid_in   (te_valid_in),
        .ready_out  (te_ready_out),
        .ready_in   (pe_ready_out),
        .valid_out  (te_valid_out),
        .vertex_in  (vm_data),
        .matrix_in  (mm_out),
        .vertex_out (te_vertex_out)
    );

    // ========================================================================
    // 3) PROJECTION ENGINE
    // ========================================================================
    wire               pe_valid_out;
    wire               pe_ready_in;
    wire               ts_in_ready;
    wire signed [31:0] pe_x0, pe_y0, pe_z0;
    wire signed [31:0] pe_x1, pe_y1, pe_z1;
    wire signed [31:0] pe_x2, pe_y2, pe_z2;
    wire        [15:0] pe_color;

    projection_engine u_projection_engine (
        .clk            (clk),
        .rst_n          (rst_n_r),
        .valid_in       (te_valid_out),
        .ready_out      (pe_ready_out),
        .ready_in       (pe_ready_in),
        .valid_out      (pe_valid_out),
        .vertex_in      (te_vertex_out),
        .out_x0 (pe_x0), .out_y0 (pe_y0), .out_z0 (pe_z0),
        .out_x1 (pe_x1), .out_y1 (pe_y1), .out_z1 (pe_z1),
        .out_x2 (pe_x2), .out_y2 (pe_y2), .out_z2 (pe_z2),
        .out_face_color (pe_color)
    );

    // ------------------------------------------------------------------------
    // Projection -> Setup handshake, with the behind-camera discard (NAV)
    //   behind = some vertex carries the W' <= 0 mark (x = y = 0x7FFFFFFF)
    //   setup.in_valid = proj.valid_out && !drop
    //   proj.ready_in  = setup.in_ready || drop        (dropped at once)
    // pe_x*/pe_y* are registers inside projection_engine, so there is no
    // combinational loop.
    // ------------------------------------------------------------------------
    localparam signed [31:0] BEHIND_MARK = 32'sh7FFF_FFFF;
    wire pe_behind = ((pe_x0 == BEHIND_MARK) && (pe_y0 == BEHIND_MARK)) ||
                     ((pe_x1 == BEHIND_MARK) && (pe_y1 == BEHIND_MARK)) ||
                     ((pe_x2 == BEHIND_MARK) && (pe_y2 == BEHIND_MARK));
    wire pe_drop       = (DISCARD_BEHIND != 0) && pe_behind;
    wire ts_in_valid   = pe_valid_out && !pe_drop;
    assign pe_ready_in = ts_in_ready || pe_drop;

    // ========================================================================
    // 4) TRIANGLE SETUP UNIT (debug / bbox variant)
    // ========================================================================
    wire               ts_out_valid;
    wire               ts_out_ready;
    wire [9:0]         ts_xmin, ts_xmax, ts_ymin, ts_ymax;
    wire signed [31:0] ts_e0A, ts_e0B, ts_e1A, ts_e1B, ts_e2A, ts_e2B;
    wire signed [33:0] ts_e0C, ts_e1C, ts_e2C;
    wire signed [31:0] ts_zA, ts_zB, ts_zC;
    wire [15:0]        ts_color;
    wire ts_overflow, ts_ov_xy, ts_ov_edge_c, ts_ov_zA, ts_ov_zB, ts_ov_zC;
    wire ts_ov_depth, ts_bbox_clamped, ts_orient_neg, ts_degenerate;

    triangle_setup_unit_debug #(
        .RECIP_LUT_FILE (RECIP_LUT_FILE)
    ) u_triangle_setup (
        .clk   (clk),
        .rst_n (rst_n_r),

        .in_valid (ts_in_valid),
        .in_ready (ts_in_ready),
        .in_x0 (pe_x0), .in_y0 (pe_y0), .in_z0 (pe_z0),
        .in_x1 (pe_x1), .in_y1 (pe_y1), .in_z1 (pe_z1),
        .in_x2 (pe_x2), .in_y2 (pe_y2), .in_z2 (pe_z2),
        .in_face_color (pe_color),

        .out_valid (ts_out_valid),
        .out_ready (ts_out_ready),

        .bbox_xmin (ts_xmin), .bbox_xmax (ts_xmax),
        .bbox_ymin (ts_ymin), .bbox_ymax (ts_ymax),

        .edge0_A (ts_e0A), .edge0_B (ts_e0B), .edge0_C (ts_e0C),
        .edge1_A (ts_e1A), .edge1_B (ts_e1B), .edge1_C (ts_e1C),
        .edge2_A (ts_e2A), .edge2_B (ts_e2B), .edge2_C (ts_e2C),

        .zA (ts_zA), .zB (ts_zB), .zC (ts_zC),
        .face_color        (ts_color),
        .overflow          (ts_overflow),
        .overflow_xy_range (ts_ov_xy),
        .overflow_edge_c   (ts_ov_edge_c),
        .overflow_zA       (ts_ov_zA),
        .overflow_zB       (ts_ov_zB),
        .overflow_zC       (ts_ov_zC),
        .overflow_depth    (ts_ov_depth),
        .bbox_clamped      (ts_bbox_clamped),
        .orient_neg        (ts_orient_neg),
        .degenerate        (ts_degenerate)
    );

    // ------------------------------------------------------------------------
    // Setup -> Rasterizer handshake (pipeline-level connection)
    //   pipe.discard          = setup.degenerate || setup.overflow_depth
    //                           || back-facing (CULL_MODE)
    //   raster.triangle_valid = setup.out_valid && !pipe.discard
    //   setup.out_ready       = raster.idle_raster || pipe.discard
    //   raster.orient_neg     = setup.orient_neg
    // A discarded triangle is consumed by the setup unit at once and never
    // reaches the rasterizer.
    // ------------------------------------------------------------------------
    wire rast_idle, rast_done;

    // setup.orient_neg = 1 when the screen-space determinant is negative.
    // Screen y points down, so a triangle wound counter-clockwise in object
    // space (and facing the camera) arrives with orient_neg = 1.
    wire back_facing    = (CULL_MODE == 1) ? !ts_orient_neg :
                          (CULL_MODE == 2) ?  ts_orient_neg : 1'b0;
    wire pipe_discard   = ts_degenerate || ts_ov_depth || back_facing;
    wire rast_tri_valid = ts_out_valid && !pipe_discard;
    assign ts_out_ready = rast_idle || pipe_discard;

    // ========================================================================
    // 5) RASTERIZER
    // ========================================================================
    wire        zf_busy;
    wire [9:0]  rs_px, rs_py;
    wire [15:0] rs_pz, rs_pcolor;
    wire        rs_pvalid;
    wire [2:0]  rs_dbg_state;
    wire [9:0]  rs_dbg_x, rs_dbg_y;

    rasterizer #(
        .COORD_W     (10),
        .COEF_W      (32),
        .FRAC_IN     (8),
        .ACC_W       (48),
        .Z_OUT_W     (16),
        .FRAC_OUT    (Z_FRAC_OUT),
        .FRAC_Z      (18),            // setup depth plane is Q14.18
        .COLOR_W     (16),
        .Z_SIGNED    (1),
        .EDGE_C_W    (34),
        .EDGE_C_FRAC (16),
        .C_ROUND     (1)
    ) u_rasterizer (
        .clk            (clk),
        .rst_n          (rst_n_r),

        .triangle_valid (rast_tri_valid),
        .bbox_min_x     (ts_xmin),
        .bbox_max_x     (ts_xmax),
        .bbox_min_y     (ts_ymin),
        .bbox_max_y     (ts_ymax),
        .edge0_A (ts_e0A), .edge0_B (ts_e0B), .edge0_C (ts_e0C),
        .edge1_A (ts_e1A), .edge1_B (ts_e1B), .edge1_C (ts_e1C),
        .edge2_A (ts_e2A), .edge2_B (ts_e2B), .edge2_C (ts_e2C),
        .z_A (ts_zA), .z_B (ts_zB), .z_C (ts_zC),
        .color_in       (ts_color),
        .orient_neg     (ts_orient_neg),

        .busy_fbuffer   (zf_busy),

        .pixel_x        (rs_px),
        .pixel_y        (rs_py),
        .pixel_z        (rs_pz),
        .pixel_color    (rs_pcolor),
        .pixel_valid    (rs_pvalid),

        .idle_raster    (rast_idle),
        .done_raster    (rast_done),

        .dbg_state      (rs_dbg_state),
        .dbg_x_cur      (rs_dbg_x),
        .dbg_y_cur      (rs_dbg_y)
    );

    // Rasterizer pixel_z is signed (Z_SIGNED=1) and z_f_buffer now takes a
    // signed depth directly (clears to +32767 = far), so no conversion here.

    // ========================================================================
    // 6) Z-BUFFER / FRAMEBUFFER
    // ========================================================================
    wire        zf_ready, zf_clear_done;
    wire [9:0]  vga_fb_x, vga_fb_y;
    wire [11:0] vga_fb_color;

    z_f_buffer #(
        .WIDTH        (300),
        .HEIGHT       (400),
        .DOUBLE_BUF   (DOUBLE_BUF),
        .INDEX_W      (INDEX_W),
        .PALETTE_FILE (PALETTE_FILE)
    ) u_z_f_buffer (
        .clk_render  (clk),
        .rst_n       (rst_n_r),

        .valid_in    (rs_pvalid),
        .pixel_x     (rs_px),
        .pixel_y     (rs_py),
        .pixel_z     (rs_pz),
        .pixel_color (rs_pcolor),

        .ready_out   (zf_ready),        // == !busy, rasterizer uses busy instead
        .clear_done  (zf_clear_done),
        .busy        (zf_busy),

        .draw_buf    (draw_buf),
        .pal_we      (pal_we),
        .pal_addr    (pal_addr),
        .pal_data    (pal_data),

        .clk_vga     (clk_vga),
        .vga_x       (vga_fb_x),
        .vga_y       (vga_fb_y),
        .show_buf    (show_buf),
        .vga_color   (vga_fb_color)
    );

    // pixels handed to the z-buffer (the same condition z_f_buffer uses to
    // accept a pixel; out-of-screen pixels are counted too, there are none
    // after the setup unit's bounding-box clamp)
    assign stat_pixel = rs_pvalid && zf_ready;

    // ========================================================================
    // 6b) "FRAMEBUFFER HAS A PICTURE" flag for the VGA controller
    //   DOUBLE_BUF = 1: the render domain is reset for every frame, so
    //   zf_clear_done drops for every frame. That used to be invisible (the
    //   clear ran while the screen was not read), but with double buffering
    //   drawing happens while the monitor shows the OTHER buffer, so it would
    //   black out the screen every frame. Instead the screen is enabled once
    //   the first frame after a full reset is finished, and stays enabled.
    //   DOUBLE_BUF = 0: unchanged (zf_clear_done).
    // ========================================================================
    reg first_frame_done;
    always @(posedge clk or negedge rst_n_f) begin
        if (!rst_n_f)         first_frame_done <= 1'b0;
        else if (render_done) first_frame_done <= 1'b1;
    end
    wire fb_has_picture = (DOUBLE_BUF != 0) ? first_frame_done : zf_clear_done;

    // ========================================================================
    // 7) VGA CONTROLLER
    // ========================================================================
    wire vga_active;

    vga_controller #(
        .FB_W  (300),
        .FB_H  (400),
        .X_OFF (170),
        .Y_OFF (40)
    ) u_vga_controller (
        .clk_pix      (clk_vga),
        .rst_n        (rst_n_v),
        .fb_ready     (fb_has_picture),  // CDC handled inside (2-FF sync)
        .fb_color     (vga_fb_color),
        .fb_x         (vga_fb_x),
        .fb_y         (vga_fb_y),
        .vga_r        (vga_r),
        .vga_g        (vga_g),
        .vga_b        (vga_b),
        .hsync        (vga_hs),
        .vsync        (vga_vs),
        .video_active (vga_active)
    );

    // ========================================================================
    // 8) RENDER DONE: fetch finished, clear finished, all links quiet 64 clocks
    //    (64 > the longest internal latency, so nothing is still in flight)
    // ========================================================================
    reg [6:0] quiet_cnt;
    wire links_quiet = (f_state == F_DONE) && zf_clear_done && rast_idle &&
                       !te_valid_out && !pe_valid_out && !ts_out_valid && !rs_pvalid;

    always @(posedge clk or negedge rst_n_r) begin
        if (!rst_n_r)          quiet_cnt <= 7'd0;
        else if (!links_quiet) quiet_cnt <= 7'd0;
        else if (!quiet_cnt[6]) quiet_cnt <= quiet_cnt + 7'd1;
    end
    assign render_done = quiet_cnt[6];

endmodule

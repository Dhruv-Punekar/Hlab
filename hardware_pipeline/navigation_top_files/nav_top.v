`timescale 1ns / 1ps
// ============================================================================
// Module: nav_top   (board top, Nexys A7-100T, xc7a100tcsg324-1)
//         Walk around a small city (96 m x 96 m), game style, 60 frames/s.
//
// CONTROLS (hold a button to keep going)
//   BTNU / BTND   walk forward / back
//   BTNL / BTNR   turn left / right          (SW1 up: step sideways instead)
//   BTNC          snap to the nearest N / E / S / W  (handy for walking a road)
//   SW0  up       run (double speed, double turn speed)
//   SW14 up       hide the mini-map
//   SW15          LED page (see below)
//   CPU_RESETN    back to the start (red button)
//
// WHAT YOU SEE
//   middle : the 3D view (300 x 400), buildings/trees from the pipeline,
//            sky and ground painted by nav_overlay
//   right  : mini-map, red = you, yellow dots = where you look
//   7-seg  : "XX-ZZ.HHH"  position x, z in metres and heading in degrees
//   LEDs   : SW15 down  compass: one LED lit, LED15 = north, LED11 = east,
//                       LED7 = south, LED3 = west (turn right -> light moves right)
//            SW15 up    LED15..10 map column, LED9..4 map row (2 m cells),
//                       LED3..2 where you look compared with where you last
//                       walked (0 ahead, 1 right, 2 behind, 3 left),
//                       LED1 bumped into something, LED0 frame heartbeat
//   RGB LED16: green walking, blue turning, red bumped
//
// FRAME LOOP (same idea as game_top)
//   0. after a reset the start view is drawn at the first frame tick
//   1. wait for the VGA frame tick (60 per second)
//   2. nav_camera: read the buttons, move / turn, stop at walls, build the
//      matrix (about 50 clocks)
//   3. nothing changed -> back to 1 (the picture on the screen is still right)
//   4. render domain in reset, 16 matrix words written, reset released
//      -> clear + draw into the hidden buffer (DOUBLE_BUF = 1)
//   5. render_done -> remember the pose of this picture for nav_overlay,
//      ask buffer_swap to show it at the next vertical blanking
//   6. swap_done -> back to 1
//
// FILES
//   new : nav_top.v, nav_camera.v, nav_overlay.v, hardware_pipeline_nav.v,
//         nav_constraints.xdc
//   from hardware_pipeline/game_files : btn_debounce.v, buffer_swap.v, seg7_driver.v
//   from hardware_pipeline/           : the pipeline modules (vertex_memory,
//         matrix_memory, transform_engine, projected_engine, triangle_setup_unit_bbox,
//         rasterizer, z_f_buffer, vga_controller)  - hardware_pipeline.v not needed
//   mem (nav_world.py): vertices.mem matrix.mem palette.mem nav_rot.mem
//         nav_trig.mem nav_floor.mem nav_map.mem  + mem_files/recip_lut_q1_17.mem
//   NUM_VERTICES etc. must match nav_mem/scene_info.txt.
// ============================================================================
module nav_top #(
    parameter NUM_VERTICES   = 3450,     // scene_info.txt
    parameter VERTEX_DEPTH   = 4096,
    parameter RECIP_LUT_FILE = "recip_lut_q1_17.mem",
    parameter PALETTE_FILE   = "palette.mem",
    parameter CULL_MODE      = 1,        // nav_world.py writes counter-clockwise faces
    // camera (scene_info.txt)
    parameter START_X        = 139264,   // 1/4096 m  (34 m)
    parameter START_Z        = 212992,   //           (52 m)
    parameter START_H        = 0,        // north
    parameter WALK_STEP      = 205,      // 3 m/s
    parameter TURN_STEP      = 1,        // 84 deg/s
    parameter RADIUS         = 4096,     // 1 m
    parameter FOG_ROWS       = 8,
    // board timing (the testbench makes these shorter)
    parameter TRIGGER_MODE   = 0,        // 0: VGA frame tick  1: free-running (simulation)
    parameter DEBOUNCE_CLKS  = 1000000,  // 10 ms
    parameter SEG_REFRESH_BITS = 17
)(
    input  wire        CLK100MHZ,
    input  wire        CPU_RESETN,

    input  wire        BTNC, BTNU, BTNL, BTNR, BTND,
    input  wire [15:0] SW,
    output wire [15:0] LED,
    output wire        LED16_R, LED16_G, LED16_B,

    output wire [3:0]  VGA_R,
    output wire [3:0]  VGA_G,
    output wire [3:0]  VGA_B,
    output wire        VGA_HS,
    output wire        VGA_VS,

    output wire [6:0]  SEG,
    output wire        DP,
    output wire [7:0]  AN
);
    // ========================================================================
    // 1) CLOCKS: 100 MHz render, 25.175 MHz VGA (same MMCM as the other tops)
    // ========================================================================
    wire clk_fb_out, clk_fb_in, clk_vga_unbuf, clk_render_unbuf;
    wire clk_vga, clk_render, mmcm_locked;

    MMCME2_BASE #(
        .BANDWIDTH          ("OPTIMIZED"),
        .CLKIN1_PERIOD      (10.000),
        .DIVCLK_DIVIDE      (1),
        .CLKFBOUT_MULT_F    (9.000),
        .CLKFBOUT_PHASE     (0.000),
        .CLKOUT0_DIVIDE_F   (35.750),
        .CLKOUT0_DUTY_CYCLE (0.500),
        .CLKOUT0_PHASE      (0.000),
        .CLKOUT1_DIVIDE     (9),
        .CLKOUT1_DUTY_CYCLE (0.500),
        .CLKOUT1_PHASE      (0.000),
        .REF_JITTER1        (0.010),
        .STARTUP_WAIT       ("FALSE")
    ) u_mmcm (
        .CLKIN1   (CLK100MHZ),
        .CLKFBIN  (clk_fb_in),
        .CLKFBOUT (clk_fb_out),
        .CLKOUT0  (clk_vga_unbuf),
        .CLKOUT1  (clk_render_unbuf),
        .LOCKED   (mmcm_locked),
        .PWRDWN   (1'b0),
        .RST      (1'b0)
    );

    BUFG u_bufg_fb     (.I(clk_fb_out),       .O(clk_fb_in));
    BUFG u_bufg_vga    (.I(clk_vga_unbuf),    .O(clk_vga));
    BUFG u_bufg_render (.I(clk_render_unbuf), .O(clk_render));

    // ========================================================================
    // 2) RESET (as game_top: synchronised, then on a BUFG for timing)
    // ========================================================================
    (* ASYNC_REG = "TRUE" *) reg [1:0] btn_sync = 2'b00;
    always @(posedge clk_render or negedge mmcm_locked)
        if (!mmcm_locked) btn_sync <= 2'b00;
        else              btn_sync <= {btn_sync[0], CPU_RESETN};
    reg rst_hi = 1'b1;
    always @(posedge clk_render or negedge mmcm_locked)
        if (!mmcm_locked) rst_hi <= 1'b1;
        else              rst_hi <= ~btn_sync[1];
    wire rst_hi_g;
    BUFG u_bufg_rst (.I(rst_hi), .O(rst_hi_g));
    wire rst_n = ~rst_hi_g;

    (* ASYNC_REG = "TRUE" *) reg [1:0] rst_v_sync = 2'b00;    // VGA-clock copy
    always @(posedge clk_vga or negedge rst_n)
        if (!rst_n) rst_v_sync <= 2'b00;
        else        rst_v_sync <= {rst_v_sync[0], 1'b1};
    wire rst_v_n = rst_v_sync[1];

    // ========================================================================
    // 3) BUTTONS + SWITCHES (debounced)
    // ========================================================================
    wire [4:0]  btn;                     // {C, U, L, R, D}
    wire [15:0] sw;
    btn_debounce #(.N(5), .STABLE_CLKS(DEBOUNCE_CLKS)) u_db_btn (
        .clk(clk_render), .rst_n(rst_n),
        .raw({BTNC, BTNU, BTNL, BTNR, BTND}), .clean(btn));
    btn_debounce #(.N(16), .STABLE_CLKS(DEBOUNCE_CLKS)) u_db_sw (
        .clk(clk_render), .rst_n(rst_n), .raw(SW), .clean(sw));

    wire key_c = btn[4];
    wire key_u = btn[3];
    wire key_l = btn[2];
    wire key_r = btn[1];
    wire key_d = btn[0];
    wire sw_run = sw[0];
    wire sw_strafe = sw[1];
    wire sw_nomap = sw[14];
    wire sw_ledpage = sw[15];

    // BTNC: remember a press until the next frame uses it
    reg key_c_q, snap_pending;

    // ========================================================================
    // 4) CAMERA
    // ========================================================================
    reg                cam_start;
    wire               cam_done, cam_changed;
    reg  [3:0]         ld_cnt;
    wire [31:0]        cam_word;
    wire signed [31:0] cam_x8, cam_z8;
    wire [7:0]         cam_h, cam_walk_dir;
    wire               cam_moving, cam_turning, cam_bumped;

    nav_camera #(
        .START_X(START_X), .START_Z(START_Z), .START_H(START_H),
        .WALK_STEP(WALK_STEP), .TURN_STEP(TURN_STEP), .RADIUS(RADIUS)
    ) u_cam (
        .clk(clk_render), .rst_n(rst_n), .start(cam_start),
        .key_fwd(key_u), .key_back(key_d), .key_left(key_l), .key_right(key_r),
        .key_snap(snap_pending), .run(sw_run), .strafe(sw_strafe),
        .done(cam_done), .changed(cam_changed),
        .rd_addr(ld_cnt), .rd_data(cam_word),
        .pos_x8(cam_x8), .pos_z8(cam_z8), .heading(cam_h), .walk_dir(cam_walk_dir),
        .moving(cam_moving), .turning(cam_turning), .bumped(cam_bumped));

    // ========================================================================
    // 5) THE PIPELINE (double buffered, behind-camera triangles dropped)
    // ========================================================================
    reg         rst_render_n;
    reg         mat_we;
    reg  [3:0]  mat_addr;
    reg  [31:0] mat_data;
    wire        render_done, stat_pixel;
    wire [3:0]  pipe_r, pipe_g, pipe_b;
    wire        pipe_hs, pipe_vs;
    wire        front_r, show_buf;

    hardware_pipeline_nav #(
        .NUM_VERTICES   (NUM_VERTICES),
        .VERTEX_DEPTH   (VERTEX_DEPTH),
        .RECIP_LUT_FILE (RECIP_LUT_FILE),
        .LOOP_RENDER    (0),
        .Z_FRAC_OUT     (9),
        .CULL_MODE      (CULL_MODE),
        .DOUBLE_BUF     (1),
        .INDEX_W        (4),
        .PALETTE_FILE   (PALETTE_FILE),
        .DISCARD_BEHIND (1)
    ) u_pipeline (
        .clk          (clk_render),
        .clk_vga      (clk_vga),
        .rst_n        (rst_n),
        .rst_render_n (rst_render_n),
        .mat_we       (mat_we),
        .mat_addr     (mat_addr),
        .mat_data     (mat_data),
        .draw_buf     (~front_r),       // always draw the hidden buffer
        .show_buf     (show_buf),
        .pal_we       (1'b0),
        .pal_addr     (4'd0),
        .pal_data     (12'h000),
        .render_done  (render_done),
        .stat_pixel   (stat_pixel),
        .vga_r        (pipe_r),
        .vga_g        (pipe_g),
        .vga_b        (pipe_b),
        .vga_hs       (pipe_hs),
        .vga_vs       (pipe_vs)
    );

    // ========================================================================
    // 6) BUFFER SWAP (from game_files)
    // ========================================================================
    reg  swap_req;
    wire swap_done, swap_busy;
    buffer_swap u_swap (
        .clk_r(clk_render), .rst_r_n(rst_n),
        .swap_req(swap_req), .swap_done(swap_done), .swap_busy(swap_busy), .front_r(front_r),
        .clk_v(clk_vga), .rst_v_n(rst_v_n), .vsync(pipe_vs), .show_buf(show_buf));

    // ========================================================================
    // 7) SKY + GROUND + MINI-MAP on the way to the VGA pins
    //    pic_* = pose of the picture being swapped in (set before swap_req,
    //    unchanged until swap_done -> nav_overlay copies it at the swap)
    // ========================================================================
    reg signed [31:0] pic_x8, pic_z8;
    reg        [7:0]  pic_h;

    (* ASYNC_REG = "TRUE" *) reg [1:0] nomap_v;
    always @(posedge clk_vga) nomap_v <= {nomap_v[0], sw_nomap};

    nav_overlay #(
        .FOG_ROWS (FOG_ROWS),
        .START_X8 (START_X >>> 4), .START_Z8 (START_Z >>> 4), .START_H (START_H)
    ) u_overlay (
        .clk(clk_vga), .rst_n(rst_v_n),
        .in_r(pipe_r), .in_g(pipe_g), .in_b(pipe_b), .in_hs(pipe_hs), .in_vs(pipe_vs),
        .show_buf(show_buf), .pose_x8(pic_x8), .pose_z8(pic_z8), .pose_h(pic_h),
        .map_on(~nomap_v[1]),
        .out_r(VGA_R), .out_g(VGA_G), .out_b(VGA_B), .out_hs(VGA_HS), .out_vs(VGA_VS));

    // ========================================================================
    // 8) VGA FRAME TICK: pulse at line 440, once per frame (as game_top)
    // ========================================================================
    (* ASYNC_REG = "TRUE" *) reg [2:0] vs_s, hs_s;
    reg [9:0] hs_count;
    reg       frame_tick;
    always @(posedge clk_render or negedge rst_n) begin
        if (!rst_n) begin
            vs_s <= 3'b111; hs_s <= 3'b111; hs_count <= 10'd0; frame_tick <= 1'b0;
        end else begin
            vs_s <= {vs_s[1:0], pipe_vs};
            hs_s <= {hs_s[1:0], pipe_hs};
            frame_tick <= 1'b0;
            if (vs_s[2] && !vs_s[1])
                hs_count <= 10'd0;
            else if (hs_s[2] && !hs_s[1]) begin
                hs_count <= hs_count + 10'd1;
                if (hs_count + 10'd1 == 10'd476) frame_tick <= 1'b1;
            end
        end
    end

    // ========================================================================
    // 9) FRAME SEQUENCER
    // ========================================================================
    localparam S_IDLE = 3'd1,   // wait for the frame tick
               S_CAM  = 3'd2,   // nav_camera moves and builds the matrix
               S_LOAD = 3'd3,   // render domain in reset, write 16 words
               S_REL  = 3'd4,   // release the render reset
               S_RUN  = 3'd5,   // wait for render_done
               S_SWAP = 3'd6;   // wait until the new picture is on the screen
    reg [2:0]  state;
    reg [4:0]  wr_cnt;
    reg        tick_pending;    // a frame tick came while we were busy
    reg [31:0] frames;          // pictures drawn (LED0 heartbeat)
    reg        first;           // after a reset: draw even if nothing moved

    wire go = (TRIGGER_MODE == 1) ? 1'b1 : (tick_pending | frame_tick);

    always @(posedge clk_render or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE; rst_render_n <= 1'b0;     // render domain waits for a matrix
            first <= 1'b1;
            mat_we <= 1'b0; mat_addr <= 4'd0; mat_data <= 32'd0;
            cam_start <= 1'b0; ld_cnt <= 4'd0; wr_cnt <= 5'd0;
            swap_req <= 1'b0; tick_pending <= 1'b0; frames <= 32'd0;
            key_c_q <= 1'b0; snap_pending <= 1'b0;
            pic_x8 <= START_X >>> 4; pic_z8 <= START_Z >>> 4; pic_h <= START_H;
        end else begin
            mat_we    <= 1'b0;
            cam_start <= 1'b0;
            swap_req  <= 1'b0;
            if (frame_tick) tick_pending <= 1'b1;

            key_c_q <= key_c;
            if (key_c && !key_c_q) snap_pending <= 1'b1;   // BTNC pressed

            case (state)
                S_IDLE: if (go) begin
                    tick_pending <= 1'b0;
                    cam_start    <= 1'b1;
                    state        <= S_CAM;
                end
                S_CAM: if (cam_done) begin
                    snap_pending <= 1'b0;                  // used in this frame
                    if (cam_changed || first) begin
                        rst_render_n <= 1'b0;              // stop + clear the render domain
                        ld_cnt <= 4'd0; wr_cnt <= 5'd0;
                        state  <= S_LOAD;
                    end else
                        state  <= S_IDLE;                  // nothing to redraw
                end
                S_LOAD: begin
                    mat_we   <= 1'b1;
                    mat_addr <= ld_cnt;
                    mat_data <= cam_word;                  // word ld_cnt of the matrix
                    ld_cnt   <= ld_cnt + 4'd1;
                    wr_cnt   <= wr_cnt + 5'd1;
                    if (wr_cnt == 5'd15) state <= S_REL;
                end
                S_REL: begin                               // last write lands on this edge
                    rst_render_n <= 1'b1;
                    first        <= 1'b0;
                    state        <= S_RUN;
                end
                S_RUN: if (render_done) begin
                    frames   <= frames + 32'd1;
                    pic_x8   <= cam_x8;                    // pose of this picture
                    pic_z8   <= cam_z8;
                    pic_h    <= cam_h;
                    swap_req <= 1'b1;                      // show it at the next vblank
                    state    <= S_SWAP;
                end
                S_SWAP: if (swap_done) state <= S_IDLE;
                default: state <= S_IDLE;
            endcase
        end
    end

    // ========================================================================
    // 10) LEDS
    // ========================================================================
    // "bumped" and "walking/turning" last a few frames so they can be seen
    reg [3:0] bump_hold, move_hold, turn_hold;
    always @(posedge clk_render or negedge rst_n) begin
        if (!rst_n) begin
            bump_hold <= 4'd0; move_hold <= 4'd0; turn_hold <= 4'd0;
        end else if (cam_done) begin
            bump_hold <= cam_bumped  ? 4'd15 : (bump_hold != 0 ? bump_hold - 4'd1 : 4'd0);
            move_hold <= cam_moving  ? 4'd4  : (move_hold != 0 ? move_hold - 4'd1 : 4'd0);
            turn_hold <= cam_turning ? 4'd4  : (turn_hold != 0 ? turn_hold - 4'd1 : 4'd0);
        end
    end

    wire [5:0] cell_x = cam_x8[14:9];                  // 2 m cells (512 in Q24.8)
    wire [5:0] cell_z = cam_z8[14:9];
    wire [7:0] rel    = cam_h - cam_walk_dir + 8'd32;  // look direction vs. walk direction
    wire [7:0]  h_c     = cam_h + 8'd8;                // centre each LED on its direction
    wire [15:0] compass = 16'h8000 >> h_c[7:4];        // 16 sectors of 22.5 deg

    assign LED = sw_ledpage ? {cell_x, cell_z, rel[7:6], (bump_hold != 0), frames[4]}
                            : compass;

    // RGB LED: dimmed to 1/8 (it is very bright)
    reg [2:0] pwm = 3'd0;
    always @(posedge clk_render) pwm <= pwm + 3'd1;
    wire dim = (pwm == 3'd0);
    assign LED16_R = dim && (bump_hold != 0);
    assign LED16_G = dim && (move_hold != 0);
    assign LED16_B = dim && (turn_hold != 0);

    // ========================================================================
    // 11) 7-SEGMENT: "XX-ZZ.HHH"  x, z in metres, heading in degrees
    // ========================================================================
    function [11:0] bcd3;                              // 0..999 -> 3 BCD digits
        input [9:0] n;
        reg [9:0] r;
        reg [3:0] hd, td;
        integer k;
        begin
            hd = 4'd0; td = 4'd0;
            for (k = 1; k < 10; k = k + 1) if (n >= k * 100) hd = k;
            r = n - hd * 100;
            for (k = 1; k < 10; k = k + 1) if (r >= k * 10) td = k;
            r = r - td * 10;
            bcd3 = {hd, td, r[3:0]};
        end
    endfunction

    wire [15:0] deg = cam_h * 16'd45;                  // heading * 360 / 256 = h * 45 / 32
    reg  [11:0] bx, bz, bh;
    always @(posedge clk_render) begin                 // registered: keeps timing easy
        bx <= bcd3(cam_x8[17:8]);                      // metres
        bz <= bcd3(cam_z8[17:8]);
        bh <= bcd3(deg[14:5]);                         // degrees
    end

    // seg7_driver digit codes: 0..9, 10 blank, 14 '-'; digit 7 is the leftmost
    wire [39:0] digits = {1'b0, bx[7:4], 1'b0, bx[3:0], 5'd14,
                          1'b0, bz[7:4], 1'b0, bz[3:0],
                          1'b0, bh[11:8], 1'b0, bh[7:4], 1'b0, bh[3:0]};

    seg7_driver #(.REFRESH_BITS(SEG_REFRESH_BITS)) u_seg7 (
        .clk(clk_render), .rst_n(rst_n), .digits(digits), .dp(8'b0000_1000),
        .an(AN), .seg(SEG), .dp_n(DP));
endmodule

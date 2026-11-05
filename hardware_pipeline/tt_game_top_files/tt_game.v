`timescale 1ns / 1ps

// ============================================================================
// Module: tt_game   (everything of the table-tennis demo except the clocking)
//
//   pixel clock (25.175 MHz)                 render clock (100 MHz)
//   ------------------------                 ----------------------
//   tt_buttons   debounce + press events      render sequencer
//   tt_game_core one step per frame  --poses-->  capture poses
//                (at VGA line 440)  --toggle-->  rst_render_n = 0
//   tt_score_overlay  score panel,               tt_obj_animator rewrites
//                frame ticks                     the 156 object vertices
//                    ^                           rst_render_n = 1 -> clear +
//                    |                           render (~3.1 ms)
//   hardware_pipeline VGA output  <------------  hardware_pipeline
//
//   Frame timing: the framebuffer is read on VGA lines 40..439 only. At line
//   440 the game steps (~1 us), publishes the poses and requests a render;
//   clear + render take ~3.1 ms of the 3.97 ms until line 40, so the picture
//   is never torn. 'overrun' (LED15) latches if a render is still running at
//   line 40 of any frame.
//
//   Clock-domain crossings (all synchronised here):
//     buttons, switches          async -> pixel   2-FF (+ debounce)
//     req_toggle                 pixel -> render  2-FF + edge detect
//     pose bus (165 bits)        pixel -> render  captured on that edge; it was
//                                written one pixel clock (40 ns) before the
//                                toggle and stays constant for a whole frame
//     render busy                render -> pixel  2-FF (level)
//
//   Controls: btn = {BTNC, BTNU, BTNL, BTNR, BTND}
//     P1 (near, red racket):  BTNR forehand, BTNL backhand  (also serve)
//     P2 (far, black racket): BTNU forehand, BTND backhand  (also serve)
//     BTNC short press = next camera view, held 1 s = new game.
//     sw[0] = P2 by computer, sw[1] = P1 by computer, sw[2] = P1 tilt aiming,
//     sw[3] = mute, sw[4] = invert tilt, sw[5] = accelerometer Y axis.
//   LEDs: 0..3 = P1 fore/back, P2 fore/back buttons, 4..11 = aim position
//         (sw[2]) or camera view, 12/13 = P1/P2 serves, 14 = game over,
//         15 = render overrun (sticky).
//   Each frame: tt_view_loader writes the selected view's static scene and
//   matrix, then tt_obj_animator writes the rackets and the ball.
// ============================================================================
module tt_game #(
    parameter RECIP_LUT_FILE = "recip_lut_q1_17.mem",
    parameter Z_FRAC_OUT     = 9,
    parameter DB_BITS        = 17       // button debounce 2^17 pixel clocks = 5.2 ms
)(
    input  wire        clk_render,
    input  wire        clk_vga,
    input  wire        arst_n,          // async: button & MMCM locked

    input  wire [4:0]  btn,             // {C, U, L, R, D}, active high
    input  wire [5:0]  sw,
    output wire [15:0] led,

    // accelerometer (ADXL362, SPI)
    output wire        acl_sclk,
    output wire        acl_mosi,
    input  wire        acl_miso,
    output wire        acl_csn,

    // audio
    output wire        aud_pwm_o,       // 1 = release, 0 = drive low (tt_top makes it open-drain)
    output wire        aud_sd,

    output wire [3:0]  vga_r,
    output wire [3:0]  vga_g,
    output wire [3:0]  vga_b,
    output wire        vga_hs,
    output wire        vga_vs
);

`include "tt_scene_map.vh"

    // ========================================================================
    // resets (async assert, sync release), one per domain
    // ========================================================================
    (* ASYNC_REG = "TRUE" *) reg [1:0] rr = 2'b00, rv = 2'b00;
    always @(posedge clk_render or negedge arst_n)
        if (!arst_n) rr <= 2'b00; else rr <= {rr[0], 1'b1};
    always @(posedge clk_vga or negedge arst_n)
        if (!arst_n) rv <= 2'b00; else rv <= {rv[0], 1'b1};
    wire rst_n_r = rr[1];
    wire rst_n_v = rv[1];

    // ========================================================================
    // pipeline
    // ========================================================================
    reg          rst_render_n;
    wire         render_done;
    wire         an_we, ld_we;
    wire [15:0]  an_addr, ld_addr;
    wire [143:0] an_data, ld_data;
    wire         ld_mat_we;
    wire [3:0]   ld_mat_addr;
    wire [31:0]  ld_mat_data;
    // vertex writes: view loader first, then animator (never at the same time)
    wire         vw_we   = ld_we | an_we;
    wire [15:0]  vw_addr = ld_we ? ld_addr : an_addr;
    wire [143:0] vw_data = ld_we ? ld_data : an_data;
    wire [3:0]   p_r, p_g, p_b;
    wire         p_hs, p_vs;

    hardware_pipeline #(
        .NUM_VERTICES       (TT_NUM_VERTICES),
        .VERTEX_DEPTH       (TT_VERTEX_DEPTH),
        .RECIP_LUT_FILE     (RECIP_LUT_FILE),
        .LOOP_RENDER        (0),
        .Z_FRAC_OUT         (Z_FRAC_OUT),
        .CULL_MODE          (1),
        .CLEAR_COLOR_TOP    (TT_SKY_RGB444),
        .CLEAR_COLOR_BOTTOM (TT_FLOOR_RGB444),
        .CLEAR_SPLIT_Y      (TT_HORIZON_Y),
        .VTX_WRITE_EN       (1)
    ) u_pipeline (
        .clk          (clk_render),
        .clk_vga      (clk_vga),
        .rst_n        (rst_n_r),
        .rst_render_n (rst_render_n),
        .mat_we       (ld_mat_we),
        .mat_addr     (ld_mat_addr),
        .mat_data     (ld_mat_data),
        .vtx_we       (vw_we),
        .vtx_addr     (vw_addr),
        .vtx_data     (vw_data),
        .render_done  (render_done),
        .vga_r        (p_r),
        .vga_g        (p_g),
        .vga_b        (p_b),
        .vga_hs       (p_hs),
        .vga_vs       (p_vs)
    );

    // ========================================================================
    // pixel clock domain: inputs, game, overlay
    // ========================================================================
    wire [4:0] btn_pend, btn_level;
    wire       ev_clear;
    wire [4:0] ev_mask;              // {new, back1, back0, fore1, fore0}

    // map the board buttons to the core's event order
    // core: fore[0] = BTNR, fore[1] = BTNU, back[0] = BTNL, back[1] = BTND, new = BTNC
    wire [4:0] btn_core = {btn[4], btn[0], btn[2], btn[3], btn[1]};   // {C, D, L, U, R}

    tt_buttons #(.N (5), .DB_BITS (DB_BITS)) u_buttons (
        .clk        (clk_vga),
        .rst_n      (rst_n_v),
        .btn_raw    (btn_core),
        .clear      (ev_clear),
        .clear_mask (ev_mask),
        .pending    (btn_pend),
        .level      (btn_level)
    );

    (* ASYNC_REG = "TRUE" *) reg [5:0] sw_s1, sw_s2;
    always @(posedge clk_vga or negedge rst_n_v)
        if (!rst_n_v) begin sw_s1 <= 6'd0; sw_s2 <= 6'd0; end
        else          begin sw_s1 <= sw;   sw_s2 <= sw_s1; end
    wire [1:0] auto_play = {sw_s2[0], sw_s2[1]};   // sw[0] -> P2, sw[1] -> P1
    wire       tilt_on   = sw_s2[2];                // P1 aims with the board tilt
    wire       mute      = sw_s2[3];

    // ------------------------------------------------------------------------
    // BTNC: short press = next camera view, hold 1 s = new game
    // ------------------------------------------------------------------------
    wire        tick_440, tick_40;
    reg  [1:0]  view;
    reg  [6:0]  c_hold;               // frames BTNC has been held
    reg         new_pend;             // new-game event for the core
    always @(posedge clk_vga or negedge rst_n_v) begin
        if (!rst_n_v) begin
            view <= 2'd0;  c_hold <= 7'd0;  new_pend <= 1'b0;
        end else begin
            if (ev_clear && ev_mask[4]) new_pend <= 1'b0;
            if (tick_440) begin
                if (btn_level[4]) begin
                    if (c_hold != 7'd127) c_hold <= c_hold + 7'd1;
                    if (c_hold == 7'd59)  new_pend <= 1'b1;          // held 1 s
                end else begin
                    if (c_hold != 7'd0 && c_hold < 7'd60)            // short press released
                        view <= (view == TT_NUM_VIEWS - 1) ? 2'd0 : view + 2'd1;
                    c_hold <= 7'd0;
                end
            end
        end
    end

    // ------------------------------------------------------------------------
    // accelerometer -> P1 aim (bounce x)
    // ------------------------------------------------------------------------
    wire signed [11:0] acc_x, acc_y, acc_z;
    wire               acc_valid, acc_ok;
    adxl362_reader #(
        .HALF_CLKS    (13),            // 25.175 MHz / 26 = 0.97 MHz SCLK
        .POWERUP_CLKS (250000),        // 10 ms
        .RESET_CLKS   (250000),
        .POLL_CLKS    (250000)         // 100 reads / s
    ) u_acl (
        .clk (clk_vga), .rst_n (rst_n_v),
        .sclk (acl_sclk), .mosi (acl_mosi), .miso (acl_miso), .cs_n (acl_csn),
        .acc_x (acc_x), .acc_y (acc_y), .acc_z (acc_z),
        .sample_valid (acc_valid), .id_ok (acc_ok)
    );
    wire [15:0] tilt_x;
    wire [7:0]  aim_leds;
    tt_tilt u_tilt (
        .clk (clk_vga), .rst_n (rst_n_v),
        .acc_x (acc_x), .acc_y (acc_y), .acc_valid (acc_valid),
        .swap_axes (sw_s2[5]), .invert (sw_s2[4]),
        .tilt_x (tilt_x), .aim_leds (aim_leds)
    );

    wire [15:0] rp_x, rp_y, rp_z, ro_x, ro_y, ro_z, bl_x, bl_y, bl_z;
    wire [5:0]  rp_yaw, ro_yaw, bl_yaw;
    wire        bl_vis, req_toggle, server, winner, step_busy;
    wire [6:0]  score0, score1;
    wire [2:0]  gstate;
    wire        snd_hit, snd_bounce, snd_point;

    tt_game_core u_core (
        .clk (clk_vga), .rst_n (rst_n_v), .frame_tick (tick_440),
        .btn_fore (btn_pend[1:0]), .btn_back (btn_pend[3:2]), .btn_new (new_pend),
        .auto_play (auto_play), .tilt_en (tilt_on), .tilt_x (tilt_x),
        .ev_clear (ev_clear), .ev_mask (ev_mask),
        .rp_x (rp_x), .rp_y (rp_y), .rp_z (rp_z), .rp_yaw (rp_yaw),
        .ro_x (ro_x), .ro_y (ro_y), .ro_z (ro_z), .ro_yaw (ro_yaw),
        .bl_x (bl_x), .bl_y (bl_y), .bl_z (bl_z), .bl_yaw (bl_yaw), .bl_vis (bl_vis),
        .req_toggle (req_toggle),
        .score0 (score0), .score1 (score1), .server (server), .state (gstate),
        .winner (winner), .step_busy (step_busy),
        .snd_hit (snd_hit), .snd_bounce (snd_bounce), .snd_point (snd_point)
    );

    tt_sound u_sound (
        .clk (clk_vga), .rst_n (rst_n_v),
        .ev_hit (snd_hit), .ev_bounce (snd_bounce), .ev_point (snd_point),
        .mute (mute), .aud_pwm_o (aud_pwm_o), .aud_sd (aud_sd)
    );

    tt_score_overlay u_overlay (
        .clk (clk_vga), .rst_n (rst_n_v),
        .in_r (p_r), .in_g (p_g), .in_b (p_b), .in_hs (p_hs), .in_vs (p_vs),
        .score0 (score0), .score1 (score1), .server (server),
        .game_over (gstate == 3'd4), .winner (winner),
        .vga_r (vga_r), .vga_g (vga_g), .vga_b (vga_b), .vga_hs (vga_hs), .vga_vs (vga_vs),
        .tick_440 (tick_440), .tick_40 (tick_40)
    );

    // render overrun: still busy when the framebuffer read starts (line 40)
    reg rbusy_r;                                         // render domain, below
    (* ASYNC_REG = "TRUE" *) reg [1:0] rbusy_s;
    reg overrun;
    always @(posedge clk_vga or negedge rst_n_v) begin
        if (!rst_n_v) begin rbusy_s <= 2'b00; overrun <= 1'b0; end
        else begin
            rbusy_s <= {rbusy_s[0], rbusy_r};
            if (tick_40 && rbusy_s[1]) overrun <= 1'b1;
        end
    end

    assign led = {overrun, gstate == 3'd4, server, ~server,
                  tilt_on ? aim_leds : {6'd0, view},
                  btn_level[3], btn_level[1], btn_level[2], btn_level[0]};

    // ========================================================================
    // render clock domain: sequencer + animator
    // ========================================================================
    (* ASYNC_REG = "TRUE" *) reg [1:0] tog_s;
    reg tog_q;
    always @(posedge clk_render or negedge rst_n_r)
        if (!rst_n_r) begin tog_s <= 2'b00; tog_q <= 1'b0; end
        else          begin tog_s <= {tog_s[0], req_toggle}; tog_q <= tog_s[1]; end
    wire frame_req = tog_s[1] ^ tog_q;

    // captured poses
    reg [15:0] c_rp_x, c_rp_y, c_rp_z, c_ro_x, c_ro_y, c_ro_z, c_bl_x, c_bl_y, c_bl_z;
    reg [5:0]  c_rp_yaw, c_ro_yaw, c_bl_yaw;
    reg        c_bl_vis;
    reg [1:0]  c_view;

    reg  ld_start;
    wire ld_done;
    tt_view_loader u_view (
        .clk (clk_render), .rst_n (rst_n_r), .start (ld_start), .view (c_view),
        .vtx_we (ld_we), .vtx_addr (ld_addr), .vtx_data (ld_data),
        .mat_we (ld_mat_we), .mat_addr (ld_mat_addr), .mat_data (ld_mat_data),
        .done (ld_done)
    );

    reg  an_start;
    wire an_busy, an_done;

    tt_obj_animator #(.BASE (TT_RACKET_P_BASE)) u_animator (
        .clk (clk_render), .rst_n (rst_n_r), .start (an_start),
        .p0_x (c_rp_x), .p0_y (c_rp_y), .p0_z (c_rp_z), .p0_yaw (c_rp_yaw),
        .p1_x (c_ro_x), .p1_y (c_ro_y), .p1_z (c_ro_z), .p1_yaw (c_ro_yaw),
        .p2_x (c_bl_x), .p2_y (c_bl_y), .p2_z (c_bl_z), .p2_yaw (c_bl_yaw), .p2_vis (c_bl_vis),
        .vtx_we (an_we), .vtx_addr (an_addr), .vtx_data (an_data),
        .busy (an_busy), .done (an_done)
    );

    localparam R_BOOT = 3'd0,   // power-up render of vertices.mem
               R_IDLE = 3'd1,   // wait for a frame request
               R_ANIM = 3'd2,   // render domain in reset, animator writes
               R_LOAD = 3'd5,   // render domain in reset, view loader writes
               R_REL  = 3'd3,   // release the render reset
               R_RUN  = 3'd4;   // clear + render
    reg [2:0]  rstate;
    reg [31:0] frames_rendered;      // debug / testbench
    reg [21:0] render_cycles;        // clocks of the last render (debug / ILA)
    reg [21:0] rcnt;

    always @(posedge clk_render or negedge rst_n_r) begin
        if (!rst_n_r) begin
            rstate <= R_BOOT;  rst_render_n <= 1'b1;  an_start <= 1'b0;  ld_start <= 1'b0;  rbusy_r <= 1'b0;
            c_view <= 2'd0;
            frames_rendered <= 32'd0;  render_cycles <= 22'd0;  rcnt <= 22'd0;
            c_rp_x <= 16'd0; c_rp_y <= 16'd0; c_rp_z <= 16'd0; c_rp_yaw <= 6'd0;
            c_ro_x <= 16'd0; c_ro_y <= 16'd0; c_ro_z <= 16'd0; c_ro_yaw <= 6'd0;
            c_bl_x <= 16'd0; c_bl_y <= 16'd0; c_bl_z <= 16'd0; c_bl_yaw <= 6'd0; c_bl_vis <= 1'b0;
        end else begin
            an_start <= 1'b0;
            ld_start <= 1'b0;
            if (rcnt != 22'h3FFFFF) rcnt <= rcnt + 22'd1;
            case (rstate)
                R_BOOT: if (render_done)           // power-up render (not checked for overrun)
                    rstate <= R_IDLE;
                R_IDLE: if (frame_req) begin
                    c_rp_x <= rp_x; c_rp_y <= rp_y; c_rp_z <= rp_z; c_rp_yaw <= rp_yaw;
                    c_ro_x <= ro_x; c_ro_y <= ro_y; c_ro_z <= ro_z; c_ro_yaw <= ro_yaw;
                    c_bl_x <= bl_x; c_bl_y <= bl_y; c_bl_z <= bl_z; c_bl_yaw <= bl_yaw; c_bl_vis <= bl_vis;
                    c_view <= view;
                    rst_render_n <= 1'b0;
                    ld_start <= 1'b1;
                    rbusy_r  <= 1'b1;
                    rcnt     <= 22'd0;
                    rstate   <= R_LOAD;
                end
                R_LOAD: if (ld_done) begin             // static scene + matrix of the view
                    an_start <= 1'b1;
                    rstate   <= R_ANIM;
                end
                R_ANIM: if (an_done) rstate <= R_REL;  // rackets + ball
                R_REL: begin
                    rst_render_n <= 1'b1;
                    rstate <= R_RUN;
                end
                R_RUN: if (render_done) begin      // render_done is 0 while in reset
                    rbusy_r <= 1'b0;
                    render_cycles <= rcnt;
                    frames_rendered <= frames_rendered + 32'd1;
                    rstate <= R_IDLE;
                end
                default: rstate <= R_IDLE;
            endcase
        end
    end

endmodule

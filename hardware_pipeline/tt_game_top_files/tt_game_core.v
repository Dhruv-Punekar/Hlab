`timescale 1ns / 1ps

// ============================================================================
// Module: tt_game_core   (pixel clock domain, 25.175 MHz)
//         Table-tennis gameplay: serve, rally, timing window, scoring,
//         ball flight and racket swing animation.
//
// Runs one game step per video frame (frame_tick = start of VGA line 440, the
// first line the framebuffer is no longer read). A step is a short micro-
// sequence of ~30 clocks (1.2 us), so every operation gets its own clock and
// the logic between registers stays shallow (40 ns period, large slack).
// At the end of the step the poses of the three moving objects are published
// on pose_* (registered) and req_toggle flips one clock later; the render
// clock domain captures pose_* when it sees the toggle (the bus is stable
// for the whole frame, so no multi-bit synchroniser is needed).
//
// Bit-exact reference: mem_files/tt_game/tt_game_model.py (class Game). The
// micro-steps below follow its step() in the same order.
//
// Players: 0 = P1, near end (z > 0), red racket.  1 = P2, far end, black.
// Buttons: btn_fore[p], btn_back[p] = press events since the last frame
//          (from tt_buttons; consumed with ev_clear). The server serves with
//          either of his buttons; the receiver returns with either.
// Hit rule: the receiver's press is accepted when the ball is TT_W_LO ..
//          TT_W_HI frames from the hit point (tta); the swing's contact frame
//          (TT_SW_CONTACT) then lands within -TT_HIT_LATE..+TT_HIT_EARLY frames
//          of the ball's arrival and the ball goes back. Press too early or
//          too late (or not at all) -> the ball flies past -> point to the
//          other player.
// Rules:   serve bounces on the server's half, then the receiver's half; a
//          return bounces once on the opponent's half. Serve changes every
//          2 points. One game = first to 11 points.
// Reach:   a press in the window only hits if the racket base is within
//          TT_REACH of the ball's x; rackets follow the ball sideways at a
//          limited speed (TT_TRACK_STEP) within +-TT_BASE_LIM, so wide balls
//          can be winners.
// Tilt:    tilt_en (P1 tilt mode): P1's shots land at x = tilt_x (board tilt),
//          and P1's racket follows the ball fast (TT_TRACK_FAST).
// auto[p]: computer player (switch): serves after TT_SERVE_AUTO frames,
//          returns with small timing jitter, misses 1 ball in 8.
//
// Units: ball state Q16.16 world units (1 unit = 10 cm); poses Q8.8.
// ============================================================================
module tt_game_core (
    input  wire        clk,            // pixel clock
    input  wire        rst_n,          // synchronous to clk (async assert)
    input  wire        frame_tick,     // 1-clock pulse per frame (VGA line 440)

    input  wire [1:0]  btn_fore,       // press events [player]
    input  wire [1:0]  btn_back,
    input  wire        btn_new,        // new game
    input  wire [1:0]  auto_play,      // computer plays player p
    input  wire        tilt_en,        // P1 aims with the board tilt (accelerometer)
    input  wire [15:0] tilt_x,         // P1 aim: bounce x, Q8.8, clamped to +-TT_TILT_MAX
    output reg         ev_clear,       // events consumed (1 clock)
    output wire [4:0]  ev_mask,        // which events were sampled {new, back[1:0], fore[1:0]}

    // poses, Q8.8 world units, yaw 0..63 (5.625 deg), vis
    output reg  [15:0] rp_x, rp_y, rp_z,   // racket P1 (near)
    output reg  [5:0]  rp_yaw,
    output reg  [15:0] ro_x, ro_y, ro_z,   // racket P2 (far)
    output reg  [5:0]  ro_yaw,
    output reg  [15:0] bl_x, bl_y, bl_z,   // ball
    output reg  [5:0]  bl_yaw,
    output reg         bl_vis,
    output reg         req_toggle,     // flips once per published frame

    // scoreboard
    output reg  [6:0]  score0,
    output reg  [6:0]  score1,
    output reg         server,
    output reg  [2:0]  state,
    output reg         winner,
    output reg         step_busy,      // 1 while a step is running (debug)

    // sound events (1-clock pulses)
    output reg         snd_hit,        // a racket hits the ball (serve or return)
    output reg         snd_bounce,     // the ball bounces on the table
    output reg         snd_point       // a point is scored
);

`include "tt_game_map.vh"

    // ------------------------------------------------------------------------
    // game states / arc kinds
    // ------------------------------------------------------------------------
    localparam S_SERVE = 3'd0, S_SERVING = 3'd1, S_FLIGHT = 3'd2, S_POINT = 3'd3, S_OVER = 3'd4;
    localparam A_NONE = 3'd0, A_SERVE1 = 3'd1, A_SERVE2 = 3'd2, A_RALLY = 3'd3, A_TOHIT = 3'd4, A_FREE = 3'd5;

    // micro-steps
    localparam U_IDLE = 4'd0, U_IN = 4'd1, U_ST = 4'd2, U_FB = 4'd3, U_PH = 4'd4,
               U_LD = 4'd5, U_LM = 4'd6, U_LW = 4'd7, U_D = 4'd8, U_HOLD = 4'd9,
               U_TRACK = 4'd10, U_PUB = 4'd11, U_TOG = 4'd12;

    reg [3:0] u;
    reg [3:0] u_ret;                   // micro-step after a launch

    // ------------------------------------------------------------------------
    // game registers
    // ------------------------------------------------------------------------
    reg  [15:0] lfsr;
    reg         first_server;
    reg  [7:0]  wait_cnt;
    reg  [1:0]  sw_act, sw_type, contact;
    reg  [4:0]  sw_frame0, sw_frame1;
    reg  signed [15:0] base_x0, base_x1;          // Q8.8
    reg         hitter, recv;
    reg  signed [7:0]  tta;                       // frames to arrival at the hit point
    reg         swung, recv_ok, miss;
    reg  [5:0]  ai_t;
    reg  signed [31:0] bx, hx;                    // Q16.16
    reg  [2:0]  arc;
    reg  [5:0]  arc_left;
    reg  signed [31:0] px, py, pz, vx, vy, vz, g; // Q16.16
    reg  signed [31:0] tx, ty, tz;                // arc target
    reg  [5:0]  yaw;
    reg         vis;
    reg  signed [31:0] bob;
    reg  [4:0]  bob_cnt;
    reg         bob_dir;

    // launch unit (one shared multiplier, one axis per 3 clocks)
    reg  [2:0]  lk;                               // arc kind being launched
    reg  [1:0]  lax;                              // axis 0 x, 1 y, 2 z
    reg  signed [32:0] l_d;                       // target - position
    reg  signed [16:0] l_rn;                      // 65536/N (positive)
    reg  signed [49:0] l_prod;

    // sampled events
    reg  [1:0] ev_f, ev_b;
    reg        ev_n;
    reg        t_en;                   // tilt inputs sampled at the start of the step
    reg signed [15:0] t_x;
    assign ev_mask = {ev_n, ev_b, ev_f};

    // ------------------------------------------------------------------------
    // constant tables
    // ------------------------------------------------------------------------
    function [5:0] arc_n;  input [2:0] k;  begin
        case (k)
            A_SERVE1: arc_n = TT_N_SERVE1;  A_SERVE2: arc_n = TT_N_SERVE2;
            A_RALLY:  arc_n = TT_N_RALLY;   default:  arc_n = TT_N_TOHIT;
        endcase end endfunction
    function [15:0] arc_rn;  input [2:0] k;  begin
        case (k)
            A_SERVE1: arc_rn = TT_RN_SERVE1;  A_SERVE2: arc_rn = TT_RN_SERVE2;
            A_RALLY:  arc_rn = TT_RN_RALLY;   default:  arc_rn = TT_RN_TOHIT;
        endcase end endfunction
    function signed [31:0] arc_g;  input [2:0] k;  begin
        case (k)
            A_SERVE1: arc_g = TT_G_SERVE1;  A_SERVE2: arc_g = TT_G_SERVE2;
            A_RALLY:  arc_g = TT_G_RALLY;   default:  arc_g = TT_G_TOHIT;
        endcase end endfunction
    function signed [31:0] arc_gh;  input [2:0] k;  begin
        case (k)
            A_SERVE1: arc_gh = TT_GH_SERVE1;  A_SERVE2: arc_gh = TT_GH_SERVE2;
            A_RALLY:  arc_gh = TT_GH_RALLY;   default:  arc_gh = TT_GH_TOHIT;
        endcase end endfunction

    localparam [7:0] TTA_SERVE = TT_N_SERVE1 + TT_N_SERVE2 + TT_N_TOHIT;
    localparam [7:0] TTA_RALLY = TT_N_RALLY + TT_N_TOHIT;

    // signed +-v by player side (+ near / - far)
    function signed [31:0] side_z;  input p;  input signed [31:0] v;  begin
        side_z = p ? -v : v;
    end endfunction

    // ------------------------------------------------------------------------
    // combinational helpers (shallow; every use is registered)
    // ------------------------------------------------------------------------
    wire [15:0] lfsr_next = {1'b0, lfsr[15:1]} ^ (lfsr[0] ? TT_LFSR_TAPS : 16'h0000);

    // pick_lateral() from the (already stepped) lfsr
    wire signed [31:0] pl_bx   = tt_bx(lfsr[2:0]);
    wire signed [31:0] pl_hx   = tt_hx(lfsr[4:0]);
    wire        [5:0]  pl_ai_t = (lfsr[10:8] == 3'd0) ? TT_AI_MISS_T[5:0] : tt_ai_t(lfsr[7:5]);

    // swings advanced (U_IN)
    wire [4:0] sf0_inc = sw_frame0 + 5'd1;
    wire [4:0] sf1_inc = sw_frame1 + 5'd1;
    wire       sw0_end = sw_act[0] && (sf0_inc == TT_SW_LEN);
    wire       sw1_end = sw_act[1] && (sf1_inc == TT_SW_LEN);

    // presses of player p: {hit, type}; forehand has priority
    wire [1:0] pr0 = ev_f[0] ? 2'b10 : ev_b[0] ? 2'b11 : 2'b00;
    wire [1:0] pr1 = ev_f[1] ? 2'b10 : ev_b[1] ? 2'b11 : 2'b00;

    // server / receiver views
    wire [1:0] pr_srv  = server ? pr1 : pr0;
    wire [1:0] pr_recv = recv   ? pr1 : pr0;
    wire       act_srv  = server ? sw_act[1] : sw_act[0];
    wire       act_recv = recv   ? sw_act[1] : sw_act[0];
    wire       con_srv  = server ? contact[1] : contact[0];
    wire       con_recv = recv   ? contact[1] : contact[0];

    // computer press for the receiver: forehand when the ball comes to its right
    wire       ai_fore  = recv ? (hx <= 0) : (hx >= 0);
    wire       ai_press = (recv ? auto_play[1] : auto_play[0]) && !tta[7] && (tta[5:0] == ai_t) && (tta[6] == 1'b0);

    // score / serve rotation (U_ST, S_POINT)
    wire [7:0] n_pts  = {1'b0, score0} + {1'b0, score1};
    // one game = first to TT_WIN_POINTS (11); serve changes every 2 points
    wire       game_won = (score0 >= TT_WIN_POINTS) || (score1 >= TT_WIN_POINTS);
    wire       next_server = first_server ^ n_pts[1];

    // tilt aiming (P1): bounce at the tilt x, hit x continues the sideways
    // direction: hx = bx + (bx - px) * 5/16
    wire signed [31:0] tbx  = {{8{t_x[15]}}, t_x, 8'd0};
    wire signed [32:0] tdx  = tbx - px;
    wire signed [35:0] tdx5 = (tdx <<< 2) + tdx;
    wire signed [31:0] thx  = tbx + (tdx5 >>> 4);
    wire       tilt_srv = t_en && (server == 1'b0);
    wire       tilt_rcv = t_en && (recv   == 1'b0);

    // reach: a press in the window only hits if the racket is near the ball
    wire signed [16:0] reach_d  = (recv ? base_x1 : base_x0) - hx[23:8];
    wire               reach_ok = (reach_d <= TT_REACH) && (reach_d >= -TT_REACH);

    // racket base x tracking target (U_TRACK)
    wire signed [15:0] hx_q8   = hx[23:8];
    wire signed [15:0] hx_c    = (hx_q8 > TT_BASE_LIM) ? TT_BASE_LIM : (hx_q8 < -TT_BASE_LIM) ? -TT_BASE_LIM : hx_q8;
    wire signed [15:0] trk_t0  = (state == S_FLIGHT && recv == 1'b0 && !miss) ? hx_c : 16'sd0;
    wire signed [15:0] trk_t1  = (state == S_FLIGHT && recv == 1'b1 && !miss) ? hx_c : 16'sd0;
    wire signed [16:0] trk_d0  = trk_t0 - base_x0;
    wire signed [16:0] trk_d1  = trk_t1 - base_x1;
    wire signed [15:0] trk_s0  = t_en ? TT_TRACK_FAST : TT_TRACK_STEP;   // P1 in tilt mode: fast

    // swing table lookups (U_PUB)
    wire [53:0] sw_e0 = tt_swing(sw_act[0] ? sw_type[0] : 1'b0, sw_act[0] ? sw_frame0 : 5'd0);
    wire [53:0] sw_e1 = tt_swing(sw_act[1] ? sw_type[1] : 1'b0, sw_act[1] ? sw_frame1 : 5'd0);

    // new game (toggle the first server)
    task new_game;
        begin
            first_server <= ~first_server;
            server       <= ~first_server;
            score0 <= 7'd0;  score1 <= 7'd0;
            winner <= 1'b0;
            state  <= S_SERVE;
            wait_cnt <= 8'd0;
            sw_act <= 2'b00;  sw_type <= 2'b00;  sw_frame0 <= 5'd0;  sw_frame1 <= 5'd0;
            base_x0 <= 16'sd0;  base_x1 <= 16'sd0;
            hitter <= 1'b0;  recv <= 1'b1;
            tta <= 8'sd0;  swung <= 1'b0;  recv_ok <= 1'b0;  miss <= 1'b0;
            ai_t <= 6'd0;  bx <= 32'sd0;  hx <= 32'sd0;
            arc <= A_NONE;  arc_left <= 6'd0;
            vx <= 32'sd0;  vy <= 32'sd0;  vz <= 32'sd0;  g <= 32'sd0;
            tx <= 32'sd0;  ty <= 32'sd0;  tz <= 32'sd0;
            yaw <= 6'd0;  vis <= 1'b1;
            bob <= 32'sd0;  bob_cnt <= 5'd0;  bob_dir <= 1'b0;
            px <= 32'sd0;  py <= TT_HOLD_Y;  pz <= first_server ? TT_Z_HIT : -TT_Z_HIT;  // side of the new server
        end
    endtask

    // ------------------------------------------------------------------------
    // the step sequencer
    // ------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            u <= U_IDLE;  u_ret <= U_IDLE;
            lfsr <= TT_LFSR_SEED;
            first_server <= 1'b0;  server <= 1'b0;
            score0 <= 7'd0;  score1 <= 7'd0;  winner <= 1'b0;
            state <= S_SERVE;  wait_cnt <= 8'd0;
            sw_act <= 2'b00;  sw_type <= 2'b00;  sw_frame0 <= 5'd0;  sw_frame1 <= 5'd0;  contact <= 2'b00;
            base_x0 <= 16'sd0;  base_x1 <= 16'sd0;
            hitter <= 1'b0;  recv <= 1'b1;
            tta <= 8'sd0;  swung <= 1'b0;  recv_ok <= 1'b0;  miss <= 1'b0;
            ai_t <= 6'd0;  bx <= 32'sd0;  hx <= 32'sd0;
            arc <= A_NONE;  arc_left <= 6'd0;
            px <= 32'sd0;  py <= TT_HOLD_Y;  pz <= TT_Z_HIT;
            vx <= 32'sd0;  vy <= 32'sd0;  vz <= 32'sd0;  g <= 32'sd0;
            tx <= 32'sd0;  ty <= 32'sd0;  tz <= 32'sd0;
            yaw <= 6'd0;  vis <= 1'b1;
            bob <= 32'sd0;  bob_cnt <= 5'd0;  bob_dir <= 1'b0;
            lk <= A_NONE;  lax <= 2'd0;  l_d <= 33'sd0;  l_rn <= 17'sd0;  l_prod <= 50'sd0;
            ev_f <= 2'b00;  ev_b <= 2'b00;  ev_n <= 1'b0;  ev_clear <= 1'b0;
            t_en <= 1'b0;  t_x <= 16'sd0;
            snd_hit <= 1'b0;  snd_bounce <= 1'b0;  snd_point <= 1'b0;
            rp_x <= 16'd0; rp_y <= 16'd0; rp_z <= 16'd0; rp_yaw <= 6'd0;
            ro_x <= 16'd0; ro_y <= 16'd0; ro_z <= 16'd0; ro_yaw <= 6'd0;
            bl_x <= 16'd0; bl_y <= 16'd0; bl_z <= 16'd0; bl_yaw <= 6'd0; bl_vis <= 1'b0;
            req_toggle <= 1'b0;  step_busy <= 1'b0;
        end else begin
            ev_clear <= 1'b0;
            snd_hit <= 1'b0;  snd_bounce <= 1'b0;  snd_point <= 1'b0;
            case (u)
            // ---------------------------------------------------------------
            U_IDLE: if (frame_tick) begin
                ev_f <= btn_fore;  ev_b <= btn_back;  ev_n <= btn_new;
                t_en <= tilt_en;   t_x <= tilt_x;
                ev_clear  <= 1'b1;
                step_busy <= 1'b1;
                u <= U_IN;
            end

            // ---------------------------------------------------------------
            // lfsr, new game, swings advance + contact flags
            U_IN: begin
                lfsr <= lfsr_next;
                if (ev_n) begin
                    new_game;
                    u <= U_PUB;
                end else begin
                    sw_act[0] <= sw_act[0] && !sw0_end;
                    sw_act[1] <= sw_act[1] && !sw1_end;
                    if (sw_act[0]) sw_frame0 <= sw0_end ? 5'd0 : sf0_inc;
                    if (sw_act[1]) sw_frame1 <= sw1_end ? 5'd0 : sf1_inc;
                    contact[0] <= sw_act[0] && !sw0_end && (sf0_inc == TT_SW_CONTACT);
                    contact[1] <= sw_act[1] && !sw1_end && (sf1_inc == TT_SW_CONTACT);
                    u <= U_ST;
                end
            end

            // ---------------------------------------------------------------
            U_ST: begin
                case (state)
                S_SERVE: begin
                    vis <= 1'b1;
                    bob <= bob_dir ? bob - TT_BOB_STEP : bob + TT_BOB_STEP;
                    if (bob_cnt == TT_BOB_HALF - 1) begin
                        bob_cnt <= 5'd0;  bob_dir <= ~bob_dir;
                    end else
                        bob_cnt <= bob_cnt + 5'd1;
                    if (wait_cnt != 8'd255) wait_cnt <= wait_cnt + 8'd1;
                    // serve: server's button, or the computer after TT_SERVE_AUTO frames
                    if (((server ? auto_play[1] : auto_play[0]) && wait_cnt + 8'd1 >= TT_SERVE_AUTO) || pr_srv[1]) begin
                        if (!act_srv) begin
                            if (server) begin
                                sw_act[1] <= 1'b1;  sw_frame1 <= 5'd0;
                                sw_type[1] <= ((auto_play[1] && wait_cnt + 8'd1 >= TT_SERVE_AUTO) ? lfsr[11] : pr_srv[0]);
                            end else begin
                                sw_act[0] <= 1'b1;  sw_frame0 <= 5'd0;
                                sw_type[0] <= ((auto_play[0] && wait_cnt + 8'd1 >= TT_SERVE_AUTO) ? lfsr[11] : pr_srv[0]);
                            end
                            state <= S_SERVING;
                        end
                    end
                    u <= U_HOLD;
                end

                S_SERVING: begin
                    if (con_srv) begin
                        bx <= tilt_srv ? tbx : pl_bx;  hx <= tilt_srv ? thx : pl_hx;  ai_t <= pl_ai_t;
                        snd_hit <= 1'b1;
                        hitter <= server;  recv <= ~server;
                        tta <= TTA_SERVE;
                        swung <= 1'b0;  recv_ok <= 1'b0;  miss <= 1'b0;
                        lk <= A_SERVE1;
                        tx <= px;  ty <= TT_Y_BOUNCE;  tz <= side_z(server, TT_Z_SB1);
                        state <= S_FLIGHT;
                        lax <= 2'd0;
                        u <= U_LD;  u_ret <= U_TRACK;
                    end else
                        u <= U_TRACK;
                end

                S_FLIGHT: begin
                    // a) receiver input / timing window
                    if (!miss && !swung) begin
                        if ((pr_recv[1] || ai_press) && !act_recv) begin
                            if (recv) begin
                                sw_act[1] <= 1'b1;  sw_frame1 <= 5'd0;
                                sw_type[1] <= ai_press ? !ai_fore : pr_recv[0];
                            end else begin
                                sw_act[0] <= 1'b1;  sw_frame0 <= 5'd0;
                                sw_type[0] <= ai_press ? !ai_fore : pr_recv[0];
                            end
                            swung <= 1'b1;
                            if (tta >= TT_W_LO && tta <= TT_W_HI && reach_ok) recv_ok <= 1'b1;
                            else                                  miss    <= 1'b1;
                        end else if (tta < TT_W_LO)
                            miss <= 1'b1;
                    end
                    u <= U_FB;
                end

                S_POINT: begin
                    vis <= 1'b0;
                    wait_cnt <= wait_cnt + 8'd1;
                    if (wait_cnt + 8'd1 == TT_POINT_PAUSE) begin
                        if (game_won) begin
                            winner <= (score0 > score1) ? 1'b0 : 1'b1;
                            state  <= S_OVER;
                            u <= U_TRACK;
                        end else begin
                            server <= next_server;
                            state  <= S_SERVE;
                            bob <= 32'sd0;  bob_cnt <= 5'd0;  bob_dir <= 1'b0;
                            u <= U_HOLD;
                        end
                        wait_cnt <= 8'd0;
                    end else
                        u <= U_TRACK;
                end

                S_OVER: begin
                    vis <= 1'b0;
                    if (wait_cnt != 8'd255) wait_cnt <= wait_cnt + 8'd1;
                    if (auto_play[0] && auto_play[1] && wait_cnt + 8'd1 == TT_OVER_PAUSE) begin
                        new_game;
                        u <= U_PUB;
                    end else
                        u <= U_TRACK;
                end

                default: u <= U_TRACK;
                endcase
            end

            // ---------------------------------------------------------------
            // b) contact by the receiver -> return the ball
            U_FB: begin
                if (con_recv && recv_ok) begin
                    bx <= tilt_rcv ? tbx : pl_bx;  hx <= tilt_rcv ? thx : pl_hx;  ai_t <= pl_ai_t;
                    snd_hit <= 1'b1;
                    hitter <= recv;  recv <= ~recv;
                    tta <= TTA_RALLY;
                    swung <= 1'b0;  recv_ok <= 1'b0;  miss <= 1'b0;
                    lk <= A_RALLY;
                    tx <= tilt_rcv ? tbx : pl_bx;  ty <= TT_Y_BOUNCE;  tz <= side_z(~recv, TT_Z_RB);
                    lax <= 2'd0;
                    u <= U_LD;  u_ret <= U_D;
                end else
                    u <= U_PH;
            end

            // ---------------------------------------------------------------
            // c) ball physics, arc ends
            U_PH: begin
                vy  <= vy - g;
                yaw <= yaw + TT_SPIN;
                if (tta != -8'sd128) tta <= tta - 8'sd1;
                u <= U_D;
                if (arc != A_FREE && arc_left == 6'd1) begin
                    arc_left <= 6'd0;
                    case (arc)
                    A_SERVE1: begin
                        snd_bounce <= 1'b1;
                        px <= tx;  py <= ty;  pz <= tz;
                        lk <= A_SERVE2;
                        tx <= bx;  ty <= TT_Y_BOUNCE;  tz <= side_z(recv, TT_Z_SB2);
                        lax <= 2'd0;  u <= U_LD;  u_ret <= U_D;
                    end
                    A_SERVE2, A_RALLY: begin
                        snd_bounce <= 1'b1;
                        px <= tx;  py <= ty;  pz <= tz;
                        lk <= A_TOHIT;
                        tx <= hx;  ty <= TT_Y_HIT;  tz <= side_z(recv, TT_Z_HIT);
                        lax <= 2'd0;  u <= U_LD;  u_ret <= U_D;
                    end
                    default: begin                       // TOHIT ends: free flight
                        px <= px + vx;  py <= py + vy;  pz <= pz + vz;
                        arc <= A_FREE;
                    end
                    endcase
                end else begin
                    px <= px + vx;  py <= py + vy;  pz <= pz + vz;
                    if (arc != A_FREE) arc_left <= arc_left - 6'd1;
                end
            end

            // ---------------------------------------------------------------
            // launch: v = (target - p) * 65536/N >> 16 (+ g(N-1)/2 for y)
            U_LD: begin
                case (lax)
                    2'd0:    l_d <= tx - px;
                    2'd1:    l_d <= ty - py;
                    default: l_d <= tz - pz;
                endcase
                l_rn <= {1'b0, arc_rn(lk)};
                u <= U_LM;
            end
            U_LM: begin
                l_prod <= l_d * l_rn;
                u <= U_LW;
            end
            U_LW: begin
                case (lax)
                    2'd0:    vx <= l_prod[47:16];
                    2'd1:    vy <= l_prod[47:16] + arc_gh(lk);
                    default: vz <= l_prod[47:16];
                endcase
                if (lax == 2'd2) begin
                    g <= arc_g(lk);
                    arc <= lk;
                    arc_left <= arc_n(lk);
                    u <= u_ret;
                end else begin
                    lax <= lax + 2'd1;
                    u <= U_LD;
                end
            end

            // ---------------------------------------------------------------
            // d) ball gone past the racket -> point to the hitter
            U_D: begin
                if (miss && tta <= -TT_MISS_FRAMES) begin
                    vis <= 1'b0;
                    snd_point <= 1'b1;
                    if (hitter) score1 <= score1 + 7'd1;
                    else        score0 <= score0 + 7'd1;
                    state <= S_POINT;
                    wait_cnt <= 8'd0;
                end
                u <= U_TRACK;
            end

            // ball held by the server (bobbing)
            U_HOLD: begin
                px <= server ? {{8{base_x1[15]}}, base_x1, 8'd0} : {{8{base_x0[15]}}, base_x0, 8'd0};
                py <= TT_HOLD_Y + bob;
                pz <= server ? -TT_Z_HIT : TT_Z_HIT;
                u <= U_TRACK;
            end

            // idle rackets follow the incoming ball sideways
            U_TRACK: begin
                if (!sw_act[0]) begin
                    if (trk_d0 <= trk_s0 && trk_d0 >= -trk_s0) base_x0 <= trk_t0;
                    else if (trk_d0 > 0) base_x0 <= base_x0 + trk_s0;
                    else                 base_x0 <= base_x0 - trk_s0;
                end
                if (!sw_act[1]) begin
                    if (trk_d1 <= TT_TRACK_STEP && trk_d1 >= -TT_TRACK_STEP) base_x1 <= trk_t1;
                    else if (trk_d1 > 0) base_x1 <= base_x1 + TT_TRACK_STEP;
                    else                 base_x1 <= base_x1 - TT_TRACK_STEP;
                end
                u <= U_PUB;
            end

            // ---------------------------------------------------------------
            // publish the poses (far racket = near table mirrored in x and z)
            U_PUB: begin
                rp_x <= base_x0 + sw_e0[53:38];  rp_y <= sw_e0[37:22];  rp_z <= sw_e0[21:6];  rp_yaw <= sw_e0[5:0];
                ro_x <= base_x1 - sw_e1[53:38];  ro_y <= sw_e1[37:22];  ro_z <= -sw_e1[21:6]; ro_yaw <= sw_e1[5:0];
                bl_x <= px[23:8];  bl_y <= py[23:8];  bl_z <= pz[23:8];  bl_yaw <= yaw;  bl_vis <= vis;
                u <= U_TOG;
            end
            U_TOG: begin
                req_toggle <= ~req_toggle;          // one clock after the bus update
                step_busy  <= 1'b0;
                u <= U_IDLE;
            end

            default: u <= U_IDLE;
            endcase
        end
    end

endmodule

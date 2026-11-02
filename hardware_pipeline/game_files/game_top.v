`timescale 1ns / 1ps
// ============================================================================
// Module: game_top   (board top, Nexys A7-100T, xc7a100tcsg324-1)
//         Interactive cube "game": buttons, switches and board tilt control
//         the cube live.
//
//   CLK100MHZ -> MMCME2_BASE -> clk_render 100 MHz, clk_vga 25.175 MHz
//   CPU_RESETN (red button)  -> full reset (render + VGA), back to start pose
//
// DOUBLE BUFFERING (2026-11-02)
//   The pipeline runs with DOUBLE_BUF = 1: two framebuffers. While the
//   monitor shows one ("front"), the next frame is drawn into the other
//   ("back"). When the back buffer is finished, buffer_swap exchanges the two
//   at the start of the next vertical blanking, so the monitor only ever shows
//   complete pictures.
//
//   SW11 down = DOUBLE buffer (normal)
//   SW11 up   = SINGLE buffer DEMO: draw straight into the buffer on the
//               screen, as fast as possible, without waiting for the blank
//               time -> you SEE the clear (black flashes) and half-drawn
//               frames (tearing). Use it to show why double buffering exists.
//
//   Frame loop (render clock):
//     1. double: wait for the next VGA frame tick (60 per second)
//        single: start at once
//     2. game_ctrl updates the cube state (only on a VGA frame tick, so the
//        game speed is the same in both modes)
//     3. matrix_builder builds the matrix (6 clocks)
//     4. render domain held in reset, 16 matrix words written, reset released
//        -> z-buffer + draw buffer clear, cube render
//     5. wait for render_done
//     6. double: ask buffer_swap to show the new frame, wait for swap_done
//     back to 1
//
// 7-SEGMENT DISPLAY (perf_monitor + seg7_driver)
//   SW10 down: [d|S] FPS  render-time(ms)    e.g.  "d  60 1.392"
//   SW10 up  : [P]   pixels drawn per frame  e.g.  "P    23456"
//
// LEDs: LED0 auto-spin  LED1 move mode  LED2 zoom mode  LED4:3 speed
//       LED7:5 colour theme  LED10 7-seg page  LED11 double buffer on
//       LED12 move/zoom limit reached  LED13 heartbeat (frames being drawn)
//       LED14 accelerometer found  LED15 tilt control on
//
// Files: this file, game_ctrl.v, matrix_builder.v, btn_debounce.v,
//        adxl362_reader.v, color_theme.v, buffer_swap.v, perf_monitor.v,
//        seg7_driver.v + the pipeline modules and mem files (vertices.mem,
//        matrix.mem, recip_lut_q1_17.mem, palette.mem).
// Constraints: game_top.xdc.
// ============================================================================
module game_top #(
    parameter NUM_VERTICES   = 36,
    parameter VERTEX_DEPTH   = 64,
    parameter RECIP_LUT_FILE = "recip_lut_q1_17.mem",
    parameter PALETTE_FILE   = "palette.mem",
    parameter Z_FRAC_OUT     = 9,
    parameter CULL_MODE      = 1,
    parameter FRAME_HOLD     = 1,        // VGA frames per game update (1 = 60/s)
    parameter TRIGGER_MODE   = 0,        // 0: VGA frame tick (board)  1: free-running (simulation)
    // input timing (board values; the testbench makes them shorter)
    parameter DEBOUNCE_CLKS  = 1000000,  // 10 ms
    parameter ACL_HALF_CLKS  = 50,       // 1 MHz SPI clock
    parameter ACL_PWR_CLKS   = 1000000,  // 10 ms
    parameter ACL_RST_CLKS   = 1000000,  // 10 ms
    parameter ACL_POLL_CLKS  = 1000000,  // 100 samples per second
    // 7-segment / performance counters (board values)
    parameter FPS_WINDOW_CLKS = 100_000_000, // 1 s
    parameter DISP_CLKS       = 25_000_000,  // display refresh 0.25 s
    parameter SEG_REFRESH_BITS = 17          // 1.3 ms per digit
)(
    input  wire        CLK100MHZ,
    input  wire        CPU_RESETN,       // red button, active low

    input  wire        BTNC, BTNU, BTNL, BTNR, BTND,
    input  wire [15:0] SW,
    output wire [15:0] LED,

    input  wire        ACL_MISO,
    output wire        ACL_MOSI,
    output wire        ACL_SCLK,
    output wire        ACL_CSN,

    output wire [3:0]  VGA_R,
    output wire [3:0]  VGA_G,
    output wire [3:0]  VGA_B,
    output wire        VGA_HS,
    output wire        VGA_VS,

    output wire [6:0]  SEG,              // CA..CG, active low
    output wire        DP,               // decimal point, active low
    output wire [7:0]  AN                // digit enables, active low
);
    // ========================================================================
    // 1) CLOCKING (unchanged)
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
    // 2) GLOBAL RESET (unchanged)
    // ========================================================================
    (* ASYNC_REG = "TRUE" *) reg [1:0] btn_sync = 2'b00;
    always @(posedge clk_render or negedge mmcm_locked)
        if (!mmcm_locked) btn_sync <= 2'b00;
        else              btn_sync <= {btn_sync[0], CPU_RESETN};
    // TIMING FIX: this reset goes to thousands of flip-flops. On normal
    // routing it arrived late (reset "recovery" failure). It now comes from
    // one more flip-flop and is carried on a BUFG (global buffer), which
    // reaches every flip-flop with almost the same, small delay.
    // Active-high inside the BUFG so the flip-flops' CLR pins are driven
    // straight from it (no inverter after the buffer).
    reg rst_hi = 1'b1;
    always @(posedge clk_render or negedge mmcm_locked)
        if (!mmcm_locked) rst_hi <= 1'b1;
        else              rst_hi <= ~btn_sync[1];
    wire rst_hi_g;
    BUFG u_bufg_rst (.I(rst_hi), .O(rst_hi_g));
    wire rst_n = ~rst_hi_g;

    // VGA-domain copy of the reset for color_theme and buffer_swap
    (* ASYNC_REG = "TRUE" *) reg [1:0] rst_v_sync = 2'b00;
    always @(posedge clk_vga or negedge rst_n)
        if (!rst_n) rst_v_sync <= 2'b00;
        else        rst_v_sync <= {rst_v_sync[0], 1'b1};
    wire rst_v_n = rst_v_sync[1];

    // ========================================================================
    // 3) INPUTS: buttons + switches debounced, accelerometer (unchanged)
    // ========================================================================
    wire [4:0]  btn;          // {C, U, L, R, D}
    wire [15:0] sw;

    btn_debounce #(.N(5), .STABLE_CLKS(DEBOUNCE_CLKS)) u_db_btn (
        .clk(clk_render), .rst_n(rst_n),
        .raw({BTNC, BTNU, BTNL, BTNR, BTND}), .clean(btn));

    btn_debounce #(.N(16), .STABLE_CLKS(DEBOUNCE_CLKS)) u_db_sw (
        .clk(clk_render), .rst_n(rst_n), .raw(SW), .clean(sw));

    wire signed [11:0] acc_x, acc_y, acc_z;
    wire               acc_valid, acc_id_ok;

    adxl362_reader #(
        .HALF_CLKS(ACL_HALF_CLKS), .POWERUP_CLKS(ACL_PWR_CLKS),
        .RESET_CLKS(ACL_RST_CLKS), .POLL_CLKS(ACL_POLL_CLKS)
    ) u_acl (
        .clk(clk_render), .rst_n(rst_n),
        .sclk(ACL_SCLK), .mosi(ACL_MOSI), .miso(ACL_MISO), .cs_n(ACL_CSN),
        .acc_x(acc_x), .acc_y(acc_y), .acc_z(acc_z),
        .sample_valid(acc_valid), .id_ok(acc_id_ok));

    wire sw_single = sw[11];      // SW11 up = single-buffer demo
    wire sw_page   = sw[10];      // SW10 up = pixels page on the 7-segment

    // ========================================================================
    // 4) GAME STATE + MATRIX (unchanged)
    // ========================================================================
    reg                tick;
    wire        [9:0]  yaw, pitch, pitch_eff, roll_eff;
    wire signed [15:0] tx, ty, cam_dist;
    wire               at_limit;
    wire signed [9:0]  tilt_pitch, tilt_roll;

    game_ctrl u_game (
        .clk(clk_render), .rst_n(rst_n), .tick(tick),
        .btn_c(btn[4]), .btn_u(btn[3]), .btn_l(btn[2]), .btn_r(btn[1]), .btn_d(btn[0]),
        .sw(sw), .acc_x(acc_x), .acc_y(acc_y), .acc_valid(acc_valid),
        .yaw(yaw), .pitch(pitch), .pitch_eff(pitch_eff), .roll_eff(roll_eff),
        .tx(tx), .ty(ty), .cam_dist(cam_dist), .at_limit(at_limit),
        .tilt_pitch(tilt_pitch), .tilt_roll(tilt_roll));

    reg         mb_start;
    wire        mb_done;
    reg  [3:0]  ld_cnt;
    wire [31:0] mb_word;

    matrix_builder u_mb (
        .clk(clk_render), .start(mb_start),
        .yaw(yaw), .pitch(pitch_eff), .roll(roll_eff),
        .tx(tx), .ty(ty), .cam_dist(cam_dist),
        .done(mb_done), .rd_addr(ld_cnt), .rd_data(mb_word));

    // ========================================================================
    // 5) THE PIPELINE (double-buffered configuration)
    // ========================================================================
    reg         rst_render_n;
    reg         mat_we;
    reg  [3:0]  mat_addr;
    reg  [31:0] mat_data;
    wire        render_done;
    wire        stat_pixel;
    wire [3:0]  pipe_r, pipe_g, pipe_b;
    wire        pipe_hs, pipe_vs;

    reg         mode_dbl;     // 1 = the current frame uses double buffering
    wire        front_r;      // buffer on the screen (render-side copy)
    wire        show_buf;     // buffer on the screen (VGA clock)
    // draw into the hidden buffer (double) or straight into the shown one (single)
    wire        draw_buf = mode_dbl ? ~front_r : front_r;

    hardware_pipeline #(
        .NUM_VERTICES   (NUM_VERTICES),
        .VERTEX_DEPTH   (VERTEX_DEPTH),
        .RECIP_LUT_FILE (RECIP_LUT_FILE),
        .LOOP_RENDER    (0),
        .Z_FRAC_OUT     (Z_FRAC_OUT),
        .CULL_MODE      (CULL_MODE),
        .DOUBLE_BUF     (1),
        .INDEX_W        (4),
        .PALETTE_FILE   (PALETTE_FILE)
    ) u_pipeline (
        .clk          (clk_render),
        .clk_vga      (clk_vga),
        .rst_n        (rst_n),
        .rst_render_n (rst_render_n),
        .mat_we       (mat_we),
        .mat_addr     (mat_addr),
        .mat_data     (mat_data),
        .draw_buf     (draw_buf),
        .show_buf     (show_buf),
        .pal_we       (1'b0),          // palette writes: reserved for lighting
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
    // 6) BUFFER SWAP (render <-> VGA clock handshake, swap during vblank)
    // ========================================================================
    reg  swap_req;
    wire swap_done, swap_busy;

    buffer_swap u_swap (
        .clk_r     (clk_render),
        .rst_r_n   (rst_n),
        .swap_req  (swap_req),
        .swap_done (swap_done),
        .swap_busy (swap_busy),
        .front_r   (front_r),
        .clk_v     (clk_vga),
        .rst_v_n   (rst_v_n),
        .vsync     (pipe_vs),
        .show_buf  (show_buf)
    );

    // ========================================================================
    // 7) COLOUR THEME on the VGA outputs (unchanged)
    // ========================================================================
    (* ASYNC_REG = "TRUE" *) reg [2:0] theme_s1, theme_s2;
    always @(posedge clk_vga) begin
        theme_s1 <= sw[7:5];
        theme_s2 <= theme_s1;
    end

    color_theme u_theme (
        .clk_pix(clk_vga), .rst_n(rst_v_n), .theme(theme_s2),
        .in_r(pipe_r), .in_g(pipe_g), .in_b(pipe_b), .in_hs(pipe_hs), .in_vs(pipe_vs),
        .out_r(VGA_R), .out_g(VGA_G), .out_b(VGA_B), .out_hs(VGA_HS), .out_vs(VGA_VS));

    // ========================================================================
    // 8) VGA FRAME TICK (same detector as before): pulse at line 440,
    //    once per VGA frame -> game speed reference (60 per second)
    // ========================================================================
    (* ASYNC_REG = "TRUE" *) reg [2:0] vs_s, hs_s;
    reg [9:0] hs_count;
    reg       win_start;

    always @(posedge clk_render or negedge rst_n) begin
        if (!rst_n) begin
            vs_s <= 3'b111; hs_s <= 3'b111;
            hs_count <= 10'd0; win_start <= 1'b0;
        end else begin
            vs_s <= {vs_s[1:0], pipe_vs};
            hs_s <= {hs_s[1:0], pipe_hs};
            win_start <= 1'b0;
            if (vs_s[2] && !vs_s[1])
                hs_count <= 10'd0;
            else if (hs_s[2] && !hs_s[1]) begin
                hs_count <= hs_count + 10'd1;
                if (hs_count + 10'd1 == 10'd476)
                    win_start <= 1'b1;
            end
        end
    end

    reg [7:0] hold_cnt;
    reg       step_req;
    always @(posedge clk_render or negedge rst_n) begin
        if (!rst_n) begin
            hold_cnt <= 8'd0; step_req <= 1'b0;
        end else begin
            step_req <= 1'b0;
            if (win_start) begin
                if (hold_cnt == FRAME_HOLD - 1) begin
                    hold_cnt <= 8'd0; step_req <= 1'b1;
                end else
                    hold_cnt <= hold_cnt + 8'd1;
            end
        end
    end

    // ========================================================================
    // 9) FRAME SEQUENCER
    // ========================================================================
    localparam S_BOOT = 3'd0,   // power-up render (matrix.mem = start pose)
               S_IDLE = 3'd1,   // wait for the next frame start
               S_CALC = 3'd2,   // state updated, build matrix
               S_WAIT = 3'd3,   // wait for matrix_builder
               S_LOAD = 3'd4,   // render domain in reset, write 16 words
               S_REL  = 3'd5,   // release the render reset
               S_RUN  = 3'd6,   // wait for render_done
               S_SWAP = 3'd7;   // double buffer: wait until the swap is done
    reg [2:0]  state;
    reg [31:0] frames_rendered;
    reg [4:0]  wr_cnt;
    reg        tick_pending;    // a VGA frame tick arrived, not used yet
    reg        render_start_p;  // pulse for perf_monitor
    reg        frame_shown_p;   // pulse for perf_monitor

    wire tick_now  = (TRIGGER_MODE == 1) ? 1'b1 : (tick_pending | step_req);
    wire next_dbl  = ~sw_single;
    // double: one new frame per VGA frame tick; single: as fast as possible
    wire go        = (TRIGGER_MODE == 1) ? 1'b1 : (next_dbl ? tick_now : 1'b1);

    always @(posedge clk_render or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_BOOT; rst_render_n <= 1'b1;
            mat_we <= 1'b0; mat_addr <= 4'd0; mat_data <= 32'd0;
            tick <= 1'b0; mb_start <= 1'b0; ld_cnt <= 4'd0; wr_cnt <= 5'd0;
            frames_rendered <= 32'd0;
            mode_dbl <= 1'b1;            // power-up frame is double buffered
            swap_req <= 1'b0; tick_pending <= 1'b0;
            render_start_p <= 1'b0; frame_shown_p <= 1'b0;
        end else begin
            mat_we         <= 1'b0;
            tick           <= 1'b0;
            mb_start       <= 1'b0;
            swap_req       <= 1'b0;
            render_start_p <= 1'b0;
            frame_shown_p  <= 1'b0;
            if (step_req) tick_pending <= 1'b1;

            case (state)
                S_BOOT: if (render_done) begin
                    frames_rendered <= frames_rendered + 32'd1;
                    swap_req <= 1'b1;              // show the power-up picture
                    state    <= S_SWAP;
                end
                S_IDLE: if (go) begin
                    tick         <= tick_now;      // game_ctrl updates on this edge
                    tick_pending <= 1'b0;
                    mode_dbl     <= next_dbl;      // render domain is idle here
                    state        <= S_CALC;
                end
                S_CALC: begin
                    mb_start <= 1'b1;              // state registers are now updated
                    state    <= S_WAIT;
                end
                S_WAIT: if (mb_done) begin
                    rst_render_n <= 1'b0;          // stop + clear the render domain
                    ld_cnt <= 4'd0; wr_cnt <= 5'd0;
                    state  <= S_LOAD;
                end
                S_LOAD: begin
                    mat_we   <= 1'b1;
                    mat_addr <= ld_cnt;
                    mat_data <= mb_word;           // word ld_cnt of the new matrix
                    ld_cnt   <= ld_cnt + 4'd1;
                    wr_cnt   <= wr_cnt + 5'd1;
                    if (wr_cnt == 5'd15) state <= S_REL;
                end
                S_REL: begin                       // last write lands on this edge
                    rst_render_n   <= 1'b1;
                    render_start_p <= 1'b1;
                    state          <= S_RUN;
                end
                S_RUN: if (render_done) begin
                    frames_rendered <= frames_rendered + 32'd1;
                    if (mode_dbl) begin
                        swap_req <= 1'b1;          // show it at the next vblank
                        state    <= S_SWAP;
                    end else begin
                        frame_shown_p <= 1'b1;     // single: it is already on screen
                        state         <= S_IDLE;
                    end
                end
                S_SWAP: if (swap_done) begin
                    frame_shown_p <= 1'b1;
                    state         <= S_IDLE;
                end
                default: state <= S_IDLE;
            endcase
        end
    end

    // ========================================================================
    // 10) PERFORMANCE COUNTERS + 7-SEGMENT DISPLAY
    // ========================================================================
    // render_done is a level; perf_monitor needs its rising edge
    reg render_done_q, stat_pixel_q;
    always @(posedge clk_render or negedge rst_n) begin
        if (!rst_n) begin render_done_q <= 1'b0; stat_pixel_q <= 1'b0; end
        else        begin render_done_q <= render_done; stat_pixel_q <= stat_pixel; end
    end
    wire render_end_p = render_done && !render_done_q;

    wire [39:0] seg_digits;
    wire [7:0]  seg_dp;

    perf_monitor #(
        .WINDOW_CLKS (FPS_WINDOW_CLKS),
        .DISP_CLKS   (DISP_CLKS),
        .US_CLKS     (100)
    ) u_perf (
        .clk          (clk_render),
        .rst_n        (rst_n),
        .frame_shown  (frame_shown_p),
        .render_start (render_start_p),
        .render_end   (render_end_p),
        .pixel        (stat_pixel_q),
        .mode_dbl     (mode_dbl),
        .page         (sw_page),
        .digits       (seg_digits),
        .dp           (seg_dp)
    );

    seg7_driver #(.REFRESH_BITS(SEG_REFRESH_BITS)) u_seg7 (
        .clk    (clk_render),
        .rst_n  (rst_n),
        .digits (seg_digits),
        .dp     (seg_dp),
        .an     (AN),
        .seg    (SEG),
        .dp_n   (DP)
    );

    // ========================================================================
    // 11) LEDS
    // ========================================================================
    assign LED[0]    = sw[0];
    assign LED[1]    = sw[1] & ~sw[2];
    assign LED[2]    = sw[2];
    assign LED[4:3]  = sw[4:3];
    assign LED[7:5]  = sw[7:5];
    assign LED[9:8]  = 2'd0;
    assign LED[10]   = sw_page;
    assign LED[11]   = mode_dbl;
    assign LED[12]   = at_limit;
    assign LED[13]   = frames_rendered[5];       // toggles every 32 frames
    assign LED[14]   = acc_id_ok;
    assign LED[15]   = sw[15];
endmodule

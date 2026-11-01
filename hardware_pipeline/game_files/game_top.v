`timescale 1ns / 1ps
// ============================================================================
// Module: game_top   (board top, Nexys A7-100T, xc7a100tcsg324-1)   (NEW)
//         Interactive cube "game": buttons, switches and board tilt control
//         the cube live. Built AROUND the unchanged hardware_pipeline: it
//         only uses the ports that already exist (rst_render_n, mat_we /
//         mat_addr / mat_data, render_done, VGA outputs).
//
//   CLK100MHZ -> MMCME2_BASE -> clk_render 100 MHz, clk_vga 25.175 MHz
//   CPU_RESETN (red button)  -> full reset (render + VGA), back to start pose
//
//   Every VGA frame (60 per second):
//     1. wait for the "framebuffer not being read" window of the VGA scan
//        (line 440 .. line 39 of the next frame, 3.97 ms; same method as cube_top)
//     2. game_ctrl updates the cube state from buttons / switches / tilt
//     3. matrix_builder turns the state into the 4x4 matrix (6 clocks)
//     4. render domain held in reset, the 16 matrix words are written into
//        matrix_memory, reset released -> z/framebuffer clear + cube render
//     5. wait for render_done, back to 1
//   color_theme re-colours the final pixels on the way to the VGA pins.
//
// Files: this file, game_ctrl.v, matrix_builder.v, btn_debounce.v,
//        adxl362_reader.v, color_theme.v + the existing pipeline modules and
//        mem files (vertices.mem, matrix.mem, recip_lut_q1_17.mem).
// Constraints: game_top.xdc.
// ============================================================================
module game_top #(
    parameter NUM_VERTICES   = 36,
    parameter VERTEX_DEPTH   = 64,
    parameter RECIP_LUT_FILE = "recip_lut_q1_17.mem",
    parameter Z_FRAC_OUT     = 9,
    parameter CULL_MODE      = 1,
    parameter FRAME_HOLD     = 1,        // VGA frames per game update (1 = 60/s)
    parameter TRIGGER_MODE   = 0,        // 0: VGA window (board)  1: free-running (simulation)
    // input timing (board values; the testbench makes them shorter)
    parameter DEBOUNCE_CLKS  = 1000000,  // 10 ms
    parameter ACL_HALF_CLKS  = 50,       // 1 MHz SPI clock
    parameter ACL_PWR_CLKS   = 1000000,  // 10 ms
    parameter ACL_RST_CLKS   = 1000000,  // 10 ms
    parameter ACL_POLL_CLKS  = 1000000   // 100 samples per second
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
    output wire        VGA_VS
);
    // ========================================================================
    // 1) CLOCKING (identical to cube_top)
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
    // 2) GLOBAL RESET (identical to cube_top)
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

    // VGA-domain copy of the reset for color_theme
    (* ASYNC_REG = "TRUE" *) reg [1:0] rst_v_sync = 2'b00;
    always @(posedge clk_vga or negedge rst_n)
        if (!rst_n) rst_v_sync <= 2'b00;
        else        rst_v_sync <= {rst_v_sync[0], 1'b1};
    wire rst_v_n = rst_v_sync[1];

    // ========================================================================
    // 3) INPUTS: buttons + switches debounced, accelerometer
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

    // ========================================================================
    // 4) GAME STATE + MATRIX
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
    // 5) THE PIPELINE (unchanged module)
    // ========================================================================
    reg         rst_render_n;
    reg         mat_we;
    reg  [3:0]  mat_addr;
    reg  [31:0] mat_data;
    wire        render_done;
    wire [3:0]  pipe_r, pipe_g, pipe_b;
    wire        pipe_hs, pipe_vs;

    hardware_pipeline #(
        .NUM_VERTICES   (NUM_VERTICES),
        .VERTEX_DEPTH   (VERTEX_DEPTH),
        .RECIP_LUT_FILE (RECIP_LUT_FILE),
        .LOOP_RENDER    (0),
        .Z_FRAC_OUT     (Z_FRAC_OUT),
        .CULL_MODE      (CULL_MODE)
    ) u_pipeline (
        .clk          (clk_render),
        .clk_vga      (clk_vga),
        .rst_n        (rst_n),
        .rst_render_n (rst_render_n),
        .mat_we       (mat_we),
        .mat_addr     (mat_addr),
        .mat_data     (mat_data),
        .render_done  (render_done),
        .vga_r        (pipe_r),
        .vga_g        (pipe_g),
        .vga_b        (pipe_b),
        .vga_hs       (pipe_hs),
        .vga_vs       (pipe_vs)
    );

    // ========================================================================
    // 6) COLOUR THEME on the VGA outputs (theme switches synced to clk_vga)
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
    // 7) VGA WINDOW DETECTOR (same as cube_top): pulse at line 440
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
    // 8) FRAME SEQUENCER
    // ========================================================================
    localparam S_BOOT = 3'd0,   // power-up render (matrix.mem = start pose)
               S_IDLE = 3'd1,   // wait for the next window
               S_CALC = 3'd2,   // state updated, build matrix
               S_WAIT = 3'd3,   // wait for matrix_builder
               S_LOAD = 3'd4,   // render domain in reset, write 16 words
               S_REL  = 3'd5,   // release the render reset
               S_RUN  = 3'd6;   // wait for render_done
    reg [2:0]  state;
    reg [31:0] frames_rendered;
    reg [4:0]  wr_cnt;

    wire go = (TRIGGER_MODE == 1) ? 1'b1 : step_req;

    always @(posedge clk_render or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_BOOT; rst_render_n <= 1'b1;
            mat_we <= 1'b0; mat_addr <= 4'd0; mat_data <= 32'd0;
            tick <= 1'b0; mb_start <= 1'b0; ld_cnt <= 4'd0; wr_cnt <= 5'd0;
            frames_rendered <= 32'd0;
        end else begin
            mat_we   <= 1'b0;
            tick     <= 1'b0;
            mb_start <= 1'b0;
            case (state)
                S_BOOT: if (render_done) begin
                    frames_rendered <= frames_rendered + 32'd1;
                    state <= S_IDLE;
                end
                S_IDLE: if (go) begin
                    tick  <= 1'b1;                 // game_ctrl updates on this edge
                    state <= S_CALC;
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
                    rst_render_n <= 1'b1;
                    state <= S_RUN;
                end
                S_RUN: if (render_done) begin
                    frames_rendered <= frames_rendered + 32'd1;
                    state <= S_IDLE;
                end
                default: state <= S_IDLE;
            endcase
        end
    end

    // ========================================================================
    // 9) LEDS
    //   LED0 auto-spin   LED1 move mode   LED2 zoom mode   LED4:3 speed
    //   LED7:5 colour theme   LED12 move/zoom limit reached
    //   LED13 heartbeat (blinks while frames are being drawn)
    //   LED14 accelerometer found   LED15 tilt control on
    // ========================================================================
    assign LED[0]    = sw[0];
    assign LED[1]    = sw[1] & ~sw[2];
    assign LED[2]    = sw[2];
    assign LED[4:3]  = sw[4:3];
    assign LED[7:5]  = sw[7:5];
    assign LED[11:8] = 4'd0;
    assign LED[12]   = at_limit;
    assign LED[13]   = frames_rendered[5];       // toggles every 32 frames (~1 Hz)
    assign LED[14]   = acc_id_ok;
    assign LED[15]   = sw[15];
endmodule

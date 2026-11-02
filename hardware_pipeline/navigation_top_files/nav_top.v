`timescale 1ns / 1ps

// ============================================================================
// Module: nav_top   (board top, Nexys A7-100T, xc7a100tcsg324-1)
//         Street-View style navigation through the crossroads scene made by
//         nav_scene.py. Same structure as cube_top: the pipeline is untouched
//         except DISCARD_BEHIND = 1 (see hardware_pipeline.v).
//
//   Buttons (camera moves one 3 m cell per press, relative to the view):
//     BTNU  forward        BTND  back
//     BTNL  step left      BTNR  step right
//     BTNC  turn right 90 deg (four presses = full look-around)
//     CPU_RESETN (red)     back to the start pose
//   A move into a building, tree, pond or off the map is refused (LED14 blinks).
//
//   LEDs:  [3:0] column   [7:4] row   [11:8] heading one-hot (N, E, S, W)
//          [14] refused move (blinks ~0.3 s)   [15] rendering
//
//   How a press is shown (same sequencer as cube_top):
//     1. the button sets the new pose (col, row, heading); it differs from the
//        pose last drawn (ld_*), so a new view is pending
//     2. at the next VGA blanking window (line 440) the render domain is held
//        in reset and the 16 words of that pose are copied from nav_matrices.mem
//        into matrix_memory
//     3. release -> z/framebuffer clear (1.2 ms) -> the whole scene is drawn
//        with the new matrix -> wait render_done
//   Presses during a render update the pose; the latest pose is drawn next.
//   No matrix arithmetic in hardware: every pose has a precomputed matrix.
//
// Files (from nav_scene.py's output folder) -> add as design sources:
//   vertices.mem        scene, NUM_VERTICES words padded to VERTEX_DEPTH
//   matrix.mem          start pose (power-up render)
//   nav_matrices.mem    GRID*GRID*4 poses x 16 words  -> mat_rom
//   nav_walk.mem        GRID lines of GRID bits       -> walk_rom
//   recip_lut_q1_17.mem reciprocal ROM (same file as for the cube)
//   NUM_VERTICES / VERTEX_DEPTH / START_* must match scene_info.txt.
//
// BRAM: vertex RAM 2048 x 144 ~ 8, framebuffer ~45, z-buffer ~59,
//       matrix ROM 4096 x 32 ~ 4, reciprocal ROMs ~1  ->  ~117 of 135.
//
// Constraints: nav_constraints.xdc (cube XDC + buttons + LEDs).
// ============================================================================
module nav_top #(
    parameter NUM_VERTICES    = 1584,   // scene_info.txt: NUM_VERTICES
    parameter VERTEX_DEPTH    = 2048,   // scene_info.txt: VERTEX_DEPTH
    parameter RECIP_LUT_FILE  = "recip_lut_q1_17.mem",
    parameter Z_FRAC_OUT      = 9,
    parameter CULL_MODE       = 1,      // nav_scene.py writes CCW faces
    // ---- navigation ----
    parameter MATRIX_FILE     = "nav_matrices.mem",
    parameter WALK_FILE       = "nav_walk.mem",
    parameter GRID            = 8,      // scene_info.txt: GRID (<= 16)
    parameter START_COL       = 3,
    parameter START_ROW       = 4,
    parameter START_HEADING   = 0,      // 0=N 1=E 2=S 3=W
    parameter DEBOUNCE_CLKS   = 1000000,// 10 ms at 100 MHz (use ~20 in simulation)
    parameter TRIGGER_MODE    = 0       // 0: VGA window (board)  1: start at once (simulation)
)(
    input  wire       CLK100MHZ,
    input  wire       CPU_RESETN,       // red button, active low

    input  wire       BTNC,
    input  wire       BTNU,
    input  wire       BTND,
    input  wire       BTNL,
    input  wire       BTNR,

    output wire [15:0] LED,

    output wire [3:0] VGA_R,
    output wire [3:0] VGA_G,
    output wire [3:0] VGA_B,
    output wire       VGA_HS,
    output wire       VGA_VS
);

    // ========================================================================
    // 1) CLOCKING: one MMCM -> 100 MHz render clock + 25.175 MHz pixel clock
    //    VCO = 100 * 9 = 900 MHz; /35.75 = 25.1748 MHz; /9 = 100 MHz
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
    // 2) GLOBAL RESET: button synchronised, MMCM lock as asynchronous reset
    // ========================================================================
    (* ASYNC_REG = "TRUE" *) reg [1:0] btn_sync = 2'b00;
    always @(posedge clk_render or negedge mmcm_locked)
        if (!mmcm_locked) btn_sync <= 2'b00;
        else              btn_sync <= {btn_sync[0], CPU_RESETN};
    wire rst_n = btn_sync[1];

    // ========================================================================
    // 3) THE PIPELINE
    // ========================================================================
    reg         rst_render_n;
    reg         mat_we;
    reg  [3:0]  mat_addr;
    reg  [31:0] mat_data;
    wire        render_done;

    hardware_pipeline #(
        .NUM_VERTICES   (NUM_VERTICES),
        .VERTEX_DEPTH   (VERTEX_DEPTH),
        .RECIP_LUT_FILE (RECIP_LUT_FILE),
        .LOOP_RENDER    (0),
        .Z_FRAC_OUT     (Z_FRAC_OUT),
        .CULL_MODE      (CULL_MODE),
        .DISCARD_BEHIND (1)             // camera walks inside the scene
    ) u_pipeline (
        .clk          (clk_render),
        .clk_vga      (clk_vga),
        .rst_n        (rst_n),
        .rst_render_n (rst_render_n),
        .mat_we       (mat_we),
        .mat_addr     (mat_addr),
        .mat_data     (mat_data),
        .render_done  (render_done),
        .vga_r        (VGA_R),
        .vga_g        (VGA_G),
        .vga_b        (VGA_B),
        .vga_hs       (VGA_HS),
        .vga_vs       (VGA_VS)
    );

    // ========================================================================
    // 4) BUTTONS: 2-FF synchroniser, debounce (stable for DEBOUNCE_CLKS),
    //    one-clock pulse on the press edge. Order: {C, R, L, D, U}
    // ========================================================================
    localparam DB_W = $clog2(DEBOUNCE_CLKS + 1);

    wire [4:0] btn_raw = {BTNC, BTNR, BTNL, BTND, BTNU};
    wire [4:0] btn_press;                 // one-clock pulses

    genvar b;
    generate
        for (b = 0; b < 5; b = b + 1) begin : g_btn
            (* ASYNC_REG = "TRUE" *) reg [1:0] s;
            reg            stable;
            reg            pulse;
            reg [DB_W-1:0] cnt;
            always @(posedge clk_render or negedge rst_n) begin
                if (!rst_n) begin
                    s <= 2'b00; stable <= 1'b0; pulse <= 1'b0; cnt <= {DB_W{1'b0}};
                end else begin
                    s     <= {s[0], btn_raw[b]};
                    pulse <= 1'b0;
                    if (s[1] == stable) begin
                        cnt <= {DB_W{1'b0}};
                    end else if (cnt == DEBOUNCE_CLKS - 1) begin
                        cnt    <= {DB_W{1'b0}};
                        stable <= s[1];
                        pulse  <= s[1];           // rising edge of the clean level
                    end else begin
                        cnt <= cnt + 1'b1;
                    end
                end
            end
            assign btn_press[b] = pulse;
        end
    endgenerate

    wire press_u = btn_press[0];
    wire press_d = btn_press[1];
    wire press_l = btn_press[2];
    wire press_r = btn_press[3];
    wire press_c = btn_press[4];

    // ========================================================================
    // 5) POSE + WALKABLE MAP
    // ========================================================================
    localparam CW = (GRID <= 2) ? 1 : $clog2(GRID);   // col/row width
    localparam [CW-1:0] COL0 = START_COL;
    localparam [CW-1:0] ROW0 = START_ROW;
    localparam [1:0]    HDG0 = START_HEADING;

    reg [GRID-1:0] walk_rom [0:GRID-1];               // bit c of row r = walkable
    initial $readmemh(WALK_FILE, walk_rom);

    reg [CW-1:0] col, row;
    reg [1:0]    heading;
    reg [CW-1:0] ld_col, ld_row;                      // pose loaded / last drawn
    reg [1:0]    ld_heading;
    // new view needed when the pose differs from the one last drawn
    wire dirty = (col != ld_col) || (row != ld_row) || (heading != ld_heading);

    // direction of the requested step: forward = heading, right = heading+1,
    // back = heading+2, left = heading+3   (0=N: row-1, 1=E: col+1, 2=S: row+1, 3=W: col-1)
    wire       step_req = press_u | press_d | press_l | press_r;
    wire [1:0] step_dir = heading + (press_u ? 2'd0 : press_r ? 2'd1 :
                                     press_d ? 2'd2 : 2'd3);

    // target cell, one bit wider so -1 and GRID can be detected
    wire signed [CW+1:0] col_s = $signed({2'b00, col});
    wire signed [CW+1:0] row_s = $signed({2'b00, row});
    wire signed [CW+1:0] tcol  = (step_dir == 2'd1) ? col_s + 1 :
                                 (step_dir == 2'd3) ? col_s - 1 : col_s;
    wire signed [CW+1:0] trow  = (step_dir == 2'd2) ? row_s + 1 :
                                 (step_dir == 2'd0) ? row_s - 1 : row_s;
    wire in_map   = (tcol >= 0) && (tcol < GRID) && (trow >= 0) && (trow < GRID);
    wire [CW-1:0] tcol_u = tcol[CW-1:0];
    wire [CW-1:0] trow_u = trow[CW-1:0];
    wire walkable = in_map && walk_rom[trow_u][tcol_u];

    reg [24:0] refuse_cnt;                            // LED14: on ~0.17 s after a refused move

    always @(posedge clk_render or negedge rst_n) begin
        if (!rst_n) begin
            col        <= COL0;
            row        <= ROW0;
            heading    <= HDG0;
            refuse_cnt <= 25'd0;
        end else begin
            if (refuse_cnt != 25'd0) refuse_cnt <= refuse_cnt - 25'd1;
            if (press_c) begin
                heading <= heading + 2'd1;            // turn right 90 deg
            end else if (step_req) begin
                if (walkable) begin
                    col   <= tcol_u;
                    row   <= trow_u;
                end else begin
                    refuse_cnt <= {25{1'b1}};
                end
            end
        end
    end

    // ========================================================================
    // 6) MATRIX ROM (nav_matrices.mem, GRID*GRID*4 poses x 16 words)
    //    address = ((row*GRID + col)*4 + heading)*16 + k
    // ========================================================================
    localparam ROM_DEPTH = GRID * GRID * 4 * 16;
    localparam ROM_AW    = $clog2(ROM_DEPTH);

    (* rom_style = "block" *) reg [31:0] mat_rom [0:ROM_DEPTH-1];
    initial $readmemh(MATRIX_FILE, mat_rom);

    reg  [4:0]        ld_cnt;                         // 0..16
    wire [ROM_AW-1:0] pose_idx = (ld_row * GRID + ld_col) * 4 + ld_heading;
    wire [ROM_AW-1:0] rom_addr = pose_idx * 16 + ld_cnt[3:0];
    reg  [31:0]       rom_q;
    always @(posedge clk_render) rom_q <= mat_rom[rom_addr];

    // ========================================================================
    // 7) VGA WINDOW DETECTOR: pulse at line 440 (framebuffer no longer read)
    //    vsync falls at line 490; the k-th hsync fall after it is line 489+k
    //    (mod 525), so the 476th is line 440.  (same as cube_top)
    // ========================================================================
    (* ASYNC_REG = "TRUE" *) reg [2:0] vs_s, hs_s;
    reg [9:0] hs_count;
    reg       win_start;

    always @(posedge clk_render or negedge rst_n) begin
        if (!rst_n) begin
            vs_s <= 3'b111; hs_s <= 3'b111;
            hs_count <= 10'd0; win_start <= 1'b0;
        end else begin
            vs_s <= {vs_s[1:0], VGA_VS};
            hs_s <= {hs_s[1:0], VGA_HS};
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

    wire go = (TRIGGER_MODE == 1) ? 1'b1 : win_start;

    // ========================================================================
    // 8) RENDER SEQUENCER (cube_top's, triggered by "dirty" instead of a timer)
    // ========================================================================
    localparam S_BOOT = 3'd0,   // power-up render (matrix.mem = start pose)
               S_IDLE = 3'd1,   // wait for a new pose + VGA window
               S_LOAD = 3'd2,   // render domain in reset, copy 16 matrix words
               S_REL  = 3'd3,   // release the render reset
               S_RUN  = 3'd4;   // wait for render_done
    reg [2:0]  state;
    reg [31:0] frames_rendered;   // read by a testbench

    always @(posedge clk_render or negedge rst_n) begin
        if (!rst_n) begin
            state           <= S_BOOT;
            rst_render_n    <= 1'b1;            // power-up render runs at once
            mat_we          <= 1'b0;
            mat_addr        <= 4'd0;
            mat_data        <= 32'd0;
            ld_cnt          <= 5'd0;
            ld_col          <= COL0;            // = matrix.mem (power-up render)
            ld_row          <= ROW0;
            ld_heading      <= HDG0;
            frames_rendered <= 32'd0;
        end else begin
            mat_we <= 1'b0;
            case (state)
                S_BOOT: if (render_done) begin
                    frames_rendered <= frames_rendered + 32'd1;
                    state <= S_IDLE;
                end

                S_IDLE: if (dirty && go) begin
                    ld_col       <= col;            // snapshot the pose; a press after
                    ld_row       <= row;            // this edge makes dirty = 1 again
                    ld_heading   <= heading;
                    rst_render_n <= 1'b0;           // stop + clear the render domain
                    ld_cnt       <= 5'd0;
                    state        <= S_LOAD;
                end

                // ROM read latency 1: word k is on rom_q when ld_cnt = k+1
                // (ld_* settle on the S_IDLE->S_LOAD edge, so the first
                //  address is valid from the first S_LOAD cycle)
                S_LOAD: begin
                    if (ld_cnt != 5'd0) begin
                        mat_we   <= 1'b1;
                        mat_addr <= ld_cnt[3:0] - 4'd1;
                        mat_data <= rom_q;
                    end
                    if (ld_cnt == 5'd16) state <= S_REL;
                    else                 ld_cnt <= ld_cnt + 5'd1;
                end

                S_REL: begin                    // last matrix write happens on this edge
                    rst_render_n <= 1'b1;
                    state        <= S_RUN;
                end

                S_RUN: if (render_done) begin   // render_done is 0 while in reset
                    frames_rendered <= frames_rendered + 32'd1;
                    state <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

    // ========================================================================
    // 9) LEDS
    // ========================================================================
    wire [3:0] col4 = col;   // zero-extended for GRID < 16
    wire [3:0] row4 = row;
    assign LED[3:0]   = col4;
    assign LED[7:4]   = row4;
    assign LED[11:8]  = 4'b0001 << heading;           // N, E, S, W
    assign LED[13:12] = 2'b00;
    assign LED[14]    = refuse_cnt[24];               // on for the first half of the timer
    assign LED[15]    = (state != S_IDLE);

endmodule

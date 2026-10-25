`timescale 1ns / 1ps

// ============================================================================
// Module: cube_top   (board top, Nexys A7-100T, xc7a100tcsg324-1)
//         Rotating cube: one matrix per animation step from matrices.mem.
//
//   CLK100MHZ -> MMCME2_BASE -> clk_render 100 MHz, clk_vga 25.175 MHz
//   CPU_RESETN (red button) -> full reset (render + VGA)
//
//   Animation sequencer (render clock domain):
//     1. wait for the "framebuffer not being read" window of the VGA scan
//        (TRIGGER_MODE 0, board) or start at once (TRIGGER_MODE 1, simulation)
//     2. hold the RENDER domain in reset (rst_render_n = 0), copy the next
//        16 matrix words from the matrices.mem ROM into matrix_memory
//     3. release -> z_f_buffer clears (1.2 ms), the 36 cube vertices are
//        fetched, transformed and drawn with the new matrix
//     4. wait for render_done, go to 1
//   The VGA domain is never reset, so the monitor keeps its lock.
//
//   VGA window: the 300x400 framebuffer is read only on lines 40..439.
//   Line 440 .. line 39 of the next frame (125 lines = 3.97 ms) nobody reads
//   it, so clear (1.2 ms) + cube render fit there -> no tearing.
//   The window start is found from the VGA_VS / VGA_HS outputs:
//   vsync falls at line 490, then the k-th hsync fall is in line 489+k (mod 525),
//   so the 476th hsync fall after vsync is line 440.
//
// Files to add to the Vivado project (design sources, so they reach both
// synthesis and the simulation folder):
//   vertices.mem         cube, 36 vertices        -> vertex_memory
//   matrix.mem           matrix of frame 0        -> matrix_memory (power-up)
//   matrices.mem         NUM_FRAMES x 16 words    -> mat_rom in this module
//   recip_lut_q1_17.mem  reciprocal ROM           -> triangle_setup
//   Matrix word order (every 16 words): M33 first ... M00 last.
//
// Constraints: nexys_a7_top.xdc works unchanged (same port names, MMCM
// instance name u_mmcm for the asynchronous clock groups).
// Needs hardware_pipeline.v with the rst_render_n / mat_* / render_done ports.
// ============================================================================
module cube_top #(
    parameter NUM_VERTICES   = 36,
    parameter VERTEX_DEPTH   = 64,
    parameter RECIP_LUT_FILE = "recip_lut_q1_17.mem",
    parameter Z_FRAC_OUT     = 9,
    parameter CULL_MODE      = 1,
    // ---- animation ----
    parameter MATRIX_FILE    = "matrices.mem",
    parameter NUM_FRAMES     = 120,     // matrices in MATRIX_FILE
    parameter MATRIX_STEP    = 1,       // matrices to advance per step (1 = every frame)
    parameter FRAME_HOLD     = 2,       // VGA frames per step (2 -> 30 steps/s, 4 s per turn)
    parameter TRIGGER_MODE   = 0        // 0: VGA window (board)  1: free-running (simulation)
)(
    input  wire       CLK100MHZ,
    input  wire       CPU_RESETN,       // red button, active low

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
        .vga_r        (VGA_R),
        .vga_g        (VGA_G),
        .vga_b        (VGA_B),
        .vga_hs       (VGA_HS),
        .vga_vs       (VGA_VS)
    );

    // ========================================================================
    // 4) MATRIX ROM (matrices.mem, NUM_FRAMES x 16 words, synchronous read)
    // ========================================================================
    localparam ROM_DEPTH = NUM_FRAMES * 16;
    localparam ROM_AW    = $clog2(ROM_DEPTH);
    localparam FR_W      = $clog2(NUM_FRAMES + 1);

    reg [31:0] mat_rom [0:ROM_DEPTH-1];
    initial $readmemh(MATRIX_FILE, mat_rom);

    reg  [FR_W-1:0]   frame_idx;         // matrix currently shown
    reg  [FR_W-1:0]   next_idx;          // matrix being loaded
    reg  [4:0]        ld_cnt;            // 0..16
    wire [ROM_AW-1:0] rom_addr = next_idx * 16 + ld_cnt[3:0];
    reg  [31:0]       rom_q;
    always @(posedge clk_render) rom_q <= mat_rom[rom_addr];

    // ========================================================================
    // 5) VGA WINDOW DETECTOR: pulse at line 440 (framebuffer no longer read)
    // ========================================================================
    (* ASYNC_REG = "TRUE" *) reg [2:0] vs_s, hs_s;   // 2 sync FFs + 1 for edges
    reg [9:0] hs_count;                      // up to 525 lines
    reg       win_start;

    always @(posedge clk_render or negedge rst_n) begin
        if (!rst_n) begin
            vs_s <= 3'b111; hs_s <= 3'b111;
            hs_count <= 10'd0; win_start <= 1'b0;
        end else begin
            vs_s <= {vs_s[1:0], VGA_VS};
            hs_s <= {hs_s[1:0], VGA_HS};
            win_start <= 1'b0;
            if (vs_s[2] && !vs_s[1])                 // vsync fell: line 490
                hs_count <= 10'd0;
            else if (hs_s[2] && !hs_s[1]) begin      // hsync fell
                hs_count <= hs_count + 10'd1;
                if (hs_count + 10'd1 == 10'd476)       // line 440
                    win_start <= 1'b1;
            end
        end
    end

    // FRAME_HOLD windows per animation step
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
    // 6) ANIMATION SEQUENCER
    // ========================================================================
    localparam S_BOOT = 3'd0,   // power-up render (matrix.mem = frame 0)
               S_IDLE = 3'd1,   // wait for the next step
               S_LOAD = 3'd2,   // render domain in reset, copy 16 matrix words
               S_REL  = 3'd3,   // release the render reset
               S_RUN  = 3'd4;   // wait for render_done
    reg [2:0]  state;
    reg [31:0] frames_rendered;   // read by the testbench

    wire go = (TRIGGER_MODE == 1) ? 1'b1 : step_req;

    always @(posedge clk_render or negedge rst_n) begin
        if (!rst_n) begin
            state           <= S_BOOT;
            rst_render_n    <= 1'b1;            // power-up render runs at once
            mat_we          <= 1'b0;
            mat_addr        <= 4'd0;
            mat_data        <= 32'd0;
            ld_cnt          <= 5'd0;
            frame_idx       <= {FR_W{1'b0}};
            next_idx        <= {FR_W{1'b0}};
            frames_rendered <= 32'd0;
        end else begin
            mat_we <= 1'b0;
            case (state)
                S_BOOT: if (render_done) begin
                    frames_rendered <= frames_rendered + 32'd1;
                    state <= S_IDLE;
                end

                S_IDLE: if (go) begin
                    next_idx     <= (frame_idx + MATRIX_STEP >= NUM_FRAMES)
                                    ? frame_idx + MATRIX_STEP - NUM_FRAMES
                                    : frame_idx + MATRIX_STEP;
                    rst_render_n <= 1'b0;       // stop + clear the render domain
                    ld_cnt       <= 5'd0;
                    state        <= S_LOAD;
                end

                // ROM read latency 1: word k is on rom_q when ld_cnt = k+1
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
                    frame_idx    <= next_idx;
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

endmodule

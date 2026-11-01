`timescale 1ns / 1ps
// ============================================================================
// Module: nexys_a7_cpu_top   (board top, Nexys A7-100T, xc7a100tcsg324-1)
//
// HLab graphics pipeline driven by a PicoRV32 RISC-V CPU. Same structure as
// hardware_pipeline/rotating_cube_top_files/cube_top.v, but the matrix for
// every frame comes from the CPU (computed live) instead of matrices.mem.
// hardware_pipeline.v is used unchanged, through its re-render ports
// (rst_render_n, mat_we/mat_addr/mat_data, render_done).
//
//   CLK100MHZ -> MMCM -+-> clk_render 100.000 MHz --+--> cpu_subsystem (gfx_ctrl)
//                      |                            |      | rst_render_n, mat_*,
//                      |                            |      v render_done
//                      |                            +--> hardware_pipeline
//                      +-> clk_vga 25.175 MHz -----------> (VGA side)
//
//   CPU_RESETN (red button) -> debounced reset of everything
//   BTNC/U/L/R/D, SW[15:0]  -> CPU      LED[15:0] <- CPU (LED15 also = CPU trap)
//   UART_TXD_IN / UART_RXD_OUT <-> CPU UART, 115200 8N1 over the USB-JTAG port
//
// VGA window (as cube_top): the 300x400 framebuffer is read on lines 40..439
// only. A frame started at line 440 (clear 1.2 ms + cube ~0.7 ms) is finished
// long before line 40 of the next frame (125 lines = 3.97 ms): no tearing.
// Line 440 is found from the VGA_VS / VGA_HS outputs exactly as in cube_top:
// vsync falls at line 490, the k-th hsync fall after it is line 489+k mod
// 525, so the 476th hsync fall is line 440. One addition to cube_top's
// detector: pulses are suppressed until the first vsync after reset has
// been seen. Before that, hs_count counts from reset (line 0), so its 476th
// hsync fall is line 475, not 440 (found in simulation: the first frame after
// reset started at line 475; still off screen, but not the intended window).
//
// Memory files (add as design sources): vertices.mem, matrix.mem (power-up
// frame), recip_lut_q1_17.mem, firmware.hex.
// ============================================================================
module nexys_a7_cpu_top #(
    parameter NUM_VERTICES    = 36,
    parameter VERTEX_DEPTH    = 64,
    parameter RECIP_LUT_FILE  = "recip_lut_q1_17.mem",
    parameter FIRMWARE_FILE   = "firmware.hex",
    parameter Z_FRAC_OUT      = 9,
    parameter CULL_MODE       = 1,
    parameter DEBOUNCE_CYCLES = 1_000_000,          // 10 ms @ 100 MHz
    parameter BAUD            = 115_200,            // UART (simulation uses a faster rate)
    parameter CPU_RAM_BLOCK   = 1                   // 0: CPU RAM in LUTs (frees 4 RAMB36)
)(
    input  wire        CLK100MHZ,
    input  wire        CPU_RESETN,                  // red button, active low

    input  wire        BTNC, BTNU, BTNL, BTNR, BTND,
    input  wire [15:0] SW,
    output wire [15:0] LED,

    input  wire        UART_TXD_IN,                 // PC -> FPGA
    output wire        UART_RXD_OUT,                // FPGA -> PC

    output wire [3:0]  VGA_R,
    output wire [3:0]  VGA_G,
    output wire [3:0]  VGA_B,
    output wire        VGA_HS,
    output wire        VGA_VS
);

    // ========================================================================
    // 1) CLOCKING (same MMCM settings as cube_top)
    // ========================================================================
    wire clk_fb_out, clk_fb_in, clk_vga_unbuf, clk_render_unbuf;
    wire clk_vga, clk_render, mmcm_locked;

    MMCME2_BASE #(
        .BANDWIDTH          ("OPTIMIZED"),
        .CLKIN1_PERIOD      (10.000),
        .DIVCLK_DIVIDE      (1),
        .CLKFBOUT_MULT_F    (9.000),         // VCO = 900 MHz
        .CLKFBOUT_PHASE     (0.000),
        .CLKOUT0_DIVIDE_F   (35.750),        // 25.1748 MHz
        .CLKOUT0_DUTY_CYCLE (0.500),
        .CLKOUT0_PHASE      (0.000),
        .CLKOUT1_DIVIDE     (9),             // 100.000 MHz
        .CLKOUT1_DUTY_CYCLE (0.500),
        .CLKOUT1_PHASE      (0.000),
        .REF_JITTER1        (0.010),
        .STARTUP_WAIT       ("FALSE")
    ) u_mmcm (
        .CLKIN1 (CLK100MHZ), .CLKFBIN (clk_fb_in), .CLKFBOUT (clk_fb_out), .CLKFBOUTB (),
        .CLKOUT0 (clk_vga_unbuf), .CLKOUT0B (), .CLKOUT1 (clk_render_unbuf), .CLKOUT1B (),
        .CLKOUT2 (), .CLKOUT2B (), .CLKOUT3 (), .CLKOUT3B (),
        .CLKOUT4 (), .CLKOUT5 (), .CLKOUT6 (),
        .LOCKED (mmcm_locked), .PWRDWN (1'b0), .RST (1'b0)
    );

    BUFG u_bufg_fb     (.I(clk_fb_out),       .O(clk_fb_in));
    BUFG u_bufg_vga    (.I(clk_vga_unbuf),    .O(clk_vga));
    BUFG u_bufg_render (.I(clk_render_unbuf), .O(clk_render));

    // ========================================================================
    // 2) RESET: button sync + release debounce + MMCM lock
    // ========================================================================
    (* ASYNC_REG = "TRUE" *) reg [1:0] btn_sync = 2'b00;
    always @(posedge clk_render or negedge mmcm_locked)
        if (!mmcm_locked) btn_sync <= 2'b00;
        else              btn_sync <= {btn_sync[0], CPU_RESETN};
    wire btn_released = btn_sync[1];

    localparam DB_W = (DEBOUNCE_CYCLES > 1) ? $clog2(DEBOUNCE_CYCLES + 1) : 1;
    reg [DB_W-1:0] db_cnt  = {DB_W{1'b0}};
    reg            rst_n_q = 1'b0;

    always @(posedge clk_render or negedge mmcm_locked) begin
        if (!mmcm_locked) begin
            db_cnt  <= {DB_W{1'b0}};
            rst_n_q <= 1'b0;
        end else if (!btn_released) begin
            db_cnt  <= {DB_W{1'b0}};
            rst_n_q <= 1'b0;
        end else if (!rst_n_q) begin
            if (db_cnt == DEBOUNCE_CYCLES - 1) rst_n_q <= 1'b1;
            else                               db_cnt  <= db_cnt + 1'b1;
        end
    end

    // CPU side uses synchronous reset; one register stage for fan-out timing
    reg cpu_rst_n = 1'b0;
    always @(posedge clk_render) cpu_rst_n <= rst_n_q;

    // ========================================================================
    // 3) VGA WINDOW DETECTOR (from cube_top): pulse at line 440
    // ========================================================================
    (* ASYNC_REG = "TRUE" *) reg [2:0] vs_s, hs_s;   // 2 sync FFs + 1 for edges
    reg [9:0] hs_count;
    reg       win_start;
    reg       vs_seen;                               // hs_count is line-aligned

    always @(posedge clk_render or negedge rst_n_q) begin
        if (!rst_n_q) begin
            vs_s <= 3'b111; hs_s <= 3'b111;
            hs_count <= 10'd0; win_start <= 1'b0; vs_seen <= 1'b0;
        end else begin
            vs_s <= {vs_s[1:0], VGA_VS};
            hs_s <= {hs_s[1:0], VGA_HS};
            win_start <= 1'b0;
            if (vs_s[2] && !vs_s[1]) begin              // vsync fell: line 490
                hs_count <= 10'd0;
                vs_seen  <= 1'b1;
            end else if (hs_s[2] && !hs_s[1]) begin     // hsync fell
                hs_count <= hs_count + 10'd1;
                if (hs_count + 10'd1 == 10'd476 && vs_seen)   // line 440
                    win_start <= 1'b1;
            end
        end
    end

    // ========================================================================
    // 4) CPU SUBSYSTEM
    // ========================================================================
    wire        rst_render_n, mat_we, render_done;
    wire [3:0]  mat_addr;
    wire [31:0] mat_data;
    wire [15:0] cpu_led;
    wire        cpu_trap;

    cpu_subsystem #(
        .FIRMWARE_FILE (FIRMWARE_FILE),
        .BAUD          (BAUD),
        .RAM_BLOCK     (CPU_RAM_BLOCK)
    ) u_cpu (
        .clk          (clk_render),
        .rst_n        (cpu_rst_n),
        .led          (cpu_led),
        .sw           (SW),
        .btn          ({BTND, BTNR, BTNL, BTNU, BTNC}),
        .uart_tx      (UART_RXD_OUT),
        .uart_rx      (UART_TXD_IN),
        .cpu_trap     (cpu_trap),
        .win_start    (win_start),
        .rst_render_n (rst_render_n),
        .mat_we       (mat_we),
        .mat_addr     (mat_addr),
        .mat_data     (mat_data),
        .render_done  (render_done)
    );

    // LED15 lights on a CPU trap (illegal instruction / misaligned access)
    assign LED = {cpu_led[15] | cpu_trap, cpu_led[14:0]};

    // ========================================================================
    // 5) THE PIPELINE (unchanged hardware_pipeline.v)
    // ========================================================================
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
        .rst_n        (rst_n_q),
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

endmodule

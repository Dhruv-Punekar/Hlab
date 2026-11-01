`timescale 1ns / 1ps
// ============================================================================
// Module: cpu_subsystem
// Purpose: PicoRV32 (RV32IM) + 16 KiB program/data RAM + peripherals that
//          drive the HLab graphics pipeline.
//
// Memory map:
//   0x0000_0000 - 0x0000_3FFF  RAM 16 KiB (BRAM, preloaded from FIRMWARE_FILE)
//   0x1000_0000                gfx_ctrl  (frame control + matrix, see gfx_ctrl.v)
//   0x2000_0000                uart      (115200 8N1, see uart.v)
//   0x3000_0000                GPIO: 0x00 LED   RW [15:0]
//                                    0x04 SW    R  [15:0]   (2-FF synchronised)
//                                    0x08 BTN   R  [4:0] = {D, R, L, U, C}
//   0x4000_0000                TIMER: 0x00 cycles [31:0], 0x04 cycles [63:32]
//                                    (read 0x00 first: it snapshots the high word)
//   anything else: reads 0, writes ignored.
//
// Bus timing: PicoRV32's native interface. Every access is acknowledged
// (mem_ready) exactly one cycle after mem_valid, with registered read data,
// so the slaves need no combinational path from address to data.
//
// Clock: runs on the pipeline's 100 MHz render clock, so gfx_ctrl talks to
// hardware_pipeline without any clock-domain crossing.
// Reset: rst_n must NOT be the pipeline's rst_render_n (gfx_ctrl drives that).
// ============================================================================
module cpu_subsystem #(
    parameter FIRMWARE_FILE = "firmware.hex",
    parameter RAM_WORDS     = 4096,                 // 16 KiB
    parameter RAM_BLOCK     = 1,                    // 1: block RAM (4 x RAMB36)
                                                    // 0: LUT RAM, if the framebuffer
                                                    //    leaves too few block RAMs
    parameter CLK_HZ        = 100_000_000,
    parameter BAUD          = 115_200
)(
    input  wire         clk,
    input  wire         rst_n,          // synchronous, active low (already synchronised)

    // board I/O
    output wire [15:0]  led,
    input  wire [15:0]  sw,             // asynchronous
    input  wire [4:0]   btn,            // asynchronous, {D, R, L, U, C}, 1 = pressed
    output wire         uart_tx,
    input  wire         uart_rx,
    output wire         cpu_trap,

    // to / from hardware_pipeline (its re-render ports, see gfx_ctrl.v)
    input  wire         win_start,      // VGA window pulse (line 440)
    output wire         rst_render_n,
    output wire         mat_we,
    output wire [3:0]   mat_addr,
    output wire [31:0]  mat_data,
    input  wire         render_done
);

    // ========================================================================
    // CPU
    // ========================================================================
    wire        mem_valid, mem_instr;
    reg         mem_ready;
    wire [31:0] mem_addr, mem_wdata;
    wire [3:0]  mem_wstrb;
    wire [31:0] mem_rdata;

    picorv32 #(
        .ENABLE_COUNTERS      (1),
        .ENABLE_COUNTERS64    (1),
        .ENABLE_REGS_DUALPORT (1),
        .BARREL_SHIFTER       (1),
        .COMPRESSED_ISA       (0),
        .CATCH_MISALIGN       (1),
        .CATCH_ILLINSN        (1),
        .ENABLE_PCPI          (0),
        .ENABLE_MUL           (0),
        .ENABLE_FAST_MUL      (1),          // MUL/MULH* on DSP48 (fixed-point math)
        .ENABLE_DIV           (1),          // DIV/REM (iterative)
        .ENABLE_IRQ           (0),
        .PROGADDR_RESET       (32'h0000_0000),
        .STACKADDR            (RAM_WORDS * 4)   // stack grows down from top of RAM
    ) u_cpu (
        .clk          (clk),
        .resetn       (rst_n),
        .trap         (cpu_trap),
        .mem_valid    (mem_valid),
        .mem_instr    (mem_instr),
        .mem_ready    (mem_ready),
        .mem_addr     (mem_addr),
        .mem_wdata    (mem_wdata),
        .mem_wstrb    (mem_wstrb),
        .mem_rdata    (mem_rdata),
        .mem_la_read  (), .mem_la_write (), .mem_la_addr (),
        .mem_la_wdata (), .mem_la_wstrb (),
        .pcpi_valid   (), .pcpi_insn (), .pcpi_rs1 (), .pcpi_rs2 (),
        .pcpi_wr      (1'b0), .pcpi_rd (32'd0), .pcpi_wait (1'b0), .pcpi_ready (1'b0),
        .irq          (32'd0), .eoi (),
        .trace_valid  (), .trace_data ()
    );

    // ========================================================================
    // ADDRESS DECODE  (one access per two cycles: valid -> ready)
    // ========================================================================
    wire start  = mem_valid && !mem_ready;      // first cycle of an access
    wire is_wr  = |mem_wstrb;
    wire [3:0] region = mem_addr[31:28];

    wire sel_ram  = (region == 4'h0) && (mem_addr[27:2] < RAM_WORDS);
    wire sel_gfx  = (region == 4'h1);
    wire sel_uart = (region == 4'h2);
    wire sel_gpio = (region == 4'h3);
    wire sel_tmr  = (region == 4'h4);

    localparam [2:0] R_NONE = 3'd0, R_RAM = 3'd1, R_GFX = 3'd2, R_UART = 3'd3,
                     R_GPIO = 3'd4, R_TMR = 3'd5;
    reg [2:0] rsel;                             // which slave answers this cycle

    always @(posedge clk) begin
        if (!rst_n) begin
            mem_ready <= 1'b0;
            rsel      <= R_NONE;
        end else begin
            mem_ready <= start;
            if (start)
                rsel <= sel_ram  ? R_RAM  : sel_gfx  ? R_GFX  : sel_uart ? R_UART :
                        sel_gpio ? R_GPIO : sel_tmr  ? R_TMR  : R_NONE;
        end
    end

    // ========================================================================
    // RAM (BRAM, byte writes, registered read)
    // ========================================================================
    localparam RAM_AW = $clog2(RAM_WORDS);
    wire [RAM_AW-1:0] ram_a = mem_addr[RAM_AW+1:2];
    wire              ram_en = start && sel_ram;
    reg  [31:0]       ram_q;

    generate
        if (RAM_BLOCK) begin : g_bram
            (* ram_style = "block" *) reg [31:0] ram [0:RAM_WORDS-1];
            initial $readmemh(FIRMWARE_FILE, ram);
            always @(posedge clk) begin
                if (ram_en) begin
                    if (mem_wstrb[0]) ram[ram_a][ 7: 0] <= mem_wdata[ 7: 0];
                    if (mem_wstrb[1]) ram[ram_a][15: 8] <= mem_wdata[15: 8];
                    if (mem_wstrb[2]) ram[ram_a][23:16] <= mem_wdata[23:16];
                    if (mem_wstrb[3]) ram[ram_a][31:24] <= mem_wdata[31:24];
                    ram_q <= ram[ram_a];
                end
            end
        end else begin : g_lutram
            (* ram_style = "distributed" *) reg [31:0] ram [0:RAM_WORDS-1];
            initial $readmemh(FIRMWARE_FILE, ram);
            always @(posedge clk) begin
                if (ram_en) begin
                    if (mem_wstrb[0]) ram[ram_a][ 7: 0] <= mem_wdata[ 7: 0];
                    if (mem_wstrb[1]) ram[ram_a][15: 8] <= mem_wdata[15: 8];
                    if (mem_wstrb[2]) ram[ram_a][23:16] <= mem_wdata[23:16];
                    if (mem_wstrb[3]) ram[ram_a][31:24] <= mem_wdata[31:24];
                    ram_q <= ram[ram_a];
                end
            end
        end
    endgenerate

    // ========================================================================
    // PERIPHERALS
    // ========================================================================
    wire [31:0] gfx_q, uart_q;

    gfx_ctrl u_gfx (
        .clk (clk), .rst_n (rst_n),
        .sel (start && sel_gfx), .we (is_wr), .re (!is_wr),
        .addr (mem_addr[7:0]), .wdata (mem_wdata), .rdata (gfx_q),
        .win_start (win_start),
        .rst_render_n (rst_render_n), .mat_we (mat_we), .mat_addr (mat_addr),
        .mat_data (mat_data), .render_done (render_done)
    );

    uart #(.CLK_HZ (CLK_HZ), .BAUD (BAUD)) u_uart (
        .clk (clk), .rst_n (rst_n),
        .sel (start && sel_uart), .we (is_wr), .re (!is_wr),
        .addr (mem_addr[7:0]), .wdata (mem_wdata), .rdata (uart_q),
        .tx (uart_tx), .rx (uart_rx)
    );

    // ---- GPIO ----
    (* ASYNC_REG = "TRUE" *) reg [15:0] sw_s1, sw_s2;
    (* ASYNC_REG = "TRUE" *) reg [4:0]  btn_s1, btn_s2;
    reg [15:0] led_r;
    reg [31:0] gpio_q;
    always @(posedge clk) begin
        sw_s1  <= sw;  sw_s2  <= sw_s1;
        btn_s1 <= btn; btn_s2 <= btn_s1;
        if (!rst_n)
            led_r <= 16'd0;
        else if (start && sel_gpio && is_wr && mem_addr[7:0] == 8'h00)
            led_r <= mem_wdata[15:0];
        if (start && sel_gpio && !is_wr)
            case (mem_addr[7:0])
                8'h00:   gpio_q <= {16'd0, led_r};
                8'h04:   gpio_q <= {16'd0, sw_s2};
                8'h08:   gpio_q <= {27'd0, btn_s2};
                default: gpio_q <= 32'd0;
            endcase
    end
    assign led = led_r;

    // ---- TIMER (free-running cycle counter) ----
    reg [63:0] cycles;
    reg [31:0] cyc_hi_snap, tmr_q;
    always @(posedge clk) begin
        if (!rst_n) cycles <= 64'd0;
        else        cycles <= cycles + 64'd1;
        if (start && sel_tmr && !is_wr) begin
            if (mem_addr[2] == 1'b0) begin
                tmr_q       <= cycles[31:0];
                cyc_hi_snap <= cycles[63:32];
            end else
                tmr_q <= cyc_hi_snap;
        end
    end

    // ========================================================================
    // READ DATA MUX (all inputs are registers)
    // ========================================================================
    assign mem_rdata = (rsel == R_RAM)  ? ram_q  :
                       (rsel == R_GFX)  ? gfx_q  :
                       (rsel == R_UART) ? uart_q :
                       (rsel == R_GPIO) ? gpio_q :
                       (rsel == R_TMR)  ? tmr_q  : 32'd0;

endmodule

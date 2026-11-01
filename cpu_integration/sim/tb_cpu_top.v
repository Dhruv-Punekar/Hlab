`timescale 1ps / 1ps
// ============================================================================
// tb_cpu_top - full-system simulation: PicoRV32 firmware + graphics pipeline
//
// Runs nexys_a7_cpu_top with the real firmware.hex and
//   * decodes the UART output (fast baud for simulation) to uart.log + stdout
//   * after every frame: writes cpu_frame_<n>.ppm (P3, 0..15 = RGB444 as
//     stored, same format as tb_cube) and logs the matrix in matrix_memory,
//     the VGA scan line at render start and the draw cycles to frames.log
//     (checked by check_frames.py)
//   * drives the board inputs:
//       frame 0       the pipeline's own power-up render (matrix.mem)
//       frames 1..2   VGA-synchronised (firmware default)
//       BTNR pressed  before frame 3   (yaw speed + 1 step)
//       SW1 = 1       from frame 3 on (free-running, keeps the sim short)
//       UART 's'      sent during frame 5 (exercise RX + stats print)
//       BTNC pressed  before frame 9   (pose reset)
//   * checks that the CPU never traps and the GPU never stalls
//
// Plus-args: +FRAMES=<n> (default 12)
// ============================================================================
module tb_cpu_top;
    localparam integer BAUD = 3_125_000;               // 32 clocks per bit at 100 MHz
    localparam integer BIT_PS = 1_000_000_000_000 / BAUD;

    reg CLK100MHZ = 0; always #5000 CLK100MHZ = ~CLK100MHZ;
    reg CPU_RESETN = 1;
    reg BTNC = 0, BTNU = 0, BTNL = 0, BTNR = 0, BTND = 0;
    reg [15:0] SW = 16'd0;
    reg uart_to_fpga = 1'b1;
    wire uart_from_fpga;
    wire [15:0] LED;
    wire [3:0] R, G, B; wire HS, VS;

    nexys_a7_cpu_top #(.DEBOUNCE_CYCLES(2000), .BAUD(BAUD)) dut (
        .CLK100MHZ(CLK100MHZ), .CPU_RESETN(CPU_RESETN),
        .BTNC(BTNC), .BTNU(BTNU), .BTNL(BTNL), .BTNR(BTNR), .BTND(BTND),
        .SW(SW), .LED(LED),
        .UART_TXD_IN(uart_to_fpga), .UART_RXD_OUT(uart_from_fpga),
        .VGA_R(R), .VGA_G(G), .VGA_B(B), .VGA_HS(HS), .VGA_VS(VS));

    // ------------------------------------------------------------ UART RX
    integer ulog;
    reg [7:0] ch;
    integer k;
    initial begin
        ulog = $fopen("uart.log", "w");
        forever begin
            @(negedge uart_from_fpga);
            #(BIT_PS / 2);
            for (k = 0; k < 8; k = k + 1) begin
                #(BIT_PS);
                ch[k] = uart_from_fpga;
            end
            #(BIT_PS);
            if (ch != 8'h0d) begin
                $fwrite(ulog, "%c", ch); $fflush(ulog);
                $write("%c", ch);
            end
        end
    end

    task uart_send(input [7:0] c);
        integer j;
        begin
            uart_to_fpga = 0; #(BIT_PS);
            for (j = 0; j < 8; j = j + 1) begin uart_to_fpga = c[j]; #(BIT_PS); end
            uart_to_fpga = 1; #(BIT_PS);
        end
    endtask

    // ------------------------------------------------------- frame capture
    integer flog, frame = 0, nframes, errors = 0;
    integer start_line;
    reg [8*40-1:0] name;
    wire clk = dut.clk_render;

    task save_ppm(input integer n);
        integer a, fd, c;
        begin
            $sformat(name, "cpu_frame_%0d.ppm", n);
            fd = $fopen(name, "w");
            $fwrite(fd, "P3\n300 400\n15\n");
            for (a = 0; a < 300*400; a = a + 1) begin
                c = dut.u_pipeline.u_z_f_buffer.frame_buffer[a];
                $fwrite(fd, "%0d %0d %0d\n", (c >> 8) & 15, (c >> 4) & 15, c & 15);
            end
            $fclose(fd);
        end
    endtask

    integer i;
    always @(posedge clk) begin
        if (dut.u_cpu.u_gfx.q_state == 3'd4)           // Q_REL: render starts
            start_line = dut.u_pipeline.u_vga_controller.v_cnt;
        if (dut.u_cpu.u_gfx.frame_done) begin
            save_ppm(frame);
            $fwrite(flog, "frame %0d sync=%0d start_line=%0d draw_cycles=%0d M=",
                    frame, frame ? dut.u_cpu.u_gfx.q_sync : 1'b0, frame ? start_line : 0,
                    dut.u_cpu.u_gfx.draw_run);
            for (i = 15; i >= 0; i = i - 1)
                $fwrite(flog, "%h%s", dut.u_pipeline.u_matrix_memory.matrix_regs[i], i ? " " : "\n");
            $fflush(flog);
            $display("\n[tb] frame %0d done (sync=%0d, start line %0d, %0d draw cycles)",
                     frame, dut.u_cpu.u_gfx.q_sync, start_line, dut.u_cpu.u_gfx.draw_run);
            frame = frame + 1;
        end
        if (dut.u_cpu.cpu_trap === 1'b1) begin
            $display("[tb] FAIL: CPU trap at pc-ish %h", dut.u_cpu.u_cpu.reg_pc);
            errors = errors + 1;
            $finish;
        end
    end

    // GPU watchdog: a frame must never take longer than 2 VGA frames
    integer busy_cnt = 0;
    always @(posedge clk) begin
        busy_cnt = dut.u_cpu.u_gfx.seq_busy ? busy_cnt + 1 : 0;
        if (busy_cnt > 3_400_000) begin
            $display("[tb] FAIL: GPU busy for 34 ms (stalled?)");
            errors = errors + 1;
            $finish;
        end
    end

    // ------------------------------------------------------------ stimulus
    initial begin
        if (!$value$plusargs("FRAMES=%d", nframes)) nframes = 12;
        flog = $fopen("frames.log", "w");

        wait (frame == 3);
        SW[1] = 1;                     // free-running from here on (short sim)
        wait (frame == 4);
        BTNR = 1; #(200_000_000); BTNR = 0;   // 200 us, sampled at the next frame
        wait (frame == 5);
        uart_send("s");
        wait (frame == 9);
        BTNC = 1; #(200_000_000); BTNC = 0;
        wait (frame == nframes);
        #(20_000_000);
        if (LED[15]) begin $display("[tb] FAIL: LED15 (trap) on"); errors = errors + 1; end
        $display("\n[tb] %0d frames, LED=%h, %0s", frame, LED, errors ? "FAILED" : "no trap / no stall");
        $fclose(flog);
        $finish;
    end
endmodule

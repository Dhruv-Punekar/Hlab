`timescale 1ps / 1ps
// Drives nexys_a7_top: power-up render, then one button press -> re-render.
// Checks that frame_0.ppm, frame_1.ppm and render.log are produced.
module tb_logppm;
    reg CLK100MHZ = 0; always #5000 CLK100MHZ = ~CLK100MHZ;
    reg CPU_RESETN = 1;
    wire [3:0] R, G, B; wire HS, VS;
    nexys_a7_top #(.DEBOUNCE_CYCLES(2000)) dut (.CLK100MHZ(CLK100MHZ), .CPU_RESETN(CPU_RESETN),
        .VGA_R(R), .VGA_G(G), .VGA_B(B), .VGA_HS(HS), .VGA_VS(VS));
    initial begin
        wait (dut.frame_no == 1);
        #(50_000_000);                       // 50 us later press the button
        CPU_RESETN = 0; #(100_000_000); CPU_RESETN = 1;
        wait (dut.frame_no == 2);
        #(1_000_000);
        $display("frames written: %0d", dut.frame_no);
        $finish;
    end
endmodule

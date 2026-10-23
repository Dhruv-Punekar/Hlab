// ============================================================================
// tb_top.v
// Full-system test of top.v as it runs on the board:
//   reset -> automatic frame -> UART packet on RsTx.
// A behavioural UART receiver decodes the packet and checks the header and
// all 64 row + 64 column sums against the golden model. Then btnU is pressed
// and the second packet is checked as well. LEDs/switch readout are checked.
// The UART is sped up (CLKS_PER_BIT = 16) to keep the simulation short.
// ============================================================================
`timescale 1ns / 1ps

`ifndef IMAGE_MEM
 `define IMAGE_MEM  "mem/image.mem"
`endif
`ifndef GOLDEN_ROW
 `define GOLDEN_ROW "mem/golden_row.mem"
`endif
`ifndef GOLDEN_COL
 `define GOLDEN_COL "mem/golden_col.mem"
`endif

module tb_top;

    localparam W            = 64;
    localparam H            = 64;
    localparam CLKS_PER_BIT = 16;
    localparam CLK_NS       = 10;
    localparam BIT_NS       = CLKS_PER_BIT * CLK_NS;
    localparam NBYTES       = 2 + 2 * (W + H);

    reg         clk = 0;
    reg         btnC = 1, btnU = 0;
    reg  [15:0] sw = 16'h0000;
    wire [15:0] led;
    wire        RsTx;

    always #(CLK_NS/2) clk = ~clk;

    top #(
        .CLKS_PER_BIT    (CLKS_PER_BIT),
        .DEBOUNCE_CYCLES (4),
        .MEM_FILE        (`IMAGE_MEM)
    ) dut (
        .clk(clk), .btnC(btnC), .btnU(btnU), .sw(sw), .led(led), .RsTx(RsTx)
    );

    reg [15:0] golden_row [0:H-1];
    reg [15:0] golden_col [0:W-1];
    reg [7:0]  pkt [0:NBYTES-1];

    integer errors = 0;

    // ---------------- receive one byte from RsTx ----------------
    task rx_byte(output [7:0] data);
        integer b;
        begin
            @(negedge RsTx);
            #(BIT_NS/2);
            if (RsTx !== 1'b0) begin
                $display("ERROR: bad start bit");
                errors = errors + 1;
            end
            for (b = 0; b < 8; b = b + 1) begin
                #(BIT_NS);
                data[b] = RsTx;
            end
            #(BIT_NS);
            if (RsTx !== 1'b1) begin
                $display("ERROR: bad stop bit");
                errors = errors + 1;
            end
        end
    endtask

    // ---------------- receive and check one packet ----------------
    task check_packet(input integer n);
        integer i;
        reg [7:0]  d;
        reg [15:0] v;
        begin
            for (i = 0; i < NBYTES; i = i + 1) begin
                rx_byte(d);
                pkt[i] = d;
            end
            if (pkt[0] !== 8'hA5 || pkt[1] !== 8'h5A) begin
                $display("ERROR packet %0d: header %02h %02h", n, pkt[0], pkt[1]);
                errors = errors + 1;
            end
            for (i = 0; i < H; i = i + 1) begin
                v = {pkt[2 + 2*i], pkt[3 + 2*i]};
                if (v !== golden_row[i]) begin
                    $display("ERROR packet %0d row[%0d] = %0d expected %0d", n, i, v, golden_row[i]);
                    errors = errors + 1;
                end
            end
            for (i = 0; i < W; i = i + 1) begin
                v = {pkt[2 + 2*H + 2*i], pkt[3 + 2*H + 2*i]};
                if (v !== golden_col[i]) begin
                    $display("ERROR packet %0d col[%0d] = %0d expected %0d", n, i, v, golden_col[i]);
                    errors = errors + 1;
                end
            end
            $display("Packet %0d received (%0d bytes) at %0t ns", n, NBYTES, $time);
        end
    endtask

    initial begin
        $dumpfile("tb_top.vcd");
        $dumpvars(0, tb_top);
        $readmemh(`GOLDEN_ROW, golden_row);
        $readmemh(`GOLDEN_COL, golden_col);

        repeat (5) @(negedge clk);
        btnC = 0;                          // release reset -> auto run

        check_packet(0);
        repeat (20) @(negedge clk);

        // LED readout via switches
        if (led[15] !== 1'b1) begin
            $display("ERROR: done LED not on"); errors = errors + 1;
        end
        sw = 16'h0020;                     // row 32
        #1;
        if (led[13:0] !== golden_row[32][13:0]) begin
            $display("ERROR: LED row[32] = %0d expected %0d", led[13:0], golden_row[32]);
            errors = errors + 1;
        end
        sw = 16'h8005;                     // column 5
        #1;
        if (led[13:0] !== golden_col[5][13:0]) begin
            $display("ERROR: LED col[5] = %0d expected %0d", led[13:0], golden_col[5]);
            errors = errors + 1;
        end

        // press btnU -> re-run and re-send
        btnU = 1;
        repeat (20) @(negedge clk);
        btnU = 0;
        check_packet(1);

        if (errors == 0) $display("PASS: tb_top (2 packets, %0d values each)", W + H);
        else             $display("FAIL: tb_top, %0d errors", errors);
        $finish;
    end

    initial begin
        #20000000 $display("FAIL: timeout"); $finish;
    end

endmodule

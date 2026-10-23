// ============================================================================
// tb_uart_tx.v
// Sends a set of bytes through uart_tx and decodes the serial line with a
// behavioural receiver sampling in the middle of each bit.
// Checks start bit, 8 data bits LSB first, stop bit, and the bit period.
// ============================================================================
`timescale 1ns / 1ps

module tb_uart_tx;

    localparam CLKS_PER_BIT = 16;          // short bit time for simulation
    localparam CLK_NS       = 10;
    localparam BIT_NS       = CLKS_PER_BIT * CLK_NS;
    localparam N            = 6;

    reg clk = 0, rst = 1;
    always #(CLK_NS/2) clk = ~clk;

    reg        tx_start = 0;
    reg  [7:0] tx_data  = 0;
    wire       tx, tx_busy, tx_done;

    uart_tx #(.CLKS_PER_BIT(CLKS_PER_BIT)) dut (
        .clk(clk), .rst(rst), .tx_start(tx_start), .tx_data(tx_data),
        .tx(tx), .tx_busy(tx_busy), .tx_done(tx_done)
    );

    reg [7:0] vectors [0:N-1];
    initial begin
        vectors[0] = 8'hA5; vectors[1] = 8'h5A; vectors[2] = 8'h00;
        vectors[3] = 8'hFF; vectors[4] = 8'h37; vectors[5] = 8'hD1;
    end

    integer errors = 0;

    // ---------------- behavioural receiver ----------------
    reg [7:0] rx_byte;
    integer   rx_count = 0;
    integer   b;
    always begin
        @(negedge tx);                       // start bit edge
        #(BIT_NS/2);
        if (tx !== 1'b0) begin
            $display("ERROR: bad start bit");
            errors = errors + 1;
        end
        for (b = 0; b < 8; b = b + 1) begin
            #(BIT_NS);
            rx_byte[b] = tx;
        end
        #(BIT_NS);
        if (tx !== 1'b1) begin
            $display("ERROR: bad stop bit on byte %0d", rx_count);
            errors = errors + 1;
        end
        if (rx_byte !== vectors[rx_count]) begin
            $display("ERROR: byte %0d received 0x%02h expected 0x%02h",
                     rx_count, rx_byte, vectors[rx_count]);
            errors = errors + 1;
        end else
            $display("  rx byte %0d = 0x%02h ok", rx_count, rx_byte);
        rx_count = rx_count + 1;
    end

    // ---------------- stimulus ----------------
    integer i;
    initial begin
        $dumpfile("tb_uart_tx.vcd");
        $dumpvars(0, tb_uart_tx);
        repeat (3) @(negedge clk);
        rst = 0;
        if (tx !== 1'b1) begin
            $display("ERROR: line not idle-high after reset");
            errors = errors + 1;
        end
        repeat (5) @(negedge clk);

        for (i = 0; i < N; i = i + 1) begin
            while (tx_busy) @(negedge clk);
            tx_data  = vectors[i];
            tx_start = 1;
            @(negedge clk);
            tx_start = 0;
            @(posedge tx_done);
            @(negedge clk);
        end
        #(BIT_NS*2);

        if (rx_count != N) begin
            $display("ERROR: received %0d bytes, expected %0d", rx_count, N);
            errors = errors + 1;
        end
        if (errors == 0) $display("PASS: tb_uart_tx (%0d bytes)", N);
        else             $display("FAIL: tb_uart_tx, %0d errors", errors);
        $finish;
    end

endmodule

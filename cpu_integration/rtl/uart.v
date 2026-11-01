`timescale 1ns / 1ps
// ============================================================================
// Module: uart   (8N1, fixed baud rate, one-byte RX buffer)
//
// Register map (byte offsets; base 0x2000_0000 in the SoC):
//   0x00 DATA    W  [7:0] byte to send (ignored while TX_BUSY)
//                R  bit8 = RX_VALID, [7:0] = received byte; reading it
//                   empties the RX buffer
//   0x04 STATUS  R  bit0 TX_BUSY, bit1 RX_VALID, bit2 RX_OVERRUN (sticky,
//                   cleared by reading DATA)
//
// Nexys A7: the FPGA transmits on UART_RXD_OUT (D4) and receives on
// UART_TXD_IN (C4); names are from the FT2232's point of view.
// ============================================================================
module uart #(
    parameter CLK_HZ = 100_000_000,
    parameter BAUD   = 115_200
)(
    input  wire        clk,
    input  wire        rst_n,       // synchronous, active low

    input  wire        sel,
    input  wire        we,
    input  wire        re,
    input  wire [7:0]  addr,
    input  wire [31:0] wdata,
    output reg  [31:0] rdata,

    output reg         tx,          // -> UART_RXD_OUT
    input  wire        rx           // <- UART_TXD_IN (asynchronous)
);
    localparam integer DIV  = (CLK_HZ + BAUD / 2) / BAUD;   // 868 at 100 MHz / 115200
    localparam integer CW   = $clog2(DIV + 1);

    // ------------------------------------------------------------------ TX
    reg [CW-1:0] tx_cnt;
    reg [3:0]    tx_bit;            // 0 = idle, 1..10 = start, 8 data, stop
    reg [9:0]    tx_sh;
    wire         tx_busy = (tx_bit != 4'd0);

    // ------------------------------------------------------------------ RX
    (* ASYNC_REG = "TRUE" *) reg [1:0] rx_sync;
    reg [CW-1:0] rx_cnt;
    reg [3:0]    rx_bit;            // 0 = idle, 1 = start, 2..9 data, 10 stop
    reg [7:0]    rx_sh;
    reg [7:0]    rx_data;
    reg          rx_valid, rx_overrun;
    wire         rx_in = rx_sync[1];

    wire wr_data = sel && we && (addr == 8'h00);
    wire rd_data = sel && re && (addr == 8'h00);

    always @(posedge clk) begin
        if (!rst_n) begin
            tx <= 1'b1; tx_bit <= 4'd0; tx_cnt <= {CW{1'b0}}; tx_sh <= 10'h3FF;
            rx_sync <= 2'b11; rx_bit <= 4'd0; rx_cnt <= {CW{1'b0}};
            rx_valid <= 1'b0; rx_overrun <= 1'b0; rx_data <= 8'd0; rx_sh <= 8'd0;
        end else begin
            // ---- transmit ----
            if (!tx_busy) begin
                tx <= 1'b1;
                if (wr_data) begin
                    tx_sh  <= {1'b1, wdata[7:0], 1'b0};   // stop, data (LSB first), start
                    tx_bit <= 4'd1;
                    tx_cnt <= {CW{1'b0}};
                end
            end else begin
                tx <= tx_sh[0];
                if (tx_cnt == DIV - 1) begin
                    tx_cnt <= {CW{1'b0}};
                    tx_sh  <= {1'b1, tx_sh[9:1]};
                    tx_bit <= (tx_bit == 4'd10) ? 4'd0 : tx_bit + 4'd1;
                end else
                    tx_cnt <= tx_cnt + 1'b1;
            end

            // ---- receive ----
            rx_sync <= {rx_sync[0], rx};
            if (rd_data) begin rx_valid <= 1'b0; rx_overrun <= 1'b0; end
            case (rx_bit)
                4'd0: if (!rx_in) begin                    // start bit edge
                          rx_bit <= 4'd1;
                          rx_cnt <= DIV / 2;               // sample mid-bit
                      end
                default: begin
                    if (rx_cnt == DIV - 1) begin
                        rx_cnt <= {CW{1'b0}};
                        if (rx_bit == 4'd1) begin          // middle of start bit
                            rx_bit <= rx_in ? 4'd0 : 4'd2; // glitch -> back to idle
                        end else if (rx_bit <= 4'd9) begin // data bits
                            rx_sh  <= {rx_in, rx_sh[7:1]};
                            rx_bit <= rx_bit + 4'd1;
                        end else begin                     // stop bit
                            rx_bit <= 4'd0;
                            if (rx_in) begin
                                rx_data  <= rx_sh;
                                if (rx_valid && !rd_data) rx_overrun <= 1'b1;
                                rx_valid <= 1'b1;
                            end
                        end
                    end else
                        rx_cnt <= rx_cnt + 1'b1;
                end
            endcase
        end
    end

    always @(posedge clk) begin
        if (sel && re) begin
            case (addr)
                8'h00:   rdata <= {23'd0, rx_valid, rx_data};
                8'h04:   rdata <= {29'd0, rx_overrun, rx_valid, tx_busy};
                default: rdata <= 32'd0;
            endcase
        end
    end
endmodule

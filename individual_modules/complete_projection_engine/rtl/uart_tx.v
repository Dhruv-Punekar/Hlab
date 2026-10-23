// ============================================================================
// uart_tx.v
// 8N1 UART transmitter. CLKS_PER_BIT = f_clk / baud
// (100 MHz / 115200 = 868).
// Pulse tx_start for one clock while tx_busy is low to send tx_data.
// tx_done pulses for one clock after the stop bit.
// ============================================================================
`timescale 1ns / 1ps

module uart_tx #(
    parameter CLKS_PER_BIT = 868
)(
    input  wire       clk,
    input  wire       rst,
    input  wire       tx_start,
    input  wire [7:0] tx_data,
    output reg        tx,
    output wire       tx_busy,
    output reg        tx_done
);

    localparam CW = $clog2(CLKS_PER_BIT);

    localparam S_IDLE  = 2'd0,
               S_START = 2'd1,
               S_DATA  = 2'd2,
               S_STOP  = 2'd3;

    reg [1:0]    state;
    reg [CW-1:0] clk_cnt;
    reg [2:0]    bit_idx;
    reg [7:0]    shreg;

    assign tx_busy = (state != S_IDLE);

    always @(posedge clk) begin
        if (rst) begin
            state   <= S_IDLE;
            tx      <= 1'b1;
            tx_done <= 1'b0;
            clk_cnt <= {CW{1'b0}};
            bit_idx <= 3'd0;
            shreg   <= 8'd0;
        end else begin
            tx_done <= 1'b0;
            case (state)
                S_IDLE: begin
                    tx <= 1'b1;
                    if (tx_start) begin
                        shreg   <= tx_data;
                        clk_cnt <= {CW{1'b0}};
                        tx      <= 1'b0;          // start bit
                        state   <= S_START;
                    end
                end

                S_START: begin
                    if (clk_cnt == CLKS_PER_BIT-1) begin
                        clk_cnt <= {CW{1'b0}};
                        bit_idx <= 3'd0;
                        tx      <= shreg[0];      // LSB first
                        state   <= S_DATA;
                    end else
                        clk_cnt <= clk_cnt + 1'b1;
                end

                S_DATA: begin
                    if (clk_cnt == CLKS_PER_BIT-1) begin
                        clk_cnt <= {CW{1'b0}};
                        if (bit_idx == 3'd7) begin
                            tx    <= 1'b1;        // stop bit
                            state <= S_STOP;
                        end else begin
                            bit_idx <= bit_idx + 1'b1;
                            tx      <= shreg[bit_idx + 1'b1];
                        end
                    end else
                        clk_cnt <= clk_cnt + 1'b1;
                end

                S_STOP: begin
                    if (clk_cnt == CLKS_PER_BIT-1) begin
                        clk_cnt <= {CW{1'b0}};
                        tx_done <= 1'b1;
                        state   <= S_IDLE;
                    end else
                        clk_cnt <= clk_cnt + 1'b1;
                end
            endcase
        end
    end

endmodule

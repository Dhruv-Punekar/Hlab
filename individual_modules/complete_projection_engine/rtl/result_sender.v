// ============================================================================
// result_sender.v
// Streams the projection results over the UART after each frame.
//
// Packet format (2 + 2*(H+W) bytes):
//   0xA5 0x5A                         header
//   H x {hi, lo}  row projection      P_row[0] .. P_row[H-1], big-endian 16-bit
//   W x {hi, lo}  column projection   P_col[0] .. P_col[W-1], big-endian 16-bit
//
// The sender drives the engine's read port (rd_sel / rd_idx) while active.
// ============================================================================
`timescale 1ns / 1ps

module result_sender #(
    parameter W  = 64,
    parameter H  = 64,
    parameter IW = 6,
    parameter SW = 14
)(
    input  wire          clk,
    input  wire          rst,
    input  wire          go,          // one-cycle pulse: start sending
    output reg           active,
    // engine read port
    output wire          rd_sel,
    output wire [IW-1:0] rd_idx,
    input  wire [SW-1:0] rd_data,
    // UART
    output reg           tx_start,
    output reg  [7:0]    tx_data,
    input  wire          tx_busy,
    input  wire          tx_done
);

    localparam NBYTES = 2 + 2 * (H + W);
    localparam CW     = $clog2(NBYTES);

    localparam S_IDLE = 2'd0,
               S_SEND = 2'd1,
               S_WAIT = 2'd2;

    reg [1:0]    state;
    reg [CW-1:0] cnt;                 // byte index in the packet

    // which result word / byte the current byte index refers to
    wire [CW-1:0] k       = cnt - 2'd2;
    wire [CW-1:0] word    = k >> 1;   // 0 .. H+W-1
    wire          hi_byte = ~k[0];

    assign rd_sel = (word >= H);
    assign rd_idx = (word >= H) ? (word - H) : word;

    wire [15:0] word16 = {{(16-SW){1'b0}}, rd_data};

    wire [7:0] byte_val = (cnt == 0) ? 8'hA5 :
                          (cnt == 1) ? 8'h5A :
                          hi_byte    ? word16[15:8] : word16[7:0];

    always @(posedge clk) begin
        if (rst) begin
            state    <= S_IDLE;
            active   <= 1'b0;
            cnt      <= {CW{1'b0}};
            tx_start <= 1'b0;
            tx_data  <= 8'h00;
        end else begin
            tx_start <= 1'b0;
            case (state)
                S_IDLE: begin
                    if (go) begin
                        cnt    <= {CW{1'b0}};
                        active <= 1'b1;
                        state  <= S_SEND;
                    end
                end

                S_SEND: begin
                    if (!tx_busy) begin
                        tx_data  <= byte_val;
                        tx_start <= 1'b1;
                        state    <= S_WAIT;
                    end
                end

                S_WAIT: begin
                    if (tx_done) begin
                        if (cnt == NBYTES-1) begin
                            active <= 1'b0;
                            state  <= S_IDLE;
                        end else begin
                            cnt   <= cnt + 1'b1;
                            state <= S_SEND;
                        end
                    end
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule

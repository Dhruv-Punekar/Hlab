// ============================================================================
// projection_engine.v
// Computes the horizontal (row) and vertical (column) projections of a
// W x H, DW-bit grayscale image stored in an external synchronous memory.
//
//   P_row[y] = sum_x I(x, y)      (H values)
//   P_col[x] = sum_y I(x, y)      (W values)
//
// Throughput: one pixel per clock -> W*H + 2 cycles per frame.
// Memory interface: `pix_addr` out, `pix_data` back one clock later
// (matches a block RAM with registered output, see image_rom.v).
//
// Results are read through a combinational port once `done` is high:
//   rd_sel = 0 -> row projection,    rd_idx = y
//   rd_sel = 1 -> column projection, rd_idx = x
// ============================================================================
`timescale 1ns / 1ps

module projection_engine #(
    parameter W  = 64,
    parameter H  = 64,
    parameter DW = 8,
    parameter XW = $clog2(W),
    parameter YW = $clog2(H),
    parameter AW = $clog2(W*H),
    parameter IW = (XW > YW) ? XW : YW,
    parameter SW = DW + ((XW > YW) ? XW : YW)
)(
    input  wire          clk,
    input  wire          rst,
    input  wire          start,      // one-cycle pulse
    // image memory interface
    output wire [AW-1:0] pix_addr,
    input  wire [DW-1:0] pix_data,   // valid 1 clock after pix_addr
    // status
    output wire          busy,
    output reg           done,       // high from frame end until next start
    // result read port
    input  wire          rd_sel,
    input  wire [IW-1:0] rd_idx,
    output wire [SW-1:0] rd_data
);

    // ---------------- stage 0: address generation ----------------
    wire [XW-1:0] g_x;
    wire [YW-1:0] g_y;
    wire          g_valid, g_last, g_busy;

    pixel_addr_gen #(.W(W), .H(H), .XW(XW), .YW(YW), .AW(AW)) u_addr_gen (
        .clk   (clk),
        .rst   (rst),
        .start (start),
        .addr  (pix_addr),
        .x     (g_x),
        .y     (g_y),
        .valid (g_valid),
        .last  (g_last),
        .busy  (g_busy)
    );

    // ---------------- stage 1: align with memory latency ----------------
    reg [XW-1:0] d_x;
    reg [YW-1:0] d_y;
    reg          d_valid, d_last;

    always @(posedge clk) begin
        if (rst) begin
            d_valid <= 1'b0;
            d_last  <= 1'b0;
            d_x     <= {XW{1'b0}};
            d_y     <= {YW{1'b0}};
        end else begin
            d_valid <= g_valid;
            d_last  <= g_last & g_valid;
            d_x     <= g_x;
            d_y     <= g_y;
        end
    end

    // ---------------- stage 2: accumulate ----------------
    wire [SW-1:0] row_rd, col_rd;

    row_projection #(.W(W), .H(H), .DW(DW), .XW(XW), .YW(YW), .SW(SW)) u_row (
        .clk      (clk),
        .in_valid (d_valid),
        .in_x     (d_x),
        .in_y     (d_y),
        .in_pix   (pix_data),
        .rd_idx   (rd_idx[YW-1:0]),
        .rd_data  (row_rd)
    );

    col_projection #(.W(W), .H(H), .DW(DW), .XW(XW), .YW(YW), .SW(SW)) u_col (
        .clk      (clk),
        .in_valid (d_valid),
        .in_x     (d_x),
        .in_y     (d_y),
        .in_pix   (pix_data),
        .rd_idx   (rd_idx[XW-1:0]),
        .rd_data  (col_rd)
    );

    // done rises on the same edge that stores the last pixel's sums
    always @(posedge clk) begin
        if (rst)
            done <= 1'b0;
        else if (start && !g_busy)
            done <= 1'b0;
        else if (d_valid && d_last)
            done <= 1'b1;
    end

    assign busy    = g_busy | d_valid;
    assign rd_data = rd_sel ? col_rd : row_rd;

endmodule

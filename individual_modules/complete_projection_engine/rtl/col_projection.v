// ============================================================================
// col_projection.v
// Vertical projection: P_col[x] = sum over y of I(x, y).
// Each column has its own accumulator held in distributed (LUT) RAM.
// Read-modify-write once per pixel: row 0 initialises, later rows add.
// A given column is touched only once every W cycles, so there is no hazard.
// ============================================================================
`timescale 1ns / 1ps

module col_projection #(
    parameter W  = 64,
    parameter H  = 64,
    parameter DW = 8,
    parameter XW = $clog2(W),
    parameter YW = $clog2(H),
    parameter SW = DW + $clog2(H)       // enough bits for H * (2^DW - 1)
)(
    input  wire          clk,
    input  wire          in_valid,
    input  wire [XW-1:0] in_x,
    input  wire [YW-1:0] in_y,
    input  wire [DW-1:0] in_pix,
    input  wire [XW-1:0] rd_idx,
    output wire [SW-1:0] rd_data
);

    reg [SW-1:0] sums [0:W-1];

    integer i;
    initial begin
        for (i = 0; i < W; i = i + 1)
            sums[i] = {SW{1'b0}};
    end

    wire [SW-1:0] old_sum = (in_y == 0) ? {SW{1'b0}} : sums[in_x];

    always @(posedge clk) begin
        if (in_valid)
            sums[in_x] <= old_sum + in_pix;
    end

    assign rd_data = sums[rd_idx];

endmodule

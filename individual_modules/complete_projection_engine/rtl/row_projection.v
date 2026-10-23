// ============================================================================
// row_projection.v
// Horizontal projection: P_row[y] = sum over x of I(x, y).
// Pixels arrive in raster order, so one running accumulator is enough;
// the finished sum is written to the result memory at the end of each row.
// Result memory is small (H x SW) and maps to distributed (LUT) RAM.
// ============================================================================
`timescale 1ns / 1ps

module row_projection #(
    parameter W  = 64,
    parameter H  = 64,
    parameter DW = 8,
    parameter XW = $clog2(W),
    parameter YW = $clog2(H),
    parameter SW = DW + $clog2(W)       // enough bits for W * (2^DW - 1)
)(
    input  wire          clk,
    input  wire          in_valid,
    input  wire [XW-1:0] in_x,
    input  wire [YW-1:0] in_y,
    input  wire [DW-1:0] in_pix,
    input  wire [YW-1:0] rd_idx,
    output wire [SW-1:0] rd_data
);

    reg [SW-1:0] acc;
    reg [SW-1:0] sums [0:H-1];

    integer i;
    initial begin
        acc = {SW{1'b0}};
        for (i = 0; i < H; i = i + 1)
            sums[i] = {SW{1'b0}};
    end

    // running sum including the current pixel
    wire [SW-1:0] acc_next = (in_x == 0) ? {{(SW-DW){1'b0}}, in_pix}
                                         : acc + in_pix;

    always @(posedge clk) begin
        if (in_valid) begin
            acc <= acc_next;
            if (in_x == W-1)
                sums[in_y] <= acc_next;
        end
    end

    assign rd_data = sums[rd_idx];

endmodule

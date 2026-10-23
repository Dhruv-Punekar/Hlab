// ============================================================================
// image_rom.v
// Image storage, initialised from a $readmemh file (mem/image.mem).
// Registered output -> infers a Xilinx block RAM (4096 x 8 = one RAMB36).
// In Vivado, add image.mem to the project as a design source so synthesis
// can find it by file name.
// ============================================================================
`timescale 1ns / 1ps

module image_rom #(
    parameter DW       = 8,
    parameter DEPTH    = 4096,
    parameter AW       = $clog2(DEPTH),
    parameter MEM_FILE = "image.mem"
)(
    input  wire          clk,
    input  wire [AW-1:0] addr,
    output reg  [DW-1:0] dout
);

    (* rom_style = "block" *)
    reg [DW-1:0] mem [0:DEPTH-1];

    initial begin
        $readmemh(MEM_FILE, mem);
    end

    always @(posedge clk)
        dout <= mem[addr];

endmodule

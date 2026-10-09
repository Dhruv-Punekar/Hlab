`timescale 1ns / 1ps

// ============================================================================
// Module: matrix_memory
// Description: Stores the 4x4 Transformation Matrix. Unlike the Vertex Mem,
// this module exposes all 16 values simultaneously on a 512-bit bus so the 
// Transform Engine can do parallel multiplications in 1 cycle.
// ============================================================================
module matrix_memory (
    input wire clk,
    input wire we,                   // Write enable (from future UART)
    input wire [3:0] write_addr,     // Address 0 to 15 (which element to update)
    input wire [31:0] write_data,    // Q24.8 data from UART
    
    // 512-bit flattened output containing all 16 matrix elements
    output wire [512-1:0] matrix_out 
);

    // 16 individual 32-bit registers (Distributed RAM / Registers)
    reg [31:0] matrix_regs [0:15];

    // Initialize the matrix with the Identity matrix from Python
    initial begin
        $readmemh("matrix.mem", matrix_regs);
    end

    // Sequential logic: Update matrix values when UART writes to them
    always @(posedge clk) begin
        if (we) begin
            matrix_regs[write_addr] <= write_data;
        end
    end

    // Combinational logic: Flatten the 16 registers into a single 512-bit bus
    // This allows the Transform Engine to multiply Vx*Mx instantly.
    genvar i;
    generate
        for (i = 0; i < 16; i = i + 1) begin : flatten_matrix
            assign matrix_out[(i * 32) +: 32] = matrix_regs[i];
        end
    endgenerate

endmodule

`timescale 1ns / 1ps

// ============================================================================
// Module: vertex_memory
// Description: Stores the initial 3D mesh (triangles) using FPGA Block RAM.
// Width: 144 bits per vertex
// ============================================================================
module vertex_memory #(
    parameter DEPTH = 64, // Supports up to 64 vertices (21 triangles)
    parameter WIDTH = 144
)(
    input wire clk,
    input wire en,             // Master enable
    input wire we,             // Write enable (for future updates)
    input wire [$clog2(DEPTH)-1:0] addr,  // Dynamic width based on DEPTH
    input wire [WIDTH-1:0] data_in,       // Data to write (if we is high)
    output reg [WIDTH-1:0] data_out       // Vertex output to FSM
);

    // Declare the Block RAM array
    reg [WIDTH-1:0] ram [0:DEPTH-1];

    // Initialize the memory contents from the Python-generated file
    initial begin
        $readmemh("vertices.mem", ram);
    end

    // Synchronous Read/Write Operation
    always @(posedge clk) begin
        if (en) begin
            if (we) begin
                ram[addr] <= data_in; // Write port (Future phase 2)
            end
            data_out <= ram[addr];    // Read port (Phase 1)
        end
    end

endmodule

// ============================================================================
// pixel_addr_gen.v
// Raster-scan address generator for the projection engine.
// After a one-cycle `start` pulse it emits one pixel per clock:
//   addr = y*W + x, with x running fastest. `last` marks the final pixel.
// Requires W >= 2.
// ============================================================================
`timescale 1ns / 1ps

module pixel_addr_gen #(
    parameter W  = 64,
    parameter H  = 64,
    parameter XW = $clog2(W),
    parameter YW = $clog2(H),
    parameter AW = $clog2(W*H)
)(
    input  wire          clk,
    input  wire          rst,     // synchronous, active high
    input  wire          start,   // one-cycle pulse, ignored while busy
    output reg  [AW-1:0] addr,
    output reg  [XW-1:0] x,
    output reg  [YW-1:0] y,
    output reg           valid,   // addr/x/y hold a pixel this cycle
    output reg           last,    // this is the final pixel of the frame
    output wire          busy
);

    reg run;
    assign busy = run;

    always @(posedge clk) begin
        if (rst) begin
            run   <= 1'b0;
            valid <= 1'b0;
            last  <= 1'b0;
            addr  <= {AW{1'b0}};
            x     <= {XW{1'b0}};
            y     <= {YW{1'b0}};
        end else if (!run) begin
            if (start) begin
                run   <= 1'b1;
                valid <= 1'b1;
                last  <= 1'b0;
                addr  <= {AW{1'b0}};
                x     <= {XW{1'b0}};
                y     <= {YW{1'b0}};
            end
        end else begin
            if (last) begin
                // final pixel has been presented; stop
                run   <= 1'b0;
                valid <= 1'b0;
                last  <= 1'b0;
            end else begin
                addr <= addr + 1'b1;
                if (x == W-1) begin
                    x <= {XW{1'b0}};
                    y <= y + 1'b1;
                end else begin
                    x <= x + 1'b1;
                end
                // next pixel is the last one when it is (W-1, H-1)
                last <= (x == W-2) && (y == H-1);
            end
        end
    end

endmodule

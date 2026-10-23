// ============================================================================
// debounce.v
// Synchronises a push button and outputs a one-cycle pulse on a clean press.
// STABLE_CYCLES = 1_000_000 -> 10 ms at 100 MHz.
// ============================================================================
`timescale 1ns / 1ps

module debounce #(
    parameter STABLE_CYCLES = 1000000
)(
    input  wire clk,
    input  wire rst,
    input  wire btn_in,
    output reg  pressed      // one-cycle pulse
);

    localparam CW = $clog2(STABLE_CYCLES + 1);

    reg          s0, s1;     // 2-FF synchroniser
    reg          stable;
    reg [CW-1:0] cnt;

    always @(posedge clk) begin
        if (rst) begin
            s0      <= 1'b0;
            s1      <= 1'b0;
            stable  <= 1'b0;
            cnt     <= {CW{1'b0}};
            pressed <= 1'b0;
        end else begin
            s0      <= btn_in;
            s1      <= s0;
            pressed <= 1'b0;
            if (s1 == stable) begin
                cnt <= {CW{1'b0}};
            end else if (cnt == STABLE_CYCLES - 1) begin
                cnt    <= {CW{1'b0}};
                stable <= s1;
                if (s1)
                    pressed <= 1'b1;
            end else begin
                cnt <= cnt + 1'b1;
            end
        end
    end

endmodule

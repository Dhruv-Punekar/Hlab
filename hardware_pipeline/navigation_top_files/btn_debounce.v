`timescale 1ns / 1ps
// ============================================================================
// Module: btn_debounce                                    (NEW, game feature)
//   Cleans up N mechanical buttons / switches.
//   1) 2-flip-flop synchroniser (the board inputs are asynchronous)
//   2) a bit changes only after the raw input has been stable for STABLE_CLKS
//      clocks (default 1,000,000 = 10 ms at 100 MHz), so button "bounce"
//      is ignored.
// ============================================================================
module btn_debounce #(
    parameter N           = 5,
    parameter STABLE_CLKS = 1000000
)(
    input  wire         clk,
    input  wire         rst_n,
    input  wire [N-1:0] raw,
    output reg  [N-1:0] clean
);
    localparam CW = (STABLE_CLKS < 2) ? 1 : $clog2(STABLE_CLKS + 1);

    (* ASYNC_REG = "TRUE" *) reg [N-1:0] s1, s2;
    always @(posedge clk or negedge rst_n)
        if (!rst_n) begin s1 <= {N{1'b0}}; s2 <= {N{1'b0}}; end
        else        begin s1 <= raw;       s2 <= s1;        end

    genvar i;
    generate
        for (i = 0; i < N; i = i + 1) begin : g_db
            reg [CW-1:0] cnt;
            always @(posedge clk or negedge rst_n) begin
                if (!rst_n) begin
                    cnt      <= {CW{1'b0}};
                    clean[i] <= 1'b0;
                end else if (s2[i] == clean[i]) begin
                    cnt <= {CW{1'b0}};                  // nothing to do
                end else if (cnt == STABLE_CLKS - 1) begin
                    clean[i] <= s2[i];                  // stable long enough
                    cnt      <= {CW{1'b0}};
                end else begin
                    cnt <= cnt + 1'b1;
                end
            end
        end
    endgenerate
endmodule
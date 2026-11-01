`timescale 1ns / 1ps

// ============================================================================
// Module: tt_buttons   (pixel clock domain)
//   Push-button conditioning for the game: 2-FF synchroniser, debounce, and a
//   press-event flag per button that stays set until the game consumes it.
//
//   debounce: a new level is accepted once it has been stable for
//             2^DB_BITS clocks (2^17 / 25.175 MHz = 5.2 ms; contact bounce
//             of the Nexys A7 buttons is well below that).
//   pending : set on every accepted 0->1 change (press). 'clear' (one clock,
//             from tt_game_core at the start of a frame step) clears only the
//             bits in clear_mask, i.e. the events the game actually sampled,
//             so a press that lands between sampling and clearing is kept.
//   level   : debounced button level (for the LEDs).
// ============================================================================
module tt_buttons #(
    parameter N       = 5,
    parameter DB_BITS = 17
)(
    input  wire         clk,
    input  wire         rst_n,
    input  wire [N-1:0] btn_raw,      // asynchronous, active high
    input  wire         clear,
    input  wire [N-1:0] clear_mask,
    output reg  [N-1:0] pending,
    output wire [N-1:0] level
);

    (* ASYNC_REG = "TRUE" *) reg [N-1:0] s1, s2;
    always @(posedge clk or negedge rst_n)
        if (!rst_n) begin s1 <= {N{1'b0}}; s2 <= {N{1'b0}}; end
        else        begin s1 <= btn_raw;   s2 <= s1;        end

    wire [N-1:0] press;

    genvar i;
    generate for (i = 0; i < N; i = i + 1) begin : g_db
        reg [DB_BITS-1:0] cnt;
        reg               lv, pr;
        always @(posedge clk or negedge rst_n) begin
            if (!rst_n) begin
                cnt <= {DB_BITS{1'b0}};
                lv  <= 1'b0;
                pr  <= 1'b0;
            end else begin
                pr <= 1'b0;
                if (s2[i] == lv)
                    cnt <= {DB_BITS{1'b0}};
                else if (&cnt) begin                // stable long enough: accept
                    cnt <= {DB_BITS{1'b0}};
                    lv  <= s2[i];
                    pr  <= s2[i];                   // rising edge = press
                end else
                    cnt <= cnt + 1'b1;
            end
        end
        assign level[i] = lv;
        assign press[i] = pr;
    end endgenerate

    always @(posedge clk or negedge rst_n)
        if (!rst_n) pending <= {N{1'b0}};
        else        pending <= press | (pending & ~(clear_mask & {N{clear}}));

endmodule

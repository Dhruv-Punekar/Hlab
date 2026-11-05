`timescale 1ns / 1ps
// ============================================================================
// Module: buffer_swap                                   (NEW, double buffering)
//
// Decides WHEN the two framebuffers change roles, safely across the two
// clocks (render 100 MHz, VGA 25.175 MHz).
//
//   render side                                  VGA side
//   -----------                                  --------
//   swap_req (1-clock pulse: "back buffer
//   is finished, please show it")
//       |  req_tog flips (a LEVEL change, not
//       |  a pulse, so the slower VGA clock
//       |  can never miss it)
//       +---- 2-FF synchronizer ---------------> req_s
//                                                at the next vsync (start of
//                                                the vertical sync pulse =
//                                                line 490, picture finished):
//                                                  show_buf flips
//                                                  ack_tog <= req_s
//   ack_s <---------- 2-FF synchronizer ----------+
//   ack_s changed -> swap_done pulse, front_r flips
//
//   * show_buf (VGA clock) : buffer the monitor reads. Changes ONLY at the
//     start of vertical blanking, never in the middle of a picture.
//   * front_r  (render clock): copy of show_buf for the render side; it flips
//     exactly when the render side learns that the swap happened, so
//     "draw into ~front_r" never touches the buffer on the screen.
//   * swap_busy: a request is waiting (do not draw until swap_done).
//
// A toggle handshake like this is the standard way to pass one event between
// two unrelated clocks. Only single bits cross, each through two flip-flops
// marked ASYNC_REG (placed close together, settles any metastability).
// The XDC already declares the two clocks asynchronous.
// ============================================================================
module buffer_swap (
    // ---- render clock domain ----
    input  wire clk_r,
    input  wire rst_r_n,      // full reset (NOT the per-frame render reset)
    input  wire swap_req,     // 1-clock pulse: back buffer ready to be shown
    output reg  swap_done,    // 1-clock pulse: the monitor now shows it
    output wire swap_busy,    // request sent, not yet acknowledged
    output reg  front_r,      // buffer on the screen (render-side copy)

    // ---- VGA clock domain ----
    input  wire clk_v,
    input  wire rst_v_n,      // full reset synchronised to clk_v
    input  wire vsync,        // from vga_controller (active low)
    output reg  show_buf      // buffer the monitor reads
);
    // ------------------------------------------------------------------------
    // render side
    // ------------------------------------------------------------------------
    reg req_tog;                                   // flips once per request
    reg ack_seen;                                  // last acknowledge handled
    (* ASYNC_REG = "TRUE" *) reg [1:0] ack_sync;   // ack_tog -> clk_r

    wire ack_s = ack_sync[1];
    assign swap_busy = (req_tog != ack_seen);

    always @(posedge clk_r or negedge rst_r_n) begin
        if (!rst_r_n) begin
            req_tog   <= 1'b0;
            ack_seen  <= 1'b0;
            ack_sync  <= 2'b00;
            swap_done <= 1'b0;
            front_r   <= 1'b0;
        end else begin
            ack_sync  <= {ack_sync[0], ack_tog};
            swap_done <= 1'b0;

            if (swap_req && !swap_busy)            // ignore a request while one is open
                req_tog <= ~req_tog;

            if (ack_s != ack_seen) begin           // VGA side has swapped
                ack_seen  <= ack_s;
                front_r   <= ~front_r;
                swap_done <= 1'b1;
            end
        end
    end

    // ------------------------------------------------------------------------
    // VGA side
    // ------------------------------------------------------------------------
    reg ack_tog;                                   // = req_s after the swap
    reg vs_d;                                      // vsync one clock ago
    (* ASYNC_REG = "TRUE" *) reg [1:0] req_sync;   // req_tog -> clk_v

    wire req_s    = req_sync[1];
    wire vs_start = vs_d && !vsync;                // vsync falls: line 490, picture done

    always @(posedge clk_v or negedge rst_v_n) begin
        if (!rst_v_n) begin
            req_sync <= 2'b00;
            ack_tog  <= 1'b0;
            vs_d     <= 1'b1;
            show_buf <= 1'b0;
        end else begin
            req_sync <= {req_sync[0], req_tog};
            vs_d     <= vsync;
            if (vs_start && (req_s != ack_tog)) begin
                show_buf <= ~show_buf;             // swap during vertical blanking
                ack_tog  <= req_s;                 // tell the render side
            end
        end
    end
endmodule

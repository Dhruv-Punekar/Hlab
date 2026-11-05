`timescale 1ns / 1ps

// ============================================================================
// Module: tt_view_loader   (render clock domain, 100 MHz)
//   Camera views for the table-tennis demo. Each view has its own camera
//   matrix and its own static scene (the floor grid and the clipping depend on
//   the camera; the hardware has no clipper). Before each render, while the
//   render domain is held in reset, this module
//     - writes the 16 matrix words of the selected view into matrix_memory
//     - copies the view's static vertices from the view ROM (tt_views.mem)
//       into vertex_memory addresses 0 .. TT_N_STATIC-1
//   then tt_obj_animator writes the moving objects after them.
//
//   tt_views.mem: TT_NUM_VIEWS x TT_N_STATIC lines of 64 bit
//                 {X, Y, Z (Q8.8 world units), RGB565}; expanded here to the
//                 144-bit vertex word {X, Y, Z, W = 1.0 (Q24.8), RGB565}.
//   Matrices:     tt_view_mat(view, i) in tt_scene_map.vh.
//
//   One vertex per clock: ROM read (BRAM, 2 registered stages), expand,
//   write port registers. TT_N_STATIC + 4 clocks (~12 us) per load.
// ============================================================================
module tt_view_loader #(
    parameter VIEWS_FILE = "tt_views.mem"
)(
    input  wire         clk,
    input  wire         rst_n,
    input  wire         start,          // 1-clock pulse
    input  wire [1:0]   view,           // stable from start to done

    output reg          vtx_we,
    output reg  [15:0]  vtx_addr,
    output reg  [143:0] vtx_data,

    output reg          mat_we,
    output reg  [3:0]   mat_addr,
    output reg  [31:0]  mat_data,

    output reg          done            // 1-clock pulse after the last write
);

`include "tt_scene_map.vh"

    localparam DEPTH = TT_NUM_VIEWS * TT_N_STATIC;
    localparam AW    = $clog2(DEPTH);

    (* rom_style = "block" *) reg [63:0] rom [0:DEPTH-1];
    initial $readmemh(VIEWS_FILE, rom);

    // first ROM line of each view
    function [AW-1:0] view_base;  input [1:0] v;  begin
        view_base = v * TT_N_STATIC;
    end endfunction

    // ------------------------------------------------------------------------
    // address counter
    // ------------------------------------------------------------------------
    reg          run;
    reg [15:0]   idx;
    reg [AW-1:0] raddr;
    reg [4:0]    midx;                  // matrix word 0..15, 16 = done
    reg          v0, v1;
    reg [15:0]   i0, i1;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            run <= 1'b0;  idx <= 16'd0;  raddr <= {AW{1'b0}};
            v0 <= 1'b0;  i0 <= 16'd0;
            midx <= 5'd16;
        end else begin
            v0 <= 1'b0;
            if (start) begin
                run   <= 1'b1;
                idx   <= 16'd0;
                raddr <= view_base(view);
                midx  <= 5'd0;
            end else if (run) begin
                v0    <= 1'b1;
                i0    <= idx;
                raddr <= raddr + 1'b1;
                idx   <= idx + 16'd1;
                if (idx == TT_N_STATIC - 1) run <= 1'b0;
            end
            if (!start && midx != 5'd16) midx <= midx + 5'd1;
        end
    end

    // ------------------------------------------------------------------------
    // ROM read: 2 registered stages (BRAM output register)
    // ------------------------------------------------------------------------
    reg [63:0] q1, q2;
    always @(posedge clk) begin
        q1 <= rom[raddr];
        q2 <= q1;
    end
    // entry i0 (raddr was its address one clock earlier) is in q1 while v0 is
    // set, and in q2 while v1 is set
    always @(posedge clk or negedge rst_n)
        if (!rst_n) begin v1 <= 1'b0; i1 <= 16'd0; end
        else        begin v1 <= v0;   i1 <= i0;    end

    // ------------------------------------------------------------------------
    // write port registers
    // ------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            vtx_we <= 1'b0;  vtx_addr <= 16'd0;  vtx_data <= 144'd0;
            mat_we <= 1'b0;  mat_addr <= 4'd0;   mat_data <= 32'd0;
            done   <= 1'b0;
        end else begin
            vtx_we   <= v1;
            vtx_addr <= i1;
            vtx_data <= {{16{q2[63]}}, q2[63:48], {16{q2[47]}}, q2[47:32],
                         {16{q2[31]}}, q2[31:16], 32'h0000_0100, q2[15:0]};
            done     <= v1 && (i1 == TT_N_STATIC - 1);

            mat_we   <= (midx != 5'd16) && !start;
            mat_addr <= midx[3:0];
            mat_data <= tt_view_mat(view, midx[3:0]);
        end
    end

endmodule

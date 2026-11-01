`timescale 1ns / 1ps
// ============================================================================
// Module: gfx_ctrl
// Purpose: CPU registers + frame sequencer for hardware_pipeline.
//          Uses the pipeline's re-render ports exactly the way cube_top does
//          (rst_render_n, matrix write port, render_done); the pipeline itself
//          is not modified. The CPU takes the place of cube_top's matrix ROM.
//
// Register map (byte offsets, 32-bit accesses; base 0x1000_0000 in the SoC):
//   0x00 CTRL        W  bit0 GO    1 = draw one frame with MATRIX (self-clearing)
//                    RW bit1 SYNC  1 = start in the next VGA window (line 440,
//                                      framebuffer off screen) -> tear-free
//                                      0 = start at once            [reset 1]
//                    R  bit0 reads back PENDING
//   0x04 STATUS      R  bit0 BUSY     = PENDING | SEQ_BUSY  (poll until 0)
//                       bit1 PENDING  = GO written, not started yet
//                       bit2 SEQ_BUSY = sequencer running (incl. power-up render)
//   0x08 FRAME_COUNT R  frames finished since reset (power-up render = frame 0)
//   0x10 FRAME_CYCLES R clk cycles of the last frame, GO accepted -> done
//                       (includes the wait for the VGA window when SYNC = 1)
//   0x14 DRAW_CYCLES R  clk cycles of the last frame, render start -> done
//                       (z/framebuffer clear + draw: the GPU's real work)
//   0x40..0x7C MATRIX RW 4x4 transform, ROW-MAJOR: 0x40 = M00, 0x44 = M01,
//                       ... 0x7C = M33, signed Q24.8. Copied into the
//                       pipeline's matrix_memory when the frame starts; do
//                       not change it while BUSY.
//
// Frame sequence (same as cube_top's animation sequencer):
//   IDLE --GO--> [WAIT_WIN: wait for win_start if SYNC] --> LOAD: hold the
//   render domain in reset (rst_render_n = 0), write the 16 matrix words
//   into matrix_memory (word k = row-major element 15-k, i.e. M33 first)
//   --> REL: release the reset (z/framebuffer clear + vertex fetch start)
//   --> RUN: wait for render_done --> IDLE.
// After reset the pipeline renders once with matrix.mem by itself; BOOT
// waits for that render, so a GO written early is simply held as PENDING.
// ============================================================================
module gfx_ctrl (
    input  wire         clk,
    input  wire         rst_n,       // synchronous, active low

    // CPU bus slave: write on the edge with we=1, registered read data
    input  wire         sel,
    input  wire         we,
    input  wire         re,
    input  wire [7:0]   addr,        // byte offset
    input  wire [31:0]  wdata,
    output reg  [31:0]  rdata,

    // VGA window: 1-cycle pulse when the scan reaches line 440
    input  wire         win_start,

    // to / from hardware_pipeline
    output reg          rst_render_n,
    output reg          mat_we,
    output reg  [3:0]   mat_addr,
    output reg  [31:0]  mat_data,
    input  wire         render_done
);

    // ------------------------------------------------------------------ regs
    reg        pending;
    reg        sync_en;
    reg [31:0] m [0:15];               // row-major M00..M33
    reg [31:0] frame_count;
    reg [31:0] frame_cycles, frame_run;
    reg [31:0] draw_cycles,  draw_run;

    // ------------------------------------------------------------- sequencer
    localparam [2:0] Q_BOOT = 3'd0, Q_IDLE = 3'd1, Q_WAIT_WIN = 3'd2,
                     Q_LOAD = 3'd3, Q_REL = 3'd4, Q_RUN = 3'd5;
    reg [2:0] q_state;
    reg [4:0] ld_cnt;                  // 0..15 = matrix word being written
    reg       q_sync;                  // SYNC of the frame in progress (for tests)

    wire seq_busy = (q_state != Q_IDLE);
    wire accept   = (q_state == Q_IDLE) && pending;
    wire frame_done = (q_state == Q_RUN || q_state == Q_BOOT) && render_done;

    wire wr = sel && we;
    wire rd = sel && re;
    wire is_mtx = (addr[7:6] == 2'b01);          // 0x40..0x7F
    wire [3:0] midx = addr[5:2];

    integer i;
    always @(posedge clk) begin
        if (!rst_n) begin
            pending      <= 1'b0;
            sync_en      <= 1'b1;
            for (i = 0; i < 16; i = i + 1) m[i] <= 32'd0;
            frame_count  <= 32'd0;
            frame_cycles <= 32'd0;  frame_run <= 32'd0;
            draw_cycles  <= 32'd0;  draw_run  <= 32'd0;
            q_state      <= Q_BOOT;
            q_sync       <= 1'b0;
            ld_cnt       <= 5'd0;
            rst_render_n <= 1'b1;   // power-up render runs by itself
            mat_we       <= 1'b0;
            mat_addr     <= 4'd0;
            mat_data     <= 32'd0;
        end else begin
            // ---- CPU writes ----
            if (wr) begin
                if (addr == 8'h00) sync_en <= wdata[1];
                if (is_mtx)        m[midx] <= wdata;
            end
            if (accept)                           pending <= 1'b0;
            if (wr && addr == 8'h00 && wdata[0])  pending <= 1'b1;   // a new GO wins

            // ---- sequencer ----
            mat_we <= 1'b0;
            case (q_state)
                Q_BOOT: if (render_done) q_state <= Q_IDLE;

                Q_IDLE: if (pending) begin
                    q_sync  <= sync_en;
                    q_state <= sync_en ? Q_WAIT_WIN : Q_LOAD;
                    ld_cnt  <= 5'd0;
                end

                Q_WAIT_WIN: if (win_start) q_state <= Q_LOAD;

                Q_LOAD: begin                       // render domain held in reset
                    rst_render_n <= 1'b0;
                    mat_we   <= 1'b1;
                    mat_addr <= ld_cnt[3:0];
                    mat_data <= m[15 - ld_cnt[3:0]];  // word k = row-major 15-k
                    if (ld_cnt == 5'd15) q_state <= Q_REL;
                    ld_cnt <= ld_cnt + 5'd1;
                end

                Q_REL: begin                        // last word is written on this edge
                    rst_render_n <= 1'b1;
                    q_state      <= Q_RUN;
                end

                Q_RUN: if (render_done) q_state <= Q_IDLE;   // 0 while in reset

                default: q_state <= Q_IDLE;
            endcase

            // ---- statistics ----
            if (accept)        frame_run <= 32'd1;
            else               frame_run <= frame_run + 32'd1;
            if (q_state == Q_REL) draw_run <= 32'd1;
            else                  draw_run <= draw_run + 32'd1;
            if (frame_done) begin
                frame_count  <= frame_count + 32'd1;
                frame_cycles <= (q_state == Q_BOOT) ? draw_run : frame_run;
                draw_cycles  <= draw_run;
            end
        end
    end

    // ---- registered read port ----
    always @(posedge clk) begin
        if (rd) begin
            if (is_mtx) rdata <= m[midx];
            else case (addr)
                8'h00:   rdata <= {30'd0, sync_en, pending};
                8'h04:   rdata <= {29'd0, seq_busy, pending, pending | seq_busy};
                8'h08:   rdata <= frame_count;
                8'h10:   rdata <= frame_cycles;
                8'h14:   rdata <= draw_cycles;
                default: rdata <= 32'd0;
            endcase
        end
    end

endmodule

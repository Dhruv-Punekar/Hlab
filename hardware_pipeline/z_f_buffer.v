`timescale 1ns / 1ps

// ============================================================================
// Module: z_f_buffer
// Purpose: Combined Z-Buffer and Framebuffer with a 3-stage pipeline,
//          data forwarding, and fast-clearing.
//          (Single Buffered architecture: 300x400 fits perfectly in BRAM)
//
// Depth format: pixel_z is SIGNED (rasterizer Z_SIGNED=1, NDC depth, smaller
//               = nearer; Q6.9 with Z_FRAC_OUT = 9). All depth compares are
//               signed and the z-buffer clears to +32767 (0x7FFF = farthest).
//               Fix 2026-10-22: compares were unsigned with a 0xFFFF clear,
//               so every negative (near) depth lost to every positive one.
//
// BRAM inference (fix 2026-10-23): the memories are read and written ONLY in
// the reset-free always blocks of section "MEMORY PORTS". Vivado cannot map a
// memory written inside an always block with an asynchronous reset to block
// RAM (Synth 8-4767 "RAM is sensitive to asynchronous reset signal" ->
// Synth 8-3391). Clear and pixel writes now share one write port through a
// mux; behaviour is cycle-for-cycle identical to the previous version.
//
// Timing fix (2026-10-23): the read address s1_addr was built inside a DSP48
// block (y*300 + x) and one DSP output drove ~97 block-RAM address pins
// across the chip (9.2 ns of wire -> 5 paths failed by up to 0.31 ns).
// Now: use_dsp = "no" keeps the address arithmetic in normal logic, and
// max_fanout lets Vivado make several copies of s1_addr, each placed close
// to its own group of block RAMs. Same values, same clock cycle: behaviour
// is unchanged.
// Latch fix (2026-10-23): s1_addr is now written in its own always block
// without a reset (it never had a reset value). Inside the async-reset block
// Vivado built "hold during reset" for it out of 19 latches (TIMING-20,
// PDRC-153). It still updates only when not clearing, exactly as before.
// ============================================================================
(* use_dsp = "no" *)   // address math in fabric so s1_addr can be replicated
module z_f_buffer #(
    parameter WIDTH = 300,
    parameter HEIGHT = 400,
    // Total pixels per buffer
    parameter PIXELS = WIDTH * HEIGHT
)(
    // Rasterizer Clock Domain
    input  wire        clk_render,
    input  wire        rst_n,

    // Rasterizer Interface
    input  wire        valid_in,
    input  wire [9:0]  pixel_x,
    input  wire [9:0]  pixel_y,
    input  wire [15:0] pixel_z,
    input  wire [15:0] pixel_color,

    // Handshake & Status
    output wire        ready_out,   // Tells rasterizer we can accept pixels
    output wire        clear_done,  // High when memory wipe is finished
    output wire        busy,        // High during wipe

    // VGA Clock Domain
    input  wire        clk_vga,
    input  wire [9:0]  vga_x,
    input  wire [9:0]  vga_y,
    output reg  [15:0] vga_color
);

    // ========================================================================
    // MEMORY ARRAYS (BRAM) - Single Buffered
    // ========================================================================
    // Explicitly enforce Xilinx Block RAM inference (RAMB36/RAMB18)
    (* ram_style = "block" *) reg [15:0] frame_buffer [0:PIXELS-1];
    (* ram_style = "block" *) reg [15:0] z_buffer     [0:PIXELS-1];

    // ========================================================================
    // FSM & FAST CLEAR LOGIC
    // ========================================================================
    localparam STATE_CLEAR = 1'b0;
    localparam STATE_IDLE  = 1'b1;

    reg state;
    reg [18:0] clear_addr; // Large enough to count to 120,000

    assign busy = (state == STATE_CLEAR);
    assign clear_done = (state == STATE_IDLE);

    // We can only accept pixels if we are IDLE, bounds are correct, and valid is high
    wire bounds_ok = (pixel_x < WIDTH) && (pixel_y < HEIGHT);
    assign ready_out = (state == STATE_IDLE);

    // ========================================================================
    // 3-STAGE PIPELINE (S1: Read, S2: Compare, S3: Write)
    // ========================================================================

    // --- STAGE 1 REGISTERS ---
    reg s1_valid;
    (* max_fanout = 16 *) reg [18:0] s1_addr;  // Vivado may duplicate this register (read-address fanout)
    reg signed [15:0] s1_z;
    reg        [15:0] s1_color;

    // --- STAGE 2 REGISTERS ---
    reg s2_valid;
    reg [18:0] s2_addr;
    reg signed [15:0] s2_z;
    reg        [15:0] s2_color;
    reg signed [15:0] s2_old_z; // From BRAM

    // --- STAGE 3 REGISTERS ---
    reg s3_we;
    reg [18:0] s3_addr;
    reg signed [15:0] s3_z;
    reg        [15:0] s3_color;

    // --- STAGE 4 REGISTERS (Distance-2 RAW Hazard Bypass) ---
    reg s4_we;
    reg [18:0] s4_addr;
    reg signed [15:0] s4_z;

    // ========================================================================
    // PIPELINE EXECUTION
    // ========================================================================
    always @(posedge clk_render or negedge rst_n) begin
        if (!rst_n) begin
            state <= STATE_CLEAR;
            clear_addr <= 0;
            s1_valid <= 0; s2_valid <= 0; s3_we <= 0; s4_we <= 0;
        end else begin

            // ----------------------------------------------------------------
            // FAST CLEAR FSM
            // ----------------------------------------------------------------
            if (state == STATE_CLEAR) begin
                // Wipe both Z and Color memories (the writes themselves are in
                // MEMORY PORTS below: z = 0x7FFF farthest, color = black)

                clear_addr <= clear_addr + 1;

                // Stop when we reach the end of the memory
                if (clear_addr == PIXELS - 1) begin
                    state <= STATE_IDLE;
                end

                s1_valid <= 0; s2_valid <= 0; s3_we <= 0; s4_we <= 0;
            end
            // ----------------------------------------------------------------
            // RENDER PIPELINE
            // ----------------------------------------------------------------
            else begin

                // ------------------------------------------------------------
                // STAGE 1: READ
                // ------------------------------------------------------------
                // (1D memory address s1_addr is computed in its own
                //  reset-free block below, see "STAGE 1 ADDRESS")
                s1_z <= pixel_z;
                s1_color <= pixel_color;
                s1_valid <= valid_in && bounds_ok; // MVP: Hardware Bounds Check

                // ------------------------------------------------------------
                // STAGE 2: COMPARE & FORWARD
                // ------------------------------------------------------------
                s2_addr <= s1_addr;
                s2_z <= s1_z;
                s2_color <= s1_color;
                s2_valid <= s1_valid;

                // (s2_old_z is the synchronous BRAM read, see MEMORY PORTS)

                // ------------------------------------------------------------
                // STAGE 3: WRITE (With Distance-1 & Distance-2 Forwarding)
                // ------------------------------------------------------------
                s3_addr <= s2_addr;
                s3_z <= s2_z;
                s3_color <= s2_color;

                // Track committed write for Distance-2 hazard bypass
                s4_we   <= s3_we;
                s4_addr <= s3_addr;
                s4_z    <= s3_z;

                // DATA FORWARDING (Distance-1 & Distance-2 RAW Hazard Bypass):
                // Distance 1: Pixel in Stage 3 is writing to the same address.
                // Distance 2: Pixel in Stage 4 committed a write on the exact same
                //             clock edge that Stage 2 sampled BRAM (stale read).
                if (s3_we && (s3_addr == s2_addr)) begin
                    s3_we <= s2_valid && (s2_z < s3_z);  // Distance-1 Forwarding (signed)
                end else if (s4_we && (s4_addr == s2_addr)) begin
                    s3_we <= s2_valid && (s2_z < s4_z);  // Distance-2 Forwarding (signed)
                end else begin
                    s3_we <= s2_valid && (s2_z < s2_old_z); // Normal BRAM Read (signed)
                end

                // (the memory write for s3_we is in MEMORY PORTS)
            end
        end
    end

    // ========================================================================
    // MEMORY PORTS (no reset here, so Vivado infers block RAM)
    //   one write port (render clock): fast clear OR the stage-3 pixel write
    //   one read port  (render clock): z_buffer[s1_addr] -> s2_old_z
    //   VGA read port  (VGA clock)   : frame_buffer, below
    // Read-during-write on the same address returns the OLD value (read-first),
    // exactly as before; the distance-2 forwarding path covers that case.
    // ========================================================================
    wire               clearing = (state == STATE_CLEAR);

    // STAGE 1 ADDRESS: y * WIDTH + x. No reset (it never had one); updates on
    // the same clock edges as the rest of stage 1 (only when not clearing).
    always @(posedge clk_render) begin
        if (!clearing)
            s1_addr <= (pixel_y * WIDTH) + pixel_x;
    end
    wire               mem_we   = clearing | s3_we;      // s3_we is 0 during clear
    wire [18:0]        mem_addr = clearing ? clear_addr : s3_addr;
    wire signed [15:0] mem_z    = clearing ? 16'sh7FFF  : s3_z;    // 0x7FFF = farthest
    wire        [15:0] mem_c    = clearing ? 16'h0000   : s3_color; // black

    always @(posedge clk_render) begin
        if (mem_we) begin
            z_buffer[mem_addr]     <= mem_z;
            frame_buffer[mem_addr] <= mem_c;
        end
    end

    always @(posedge clk_render) begin
        if (!clearing)
            s2_old_z <= z_buffer[s1_addr];
    end

    // ========================================================================
    // VGA DISPLAY PORT (Independent Clock Domain)
    // ========================================================================
    wire [18:0] vga_addr_1d = (vga_y * WIDTH) + vga_x;
    wire vga_bounds_ok = (vga_x < WIDTH) && (vga_y < HEIGHT);

    always @(posedge clk_vga) begin
        if (vga_bounds_ok)
            vga_color <= frame_buffer[vga_addr_1d];
        else
            vga_color <= 16'h0000;
    end

endmodule

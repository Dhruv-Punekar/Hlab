`timescale 1ns / 1ps

// ============================================================================
// Module: z_f_buffer
// Purpose: Combined Z-Buffer and Framebuffer with a 3-stage pipeline,
//          data forwarding, and fast-clearing.
//
// Depth format: pixel_z is SIGNED (rasterizer Z_SIGNED=1, NDC depth, smaller
//               = nearer; Q6.9 with Z_FRAC_OUT = 9). All depth compares are
//               signed and the z-buffer clears to +32767 (0x7FFF = farthest).
//               Fix 2026-10-22: compares were unsigned with a 0xFFFF clear,
//               so every negative (near) depth lost to every positive one.
//
// Colour format (change 2026-10-23, BRAM reduction):
//   pixel_color still arrives as RGB565 (port unchanged), but it is converted
//   to RGB444 as soon as it enters stage 1. The conversion is top-bit truncation
//   R4 = c[15:12], G4 = c[10:7], B4 = c[4:1]
// ============================================================================

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
    input  wire [15:0] pixel_color,   // RGB565 (converted to RGB444 on entry to stage 1)

    // Handshake & Status
    output wire        ready_out,   // Tells rasterizer we can accept pixels
    output wire        clear_done,  // High when memory wipe is finished
    output wire        busy,        // High during wipe

    // VGA Clock Domain
    input  wire        clk_vga,
    input  wire [9:0]  vga_x,
    input  wire [9:0]  vga_y,
    output reg  [11:0] vga_color    // RGB444 {R[3:0], G[3:0], B[3:0]}
);

    // ========================================================================
    // MEMORY ARRAYS (BRAM) - Single Buffered
    // ========================================================================

    (* ram_style = "block" *) reg [11:0] frame_buffer [0:PIXELS-1];   // RGB444
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
    reg [18:0] s1_addr;
    reg signed [15:0] s1_z;
    reg        [11:0] s1_color;   // RGB444

    // --- STAGE 2 REGISTERS ---
    reg s2_valid;
    reg [18:0] s2_addr;
    reg signed [15:0] s2_z;
    reg        [11:0] s2_color;   // RGB444
    reg signed [15:0] s2_old_z; // From BRAM

    // --- STAGE 3 REGISTERS ---
    reg s3_we;
    reg [18:0] s3_addr;
    reg signed [15:0] s3_z;
    reg        [11:0] s3_color;   // RGB444

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
                // Wipe both Z and Color memories
                z_buffer[clear_addr] <= 16'sh7FFF; // Farthest signed depth (0x7FFF)
                frame_buffer[clear_addr] <= 12'h000; // Black

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
                // Calculate 1D Memory Address
                s1_addr <= (pixel_y * WIDTH) + pixel_x;
                s1_z <= pixel_z;
                s1_color <= {pixel_color[15:12], pixel_color[10:7], pixel_color[4:1]}; // RGB565 -> RGB444
                s1_valid <= valid_in && bounds_ok; // Hardware Bounds Check

                // ------------------------------------------------------------
                // STAGE 2: COMPARE & FORWARD
                // ------------------------------------------------------------
                s2_addr <= s1_addr;
                s2_z <= s1_z;
                s2_color <= s1_color;
                s2_valid <= s1_valid;

                // Synchronous BRAM Read (Fetching the old Z value)
                s2_old_z <= z_buffer[s1_addr];

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

                // Execute the actual memory write
                if (s3_we) begin
                    z_buffer[s3_addr] <= s3_z;
                    frame_buffer[s3_addr] <= s3_color;
                end
            end
        end
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
            vga_color <= 12'h000;
    end

endmodule

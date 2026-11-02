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
// Colour / framebuffer modes (parameter DOUBLE_BUF, change 2026-11-02):
//
//   DOUBLE_BUF = 0  (default - EXACTLY the old behaviour, used by cube_top,
//                   image_top, nexys_a7_top)
//     One framebuffer, RGB444 per pixel. pixel_color arrives as RGB565 and is
//     truncated to RGB444 on entry to stage 1:
//       R4 = c[15:12], G4 = c[10:7], B4 = c[4:1]
//     draw_buf / show_buf / pal_* are not used.
//
//   DOUBLE_BUF = 1  (game_top)
//     TWO framebuffers (fb0, fb1) that store a small colour NUMBER (palette
//     index, INDEX_W bits, default 4 -> 16 colours) instead of the colour.
//       * draw_buf (render clock) : which buffer the pixels and the clear go to
//       * show_buf (VGA clock)    : which buffer the monitor is reading
//       The caller (buffer_swap) keeps them different, so the picture on the
//       screen is never the one being drawn.
//     Colour in : RGB565 -> RGB444 (same truncation as above) -> looked up in
//                 the colour table loaded from PALETTE_FILE; the first entry
//                 with the same RGB444 gives the index. A colour that is not
//                 in the table gets the LAST index (2^INDEX_W - 1).
//                 With COLOR_IN_IDX = 1 the low INDEX_W bits of pixel_color
//                 are used as the index directly (for future lighting).
//     Colour out: index -> palette -> RGB444 on vga_color, still exactly
//                 1 clk_vga of latency, so vga_controller is unchanged.
//     Palette   : two copies (pal0, pal1), one per framebuffer, loaded from
//                 PALETTE_FILE at power-up. pal_we/pal_addr/pal_data (render
//                 clock) write the copy that belongs to the DRAW buffer, so a
//                 new palette appears on screen together with its frame.
//                 (Not used yet - tie pal_we to 0.)
//     BRAM      : 2 x 120000 x 4 bits ~ 32 RAMB36 (old: 120000 x 12 ~ 48).
//
//   The z-buffer and the whole 3-stage pipeline / forwarding logic are the
//   same in both modes; only the width of the colour that travels with the
//   pixel changes (12 bits or INDEX_W bits).
// ============================================================================

module z_f_buffer #(
    parameter WIDTH  = 300,
    parameter HEIGHT = 400,
    // Total pixels per buffer
    parameter PIXELS = WIDTH * HEIGHT,

    // ---- double buffering (see header) ----
    parameter DOUBLE_BUF   = 0,             // 0: old single RGB444 buffer, 1: two index buffers
    parameter INDEX_W      = 4,             // palette index bits (DOUBLE_BUF = 1)
    parameter PALETTE_FILE = "palette.mem", // 2^INDEX_W lines of RGB444 (3 hex digits)
    parameter COLOR_IN_IDX = 0,             // 1: pixel_color[INDEX_W-1:0] is already an index
    parameter [INDEX_W-1:0] CLEAR_INDEX = 0 // background index written by the clear
)(
    // Rasterizer Clock Domain
    input  wire        clk_render,
    input  wire        rst_n,

    // Rasterizer Interface
    input  wire        valid_in,
    input  wire [9:0]  pixel_x,
    input  wire [9:0]  pixel_y,
    input  wire [15:0] pixel_z,
    input  wire [15:0] pixel_color,   // RGB565

    // Handshake & Status
    output wire        ready_out,   // Tells rasterizer we can accept pixels
    output wire        clear_done,  // High when memory wipe is finished
    output wire        busy,        // High during wipe

    // Double buffering, render clock domain (DOUBLE_BUF = 1 only)
    input  wire               draw_buf,  // buffer that is drawn / cleared (keep stable while rendering)
    input  wire               pal_we,    // palette write (goes to the draw buffer's palette)
    input  wire [INDEX_W-1:0] pal_addr,
    input  wire [11:0]        pal_data,  // RGB444

    // VGA Clock Domain
    input  wire        clk_vga,
    input  wire [9:0]  vga_x,
    input  wire [9:0]  vga_y,
    input  wire        show_buf,    // DOUBLE_BUF = 1: buffer shown on the monitor (VGA clock)
    output wire [11:0] vga_color    // RGB444 {R[3:0], G[3:0], B[3:0]}, 1 clk_vga latency
);

    // width of the colour that travels through the pipeline and is stored
    localparam CW   = DOUBLE_BUF ? INDEX_W : 12;
    localparam NPAL = 1 << INDEX_W;

    // ========================================================================
    // MEMORY ARRAYS (BRAM)
    //   Only the arrays of the selected mode are used; Vivado removes the rest.
    // ========================================================================
    (* ram_style = "block" *) reg [11:0]        frame_buffer [0:PIXELS-1];   // DOUBLE_BUF = 0 (RGB444)
    (* ram_style = "block" *) reg [INDEX_W-1:0] fb0          [0:PIXELS-1];   // DOUBLE_BUF = 1, buffer 0
    (* ram_style = "block" *) reg [INDEX_W-1:0] fb1          [0:PIXELS-1];   // DOUBLE_BUF = 1, buffer 1
    (* ram_style = "block" *) reg [15:0]        z_buffer     [0:PIXELS-1];

    // Palettes (DOUBLE_BUF = 1): tiny, built from LUTs, not BRAM
    reg [11:0] match_rom [0:NPAL-1];   // fixed copy used to turn RGB444 into an index
    (* ram_style = "distributed" *) reg [11:0] pal0 [0:NPAL-1];   // palette of fb0
    (* ram_style = "distributed" *) reg [11:0] pal1 [0:NPAL-1];   // palette of fb1

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
    // COLOUR CONVERSION (combinational, in front of stage 1)
    // ========================================================================
    wire [11:0] in_rgb444 = {pixel_color[15:12], pixel_color[10:7], pixel_color[4:1]}; // RGB565 -> RGB444

    reg [INDEX_W-1:0] in_index;
    reg               idx_found;
    integer           k;
    always @* begin
        in_index  = NPAL - 1;                 // not in the table -> last entry
        idx_found = 1'b0;
        for (k = 0; k < NPAL; k = k + 1)
            if (!idx_found && (match_rom[k] == in_rgb444)) begin
                in_index  = k;
                idx_found = 1'b1;
            end
        if (COLOR_IN_IDX != 0)
            in_index = pixel_color[INDEX_W-1:0];
    end

    wire [CW-1:0] in_color;
    generate
        if (DOUBLE_BUF != 0) begin : g_in_idx
            assign in_color = in_index;
        end else begin : g_in_rgb
            assign in_color = in_rgb444;
        end
    endgenerate

    // ========================================================================
    // 3-STAGE PIPELINE (S1: Read, S2: Compare, S3: Write)
    // ========================================================================

    // --- STAGE 1 REGISTERS ---
    reg s1_valid;
    reg [18:0] s1_addr;
    reg signed [15:0] s1_z;
    reg        [CW-1:0] s1_color;

    // --- STAGE 2 REGISTERS ---
    reg s2_valid;
    reg [18:0] s2_addr;
    reg signed [15:0] s2_z;
    reg        [CW-1:0] s2_color;
    reg signed [15:0] s2_old_z; // From BRAM

    // --- STAGE 3 REGISTERS ---
    reg s3_we;
    reg [18:0] s3_addr;
    reg signed [15:0] s3_z;
    reg        [CW-1:0] s3_color;

    // --- STAGE 4 REGISTERS (Distance-2 RAW Hazard Bypass) ---
    reg s4_we;
    reg [18:0] s4_addr;
    reg signed [15:0] s4_z;

    // ========================================================================
    // PIPELINE EXECUTION  (unchanged)
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
                // Wipe Z and the colour memory being drawn (write port below)
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
                s1_addr <= (pixel_y * WIDTH) + pixel_x;
                s1_z <= pixel_z;
                s1_color <= in_color;                 // RGB444 or palette index
                s1_valid <= valid_in && bounds_ok;    // Hardware Bounds Check

                // ------------------------------------------------------------
                // STAGE 2: COMPARE & FORWARD
                // ------------------------------------------------------------
                s2_addr <= s1_addr;
                s2_z <= s1_z;
                s2_color <= s1_color;
                s2_valid <= s1_valid;

                // ------------------------------------------------------------
                // STAGE 3: WRITE (With Distance-1 & Distance-2 Forwarding)
                // ------------------------------------------------------------
                s3_addr <= s2_addr;
                s3_z <= s2_z;
                s3_color <= s2_color;

                s4_we   <= s3_we;
                s4_addr <= s3_addr;
                s4_z    <= s3_z;

                if (s3_we && (s3_addr == s2_addr)) begin
                    s3_we <= s2_valid && (s2_z < s3_z);  // Distance-1 Forwarding (signed)
                end else if (s4_we && (s4_addr == s2_addr)) begin
                    s3_we <= s2_valid && (s2_z < s4_z);  // Distance-2 Forwarding (signed)
                end else begin
                    s3_we <= s2_valid && (s2_z < s2_old_z); // Normal BRAM Read (signed)
                end
            end
        end
    end

    // ========================================================================
    // MEMORY WRITE PORT (render clock) - clear and pixel write share it
    // ========================================================================
    wire          clearing  = (state == STATE_CLEAR);
    wire          mem_we    = clearing | s3_we;
    wire [18:0]   mem_addr  = clearing ? clear_addr  : s3_addr;
    wire [15:0]   mem_z     = clearing ? 16'sh7FFF   : s3_z;      // far on clear
    wire [CW-1:0] clr_color = DOUBLE_BUF ? CLEAR_INDEX : {CW{1'b0}};
    wire [CW-1:0] mem_color = clearing ? clr_color   : s3_color;  // background on clear

    // z-buffer: identical to before (read-first, the distance-2 forwarding
    // relies on that "stale read")
    always @(posedge clk_render) begin
        s2_old_z <= z_buffer[s1_addr];                 // stage-2 read
        if (mem_we)
            z_buffer[mem_addr] <= mem_z;
    end

    // ========================================================================
    // VGA DISPLAY PORT (independent clock domain)
    // ========================================================================
    wire [18:0] vga_addr_1d   = (vga_y * WIDTH) + vga_x;
    wire        vga_bounds_ok = (vga_x < WIDTH) && (vga_y < HEIGHT);

    generate
        if (DOUBLE_BUF == 0) begin : g_single
            // ---------------- old behaviour: one RGB444 framebuffer ----------
            always @(posedge clk_render)
                if (mem_we)
                    frame_buffer[mem_addr] <= mem_color;

            reg [11:0] vga_q;
            always @(posedge clk_vga) begin
                if (vga_bounds_ok)
                    vga_q <= frame_buffer[vga_addr_1d];
                else
                    vga_q <= 12'h000;
            end
            assign vga_color = vga_q;

        end else begin : g_double
            // ---------------- two index framebuffers + palettes --------------
            initial begin
                $readmemh(PALETTE_FILE, match_rom);
                $readmemh(PALETTE_FILE, pal0);
                $readmemh(PALETTE_FILE, pal1);
            end

            // write: only the DRAW buffer (clear and pixels)
            always @(posedge clk_render)
                if (mem_we && !draw_buf)
                    fb0[mem_addr] <= mem_color;
            always @(posedge clk_render)
                if (mem_we && draw_buf)
                    fb1[mem_addr] <= mem_color;

            // palette write: the palette of the DRAW buffer
            always @(posedge clk_render)
                if (pal_we && !draw_buf)
                    pal0[pal_addr] <= pal_data;
            always @(posedge clk_render)
                if (pal_we && draw_buf)
                    pal1[pal_addr] <= pal_data;

            // read: both buffers every clock, pick the SHOWN one.
            // show_buf only changes during vertical blanking (buffer_swap),
            // so the choice never changes in the middle of a picture.
            wire [18:0] vga_addr = vga_bounds_ok ? vga_addr_1d : 19'd0;
            reg  [INDEX_W-1:0] q0, q1;
            reg                sel_q, in_q;
            always @(posedge clk_vga) begin
                q0    <= fb0[vga_addr];
                q1    <= fb1[vga_addr];
                sel_q <= show_buf;
                in_q  <= vga_bounds_ok;
            end

            // index -> RGB444 in the same clock (keeps the 1-clock latency
            // that vga_controller expects)
            wire [INDEX_W-1:0] idx = sel_q ? q1 : q0;
            wire [11:0]        rgb = sel_q ? pal1[idx] : pal0[idx];
            assign vga_color = in_q ? rgb : 12'h000;
        end
    endgenerate

endmodule

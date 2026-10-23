`timescale 1ns / 1ps

// ============================================================================
// Module: vga_controller
// Purpose: 640x480 @ 60 Hz VGA timing generator for the z_f_buffer read port.
//          Centers a 300x400 framebuffer in the visible area and converts
//          RGB565 -> RGB444 for the Nexys A7 VGA DAC.
//
// Pixel clock: 25.175 MHz ideal (25.0 - 25.2 MHz works on most monitors /
//              VGA-to-HDMI converters).
//
// Pipeline (all on clk_pix):
//   cycle N   : h_cnt/v_cnt -> fb_x/fb_y (combinational from counters)
//   cycle N+1 : z_f_buffer registers vga_color  (1-cycle read latency)
//               hsync/vsync/active delayed 1 cycle here  -> *_d1
//   cycle N+2 : output registers (RGB gated by active_d1, syncs)
//   => the whole image is shifted by a constant 2 clocks, which is harmless;
//      what matters is that color, hsync, vsync and blanking all share it.
// ============================================================================
module vga_controller #(
    // Framebuffer geometry
    parameter FB_W   = 300,
    parameter FB_H   = 400,
    // Image origin on screen (centered in 640x480)
    parameter X_OFF  = 170,   // (640-300)/2
    parameter Y_OFF  = 40,    // (480-400)/2
    // 640x480@60 timing (pixels / lines)
    parameter H_VISIBLE = 640,
    parameter H_FRONT   = 16,
    parameter H_SYNC    = 96,
    parameter H_BACK    = 48,
    parameter V_VISIBLE = 480,
    parameter V_FRONT   = 10,
    parameter V_SYNC    = 2,
    parameter V_BACK    = 33
)(
    input  wire        clk_pix,
    input  wire        rst_n,       // async assert, active low
    input  wire        fb_ready,    // from z_f_buffer.clear_done (render clk domain)
                                    // tie to 1'b1 if not used
    input  wire [15:0] fb_color,    // RGB565 from z_f_buffer.vga_color (1-cycle latency)

    output wire [9:0]  fb_x,        // -> z_f_buffer.vga_x
    output wire [9:0]  fb_y,        // -> z_f_buffer.vga_y
    output reg  [3:0]  vga_r,
    output reg  [3:0]  vga_g,
    output reg  [3:0]  vga_b,
    output reg         hsync,       // active low
    output reg         vsync,       // active low
    output wire        video_active // aligned with RGB output stage input (debug)
);

    localparam H_TOTAL = H_VISIBLE + H_FRONT + H_SYNC + H_BACK; // 800
    localparam V_TOTAL = V_VISIBLE + V_FRONT + V_SYNC + V_BACK; // 525

    localparam H_SYNC_START = H_VISIBLE + H_FRONT;              // 656
    localparam H_SYNC_END   = H_SYNC_START + H_SYNC;            // 752
    localparam V_SYNC_START = V_VISIBLE + V_FRONT;              // 490
    localparam V_SYNC_END   = V_SYNC_START + V_SYNC;            // 492

    // ------------------------------------------------------------------------
    // fb_ready CDC: 2-FF synchronizer from render clock into clk_pix.
    // clear_done is a level that goes high once and stays, so this is safe.
    // ------------------------------------------------------------------------
    (* ASYNC_REG = "TRUE" *) reg [1:0] ready_sync;
    always @(posedge clk_pix or negedge rst_n) begin
        if (!rst_n) ready_sync <= 2'b00;
        else        ready_sync <= {ready_sync[0], fb_ready};
    end
    wire ready_pix = ready_sync[1];

    // ------------------------------------------------------------------------
    // Stage 0: scan counters
    // ------------------------------------------------------------------------
    reg [9:0] h_cnt;   // 0..799
    reg [9:0] v_cnt;   // 0..524

    always @(posedge clk_pix or negedge rst_n) begin
        if (!rst_n) begin
            h_cnt <= 10'd0;
            v_cnt <= 10'd0;
        end else begin
            if (h_cnt == H_TOTAL - 1) begin
                h_cnt <= 10'd0;
                if (v_cnt == V_TOTAL - 1) v_cnt <= 10'd0;
                else                      v_cnt <= v_cnt + 10'd1;
            end else begin
                h_cnt <= h_cnt + 10'd1;
            end
        end
    end

    // Raw (undelayed) sync, negative polarity for 640x480@60
    wire hsync_raw = ~((h_cnt >= H_SYNC_START) && (h_cnt < H_SYNC_END));
    wire vsync_raw = ~((v_cnt >= V_SYNC_START) && (v_cnt < V_SYNC_END));

    // Window check BEFORE subtracting offsets (no underflow into valid addrs)
    wire in_window = (h_cnt >= X_OFF) && (h_cnt < X_OFF + FB_W) &&
                     (v_cnt >= Y_OFF) && (v_cnt < Y_OFF + FB_H);

    // Out-of-range coords make z_f_buffer return black
    assign fb_x = in_window ? (h_cnt - X_OFF) : FB_W;   // 300 when outside
    assign fb_y = in_window ? (v_cnt - Y_OFF) : FB_H;   // 400 when outside

    // ------------------------------------------------------------------------
    // Stage 1: delay control signals to match z_f_buffer's registered vga_color
    // ------------------------------------------------------------------------
    reg hsync_d1, vsync_d1, active_d1;
    always @(posedge clk_pix or negedge rst_n) begin
        if (!rst_n) begin
            hsync_d1  <= 1'b1;   // idle = inactive (high)
            vsync_d1  <= 1'b1;
            active_d1 <= 1'b0;
        end else begin
            hsync_d1  <= hsync_raw;
            vsync_d1  <= vsync_raw;
            active_d1 <= in_window;
        end
    end

    // active here is aligned with fb_color (this cycle's fb_color corresponds
    // to the coordinates presented one cycle ago, same as active_d1)
    assign video_active = active_d1;

    // ------------------------------------------------------------------------
    // Stage 2: output registers. RGB565 -> RGB444 by top-bit truncation.
    // Black when outside window, during reset, or before framebuffer clear.
    // ------------------------------------------------------------------------
    wire show = active_d1 & ready_pix;

    always @(posedge clk_pix or negedge rst_n) begin
        if (!rst_n) begin
            vga_r <= 4'h0;
            vga_g <= 4'h0;
            vga_b <= 4'h0;
            hsync <= 1'b1;
            vsync <= 1'b1;
        end else begin
            hsync <= hsync_d1;
            vsync <= vsync_d1;
            if (show) begin
                vga_r <= fb_color[15:12];	// truncation to 4bits
                vga_g <= fb_color[10:7];	// truncation to 4bits
                vga_b <= fb_color[4:1];		// truncation to 4bits
            end else begin
                vga_r <= 4'h0;
                vga_g <= 4'h0;
                vga_b <= 4'h0;
            end
        end
    end

endmodule

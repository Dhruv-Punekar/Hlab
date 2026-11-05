`timescale 1ns / 1ps
// ============================================================================
// Module: nav_overlay                              (navigation demo helper 2/2)
//
// Sits between the pipeline's VGA outputs and the VGA pins (like
// color_theme) and paints three things the pipeline does not draw:
//
//   1. SKY    : picture pixels that are still black (nothing drawn) in the
//               upper half get a 4-band blue gradient.
//   2. GROUND : black pixels in the lower half get the colour of the ground
//               spot you look at (grass, road, sidewalk, water ...).
//   3. MINI-MAP in the right margin: the 48 x 48 cell map, a red square for
//               you and three yellow dots showing where you look.
//
// HOW THE GROUND WORKS
//   The camera never tilts, so a whole screen row looks at the ground at one
//   distance D (row 399: 1.7 m, rows near the middle: far away). nav_world.py
//   stores, per row, D, the sideways step K from one pixel to the next and the
//   offset G = 149.5*K of the row start. At the start of each line (hc 2..5):
//       A = (eye x, eye z) + D*forward - G*right       ground spot of pixel 0
//       B = K*right                                    step per pixel
//   then for every pixel: spot += B  (one add, no division).
//   The spot's map cell (2 m) gives the colour. Rows further than 45 m are
//   mixed 50/50 with a haze colour.
//
// WHICH POSITION?  The ground must match the picture on the screen, so the
// pose is copied when the monitor switches to a new picture (show_buf
// changes, during vertical blanking). nav_top keeps pose_* stable from the
// swap request until the swap, so this copy across clocks is safe.
//
// Timing: 3 pixel clocks of delay; HSYNC/VSYNC are delayed the same, so the
// picture stays aligned. The screen position is rebuilt from the sync pulses
// (hsync falls at pixel 656, vsync falls at line 490).
// ============================================================================
module nav_overlay #(
    parameter MAP_FILE   = "nav_map.mem",
    parameter TRIG_FILE  = "nav_trig.mem",
    parameter FLOOR_FILE = "nav_floor.mem",
    parameter MAP_N      = 48,
    parameter FOG_ROWS   = 8,           // scene_info.txt: FOG_ROWS
    parameter START_X8   = 8704,        // start pose, Q24.8 (34 m)
    parameter START_Z8   = 13312,       //                   (52 m)
    parameter START_H    = 0,
    parameter MM_X0      = 485,         // mini-map top-left corner on the screen
    parameter MM_Y0      = 60
)(
    input  wire        clk,             // VGA pixel clock
    input  wire        rst_n,           // VGA-domain reset

    input  wire [3:0]  in_r, in_g, in_b,  // from the pipeline
    input  wire        in_hs, in_vs,

    input  wire        show_buf,        // buffer on the screen (flips at a swap)
    input  wire signed [31:0] pose_x8,  // pose of the picture being swapped in
    input  wire signed [31:0] pose_z8,  //   (render clock, stable during the swap)
    input  wire [7:0]  pose_h,
    input  wire        map_on,          // show the mini-map

    output reg  [3:0]  out_r, out_g, out_b,
    output reg         out_hs, out_vs
);
    // ========================================================================
    // colours (RGB444) - same numbers as nav_world.py
    // ========================================================================
    localparam GRASS = 3'd0, ROAD = 3'd1, WALK = 3'd2, WATER = 3'd3,
               BUILDING = 3'd4, TREE = 3'd5, PATH = 3'd6, OUTSIDE = 3'd7;

    function [11:0] ground_rgb;
        input [2:0] t;
        case (t)
            GRASS:    ground_rgb = 12'h5A3;
            ROAD:     ground_rgb = 12'h555;
            WALK:     ground_rgb = 12'h999;
            WATER:    ground_rgb = 12'h37C;
            BUILDING: ground_rgb = 12'h888;
            TREE:     ground_rgb = 12'h5A3;
            PATH:     ground_rgb = 12'hCB8;
            default:  ground_rgb = 12'h483;   // outside the world
        endcase
    endfunction

    function [11:0] mini_rgb;
        input [2:0] t;
        case (t)
            BUILDING: mini_rgb = 12'h336;
            TREE:     mini_rgb = 12'h262;
            default:  mini_rgb = ground_rgb(t);
        endcase
    endfunction

    function [11:0] sky_rgb;            // 4 bands, row v = 0..199
        input [9:0] v;
        sky_rgb = (v < 50) ? 12'h58E : (v < 100) ? 12'h7AF : (v < 150) ? 12'h9CF : 12'hBDF;
    endfunction

    localparam [11:0] FOG = 12'hBDF;
    function [11:0] haze;               // 50 / 50 mix with the fog colour
        input [11:0] c;
        reg   [4:0]  r, g, b;           // 5 bits: the sum of two 4-bit values
        begin
            r = c[11:8] + FOG[11:8];
            g = c[7:4]  + FOG[7:4];
            b = c[3:0]  + FOG[3:0];
            haze = {r[4:1], g[4:1], b[4:1]};   // divide by 2
        end
    endfunction

    // ========================================================================
    // ROMs
    // ========================================================================
    reg [2:0]  map_rom   [0:MAP_N*MAP_N-1];
    reg [31:0] trig_rom  [0:255];
    reg [95:0] floor_rom [0:199];       // {D, K, G}, Q16.16 metres
    initial begin
        $readmemh(MAP_FILE,   map_rom);
        $readmemh(TRIG_FILE,  trig_rom);
        $readmemh(FLOOR_FILE, floor_rom);
    end

    // ========================================================================
    // POSE of the picture on the screen (copied at every buffer swap)
    // ========================================================================
    reg               show_q;
    reg signed [31:0] px8, pz8;         // eye position, Q24.8
    reg        [7:0]  ph;               // heading
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            show_q <= 1'b0; px8 <= START_X8; pz8 <= START_Z8; ph <= START_H;
        end else begin
            show_q <= show_buf;
            if (show_buf != show_q) begin   // the monitor just got a new picture
                px8 <= pose_x8; pz8 <= pose_z8; ph <= pose_h;
            end
        end
    end

    reg  [31:0] trig_q;                 // {sin, cos} of the heading, Q1.14
    always @(posedge clk) trig_q <= trig_rom[ph];
    wire signed [15:0] sn = trig_q[31:16];
    wire signed [15:0] cs = trig_q[15:0];
    wire signed [19:0] sn20 = sn;       // wider copies for the dot offsets below
    wire signed [19:0] cs20 = cs;

    // ========================================================================
    // STAGE 1: delay the input one clock and know where that pixel is
    //   hc = 0..799 pixel in the line, vc = 0..524 line
    // ========================================================================
    reg        hs_q, vs_q;
    reg [11:0] s1_rgb;
    reg        s1_hs, s1_vs;
    reg [9:0]  hc, vc;
    always @(posedge clk) begin
        hs_q   <= in_hs;
        vs_q   <= in_vs;
        s1_rgb <= {in_r, in_g, in_b};
        s1_hs  <= in_hs;
        s1_vs  <= in_vs;
        if (hs_q && !in_hs)   hc <= 10'd656;          // hsync starts at pixel 656
        else if (hc == 10'd799) hc <= 10'd0;
        else                  hc <= hc + 10'd1;
        if (vs_q && !in_vs)   vc <= 10'd490;          // vsync starts at line 490
        else if (hc == 10'd799) vc <= (vc == 10'd524) ? 10'd0 : vc + 10'd1;
    end

    // picture window: 300 x 400 at (170, 40); u, v = position inside it
    wire       in_win = (hc >= 170) && (hc < 470) && (vc >= 40) && (vc < 440);
    wire [9:0] v      = vc - 10'd40;
    wire       ground = (vc >= 240);                  // v >= 200: below the horizon

    // ========================================================================
    // GROUND: row set-up at the start of each line, then one add per pixel
    // ========================================================================
    wire [7:0] frow = (vc >= 240 && vc < 440) ? (vc - 10'd240) : 8'd0;
    reg  [95:0] floor_q;
    always @(posedge clk) floor_q <= floor_rom[frow];
    wire signed [32:0] D = {1'b0, floor_q[95:64]};    // distance of this row
    wire signed [32:0] K = {1'b0, floor_q[63:32]};    // sideways step per pixel
    wire signed [32:0] G = {1'b0, floor_q[31:0]};     // offset of pixel 0

    reg signed [48:0] pDs, pDc, pGs, pGc, pKs, pKc;
    reg signed [31:0] ax, az, bx, bz;                 // row start + step, Q16.16
    reg signed [31:0] fx, fz;                         // ground spot of this pixel
    always @(posedge clk) begin
        if (hc == 10'd4) begin                        // floor_q of this line is ready
            pDs <= D * sn;  pDc <= D * cs;
            pGs <= G * sn;  pGc <= G * cs;
            pKs <= K * sn;  pKc <= K * cs;
        end
        if (hc == 10'd5) begin
            // forward = (sin, -cos), right = (cos, sin)
            ax <= (px8 <<< 8) + (pDs >>> 14) - (pGc >>> 14);
            az <= (pz8 <<< 8) - (pDc >>> 14) - (pGs >>> 14);
            bx <= pKc >>> 14;
            bz <= pKs >>> 14;
        end
        if (hc == 10'd169) begin fx <= ax;      fz <= az;      end   // pixel u = 0 next
        else               begin fx <= fx + bx; fz <= fz + bz; end
    end

    localparam signed [31:0] WLIM = MAP_N <<< 17;     // world size in Q16.16 (2 m cells)
    wire in_world = (fx >= 0) && (fz >= 0) && (fx < WLIM) && (fz < WLIM);
    wire [31:0] ground_cell = (fz >>> 17) * MAP_N + (fx >>> 17);

    // ========================================================================
    // MINI-MAP: 3 x 3 pixels per cell;  n / 3 = (n * 171) >> 9  for n < 144
    // ========================================================================
    localparam MM_W = MAP_N * 3;
    wire       in_mm  = (hc >= MM_X0) && (hc < MM_X0 + MM_W) && (vc >= MM_Y0) && (vc < MM_Y0 + MM_W);
    wire       in_box = (hc >= MM_X0 - 1) && (hc <= MM_X0 + MM_W) && (vc >= MM_Y0 - 1) && (vc <= MM_Y0 + MM_W);
    wire [9:0] mm_dx  = hc - MM_X0;
    wire [9:0] mm_dy  = vc - MM_Y0;
    wire [17:0] mm_cx = (mm_dx * 171) >> 9;
    wire [17:0] mm_cy = (mm_dy * 171) >> 9;
    wire [31:0] mm_cell = mm_cy * MAP_N + mm_cx;

    // you: 1.5 mini-map pixels per metre; heading dots 4, 8, 12 pixels ahead
    reg [9:0] mk_x, mk_y;
    reg signed [9:0] d1x, d1y, d2x, d2y, d3x, d3y;
    always @(posedge clk) begin
        mk_x <= MM_X0 + ((px8 * 3) >>> 9);
        mk_y <= MM_Y0 + ((pz8 * 3) >>> 9);
        d1x  <=  ((sn20 * 4)  >>> 14);    d1y <= -((cs20 * 4)  >>> 14);
        d2x  <=  ((sn20 * 8)  >>> 14);    d2y <= -((cs20 * 8)  >>> 14);
        d3x  <=  ((sn20 * 12) >>> 14);    d3y <= -((cs20 * 12) >>> 14);
    end
    wire [9:0] e1x = mk_x + d1x;
    wire [9:0] e1y = mk_y + d1y;
    wire [9:0] e2x = mk_x + d2x;
    wire [9:0] e2y = mk_y + d2y;
    wire [9:0] e3x = mk_x + d3x;
    wire [9:0] e3y = mk_y + d3y;
    // 2 x 2 dot covering (ex-1 .. ex, ey-1 .. ey)
    function dot;
        input [9:0] h, l, ex, ey;
        dot = ((h == ex) || (h + 10'd1 == ex)) && ((l == ey) || (l + 10'd1 == ey));
    endfunction
    wire me   = (hc + 10'd2 >= mk_x) && (hc <= mk_x + 10'd2) &&
                (vc + 10'd2 >= mk_y) && (vc <= mk_y + 10'd2);
    wire dots = dot(hc, vc, e1x, e1y) || dot(hc, vc, e2x, e2y) || dot(hc, vc, e3x, e3y);

    // ========================================================================
    // STAGE 2: one map read (ground cell inside the picture, mini-map cell
    //          in the margin) + remember what kind of pixel this is
    // ========================================================================
    wire [11:0] map_addr = in_win ? ground_cell[11:0] : mm_cell[11:0];
    reg  [2:0]  map_q;
    reg  [11:0] s2_rgb, s2_sky;
    reg         s2_hs, s2_vs, s2_win, s2_ground, s2_world, s2_fog;
    reg         s2_mm, s2_box, s2_me, s2_dot;
    always @(posedge clk) begin
        map_q     <= map_rom[map_addr];
        s2_rgb    <= s1_rgb;
        s2_hs     <= s1_hs;
        s2_vs     <= s1_vs;
        s2_win    <= in_win;
        s2_ground <= ground;
        s2_world  <= in_world;
        s2_fog    <= (frow < FOG_ROWS);
        s2_sky    <= sky_rgb(v);
        s2_mm     <= map_on && in_mm;
        s2_box    <= map_on && in_box;
        s2_me     <= map_on && me;
        s2_dot    <= map_on && dots;
    end

    // ========================================================================
    // STAGE 3: choose the colour
    // ========================================================================
    wire [11:0] gnd_raw = ground_rgb(s2_world ? map_q : OUTSIDE);
    wire [11:0] gnd     = s2_fog ? haze(gnd_raw) : gnd_raw;

    reg [11:0] col;
    always @* begin
        col = s2_rgb;                                  // the pipeline's pixel
        if (s2_win && s2_rgb == 12'h000)
            col = s2_ground ? gnd : s2_sky;            // nothing drawn here
        if (!s2_win) begin
            if (s2_me)        col = 12'hF00;           // you
            else if (s2_dot)  col = 12'hFF0;           // where you look
            else if (s2_mm)   col = mini_rgb(map_q);   // the map
            else if (s2_box)  col = 12'hFFF;           // white frame
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            out_r <= 4'd0; out_g <= 4'd0; out_b <= 4'd0; out_hs <= 1'b1; out_vs <= 1'b1;
        end else begin
            {out_r, out_g, out_b} <= col;
            out_hs <= s2_hs;
            out_vs <= s2_vs;
        end
    end
endmodule

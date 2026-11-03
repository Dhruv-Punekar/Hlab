`timescale 1ns / 1ps
// ============================================================================
// Module: nav_camera                               (navigation demo helper 1/2)
//
// Keeps WHERE you stand and WHICH WAY you look, moves you a little every
// frame while a button is held, stops you at walls, and builds the camera
// matrix for the pipeline.
//
//   position  pos_x, pos_z : metres in 1/4096 m units (Q.12)
//   heading   0..255       : 0 = north, 64 = east, 128 = south, 192 = west
//                            (1.4 deg per step)
//
// ONE FRAME (start pulse -> done pulse, about 50 clocks)
//   1. TURN : left / right buttons change the heading by TURN_STEP
//             (2x with run), centre button snaps to the nearest N/E/S/W.
//             In strafe mode left / right walk sideways instead of turning.
//   2. STEP : look up sin/cos of the walking direction (nav_trig.mem) and
//             make the step   dx = step*sin,  dz = -step*cos.
//   3. WALLS: try the x part of the step, then the z part. A part is only
//             taken if all 4 corners of your 2*RADIUS wide square are on
//             walkable map cells (nav_map.mem). Doing x and z separately
//             lets you slide along a wall instead of sticking to it.
//   4. MATRIX: read the 16 words of this heading from nav_rot.mem (the
//             rotation + projection part, made by nav_world.py) and add the
//             position part to the 4th column of every row:
//                 m[r][3] = rom[r][3] - (m[r][0]*x + m[r][2]*z) >> 8
//             (x, z in Q24.8). That is all the maths the camera needs.
//
// Read the result with rd_addr / rd_data in matrix_memory order
// (word k = element 15-k: M33 first ... M00 last), like matrix_builder.
// ============================================================================
module nav_camera #(
    parameter MAP_FILE  = "nav_map.mem",   // MAP_N*MAP_N cell codes (1 hex digit each)
    parameter TRIG_FILE = "nav_trig.mem",  // 256 x {sin, cos} Q1.14
    parameter ROT_FILE  = "nav_rot.mem",   // 256 headings x 16 words
    parameter MAP_N     = 48,              // map cells per side (2 m each)
    parameter START_X   = 139264,          // start position, 1/4096 m (34 m)
    parameter START_Z   = 212992,          //                         (52 m)
    parameter START_H   = 0,               // start heading (north)
    parameter WALK_STEP = 205,             // 1/4096 m per frame (3 m/s at 60 fps)
    parameter TURN_STEP = 1,               // heading steps per frame (84 deg/s)
    parameter RADIUS    = 4096             // half width of your square, 1/4096 m (1 m)
)(
    input  wire        clk,
    input  wire        rst_n,

    input  wire        start,      // 1-clock pulse: do one frame
    input  wire        key_fwd,    // buttons, held = keep going
    input  wire        key_back,
    input  wire        key_left,
    input  wire        key_right,
    input  wire        key_snap,   // 1 = snap to N/E/S/W this frame
    input  wire        run,        // double speed
    input  wire        strafe,     // left/right walk sideways

    output reg         done,       // 1-clock pulse: matrix ready
    output reg         changed,    // the pose changed in this frame
    input  wire [3:0]  rd_addr,    // matrix_memory word 0..15
    output wire [31:0] rd_data,

    output wire signed [31:0] pos_x8,   // position, Q24.8 metres
    output wire signed [31:0] pos_z8,
    output reg  [7:0]  heading,
    output reg  [7:0]  walk_dir,   // direction of the last step taken
    output reg         moving,     // walked this frame
    output reg         turning,    // turned this frame
    output reg         bumped      // a wall stopped (part of) the step
);
    // ========================================================================
    // ROMs (all made by nav_world.py)
    // ========================================================================
    localparam GRASS = 3'd0, ROAD = 3'd1, WALK = 3'd2, PATH = 3'd6;

    reg [2:0]  map_rom  [0:MAP_N*MAP_N-1];   // small: LUTs, read 4x at once
    reg [31:0] trig_rom [0:255];
    (* rom_style = "block" *) reg [31:0] rot_rom [0:4095];
    initial begin
        $readmemh(MAP_FILE,  map_rom);
        $readmemh(TRIG_FILE, trig_rom);
        $readmemh(ROT_FILE,  rot_rom);
    end

    // ---- which map cell is the point (px, pz)?  (1/4096 m units) ----
    //   returns {inside the world, cell number}
    localparam signed [31:0] LIM = MAP_N * 8192;      // world size, 2 m = 8192 units per cell
    function [12:0] cell_of;
        input signed [31:0] px, pz;
        reg   [31:0] n;
        begin
            n = (pz >>> 13) * MAP_N + (px >>> 13);    // row * MAP_N + column
            if (px < 0 || pz < 0 || px >= LIM || pz >= LIM)
                cell_of = 13'd0;                      // outside: not walkable
            else
                cell_of = {1'b1, n[11:0]};
        end
    endfunction

    // ---- can you stand on this cell? ----
    function walkable;
        input [12:0] c;                               // from cell_of
        reg   [2:0]  code;
        begin
            code     = map_rom[c[11:0]];
            walkable = c[12] && ((code == GRASS) || (code == ROAD) || (code == WALK) || (code == PATH));
        end
    endfunction

    // ========================================================================
    // STATE
    // ========================================================================
    reg signed [31:0] pos_x, pos_z;               // 1/4096 m
    assign pos_x8 = pos_x >>> 4;                  // -> Q24.8 (1/256 m)
    assign pos_z8 = pos_z >>> 4;

    reg signed [31:0] m [0:15];                   // the matrix, element 4*row + col
    assign rd_data = m[15 - rd_addr];

    // ========================================================================
    // 1. TURN + choose the walking direction (combinational, used in S_IDLE)
    // ========================================================================
    wire [7:0] tstep   = run ? (2 * TURN_STEP) : TURN_STEP;
    wire       turn_r  = !strafe && key_right && !key_left;
    wire       turn_l  = !strafe && key_left  && !key_right;
    wire [7:0] h_turn  = heading + (turn_r ? tstep : 8'd0) - (turn_l ? tstep : 8'd0);
    wire [7:0] h_new   = key_snap ? ((h_turn + 8'd32) & 8'hC0) : h_turn;

    wire go_f = key_fwd  && !key_back;
    wire go_b = key_back && !key_fwd;
    wire go_r = !go_f && !go_b && strafe && key_right && !key_left;
    wire go_l = !go_f && !go_b && strafe && key_left  && !key_right;
    wire walk = go_f || go_b || go_r || go_l;
    wire [7:0] dir = h_new + (go_f ? 8'd0 : go_b ? 8'd128 : go_r ? 8'd64 : 8'd192);

    // ========================================================================
    // FRAME SEQUENCER
    // ========================================================================
    localparam S_IDLE = 4'd0, S_TRIG = 4'd1, S_STEP = 4'd2,
               S_PREPX = 4'd3, S_CHKX = 4'd4, S_PREPZ = 4'd5, S_CHKZ = 4'd6,
               S_ROT = 4'd7, S_MUL = 4'd8, S_ADD = 4'd9;
    reg [3:0] state;

    reg  [7:0]        trig_addr;
    reg  [31:0]       trig_q;                     // {sin, cos}, read 1 clock later
    always @(posedge clk) trig_q <= trig_rom[trig_addr];

    reg  [4:0]        cnt;                        // word counter (S_ROT) / row (S_MUL, S_ADD)
    reg  [31:0]       rot_q;
    always @(posedge clk) rot_q <= rot_rom[{heading, cnt[3:0]}];

    reg  [10:0]       step;                       // length of this step (1/4096 m)
    reg signed [31:0] dx, dz;
    reg signed [35:0] p0, p1;                     // m[r][0]*x, m[r][2]*z

    wire signed [15:0] s_tab = trig_q[31:16];
    wire signed [15:0] c_tab = trig_q[15:0];
    wire signed [31:0] nx = pos_x + dx;           // where the x part of the step goes
    wire signed [31:0] nz = pos_z + dz;
    // the point being tested: (nx, z) for the x part, (x, nz) for the z part
    wire signed [31:0] tx = (state == S_PREPX) ? nx : pos_x;
    wire signed [31:0] tz = (state == S_PREPX) ? pos_z : nz;
    reg  [12:0] c0, c1, c2, c3;                   // the 4 corner cells of your square
    wire box_free = walkable(c0) && walkable(c1) && walkable(c2) && walkable(c3);
    wire [1:0]  row = cnt[1:0];
    // 18-bit operands: matrix entries are below 13000, positions below 96 m
    wire signed [17:0] mx18 = m[{row, 2'd0}];
    wire signed [17:0] mz18 = m[{row, 2'd2}];
    wire signed [17:0] x18  = pos_x8;
    wire signed [17:0] z18  = pos_z8;

    integer i;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE; done <= 1'b0; changed <= 1'b0;
            pos_x <= START_X; pos_z <= START_Z; heading <= START_H; walk_dir <= START_H;
            moving <= 1'b0; turning <= 1'b0; bumped <= 1'b0;
            trig_addr <= 8'd0; cnt <= 5'd0; step <= 11'd0; dx <= 0; dz <= 0;
            p0 <= 0; p1 <= 0;
            c0 <= 13'd0; c1 <= 13'd0; c2 <= 13'd0; c3 <= 13'd0;
            for (i = 0; i < 16; i = i + 1) m[i] <= 32'd0;
        end else begin
            done <= 1'b0;
            case (state)
                // 1. turn, pick the direction, ask the sin/cos table
                S_IDLE: if (start) begin
                    turning   <= (h_new != heading);
                    changed   <= (h_new != heading);
                    heading   <= h_new;
                    trig_addr <= dir;
                    step      <= walk ? (run ? 2 * WALK_STEP : WALK_STEP) : 11'd0;
                    moving    <= 1'b0;
                    bumped    <= 1'b0;
                    if (walk) walk_dir <= dir;
                    state     <= S_TRIG;
                end
                S_TRIG: state <= S_STEP;          // trig_q is being read

                // 2. the step:  dx = step*sin,  dz = -step*cos   (Q1.14 table)
                S_STEP: begin
                    dx    <= ($signed({1'b0, step}) * s_tab) >>> 14;
                    dz    <= -(($signed({1'b0, step}) * c_tab) >>> 14);
                    state <= S_PREPX;
                end

                // 3. walls: x first, then z (slide along walls).
                //    PREP: find the 4 corner cells of the square at the new
                //    point. CHK: read them from the map, move if all walkable.
                S_PREPX, S_PREPZ: begin
                    c0 <= cell_of(tx - RADIUS, tz - RADIUS);
                    c1 <= cell_of(tx + RADIUS, tz - RADIUS);
                    c2 <= cell_of(tx - RADIUS, tz + RADIUS);
                    c3 <= cell_of(tx + RADIUS, tz + RADIUS);
                    state <= (state == S_PREPX) ? S_CHKX : S_CHKZ;
                end
                S_CHKX: begin
                    if (dx != 0) begin
                        if (box_free) begin pos_x <= nx; moving <= 1'b1; changed <= 1'b1; end
                        else bumped <= 1'b1;
                    end
                    state <= S_PREPZ;
                end
                S_CHKZ: begin
                    if (dz != 0) begin
                        if (box_free) begin pos_z <= nz; moving <= 1'b1; changed <= 1'b1; end
                        else bumped <= 1'b1;
                    end
                    cnt   <= 5'd0;
                    state <= S_ROT;
                end

                // 4a. copy the 16 words of this heading (ROM latency 1:
                //     word k arrives when cnt = k+1)
                S_ROT: begin
                    if (cnt != 5'd0) m[cnt - 5'd1] <= rot_q;
                    if (cnt == 5'd16) begin cnt <= 5'd0; state <= S_MUL; end
                    else cnt <= cnt + 5'd1;
                end

                // 4b. position part of row `row`:  m[r][3] -= (m[r][0]*x + m[r][2]*z) >> 8
                S_MUL: begin
                    p0    <= mx18 * x18;
                    p1    <= mz18 * z18;
                    state <= S_ADD;
                end
                S_ADD: begin
                    m[{row, 2'd3}] <= m[{row, 2'd3}] - ((p0 + p1) >>> 8);
                    if (row == 2'd3) begin
                        done  <= 1'b1;
                        state <= S_IDLE;
                    end else begin
                        cnt   <= cnt + 5'd1;
                        state <= S_MUL;
                    end
                end
                default: state <= S_IDLE;
            endcase
        end
    end
endmodule

`timescale 1ns / 1ps
// ============================================================================
// tb_memory.v
// Self-checking testbench for:   vertex_memory   and   matrix_memory
// (entry point of the hardware graphics pipeline, see initial_modules.txt)
//
// Written in Verilog-2001 (+ $clog2 used by the DUT), so it runs on
// Icarus (-g2012), Vivado xsim and ModelSim/Questa without changes.
//
// WHAT IS CHECKED
//   vertex_memory (two instances: DEPTH=64 default, and DEPTH=36 per spec)
//     V01  power-up contents == vertices.mem, X before first read (info)
//     V02  file/format sanity: Q24.8 fields, W=1.0, RGB565 packing, cube geometry
//     V03  synchronous read: 1-cycle latency, no combinational path, en=0 holds,
//          en=0 blocks writes
//     V04  streaming read, 1 vertex/clock, forward / reverse / random order
//     V05  write + read back (incl. overwrite of initial data), we glitch ignored
//     V06  read-during-write returns OLD data (read-first), new data next read
//     V07  full 144-bit data path: zeros, ones, A5/5A, walking-1, walking-0
//     V08  address decode: fill all 64 words, readback, neighbour isolation
//     V09  data_in activity with we=0 never disturbs memory
//     V10  back-to-back writes to same address (last wins)
//     V11  Q24.8 / RGB565 pack-unpack through the memory incl. negative, min, max
//     V12  constrained-random vs. scoreboard (en/we/addr/data)
//     V13  DEPTH=36 instance: out-of-range read behaviour (info only)
//   matrix_memory
//     M01  power-up contents == matrix.mem (checked BEFORE the first clock edge,
//          proves output is pure wiring); identity check (disable with +NOIDENT)
//     M02  output stable with we=0 while addr/data toggle
//     M03  write is synchronous (no change before edge, change right after),
//          we glitch between edges ignored
//     M04  per-element write isolation (16 elements)
//     M05  flattening map: element i lives in matrix_out[i*32 +: 32]
//     M06  full 32-bit data path on every element
//     M07  Phase-2: full 16-word overwrite via write port (spaced, gapped,
//          back-to-back, reverse order)
//     M08  repeated writes to the same element (last wins)
//     M09  constrained-random vs. scoreboard
//
// Only behaviour stated in initial_modules.txt / the two .v files is checked.
//
// PLUSARGS
//   +NOCUBE        skip cube-geometry checks on vertices.mem (use for other meshes)
//   +NOIDENT       skip identity check on matrix.mem   (use for rotated matrix)
//   +SEED=<n>      random seed        (default 1)
//   +NRAND=<n>     random cycles per random test (default 4000)
//
// Run:  python3 gen_mem.py && ./run_sim.sh
// ============================================================================

module tb_memory;

    // ------------------------------------------------------------------
    // Parameters
    // ------------------------------------------------------------------
    localparam CLK_PERIOD = 10;
    localparam NUM_VERTS  = 36;      // cube: 12 triangles * 3
    localparam VDEPTH     = 64;
    localparam VWIDTH     = 144;

    // ------------------------------------------------------------------
    // Clock
    // ------------------------------------------------------------------
    reg clk;
    initial clk = 1'b0;
    always #(CLK_PERIOD/2) clk = ~clk;

    // ------------------------------------------------------------------
    // DUT hook-up
    // ------------------------------------------------------------------
    reg          v_en, v_we;
    reg  [5:0]   v_addr;
    reg  [143:0] v_din;
    wire [143:0] v_dout;     // DEPTH = 64
    wire [143:0] v36_dout;   // DEPTH = 36

    vertex_memory #(.DEPTH(64), .WIDTH(144)) dut_v (
        .clk(clk), .en(v_en), .we(v_we), .addr(v_addr),
        .data_in(v_din), .data_out(v_dout));

    vertex_memory #(.DEPTH(36), .WIDTH(144)) dut_v36 (
        .clk(clk), .en(v_en), .we(v_we), .addr(v_addr),
        .data_in(v_din), .data_out(v36_dout));

    reg          m_we;
    reg  [3:0]   m_addr;
    reg  [31:0]  m_data;
    wire [511:0] m_out;

    matrix_memory dut_m (
        .clk(clk), .we(m_we), .write_addr(m_addr),
        .write_data(m_data), .matrix_out(m_out));

    // ------------------------------------------------------------------
    // Reference models / golden data
    // ------------------------------------------------------------------
    reg  [143:0] vgold   [0:63];   // straight from vertices.mem, never modified
    reg  [143:0] vmodel  [0:63];   // scoreboard for DEPTH=64
    reg  [143:0] vmodel36[0:35];   // scoreboard for DEPTH=36
    reg  [143:0] vmodel_out;
    reg  [143:0] vmodel36_out;
    reg          oob_taint;        // 1 = last read of the 36-deep DUT was out of range

    reg  [31:0]  mgold  [0:15];    // straight from matrix.mem
    reg  [31:0]  mmodel [0:15];
    reg  [31:0]  newm   [0:15];    // matrix prepared for the "UART" tests

    // ------------------------------------------------------------------
    // Bookkeeping
    // ------------------------------------------------------------------
    integer errors, checks;
    integer v_cycles, v_reads, v_writes, v_rdw;
    integer m_cycles, m_writes;
    integer seed, nrand;
    reg     cube_check, ident_check;

    // ------------------------------------------------------------------
    // Helper functions
    // ------------------------------------------------------------------
    function [31:0] rand32;
        input dummy;
        begin
            rand32 = $random(seed);
        end
    endfunction

    function [143:0] rand144;
        input dummy;
        begin
            rand144 = {$random(seed), $random(seed), $random(seed),
                       $random(seed), $random(seed)};
        end
    endfunction

    // real -> Q24.8 (round to nearest, two's complement)
    function [31:0] to_q;
        input real r;
        real tmp;
        begin
            tmp = r * 256.0;
            if (tmp >= 0.0) tmp = tmp + 0.5;
            else            tmp = tmp - 0.5;
            to_q = $rtoi(tmp);
        end
    endfunction

    // Q24.8 -> real
    function real from_q;
        input [31:0] q;
        reg signed [31:0] s;
        begin
            s = q;
            from_q = $itor(s) / 256.0;
        end
    endfunction

    function [15:0] rgb565;
        input [7:0] r;
        input [7:0] g;
        input [7:0] b;
        begin
            rgb565 = {r[7:3], g[7:2], b[7:3]};
        end
    endfunction

    function [511:0] mm_flat;      // flattened scoreboard matrix
        input dummy;
        integer k;
        begin
            for (k = 0; k < 16; k = k + 1)
                mm_flat[k*32 +: 32] = mmodel[k];
        end
    endfunction

    // ------------------------------------------------------------------
    // Check tasks
    // ------------------------------------------------------------------
    task bail_if_too_many;
        begin
            if (errors >= 100) begin
                $display("Too many errors (%0d) - aborting.", errors);
                $display("*** TESTBENCH FAILED ***");
                $finish;
            end
        end
    endtask

    task check;
        input        cond;
        input [8*80-1:0] msg;
        begin
            checks = checks + 1;
            if (cond !== 1'b1) begin
                errors = errors + 1;
                $display("[%0t] FAIL: %0s", $time, msg);
                bail_if_too_many;
            end
        end
    endtask

    task check_v;                  // 144-bit compare
        input [143:0] got;
        input [143:0] exp_v;
        input [8*80-1:0] msg;
        begin
            checks = checks + 1;
            if (got !== exp_v) begin
                errors = errors + 1;
                $display("[%0t] FAIL: %0s\n        got=%h\n        exp=%h", $time, msg, got, exp_v);
                bail_if_too_many;
            end
        end
    endtask

    task check_32;
        input [31:0] got;
        input [31:0] exp_v;
        input [8*80-1:0] msg;
        begin
            checks = checks + 1;
            if (got !== exp_v) begin
                errors = errors + 1;
                $display("[%0t] FAIL: %0s  got=%h exp=%h", $time, msg, got, exp_v);
                bail_if_too_many;
            end
        end
    endtask

    task check_m;                  // 512-bit compare
        input [511:0] got;
        input [511:0] exp_v;
        input [8*80-1:0] msg;
        begin
            checks = checks + 1;
            if (got !== exp_v) begin
                errors = errors + 1;
                $display("[%0t] FAIL: %0s\n        got=%h\n        exp=%h", $time, msg, got, exp_v);
                bail_if_too_many;
            end
        end
    endtask

    task info;
        input [8*80-1:0] msg;
        begin
            $display("[%0t] INFO: %0s", $time, msg);
        end
    endtask

    // ------------------------------------------------------------------
    // vertex_memory bus tasks (scoreboard built in)
    //   * inputs are driven at the falling edge
    //   * 1 ns after driving (still before the rising edge) the output must
    //     NOT have moved -> proves the read is synchronous
    //   * 1 ns after the rising edge the output is compared with the model
    // ------------------------------------------------------------------
    task vm_compare;
        input [8*32-1:0] tag;
        begin
            checks = checks + 1;
            if (v_dout !== vmodel_out) begin
                errors = errors + 1;
                $display("[%0t] FAIL (%0s): vertex_memory[64] en=%b we=%b addr=%0d\n        got=%h\n        exp=%h",
                         $time, tag, v_en, v_we, v_addr, v_dout, vmodel_out);
                bail_if_too_many;
            end
            if (!oob_taint) begin
                checks = checks + 1;
                if (v36_dout !== vmodel36_out) begin
                    errors = errors + 1;
                    $display("[%0t] FAIL (%0s): vertex_memory[36] en=%b we=%b addr=%0d\n        got=%h\n        exp=%h",
                             $time, tag, v_en, v_we, v_addr, v36_dout, vmodel36_out);
                    bail_if_too_many;
                end
            end
        end
    endtask

    task vm_cycle;
        input         e;
        input         w;
        input  [5:0]  a;
        input  [143:0] d;
        begin
            @(negedge clk);
            v_en = e; v_we = w; v_addr = a; v_din = d;
            #1;
            vm_compare("pre-edge, must not change");
            @(posedge clk);
            // ---- reference model: read-first BRAM behaviour ----
            if (e) begin
                vmodel_out = vmodel[a];
                v_reads = v_reads + 1;
                if (w) begin
                    vmodel[a] = d;
                    v_writes = v_writes + 1;
                    v_rdw = v_rdw + 1;
                end
                if (a < 36) begin
                    vmodel36_out = vmodel36[a];
                    if (w) vmodel36[a] = d;
                    oob_taint = 1'b0;
                end else begin
                    oob_taint = 1'b1;
                end
            end
            #1;
            v_cycles = v_cycles + 1;
            vm_compare("post-edge");
        end
    endtask

    // we pulse that is gone again before the clock edge -> must be ignored
    task vm_glitch_we;
        input  [5:0]   a;
        input  [143:0] d;
        begin
            @(negedge clk);
            v_en = 1'b1; v_we = 1'b1; v_addr = a; v_din = d;
            #2;
            v_we = 1'b0;
            @(posedge clk);
            vmodel_out = vmodel[a];
            if (a < 36) begin
                vmodel36_out = vmodel36[a];
                oob_taint = 1'b0;
            end else oob_taint = 1'b1;
            #1;
            v_cycles = v_cycles + 1;
            vm_compare("we glitch");
        end
    endtask

    task vm_idle;
        input integer n;
        integer i;
        begin
            for (i = 0; i < n; i = i + 1)
                vm_cycle(1'b0, 1'b0, v_addr, v_din);
        end
    endtask

    // ------------------------------------------------------------------
    // matrix_memory bus tasks (scoreboard built in)
    // ------------------------------------------------------------------
    task mm_compare;
        input [8*32-1:0] tag;
        begin
            checks = checks + 1;
            if (m_out !== mm_flat(1'b0)) begin
                errors = errors + 1;
                $display("[%0t] FAIL (%0s): matrix_memory we=%b addr=%0d data=%h\n        got=%h\n        exp=%h",
                         $time, tag, m_we, m_addr, m_data, m_out, mm_flat(1'b0));
                bail_if_too_many;
            end
        end
    endtask

    task mm_cycle;
        input         w;
        input  [3:0]  a;
        input  [31:0] d;
        begin
            @(negedge clk);
            m_we = w; m_addr = a; m_data = d;
            #1;
            mm_compare("pre-edge, must not change");
            @(posedge clk);
            if (w) begin
                mmodel[a] = d;
                m_writes = m_writes + 1;
            end
            #1;
            m_cycles = m_cycles + 1;
            mm_compare("post-edge");
        end
    endtask

    task mm_glitch_we;
        input [3:0]  a;
        input [31:0] d;
        begin
            @(negedge clk);
            m_we = 1'b1; m_addr = a; m_data = d;
            #2;
            m_we = 1'b0;
            @(posedge clk);
            #1;
            m_cycles = m_cycles + 1;
            mm_compare("we glitch");
        end
    endtask

    // Fill newm[] with 16 random 32-bit words (stand-in for a matrix sent over UART)
    task set_random_matrix;
        integer k;
        begin
            for (k = 0; k < 16; k = k + 1) newm[k] = rand32(0);
        end
    endtask

    // Write newm[] through the write port, <gap> idle cycles after each word
    task mm_write_all;
        input integer gap;
        integer i, g;
        begin
            for (i = 0; i < 16; i = i + 1) begin
                mm_cycle(1'b1, i[3:0], newm[i]);
                for (g = 0; g < gap; g = g + 1)
                    mm_cycle(1'b0, rand32(0), rand32(0));
            end
        end
    endtask

    task check_matrix_is_newm;
        input [8*80-1:0] msg;
        integer k;
        begin
            for (k = 0; k < 16; k = k + 1)
                check_32(m_out[k*32 +: 32], newm[k], msg);
        end
    endtask

    // ======================================================================
    //                              VERTEX TESTS
    // ======================================================================

    // ---- V01 --------------------------------------------------------------
    task t_v01;
        integer i, loaded;
        reg [143:0] w;
        begin
            $display("--- V01: initial contents of vertex memory ---");
            // Before any read the output register is uninitialised (X in sim)
            if (^v_dout === 1'bx) info("data_out is X before first read (BRAM output reg, expected in simulation)");
            else                  info("data_out is defined before first read");

            loaded = 0;
            for (i = 0; i < 64; i = i + 1) begin
                w = vgold[i];
                if (^w !== 1'bx) loaded = loaded + 1;
            end
            check(loaded >= NUM_VERTS, "vertices.mem provides at least 36 fully-defined words");
            $display("        vertices.mem: %0d defined words", loaded);

            for (i = 0; i < 64; i = i + 1) begin
                vm_cycle(1'b1, 1'b0, i[5:0], 144'd0);
                if (i < NUM_VERTS) begin
                    check_v(v_dout,   vgold[i], "V01 DEPTH=64 word == vertices.mem");
                    check_v(v36_dout, vgold[i], "V01 DEPTH=36 word == vertices.mem");
                end
            end
        end
    endtask

    // ---- V02 --------------------------------------------------------------
    task t_v02;
        integer i, t, f, g, same;
        reg [143:0] w, w1, w2, w3;
        reg [31:0]  x, y, z, ww;
        reg [15:0]  fc [0:5];
        reg [15:0]  c1, c2, c3;
        begin
            $display("--- V02: vertex word format (data in vertices.mem) ---");
            for (i = 0; i < NUM_VERTS; i = i + 1) begin
                w  = vgold[i];
                x  = w[143:112];
                y  = w[111:80];
                z  = w[79:48];
                ww = w[47:16];
                check((^w) !== 1'bx,         "vertex word has no X/Z bits");
                check_32(ww, 32'h0000_0100,  "W field == 1.0 (0x00000100)");
                if (cube_check) begin
                    check(x === 32'h0000_0100 || x === 32'hFFFF_FF00, "X is +-1.0 in Q24.8");
                    check(y === 32'h0000_0100 || y === 32'hFFFF_FF00, "Y is +-1.0 in Q24.8");
                    check(z === 32'h0000_0100 || z === 32'hFFFF_FF00, "Z is +-1.0 in Q24.8");
                end
            end

            if (cube_check) begin
                for (t = 0; t < 12; t = t + 1) begin
                    w1 = vgold[3*t]; w2 = vgold[3*t+1]; w3 = vgold[3*t+2];
                    c1 = w1[15:0];   c2 = w2[15:0];     c3 = w3[15:0];
                    check(c1 === c2 && c2 === c3, "all 3 vertices of a triangle share one RGB565 colour");
                    check(w1 !== w2 && w2 !== w3 && w1 !== w3, "triangle has 3 distinct vertices");
                    same = 0;
                    if (w1[143:112] === w2[143:112] && w2[143:112] === w3[143:112]) same = same + 1;
                    if (w1[111:80]  === w2[111:80]  && w2[111:80]  === w3[111:80])  same = same + 1;
                    if (w1[79:48]   === w2[79:48]   && w2[79:48]   === w3[79:48])   same = same + 1;
                    check(same == 1, "triangle lies on exactly one cube face plane");
                end
                for (f = 0; f < 6; f = f + 1) begin
                    w1 = vgold[6*f]; w2 = vgold[6*f+3];
                    c1 = w1[15:0];   c2 = w2[15:0];
                    check(c1 === c2, "two triangles of a face share the same colour");
                    fc[f] = c1;
                end
                for (f = 0; f < 6; f = f + 1)
                    for (g = f + 1; g < 6; g = g + 1)
                        check(fc[f] !== fc[g], "each cube face has a different colour");
            end
        end
    endtask

    // ---- V03 --------------------------------------------------------------
    task t_v03;
        integer i;
        reg [143:0] hold;
        begin
            $display("--- V03: synchronous read, latency and enable ---");
            // latency: address presented in cycle n -> data after that edge
            vm_cycle(1'b1, 1'b0, 6'd3, 144'd0);
            check_v(v_dout, vgold[3], "latency: data valid 1 cycle after address");
            vm_cycle(1'b1, 1'b0, 6'd17, 144'd0);
            check_v(v_dout, vgold[17], "latency: next address, next data");

            // en = 0 : output must hold even though address changes
            vm_cycle(1'b1, 1'b0, 6'd4, 144'd0);
            hold = v_dout;
            check_v(hold, vgold[4], "hold: reference read");
            for (i = 0; i < 12; i = i + 1) begin
                vm_cycle(1'b0, 1'b0, i*3, rand144(0));
                check_v(v_dout, hold, "en=0 holds data_out");
            end

            // en = 0 with we = 1 : no write may happen
            vm_cycle(1'b0, 1'b1, 6'd10, 144'hDEAD_BEEF_DEAD_BEEF_DEAD_BEEF_DEAD_BEEF_DEAD);
            vm_cycle(1'b1, 1'b0, 6'd10, 144'd0);
            check_v(v_dout, vgold[10], "we=1 while en=0 must not write");

            // en re-asserted -> read resumes with the *current* address
            vm_cycle(1'b0, 1'b0, 6'd20, 144'd0);
            vm_cycle(1'b1, 1'b0, 6'd20, 144'd0);
            check_v(v_dout, vgold[20], "read resumes after en was low");
        end
    endtask

    // ---- V04 --------------------------------------------------------------
    task t_v04;
        integer i, r;
        begin
            $display("--- V04: streaming reads (1 vertex per clock) ---");
            for (i = 0; i < NUM_VERTS; i = i + 1) begin
                vm_cycle(1'b1, 1'b0, i[5:0], 144'd0);
                check_v(v_dout, vgold[i], "stream forward");
            end
            for (i = NUM_VERTS - 1; i >= 0; i = i - 1) begin
                vm_cycle(1'b1, 1'b0, i[5:0], 144'd0);
                check_v(v_dout, vgold[i], "stream reverse");
            end
            for (i = 0; i < 200; i = i + 1) begin
                r = {$random(seed)} % NUM_VERTS;
                vm_cycle(1'b1, 1'b0, r[5:0], 144'd0);
                check_v(v_dout, vgold[r], "random-order read");
            end
        end
    endtask

    // ---- V05 --------------------------------------------------------------
    task t_v05;
        integer i;
        reg [143:0] d;
        reg [5:0]   alist [0:5];
        begin
            $display("--- V05: write and read back ---");
            alist[0] = 6'd0;  alist[1] = 6'd1;  alist[2] = 6'd35;
            alist[3] = 6'd36; alist[4] = 6'd40; alist[5] = 6'd63;
            for (i = 0; i < 6; i = i + 1) begin
                d = rand144(0);
                vm_cycle(1'b1, 1'b1, alist[i], d);
                vm_cycle(1'b1, 1'b0, alist[i], 144'd0);
                check_v(v_dout, d, "write then read back");
            end
            // reading something else afterwards must still give original data
            vm_cycle(1'b1, 1'b0, 6'd2, 144'd0);
            check_v(v_dout, vgold[2], "untouched neighbour of address 1 unchanged");
            vm_cycle(1'b1, 1'b0, 6'd34, 144'd0);
            check_v(v_dout, vgold[34], "untouched neighbour of address 35 unchanged");

            // write-enable glitch that disappears before the clock edge
            vm_cycle(1'b1, 1'b0, 6'd22, 144'd0);
            d = v_dout;
            vm_glitch_we(6'd22, rand144(0));
            check_v(v_dout, d, "glitch read returns old data");
            vm_cycle(1'b1, 1'b0, 6'd22, 144'd0);
            check_v(v_dout, d, "we glitch between edges must not write");
        end
    endtask

    // ---- V06 --------------------------------------------------------------
    task t_v06;
        reg [143:0] old_d, new_d;
        begin
            $display("--- V06: read during write (read-first) ---");
            vm_cycle(1'b1, 1'b0, 6'd25, 144'd0);
            old_d = v_dout;
            new_d = ~old_d;
            vm_cycle(1'b1, 1'b1, 6'd25, new_d);
            check_v(v_dout, old_d, "same-cycle read returns OLD data");
            vm_cycle(1'b1, 1'b0, 6'd25, 144'd0);
            check_v(v_dout, new_d, "next read returns NEW data");
            check_v(v36_dout, new_d, "DEPTH=36 instance behaves identically");
        end
    endtask

    // ---- V07 --------------------------------------------------------------
    task t_v07;
        integer a, k;
        reg [143:0] p;
        reg [5:0] al [0:1];
        begin
            $display("--- V07: full 144-bit data path ---");
            al[0] = 6'd30; al[1] = 6'd63;
            for (a = 0; a < 2; a = a + 1) begin
                p = 144'd0;
                vm_cycle(1'b1, 1'b1, al[a], p);
                vm_cycle(1'b1, 1'b0, al[a], 144'd0); check_v(v_dout, p, "all zeros");
                p = {144{1'b1}};
                vm_cycle(1'b1, 1'b1, al[a], p);
                vm_cycle(1'b1, 1'b0, al[a], 144'd0); check_v(v_dout, p, "all ones");
                p = {36{4'hA}} ^ {36{4'h0}};
                vm_cycle(1'b1, 1'b1, al[a], p);
                vm_cycle(1'b1, 1'b0, al[a], 144'd0); check_v(v_dout, p, "0xAAAA...");
                p = {36{4'h5}};
                vm_cycle(1'b1, 1'b1, al[a], p);
                vm_cycle(1'b1, 1'b0, al[a], 144'd0); check_v(v_dout, p, "0x5555...");
                for (k = 0; k < 144; k = k + 1) begin
                    p = 144'd1 << k;
                    vm_cycle(1'b1, 1'b1, al[a], p);
                    vm_cycle(1'b1, 1'b0, al[a], 144'd0);
                    check_v(v_dout, p, "walking one");
                end
                for (k = 0; k < 144; k = k + 1) begin
                    p = ~(144'd1 << k);
                    vm_cycle(1'b1, 1'b1, al[a], p);
                    vm_cycle(1'b1, 1'b0, al[a], 144'd0);
                    check_v(v_dout, p, "walking zero");
                end
            end
        end
    endtask

    // ---- V08 --------------------------------------------------------------
    task t_v08;
        integer a, n, nn;
        reg [143:0] ex [0:63];
        reg [143:0] d;
        begin
            $display("--- V08: address decoding / isolation ---");
            for (a = 0; a < 64; a = a + 1) begin
                ex[a] = rand144(0);
                vm_cycle(1'b1, 1'b1, a[5:0], ex[a]);
            end
            for (a = 0; a < 64; a = a + 1) begin
                vm_cycle(1'b1, 1'b0, a[5:0], 144'd0);
                check_v(v_dout, ex[a], "fill + readback (unique data per address)");
            end
            // overwrite each word, then confirm the two neighbours did not change
            for (a = 0; a < 64; a = a + 1) begin
                d = rand144(0);
                vm_cycle(1'b1, 1'b1, a[5:0], d);
                ex[a] = d;
                for (nn = -1; nn <= 1; nn = nn + 1) begin
                    n = a + nn;
                    if (n >= 0 && n < 64) begin
                        vm_cycle(1'b1, 1'b0, n[5:0], 144'd0);
                        check_v(v_dout, ex[n], "neighbour isolation");
                    end
                end
            end
            // power-of-two addresses (catches stuck/aliased address bits)
            for (a = 0; a < 6; a = a + 1) begin
                vm_cycle(1'b1, 1'b0, (6'd1 << a), 144'd0);
                check_v(v_dout, ex[1 << a], "single-bit address");
            end
        end
    endtask

    // ---- V09 --------------------------------------------------------------
    task t_v09;
        integer i;
        reg [143:0] first;
        begin
            $display("--- V09: data_in activity with we=0 ---");
            vm_cycle(1'b1, 1'b0, 6'd12, 144'd0);
            first = v_dout;
            for (i = 0; i < 25; i = i + 1) begin
                vm_cycle(1'b1, 1'b0, 6'd12, rand144(0));
                check_v(v_dout, first, "data_in toggling with we=0 must not change memory");
            end
        end
    endtask

    // ---- V10 --------------------------------------------------------------
    task t_v10;
        reg [143:0] a, b, c;
        begin
            $display("--- V10: back-to-back writes ---");
            a = rand144(0); b = rand144(0); c = rand144(0);
            vm_cycle(1'b1, 1'b1, 6'd7, a);
            vm_cycle(1'b1, 1'b1, 6'd7, b);
            check_v(v_dout, a, "2nd write: read-first returns first write");
            vm_cycle(1'b1, 1'b1, 6'd7, c);
            check_v(v_dout, b, "3rd write: read-first returns second write");
            vm_cycle(1'b1, 1'b0, 6'd7, 144'd0);
            check_v(v_dout, c, "last write wins");
        end
    endtask

    // ---- V11 --------------------------------------------------------------
    task t_v11;
        integer i;
        real tv [0:7];
        real rr;
        reg [143:0] pk;
        reg [15:0]  col;
        begin
            $display("--- V11: Q24.8 / RGB565 pack-unpack through memory ---");
            tv[0] = -1.5;
            tv[1] = -0.00390625;                 // -1/256 = smallest negative step
            tv[2] = 0.0;
            tv[3] = 0.00390625;                  // +1/256 = smallest positive step
            tv[4] = 1.0;
            tv[5] = 100.25;
            tv[6] = 8388607.99609375;            // max  Q24.8  (0x7FFFFFFF)
            tv[7] = -8388608.0;                  // min  Q24.8  (0x80000000)
            for (i = 0; i < 8; i = i + 1) begin
                col = rgb565(i*32, 255 - i*32, i*7);
                pk  = {to_q(tv[i]), to_q(tv[(i+1)%8]), to_q(tv[(i+2)%8]),
                       to_q(tv[(i+3)%8]), col};
                vm_cycle(1'b1, 1'b1, 6'd50, pk);
                vm_cycle(1'b1, 1'b0, 6'd50, 144'd0);
                check_v(v_dout, pk, "packed vertex round-trips bit-exact");
                rr = from_q(v_dout[143:112]); check(rr == tv[i],         "X value (Q24.8 -> real)");
                rr = from_q(v_dout[111:80]);  check(rr == tv[(i+1)%8],   "Y value (Q24.8 -> real)");
                rr = from_q(v_dout[79:48]);   check(rr == tv[(i+2)%8],   "Z value (Q24.8 -> real)");
                rr = from_q(v_dout[47:16]);   check(rr == tv[(i+3)%8],   "W value (Q24.8 -> real)");
                check(v_dout[15:11] === col[15:11], "R field = bits [15:11]");
                check(v_dout[10:5]  === col[10:5],  "G field = bits [10:5]");
                check(v_dout[4:0]   === col[4:0],   "B field = bits [4:0]");
            end
            // spot-check the conversion helpers themselves
            check_32(to_q(-1.0),   32'hFFFF_FF00, "to_q(-1.0)   == 0xFFFFFF00");
            check_32(to_q(1.0),    32'h0000_0100, "to_q(1.0)    == 0x00000100");
            check_32(to_q(-0.5),   32'hFFFF_FF80, "to_q(-0.5)   == 0xFFFFFF80");
            check_32(to_q(8388607.99609375), 32'h7FFF_FFFF, "to_q(max)     == 0x7FFFFFFF");
            check_32(to_q(-8388608.0),       32'h8000_0000, "to_q(min)     == 0x80000000");
            check(rgb565(8'hFF, 8'h00, 8'h00) === 16'hF800, "RGB565 red   == 0xF800");
            check(rgb565(8'h00, 8'hFF, 8'h00) === 16'h07E0, "RGB565 green == 0x07E0");
            check(rgb565(8'h00, 8'h00, 8'hFF) === 16'h001F, "RGB565 blue  == 0x001F");
        end
    endtask

    // ---- V12 --------------------------------------------------------------
    task t_v12;
        integer i, r;
        reg e, w;
        reg [5:0] a;
        begin
            $display("--- V12: constrained random (%0d cycles) ---", nrand);
            for (i = 0; i < nrand; i = i + 1) begin
                r = $random(seed);
                e = (r[1:0] != 2'b00);            // ~75 % enabled
                w = r[2];                         // ~50 % writes
                a = r[10:5];
                if (r[15:12] == 4'd0) a = (r[19:16] % 2) ? 6'd0 : 6'd63;   // hammer boundaries
                vm_cycle(e, w, a, rand144(0));
            end
        end
    endtask

    // ---- V13 --------------------------------------------------------------
    task t_v13;
        integer i, xs;
        begin
            $display("--- V13: DEPTH=36 instance, out-of-range addresses (info) ---");
            xs = 0;
            for (i = 36; i < 64; i = i + 1) begin
                vm_cycle(1'b1, 1'b0, i[5:0], 144'd0);
                if (^v36_dout === 1'bx) xs = xs + 1;
            end
            $display("        %0d of 28 out-of-range reads returned X (X is the expected simulator result)", xs);
            // instance must recover cleanly afterwards
            vm_cycle(1'b1, 1'b0, 6'd0, 144'd0);
            vm_cycle(1'b1, 1'b0, 6'd35, 144'd0);
            check_v(v36_dout, vmodel36[35], "DEPTH=36 recovers after OOB reads");
        end
    endtask

    // ======================================================================
    //                              MATRIX TESTS
    // ======================================================================

    // ---- M01 --------------------------------------------------------------
    task t_m01;
        integer i;
        begin
            $display("--- M01: initial matrix contents (before first clock edge) ---");
            for (i = 0; i < 16; i = i + 1) begin
                check((^mgold[i]) !== 1'bx, "matrix.mem word has no X/Z bits");
                check_32(m_out[i*32 +: 32], mgold[i], "matrix_out element == matrix.mem");
            end
            if (ident_check) begin
                for (i = 0; i < 16; i = i + 1) begin
                    if (i % 5 == 0) check_32(m_out[i*32 +: 32], 32'h0000_0100, "identity: diagonal == 1.0");
                    else            check_32(m_out[i*32 +: 32], 32'h0000_0000, "identity: off-diagonal == 0");
                end
            end
        end
    endtask

    // ---- M02 --------------------------------------------------------------
    task t_m02;
        integer i;
        begin
            $display("--- M02: output stable when we=0 ---");
            for (i = 0; i < 40; i = i + 1)
                mm_cycle(1'b0, rand32(0), rand32(0));
        end
    endtask

    // ---- M03 --------------------------------------------------------------
    task t_m03;
        reg [511:0] pre;
        begin
            $display("--- M03: write timing ---");
            pre = m_out;
            @(negedge clk);
            m_we = 1'b1; m_addr = 4'd5; m_data = 32'hDEAD_BEEF;
            #1;
            check_m(m_out, pre, "no change before the rising edge");
            @(posedge clk); #1;
            mmodel[5] = 32'hDEAD_BEEF;
            m_cycles = m_cycles + 1; m_writes = m_writes + 1;
            check_32(m_out[5*32 +: 32], 32'hDEAD_BEEF, "element visible right after the edge");
            mm_compare("after M03 write");
            mm_cycle(1'b0, 4'd0, 32'd0);

            pre = m_out;
            mm_glitch_we(4'd9, 32'h1234_5678);
            check_m(m_out, pre, "we glitch between edges must not write");
            mm_cycle(1'b0, 4'd0, 32'd0);
            check_m(m_out, pre, "still unchanged one cycle later");
        end
    endtask

    // ---- M04 --------------------------------------------------------------
    task t_m04;
        integer i, j;
        reg [511:0] pre, post;
        reg [31:0]  v;
        begin
            $display("--- M04: per-element write isolation ---");
            for (i = 0; i < 16; i = i + 1) begin
                pre = m_out;
                v   = rand32(0) | 32'h8000_0001;
                mm_cycle(1'b1, i[3:0], v);
                post = m_out;
                for (j = 0; j < 16; j = j + 1) begin
                    if (j == i) check_32(post[j*32 +: 32], v,                "written element holds new value");
                    else        check_32(post[j*32 +: 32], pre[j*32 +: 32],  "other elements unchanged");
                end
            end
        end
    endtask

    // ---- M05 --------------------------------------------------------------
    task t_m05;
        integer i;
        reg [31:0] v;
        begin
            $display("--- M05: flattening map (element i -> bits [32i+31:32i]) ---");
            for (i = 0; i < 16; i = i + 1) begin
                v = 32'hC0DE_0000 | (i << 8) | i;
                mm_cycle(1'b1, i[3:0], v);
            end
            for (i = 0; i < 16; i = i + 1)
                check_32(m_out[i*32 +: 32], (32'hC0DE_0000 | (i << 8) | i), "slice i holds element i");
            check_32(m_out[31:0],    32'hC0DE_0000, "element 0  at bits [31:0]");
            check_32(m_out[511:480], 32'hC0DE_0F0F, "element 15 at bits [511:480]");
        end
    endtask

    // ---- M06 --------------------------------------------------------------
    task t_m06;
        integer i, k;
        reg [31:0] p;
        begin
            $display("--- M06: full 32-bit data path on every element ---");
            for (i = 0; i < 16; i = i + 1) begin
                p = 32'h0000_0000; mm_cycle(1'b1, i[3:0], p); check_32(m_out[i*32 +: 32], p, "zeros");
                p = 32'hFFFF_FFFF; mm_cycle(1'b1, i[3:0], p); check_32(m_out[i*32 +: 32], p, "ones");
                p = 32'h5555_5555; mm_cycle(1'b1, i[3:0], p); check_32(m_out[i*32 +: 32], p, "0x55555555");
                p = 32'hAAAA_AAAA; mm_cycle(1'b1, i[3:0], p); check_32(m_out[i*32 +: 32], p, "0xAAAAAAAA");
                for (k = 0; k < 32; k = k + 1) begin
                    p = 32'd1 << k;  mm_cycle(1'b1, i[3:0], p); check_32(m_out[i*32 +: 32], p, "walking one");
                    p = ~(32'd1 << k); mm_cycle(1'b1, i[3:0], p); check_32(m_out[i*32 +: 32], p, "walking zero");
                end
            end
        end
    endtask

    // ---- M07 --------------------------------------------------------------
    // Phase-2 behaviour from the spec: 16 x 32-bit words overwrite the matrix
    // memory through the write port (byte assembly is the UART receiver's job
    // and is not part of these two modules, so it is not modelled here).
    task t_m07;
        integer i, g, k;
        reg [511:0] old_m, partial, new_flat;
        begin
            $display("--- M07: full-matrix overwrite through the write port ---");

            // (a) 4 cycles between words
            set_random_matrix;
            for (k = 0; k < 16; k = k + 1) new_flat[k*32 +: 32] = newm[k];
            old_m = m_out;
            for (i = 0; i < 16; i = i + 1) begin
                mm_cycle(1'b1, i[3:0], newm[i]);
                for (g = 0; g < 3; g = g + 1) mm_cycle(1'b0, rand32(0), rand32(0));
                if (i == 7) partial = m_out;
            end
            check_matrix_is_newm("matrix after overwrite (4-cycle spacing)");
            if (partial !== old_m && partial !== new_flat)
                info("matrix_out shows a MIXED old/new matrix mid-update (no double buffering)");

            // (b) one-cycle gaps
            set_random_matrix;
            mm_write_all(1);
            check_matrix_is_newm("overwrite with 1-cycle gaps");

            // (c) back-to-back: we stays high, address increments every clock
            set_random_matrix;
            mm_write_all(0);
            check_matrix_is_newm("back-to-back overwrite");

            // (d) reverse order
            set_random_matrix;
            for (i = 15; i >= 0; i = i - 1)
                mm_cycle(1'b1, i[3:0], newm[i]);
            check_matrix_is_newm("reverse order overwrite");
        end
    endtask

    // ---- M08 --------------------------------------------------------------
    task t_m08;
        reg [31:0] a, b, c;
        begin
            $display("--- M08: repeated writes to the same element ---");
            a = rand32(0); b = rand32(0); c = rand32(0);
            mm_cycle(1'b1, 4'd3, a);
            mm_cycle(1'b1, 4'd3, b);
            mm_cycle(1'b1, 4'd3, c);
            check_32(m_out[3*32 +: 32], c, "last write wins");
            mm_cycle(1'b1, 4'd3, c);           // same value again
            check_32(m_out[3*32 +: 32], c, "re-writing same value is harmless");
            mm_cycle(1'b1, 4'd0,  32'h1111_1111);
            mm_cycle(1'b1, 4'd15, 32'h2222_2222);
            mm_cycle(1'b1, 4'd0,  32'h3333_3333);
            check_32(m_out[31:0],    32'h3333_3333, "element 0 overwritten");
            check_32(m_out[511:480], 32'h2222_2222, "element 15 unaffected by element 0 rewrite");
        end
    endtask

    // ---- M09 --------------------------------------------------------------
    task t_m09;
        integer i, r;
        begin
            $display("--- M09: constrained random (%0d cycles) ---", nrand);
            for (i = 0; i < nrand; i = i + 1) begin
                r = $random(seed);
                mm_cycle(r[0], r[7:4], rand32(0));
            end
        end
    endtask

    // ======================================================================
    //                                MAIN
    // ======================================================================
    integer ii;

    initial begin
        // ---- init bookkeeping ----
        errors = 0; checks = 0;
        v_cycles = 0; v_reads = 0; v_writes = 0; v_rdw = 0;
        m_cycles = 0; m_writes = 0;
        seed = 1; nrand = 4000;
        cube_check = 1'b1; ident_check = 1'b1;
        if ($test$plusargs("NOCUBE"))  cube_check  = 1'b0;
        if ($test$plusargs("NOIDENT")) ident_check = 1'b0;
        if ($value$plusargs("SEED=%d",  seed))  ;
        if ($value$plusargs("NRAND=%d", nrand)) ;

        // ---- idle bus ----
        v_en = 1'b0; v_we = 1'b0; v_addr = 6'd0; v_din = 144'd0;
        m_we = 1'b0; m_addr = 4'd0; m_data = 32'd0;
        oob_taint = 1'b1;

        // ---- golden data straight from the same files the DUTs load ----
        $readmemh("vertices.mem", vgold);
        $readmemh("matrix.mem",   mgold);
        for (ii = 0; ii < 64; ii = ii + 1) vmodel[ii] = vgold[ii];
        for (ii = 0; ii < 36; ii = ii + 1) vmodel36[ii] = vgold[ii];
        for (ii = 0; ii < 16; ii = ii + 1) mmodel[ii] = mgold[ii];
        vmodel_out = {144{1'bx}};
        vmodel36_out = {144{1'bx}};

        $display("================================================================");
        $display(" tb_memory : vertex_memory + matrix_memory");
        $display(" seed=%0d  nrand=%0d  cube_check=%0d  ident_check=%0d", seed, nrand, cube_check, ident_check);
        $display("================================================================");

        $dumpfile("tb_memory.vcd");
        $dumpvars(0, tb_memory);

        // Matrix power-up check happens at t=1 ns, i.e. BEFORE the first clock edge
        #1;
        t_m01;

        // ---- read-only vertex tests first (memory still holds the file) ----
        t_v01;
        t_v02;
        t_v03;
        t_v04;

        // ---- write tests ----
        t_v05;
        t_v06;
        t_v07;
        t_v08;
        t_v09;
        t_v10;
        t_v11;
        t_v12;
        t_v13;

        // ---- matrix tests ----
        t_m02;
        t_m03;
        t_m04;
        t_m05;
        t_m06;
        t_m07;
        t_m08;
        t_m09;

        vm_idle(3);

        // ---- summary ----
        $display("================================================================");
        $display(" vertex_memory : %0d cycles, %0d reads, %0d writes (%0d read-during-write)",
                 v_cycles, v_reads, v_writes, v_rdw);
        $display(" matrix_memory : %0d cycles, %0d writes", m_cycles, m_writes);
        $display(" checks        : %0d", checks);
        $display(" errors        : %0d", errors);
        if (errors == 0) $display(" *** ALL TESTS PASSED ***");
        else             $display(" *** TESTBENCH FAILED ***");
        $display("================================================================");
        $finish;
    end

    // ---- watchdog ----
    initial begin
        #200_000_000;
        $display("TIMEOUT - simulation did not finish");
        $display("*** TESTBENCH FAILED ***");
        $finish;
    end

endmodule
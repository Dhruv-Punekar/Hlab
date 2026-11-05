`timescale 1ns / 1ps

// ============================================================================
// tb_tt_game_core.v - tt_game_core against the Python reference model
//
// Runs the game core alone with a fast frame tick and writes one log line per
// frame in the same format as tt_game_model.py (golden_<scenario>.log), so
// the two can be compared with diff:
//   +SCEN=0 auto  : both players computer                 (3000 frames)
//   +SCEN=1 human : P1 by scripted button presses, P2 computer (3000)
//   +SCEN=2 long  : computer vs computer, new-game button at step 700 (40000)
//   +SCEN=3 rotation : both by buttons, receiver never returns (4000)
//   +SCEN=4 tilt  : both computer, P1 aims with a sweeping tilt (3000)
// Output: core_<scenario>.log
// ============================================================================
module tb_tt_game_core;

    reg clk = 1'b0, rst_n = 1'b0;
    always #20 clk = ~clk;

    reg        frame_tick = 1'b0;
    reg  [1:0] btn_fore = 2'b00, btn_back = 2'b00;
    reg        btn_new = 1'b0;
    reg  [1:0] auto_play = 2'b00;
    reg        tilt_en = 1'b0;
    reg  [15:0] tilt_x = 16'd0;
    wire       snd_hit, snd_bounce, snd_point;
    reg  [2:0] evs;
    wire       ev_clear;
    wire [4:0] ev_mask;

    wire [15:0] rp_x, rp_y, rp_z, ro_x, ro_y, ro_z, bl_x, bl_y, bl_z;
    wire [5:0]  rp_yaw, ro_yaw, bl_yaw;
    wire        bl_vis, req_toggle, server, winner, step_busy;
    wire [6:0]  score0, score1;
    wire [2:0]  state;

    tt_game_core dut (
        .clk (clk), .rst_n (rst_n), .frame_tick (frame_tick),
        .btn_fore (btn_fore), .btn_back (btn_back), .btn_new (btn_new),
        .auto_play (auto_play), .tilt_en (tilt_en), .tilt_x (tilt_x),
        .ev_clear (ev_clear), .ev_mask (ev_mask),
        .rp_x (rp_x), .rp_y (rp_y), .rp_z (rp_z), .rp_yaw (rp_yaw),
        .ro_x (ro_x), .ro_y (ro_y), .ro_z (ro_z), .ro_yaw (ro_yaw),
        .bl_x (bl_x), .bl_y (bl_y), .bl_z (bl_z), .bl_yaw (bl_yaw), .bl_vis (bl_vis),
        .req_toggle (req_toggle),
        .score0 (score0), .score1 (score1), .server (server), .state (state),
        .winner (winner), .step_busy (step_busy),
        .snd_hit (snd_hit), .snd_bounce (snd_bounce), .snd_point (snd_point)
    );

    integer scen, frames, k, fd, hseq_n, toggles;
    integer hseq [0:11];

    // signed 16-bit as integer
    function integer s16;  input [15:0] v;  begin s16 = v[15] ? v - 65536 : v; end endfunction

    initial begin
        hseq[0] = 9;  hseq[1] = 4;  hseq[2] = 15; hseq[3] = 9;  hseq[4] = 20; hseq[5] = 9;
        hseq[6] = 3;  hseq[7] = 2;  hseq[8] = 16; hseq[9] = 8;  hseq[10] = 17; hseq[11] = 6;
        if (!$value$plusargs("SCEN=%d", scen)) scen = 0;
        case (scen)
            0: begin fd = $fopen("core_auto.log", "w");  frames = 3000;  auto_play = 2'b11; end
            1: begin fd = $fopen("core_human.log", "w"); frames = 3000;  auto_play = 2'b10; end
            2: begin fd = $fopen("core_long.log", "w");  frames = 40000; auto_play = 2'b11; end
            3: begin fd = $fopen("core_rotation.log", "w"); frames = 4000; auto_play = 2'b00; end
            default: begin fd = $fopen("core_tilt.log", "w"); frames = 3000; auto_play = 2'b11; tilt_en = 1'b1; end
        endcase
        hseq_n = 0;
        toggles = 0;
        #100 rst_n = 1'b1;
        repeat (4) @(posedge clk);

        for (k = 0; k < frames; k = k + 1) begin
            // ---- scripted presses, decided on the state before the step
            btn_fore = 2'b00;  btn_back = 2'b00;  btn_new = 1'b0;
            if (scen == 1) begin
                if (dut.state == 0 && dut.server == 0 && dut.wait_cnt == 30) btn_back[0] = 1'b1;
                if (dut.state == 2 && dut.recv == 0 && !dut.swung && !dut.miss &&
                    $signed(dut.tta) == hseq[hseq_n % 12]) begin
                    if (!dut.hx[31]) btn_fore[0] = 1'b1; else btn_back[0] = 1'b1;
                    hseq_n = hseq_n + 1;
                end
            end
            if (scen == 2 && k == 700) btn_new = 1'b1;
            if (scen == 3 && dut.state == 0 && dut.wait_cnt == 20) begin   // rotation_script
                if (dut.server == 0) btn_fore[0] = 1'b1; else btn_back[1] = 1'b1;
            end

            if (scen == 4) tilt_x = ((k % 240) - 120) * 9;     // tilt_wave() in the model
            // ---- one frame step
            evs = 3'b000;
            @(negedge clk) frame_tick = 1'b1;
            @(negedge clk) frame_tick = 1'b0;
            @(negedge clk) btn_fore = 2'b00;  btn_back = 2'b00;  btn_new = 1'b0;
            wait (!dut.step_busy);
            @(negedge clk);
            toggles = toggles + 1;
            $fdisplay(fd, "%0d st %0d sc %0d %0d srv %0d tta %0d | P %0d %0d %0d %0d 1 | O %0d %0d %0d %0d 1 | B %0d %0d %0d %0d %0d | E %0d %0d %0d",
                      k + 1, state, score0, score1, server, $signed(dut.tta),
                      s16(rp_x), s16(rp_y), s16(rp_z), rp_yaw,
                      s16(ro_x), s16(ro_y), s16(ro_z), ro_yaw,
                      s16(bl_x), s16(bl_y), s16(bl_z), bl_yaw, bl_vis, evs[0], evs[1], evs[2]);
            repeat (20) @(posedge clk);
        end
        $fclose(fd);
        $display("tb_tt_game_core scenario %0d: %0d frames, score %0d-%0d, longest step %0d clocks", scen, frames, score0, score1, tmax);
        $finish;
    end

    always @(posedge clk) if (snd_hit | snd_bounce | snd_point) evs = evs | {snd_point, snd_bounce, snd_hit};

    // step length (clocks) - must stay far below one frame
    integer t0, tmax = 0;
    always @(posedge clk) begin
        if (frame_tick) t0 = $time;
        if (dut.u == 12 && ($time - t0) / 40 > tmax) begin
            tmax = ($time - t0) / 40;
        end
    end

endmodule

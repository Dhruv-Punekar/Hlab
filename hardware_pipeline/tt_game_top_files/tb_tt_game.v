`timescale 1ns / 1ps
// ============================================================================
// tb_tt_game.v - whole game (tt_game: pipeline, game core, animator, overlay)
// with the real clocks: 100 MHz render, 25.175 MHz pixel.
//   * P1 serves with BTNR at frame 2, then the frames run on their own
//   * per frame: captured poses, object vertices in vertex memory, render
//     time and overrun flag -> frames.txt (check: tt_game_model.py --check-sim frames.txt)
//   * BTNC short press at frame 5 -> side view (view 1)
//   * the VGA output (with the score panel) of chosen frames -> screen_a.ppm (frame 4, view 0), screen_b.ppm (last frame, view 1)
// Debounce shortened (DB_BITS = 4) so the press is seen at once.
// ============================================================================
module tb_tt_game;
    reg clk_r = 0, clk_v = 0, arst_n = 0;
    always #5 clk_r = ~clk_r;
    always #19.861 clk_v = ~clk_v;

    reg  [4:0] btn = 5'd0;          // {C, U, L, R, D}
    reg  [5:0] sw  = 6'b000000;
    wire [15:0] led;
    wire [3:0] r, g, b;
    wire hs, vs;

    tt_game #(.DB_BITS (4)) dut (
        .clk_render (clk_r), .clk_vga (clk_v), .arst_n (arst_n),
        .btn (btn), .sw (sw), .led (led),
        .acl_sclk (), .acl_mosi (), .acl_miso (1'b0), .acl_csn (),
        .aud_pwm_o (), .aud_sd (),
        .vga_r (r), .vga_g (g), .vga_b (b), .vga_hs (hs), .vga_vs (vs));

`include "tt_scene_map.vh"
    integer NFR;
    integer fr = 0, fd, i, x, y;
    reg [11:0] scr [0:640*480-1];
    reg [9:0]  hx1, hx2, vy1, vy2;

    // screen capture from the pins (overlay output = its input position + 2 clocks)
    always @(posedge clk_v) begin
        hx1 <= dut.u_overlay.hx;  hx2 <= hx1;
        vy1 <= dut.u_overlay.vy;  vy2 <= vy1;
        if (hx2 < 640 && vy2 < 480) scr[vy2 * 640 + hx2] <= {r, g, b};
    end

    task dump_screen;  input integer n;  integer fp;  begin
        fp = (n == 4) ? $fopen("screen_a.ppm", "w") : $fopen("screen_b.ppm", "w");
        $fwrite(fp, "P3\n640 480\n15\n");
        for (i = 0; i < 640 * 480; i = i + 1)
            $fwrite(fp, "%0d %0d %0d\n", scr[i][11:8], scr[i][7:4], scr[i][3:0]);
        $fclose(fp);
    end endtask

    // per frame record, at line 40 (render must be finished by then)
    always @(posedge clk_v) if (dut.tick_40 && dut.u_core.req_toggle !== 1'bx && fr > 0) begin
        $fwrite(fd, "F %0d rstate %0d busy %0d overrun %0d render_cycles %0d frames_rendered %0d score %0d %0d state %0d\n",
                fr, dut.rstate, dut.rbusy_r, dut.overrun, dut.render_cycles, dut.frames_rendered,
                dut.score0, dut.score1, dut.gstate);
        $fwrite(fd, "P %h %h %h %h %h %h %h %h %h %h %h %h %h\n",
                dut.c_rp_x, dut.c_rp_y, dut.c_rp_z, dut.c_rp_yaw, dut.c_ro_x, dut.c_ro_y, dut.c_ro_z, dut.c_ro_yaw,
                dut.c_bl_x, dut.c_bl_y, dut.c_bl_z, dut.c_bl_yaw, dut.c_bl_vis);
        for (i = TT_N_STATIC; i < TT_NUM_VERTICES; i = i + 1)
            $fwrite(fd, "V %h\n", dut.u_pipeline.u_vertex_memory.ram[i]);
    end

    initial forever begin #1000000; $display("t = %0d ms", $time / 1000000); $fflush; end

    always @(posedge clk_v) if (dut.tick_440) begin
        fr = fr + 1;
        $display("[%0t] frame %0d  render_cycles(last) %0d  overrun %0d  score %0d-%0d  state %0d",
                 $time, fr, dut.render_cycles, dut.overrun, dut.score0, dut.score1, dut.gstate);
        $fflush;
    end

    initial begin
        if (!$value$plusargs("NFR=%d", NFR)) NFR = 10;
        fd = $fopen("frames.txt", "w");
        #200 arst_n = 1;
        wait (fr == 2);
        #1000 btn[1] = 1'b1;               // BTNR: P1 forehand serve
        #3000 btn[1] = 1'b0;
        wait (fr == 5);  #1000 btn[4] = 1'b1;   // BTNC short press -> side view
        wait (fr == 6);  #1000 btn[4] = 1'b0;
        while (fr < NFR) begin
            @(posedge clk_v);
            if (dut.tick_440 && (fr == 4 || fr == NFR - 1)) dump_screen(fr);
        end
        $fclose(fd);
        $display("done: %0d frames, overrun %0d", fr, dut.overrun);
        $finish;
    end
endmodule

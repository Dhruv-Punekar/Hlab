`timescale 1ns/1ps
// Self-checking-by-comparison testbench: streams N triangles from stim.hex
// through DUT with random in_valid / out_ready, logs every accepted output.
// Compile with -DDUT=<module name> -DOUTFILE="..." .
module tb;
    parameter N = 4000;
    reg clk = 0, rst_n = 0;
    always #5 clk = ~clk;

    reg [31:0] stim [0:N*10-1];
    initial $readmemh("stim.hex", stim);

    reg in_valid = 0; wire in_ready;
    reg out_ready = 0; wire out_valid;
    reg signed [31:0] x0,y0,z0,x1,y1,z1,x2,y2,z2; reg [15:0] col;

    wire [9:0] bxmin,bxmax,bymin,bymax;
    wire signed [31:0] e0a,e0b,e1a,e1b,e2a,e2b,za,zb,zc;
    wire signed [33:0] e0c,e1c,e2c;
    wire [15:0] fc;
    wire ov, ovxy, ovec, ova, ovb, ovc, ovd, bcl, oneg, deg;

    `DUT dut(.clk(clk),.rst_n(rst_n),.in_valid(in_valid),.in_ready(in_ready),
        .in_x0(x0),.in_y0(y0),.in_z0(z0),.in_x1(x1),.in_y1(y1),.in_z1(z1),
        .in_x2(x2),.in_y2(y2),.in_z2(z2),.in_face_color(col),
        .out_valid(out_valid),.out_ready(out_ready),
        .bbox_xmin(bxmin),.bbox_xmax(bxmax),.bbox_ymin(bymin),.bbox_ymax(bymax),
        .edge0_A(e0a),.edge0_B(e0b),.edge0_C(e0c),
        .edge1_A(e1a),.edge1_B(e1b),.edge1_C(e1c),
        .edge2_A(e2a),.edge2_B(e2b),.edge2_C(e2c),
        .zA(za),.zB(zb),.zC(zc),.face_color(fc),
        .overflow(ov),.overflow_xy_range(ovxy),.overflow_edge_c(ovec),
        .overflow_zA(ova),.overflow_zB(ovb),.overflow_zC(ovc),
        .overflow_depth(ovd),.bbox_clamped(bcl),.orient_neg(oneg),.degenerate(deg));

    integer sent = 0, got = 0, cyc = 0, fd, seed = 1;
    integer first_in_cyc = -1, first_out_cyc = -1;
    integer full_rate_start, full_rate_outs;

    task load(input integer i);
        begin
            x0 <= stim[i*10+0]; y0 <= stim[i*10+1]; z0 <= stim[i*10+2];
            x1 <= stim[i*10+3]; y1 <= stim[i*10+4]; z1 <= stim[i*10+5];
            x2 <= stim[i*10+6]; y2 <= stim[i*10+7]; z2 <= stim[i*10+8];
            col <= stim[i*10+9][15:0];
        end
    endtask

    initial begin
        fd = $fopen(`OUTFILE, "w");
        repeat (3) @(posedge clk);
        load(0);
        #1 rst_n = 1;
    end

    // Phase 1 (first 64 triangles): in_valid=1, out_ready=1 -> latency + II=1.
    // Phase 2: random valid/ready; mid-run asynchronous reset is NOT applied
    // so that both DUTs see the same transaction stream.
    always @(posedge clk) begin
        if (rst_n) begin
            cyc <= cyc + 1;
            if (in_valid && in_ready) begin
                if (first_in_cyc < 0) first_in_cyc = cyc;
                sent = sent + 1;
                if (sent < N) load(sent);
            end
            if (out_valid && out_ready) begin
                if (first_out_cyc < 0) first_out_cyc = cyc;
                got = got + 1;
                $fdisplay(fd, "%03x %03x %03x %03x %08x %08x %09x %08x %08x %09x %08x %08x %09x %08x %08x %08x %04x %b%b%b%b%b%b%b%b%b%b",
                    bxmin,bxmax,bymin,bymax,e0a,e0b,e0c,e1a,e1b,e1c,e2a,e2b,e2c,
                    za,zb,zc,fc,ov,ovxy,ovec,ova,ovb,ovc,ovd,bcl,oneg,deg);
                if (got == 64) $display("full-rate phase: 64 outputs by cycle %0d", cyc);
            end
        end
    end

    always @(negedge clk) begin
        if (rst_n) begin
            if (sent < 64) begin in_valid <= (sent < N); out_ready <= 1; end
            else begin
                in_valid  <= (sent < N) && ($random(seed) % 10 < 7);
                out_ready <= ($random(seed) % 10 < 6) | (($random(seed) & 3) == 0);
            end
        end
    end

    initial begin
        wait (got == N);
        @(posedge clk);
        $display("%s: sent=%0d got=%0d latency(first accept->first out)=%0d cycles",
                 `OUTFILE, sent, got, first_out_cyc - first_in_cyc);
        $fclose(fd);
        $finish;
    end
    initial begin #50_000_000; $display("TIMEOUT sent=%0d got=%0d", sent, got); $finish; end
endmodule

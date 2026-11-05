`timescale 1ns / 1ps

// ============================================================================
// Module: tt_sound   (pixel clock domain, 25.175 MHz)
//   Game sounds on the Nexys A7 mono audio jack (AUD_PWM, low-pass filtered
//   on the board). The sounds are synthesised, so no sample files / BRAM:
//     hit    : racket hits the ball  - 1.3 kHz "tock" with a short noise click
//     bounce : ball on the table     - 820 Hz, softer, shorter
//     point  : a point is scored     - 523 Hz (C5) tone, longer decay
//   One voice: a new event restarts it.
//
//   Sample rate 25.175 MHz / 512 = 49.2 kHz. Each sample:
//     phase += f * 65536 / fs ;  wave = sin(phase)  (64-entry table)
//     env   -= env >> DECAY    (exponential decay, time constant 2^DECAY samples)
//     out    = 128 + wave * env + noise * env (first part of the hit)
//   PWM: 8-bit, 98 kHz carrier. AUD_PWM is driven low or released (the board
//   pulls it up), as Digilent specifies; aud_sd = amplifier on.
// ============================================================================
module tt_sound (
    input  wire clk,
    input  wire rst_n,
    input  wire ev_hit,
    input  wire ev_bounce,
    input  wire ev_point,
    input  wire mute,
    output wire aud_pwm_o,      // 1 = release (Z), 0 = drive low  (see tt_top)
    output wire aud_sd          // amplifier enable
);

    // 64-entry sine, signed 8 bit
    function signed [7:0] sine;  input [5:0] i;  begin
        case (i)
            6'd0:  sine =  8'sd0;    6'd1:  sine =  8'sd12;   6'd2:  sine =  8'sd25;   6'd3:  sine =  8'sd37;
            6'd4:  sine =  8'sd49;   6'd5:  sine =  8'sd60;   6'd6:  sine =  8'sd71;   6'd7:  sine =  8'sd81;
            6'd8:  sine =  8'sd90;   6'd9:  sine =  8'sd98;   6'd10: sine =  8'sd106;  6'd11: sine =  8'sd112;
            6'd12: sine =  8'sd117;  6'd13: sine =  8'sd122;  6'd14: sine =  8'sd125;  6'd15: sine =  8'sd126;
            6'd16: sine =  8'sd127;  6'd17: sine =  8'sd126;  6'd18: sine =  8'sd125;  6'd19: sine =  8'sd122;
            6'd20: sine =  8'sd117;  6'd21: sine =  8'sd112;  6'd22: sine =  8'sd106;  6'd23: sine =  8'sd98;
            6'd24: sine =  8'sd90;   6'd25: sine =  8'sd81;   6'd26: sine =  8'sd71;   6'd27: sine =  8'sd60;
            6'd28: sine =  8'sd49;   6'd29: sine =  8'sd37;   6'd30: sine =  8'sd25;   6'd31: sine =  8'sd12;
            default: sine = -sine_pos(i - 6'd32);
        endcase
    end endfunction
    function signed [7:0] sine_pos;  input [5:0] i;  begin
        case (i[4:0])
            5'd0:  sine_pos = 8'sd0;    5'd1:  sine_pos = 8'sd12;   5'd2:  sine_pos = 8'sd25;   5'd3:  sine_pos = 8'sd37;
            5'd4:  sine_pos = 8'sd49;   5'd5:  sine_pos = 8'sd60;   5'd6:  sine_pos = 8'sd71;   5'd7:  sine_pos = 8'sd81;
            5'd8:  sine_pos = 8'sd90;   5'd9:  sine_pos = 8'sd98;   5'd10: sine_pos = 8'sd106;  5'd11: sine_pos = 8'sd112;
            5'd12: sine_pos = 8'sd117;  5'd13: sine_pos = 8'sd122;  5'd14: sine_pos = 8'sd125;  5'd15: sine_pos = 8'sd126;
            5'd16: sine_pos = 8'sd127;  5'd17: sine_pos = 8'sd126;  5'd18: sine_pos = 8'sd125;  5'd19: sine_pos = 8'sd122;
            5'd20: sine_pos = 8'sd117;  5'd21: sine_pos = 8'sd112;  5'd22: sine_pos = 8'sd106;  5'd23: sine_pos = 8'sd98;
            5'd24: sine_pos = 8'sd90;   5'd25: sine_pos = 8'sd81;   5'd26: sine_pos = 8'sd71;   5'd27: sine_pos = 8'sd60;
            5'd28: sine_pos = 8'sd49;   5'd29: sine_pos = 8'sd37;   5'd30: sine_pos = 8'sd25;   default: sine_pos = 8'sd12;
        endcase
    end endfunction

    // sound parameters: phase step (f * 65536 / 49170), start level, decay, noise
    localparam [15:0] INC_HIT = 16'd1733, INC_BNC = 16'd1093, INC_PNT = 16'd697;

    reg  [8:0]  div;                 // 512 clocks per sample
    wire        tick = (div == 9'd511);
    reg  [15:0] phase, inc;
    reg  [15:0] env;                 // envelope, 8.8
    reg  [3:0]  decay;
    reg  [11:0] noise_left;          // samples of noise still to add
    reg  [15:0] lfsr;
    reg  [7:0]  sample;              // unsigned, 128 = silence
    reg  [7:0]  pwm_cnt;
    reg         pwm;

    // next sample (registered once per sample; all products 8 x 8 bit)
    wire signed [7:0]  wv    = sine(phase[15:10]);
    wire signed [16:0] tone  = wv * $signed({1'b0, env[15:8]});          // +-127*255
    wire signed [16:0] nse   = $signed(lfsr[7:0]) * $signed({1'b0, env[15:8]});
    wire signed [10:0] mix   = (tone >>> 8) + ((noise_left != 12'd0) ? (nse >>> 9) : 17'sd0);
    wire signed [10:0] s_off = mix + 11'sd128;
    wire        [7:0]  s_clp = (s_off < 0) ? 8'd0 : (s_off > 255) ? 8'd255 : s_off[7:0];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            div <= 9'd0;  phase <= 16'd0;  inc <= INC_HIT;  env <= 16'd0;  decay <= 4'd9;
            noise_left <= 12'd0;  lfsr <= 16'hACE1;  sample <= 8'd128;
        end else begin
            div <= div + 9'd1;
            if (ev_hit) begin
                inc <= INC_HIT;  env <= 16'hFF00;  decay <= 4'd9;   noise_left <= 12'd150;  phase <= 16'd0;
            end else if (ev_bounce) begin
                inc <= INC_BNC;  env <= 16'hB000;  decay <= 4'd8;   noise_left <= 12'd60;   phase <= 16'd0;
            end else if (ev_point) begin
                inc <= INC_PNT;  env <= 16'hA000;  decay <= 4'd12;  noise_left <= 12'd0;    phase <= 16'd0;
            end else if (tick) begin
                phase  <= phase + inc;
                env    <= ((env >> decay) != 16'd0) ? env - (env >> decay) :   // exponential,
                          (env != 16'd0) ? env - 16'd1 : 16'd0;               // then linear to 0
                lfsr   <= {1'b0, lfsr[15:1]} ^ (lfsr[0] ? 16'hB400 : 16'h0000);
                if (noise_left != 12'd0) noise_left <= noise_left - 12'd1;
                sample <= s_clp;
            end
        end
    end

    // 8-bit PWM
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin pwm_cnt <= 8'd0; pwm <= 1'b0; end
        else begin
            pwm_cnt <= pwm_cnt + 8'd1;
            pwm     <= mute ? 1'b0 : (pwm_cnt < sample);
        end
    end

    assign aud_pwm_o = pwm;
    assign aud_sd    = ~mute;

endmodule

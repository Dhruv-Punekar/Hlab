// ============================================================================
// top_projection_engine.v
// Minimal FPGA top for projection_engine.v only (no UART) - Basys 3.
//
// Files needed in Vivado:
//   rtl/top_projection_engine.v   (this file, set as top)
//   rtl/projection_engine.v
//   rtl/pixel_addr_gen.v
//   rtl/row_projection.v
//   rtl/col_projection.v
//   mem/image.mem                 (64x64 8-bit image, add as a design source)
//   constraints/top_projection_engine.xdc
//
// Board controls
//   btnC      reset; the image is processed automatically after reset
//   btnU      run the engine again
//   sw[5:0]   which row / column to show (0..63)
//   sw[15]    0 = row projection, 1 = column projection
//   7-seg     selected sum in hexadecimal (max 0x3FC0 = 16320)
//   led[13:0] selected sum in binary
//   led[14]   busy
//   led[15]   done
// ============================================================================
`timescale 1ns / 1ps

module top_projection_engine #(
    parameter MEM_FILE    = "image.mem",
    parameter TICK_CYCLES = 100000          // 1 kHz button sampling at 100 MHz
)(
    input  wire        clk,                 // 100 MHz
    input  wire        btnC,
    input  wire        btnU,
    input  wire [15:0] sw,
    output wire [15:0] led,
    output reg  [6:0]  seg,                 // active low, seg[0] = a ... seg[6] = g
    output wire        dp,
    output reg  [3:0]  an                   // active low
);

    localparam W  = 64;
    localparam H  = 64;
    localparam DW = 8;
    localparam AW = 12;
    localparam IW = 6;
    localparam SW = 14;

    // ---------------- reset (power-on + btnC), synchronised ----------------
    reg [1:0] rst_ff = 2'b11;
    always @(posedge clk or posedge btnC) begin
        if (btnC) rst_ff <= 2'b11;
        else      rst_ff <= {rst_ff[0], 1'b0};
    end
    wire rst = rst_ff[1];

    // ---------------- btnU: sample at 1 kHz, detect a press ----------------
    reg [16:0] tick_cnt;
    reg        tick;
    always @(posedge clk) begin
        if (rst) begin
            tick_cnt <= 0;
            tick     <= 1'b0;
        end else if (tick_cnt == TICK_CYCLES - 1) begin
            tick_cnt <= 0;
            tick     <= 1'b1;
        end else begin
            tick_cnt <= tick_cnt + 1'b1;
            tick     <= 1'b0;
        end
    end

    reg [1:0] btn_sync = 2'b00;              // 2-FF synchroniser
    reg       btn_sample = 1'b0;             // value seen at the previous tick
    always @(posedge clk) begin
        btn_sync <= {btn_sync[0], btnU};
        if (rst)
            btn_sample <= 1'b0;
        else if (tick)
            btn_sample <= btn_sync[1];       // bounces are shorter than 1 ms
    end
    wire btn_press = tick & btn_sync[1] & ~btn_sample;

    // ---------------- start: once after reset, then on each press ---------
    wire busy, done;
    reg  auto_started, start;
    always @(posedge clk) begin
        if (rst) begin
            auto_started <= 1'b0;
            start        <= 1'b0;
        end else begin
            start <= 1'b0;
            if (!auto_started) begin
                auto_started <= 1'b1;
                start        <= 1'b1;
            end else if (btn_press && !busy) begin
                start <= 1'b1;
            end
        end
    end

    // ---------------- image memory (block RAM, 1-cycle read) --------------
    wire [AW-1:0] pix_addr;
    reg  [DW-1:0] pix_data;

    (* rom_style = "block" *)
    reg [DW-1:0] image [0:W*H-1];
    initial $readmemh(MEM_FILE, image);

    always @(posedge clk)
        pix_data <= image[pix_addr];

    // ---------------- projection engine ----------------
    wire [SW-1:0] rd_data;

    projection_engine #(.W(W), .H(H), .DW(DW)) u_engine (
        .clk      (clk),
        .rst      (rst),
        .start    (start),
        .pix_addr (pix_addr),
        .pix_data (pix_data),
        .busy     (busy),
        .done     (done),
        .rd_sel   (sw[15]),
        .rd_idx   (sw[IW-1:0]),
        .rd_data  (rd_data)
    );

    // register the result once for the displays (switches are asynchronous)
    reg [SW-1:0] value;
    always @(posedge clk) value <= rd_data;

    assign led[SW-1:0] = value;
    assign led[14]     = busy;
    assign led[15]     = done;

    // ---------------- 4-digit 7-segment display, hexadecimal --------------
    reg [19:0] refresh = 20'd0;                   // digit changes every ~2.6 ms
    always @(posedge clk) refresh <= refresh + 1'b1;

    wire [15:0] hex = {{(16-SW){1'b0}}, value};
    reg  [3:0]  nibble;

    always @(*) begin
        case (refresh[19:18])
            2'd0: begin an = 4'b1110; nibble = hex[3:0];   end
            2'd1: begin an = 4'b1101; nibble = hex[7:4];   end
            2'd2: begin an = 4'b1011; nibble = hex[11:8];  end
            default: begin an = 4'b0111; nibble = hex[15:12]; end
        endcase
    end

    always @(*) begin
        case (nibble)            //  gfedcba  (0 = segment on)
            4'h0: seg = 7'b1000000;
            4'h1: seg = 7'b1111001;
            4'h2: seg = 7'b0100100;
            4'h3: seg = 7'b0110000;
            4'h4: seg = 7'b0011001;
            4'h5: seg = 7'b0010010;
            4'h6: seg = 7'b0000010;
            4'h7: seg = 7'b1111000;
            4'h8: seg = 7'b0000000;
            4'h9: seg = 7'b0010000;
            4'hA: seg = 7'b0001000;
            4'hB: seg = 7'b0000011;
            4'hC: seg = 7'b1000110;
            4'hD: seg = 7'b0100001;
            4'hE: seg = 7'b0000110;
            default: seg = 7'b0001110;   // F
        endcase
    end

    assign dp = 1'b1;                    // decimal point off

endmodule

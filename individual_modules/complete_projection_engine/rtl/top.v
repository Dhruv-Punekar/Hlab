// ============================================================================
// top.v  -  Projection engine FPGA top level (Digilent Basys 3, Artix-7 35T)
//
//   image_rom (BRAM, mem/image.mem) -> projection_engine -> result_sender -> UART
//
// Board I/O
//   clk      100 MHz oscillator
//   btnC     reset (the frame is processed automatically after reset)
//   btnU     re-run the frame and re-send the results
//   sw[5:0]  result index to show on the LEDs
//   sw[15]   0 = row projection, 1 = column projection
//   led[13:0] selected projection value (14 bits)
//   led[14]  busy (computing or sending)
//   led[15]  done
//   RsTx     UART to the USB-serial bridge, 115200 8N1
//            (packet format: see result_sender.v; reader: scripts/read_uart.py)
// ============================================================================
`timescale 1ns / 1ps

module top #(
    parameter CLKS_PER_BIT   = 868,         // 100 MHz / 115200
    parameter DEBOUNCE_CYCLES = 1000000,    // 10 ms
    parameter MEM_FILE       = "image.mem"
)(
    input  wire        clk,
    input  wire        btnC,
    input  wire        btnU,
    input  wire [15:0] sw,
    output wire [15:0] led,
    output wire        RsTx
);

    localparam W  = 64;
    localparam H  = 64;
    localparam DW = 8;
    localparam AW = $clog2(W*H);
    localparam IW = 6;
    localparam SW = DW + IW;                // 14

    // ---------------- reset synchroniser (also gives a power-on reset) ----
    reg [1:0] rst_ff = 2'b11;
    always @(posedge clk or posedge btnC) begin
        if (btnC) rst_ff <= 2'b11;
        else      rst_ff <= {rst_ff[0], 1'b0};
    end
    wire rst = rst_ff[1];

    wire eng_busy, eng_done;   // from projection_engine
    wire snd_active;           // from result_sender

    // ---------------- start: once after reset, then on btnU ---------------
    wire btn_pressed;
    debounce #(.STABLE_CYCLES(DEBOUNCE_CYCLES)) u_db (
        .clk     (clk),
        .rst     (rst),
        .btn_in  (btnU),
        .pressed (btn_pressed)
    );

    reg auto_started;
    reg start;
    always @(posedge clk) begin
        if (rst) begin
            auto_started <= 1'b0;
            start        <= 1'b0;
        end else begin
            start <= 1'b0;
            if (!auto_started) begin
                auto_started <= 1'b1;
                start        <= 1'b1;
            end else if (btn_pressed && !eng_busy && !snd_active) begin
                start <= 1'b1;
            end
        end
    end

    // ---------------- image memory + engine -------------------------------
    wire [AW-1:0] pix_addr;
    wire [DW-1:0] pix_data;

    image_rom #(.DW(DW), .DEPTH(W*H), .MEM_FILE(MEM_FILE)) u_rom (
        .clk  (clk),
        .addr (pix_addr),
        .dout (pix_data)
    );

    wire          rd_sel;
    wire [IW-1:0] rd_idx;
    wire [SW-1:0] rd_data;

    projection_engine #(.W(W), .H(H), .DW(DW)) u_engine (
        .clk      (clk),
        .rst      (rst),
        .start    (start),
        .pix_addr (pix_addr),
        .pix_data (pix_data),
        .busy     (eng_busy),
        .done     (eng_done),
        .rd_sel   (rd_sel),
        .rd_idx   (rd_idx),
        .rd_data  (rd_data)
    );

    // ---------------- send results when a frame finishes -----------------
    reg done_d;
    always @(posedge clk) done_d <= rst ? 1'b0 : eng_done;
    wire send_go = eng_done & ~done_d;

    wire          snd_sel;
    wire [IW-1:0] snd_idx;
    wire          tx_start, tx_busy, tx_done;
    wire [7:0]    tx_data;

    result_sender #(.W(W), .H(H), .IW(IW), .SW(SW)) u_sender (
        .clk      (clk),
        .rst      (rst),
        .go       (send_go),
        .active   (snd_active),
        .rd_sel   (snd_sel),
        .rd_idx   (snd_idx),
        .rd_data  (rd_data),
        .tx_start (tx_start),
        .tx_data  (tx_data),
        .tx_busy  (tx_busy),
        .tx_done  (tx_done)
    );

    uart_tx #(.CLKS_PER_BIT(CLKS_PER_BIT)) u_uart (
        .clk      (clk),
        .rst      (rst),
        .tx_start (tx_start),
        .tx_data  (tx_data),
        .tx       (RsTx),
        .tx_busy  (tx_busy),
        .tx_done  (tx_done)
    );

    // read port: sender while it is sending, otherwise the switches
    assign rd_sel = snd_active ? snd_sel : sw[15];
    assign rd_idx = snd_active ? snd_idx : sw[IW-1:0];

    assign led[SW-1:0] = rd_data;
    assign led[14]     = eng_busy | snd_active;
    assign led[15]     = eng_done;

endmodule

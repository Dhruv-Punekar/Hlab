`timescale 1ns / 1ps
// ============================================================================
// Module: adxl362_reader                                  (NEW, game feature)
//   Talks to the ADXL362 accelerometer on the Nexys A7 over SPI (mode 0) and
//   keeps the latest X / Y / Z acceleration.
//
//   Start-up:  wait POWERUP_CLKS -> soft reset (write 0x52 to reg 0x1F)
//              -> wait RESET_CLKS -> write 0x02 to POWER_CTL (0x2D) = measure
//              (default range +/-2 g, 100 samples/s, 1 mg per LSB)
//              -> read DEVID_AD (reg 0x00); 0xAD means the chip answered.
//   Running:   every POLL_CLKS read 6 bytes from 0x0E (XDATA_L .. ZDATA_H).
//
//   SPI clock = clk / (2*HALF_CLKS) = 1 MHz by default (chip allows 1..8 MHz).
//   x/y/z: signed 12-bit two's complement in mg (the chip sign-extends to 16).
// ============================================================================
module adxl362_reader #(
    parameter HALF_CLKS    = 50,         // 100 MHz / (2*50) = 1 MHz SCLK
    parameter POWERUP_CLKS = 1000000,    // 10 ms after FPGA configuration
    parameter RESET_CLKS   = 1000000,    // 10 ms after soft reset
    parameter POLL_CLKS    = 1000000     // 10 ms -> 100 reads per second
)(
    input  wire               clk,
    input  wire               rst_n,
    // SPI pins
    output reg                sclk,
    output reg                mosi,
    input  wire               miso,
    output reg                cs_n,
    // results
    output reg  signed [11:0] acc_x,
    output reg  signed [11:0] acc_y,
    output reg  signed [11:0] acc_z,
    output reg                sample_valid,    // 1-clock pulse per new sample
    output reg                id_ok            // DEVID_AD read back as 0xAD
);
    // ---------------- byte-transfer engine (mode 0) ------------------------
    reg  [7:0]  tx_buf [0:7];
    reg  [7:0]  rx_buf [0:7];
    reg  [3:0]  xfer_len;       // bytes in this transaction
    reg  [3:0]  byte_i;
    reg  [2:0]  bit_i;
    reg  [7:0]  sh_tx, sh_rx;
    reg  [15:0] div;
    reg         xfer_go, xfer_busy, xfer_done;
    reg  [1:0]  ph;             // 0: CS setup, 1: SCLK low half, 2: SCLK high half, 3: CS hold

    (* ASYNC_REG = "TRUE" *) reg [1:0] miso_s;
    always @(posedge clk) miso_s <= {miso_s[0], miso};

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sclk <= 1'b0; mosi <= 1'b0; cs_n <= 1'b1;
            xfer_busy <= 1'b0; xfer_done <= 1'b0;
            div <= 16'd0; ph <= 2'd0; byte_i <= 4'd0; bit_i <= 3'd0;
            sh_tx <= 8'd0; sh_rx <= 8'd0;
        end else begin
            xfer_done <= 1'b0;
            if (!xfer_busy) begin
                if (xfer_go) begin
                    xfer_busy <= 1'b1;
                    cs_n   <= 1'b0;
                    sclk   <= 1'b0;
                    byte_i <= 4'd0;
                    bit_i  <= 3'd7;
                    sh_tx  <= tx_buf[0];
                    mosi   <= tx_buf[0][7];
                    ph     <= 2'd0;
                    div    <= 16'd0;
                end
            end else if (div != HALF_CLKS - 1) begin
                div <= div + 16'd1;
            end else begin
                div <= 16'd0;
                case (ph)
                    2'd0: ph <= 2'd2;                       // CS setup done -> first rising edge
                    2'd2: begin                             // rising edge: sample MISO
                        sclk  <= 1'b1;
                        sh_rx <= {sh_rx[6:0], miso_s[1]};
                        ph    <= 2'd1;
                    end
                    2'd1: begin                             // falling edge: next bit out
                        sclk <= 1'b0;
                        if (bit_i == 3'd0) begin
                            rx_buf[byte_i] <= sh_rx;
                            if (byte_i == xfer_len - 1) begin
                                ph <= 2'd3;                 // last bit of last byte
                            end else begin
                                byte_i <= byte_i + 4'd1;
                                bit_i  <= 3'd7;
                                sh_tx  <= tx_buf[byte_i + 1];
                                mosi   <= tx_buf[byte_i + 1][7];
                                ph     <= 2'd2;
                            end
                        end else begin
                            bit_i <= bit_i - 3'd1;
                            sh_tx <= {sh_tx[6:0], 1'b0};
                            mosi  <= sh_tx[6];
                            ph    <= 2'd2;
                        end
                    end
                    2'd3: begin                             // CS hold, then release
                        cs_n      <= 1'b1;
                        xfer_busy <= 1'b0;
                        xfer_done <= 1'b1;
                    end
                endcase
            end
        end
    end

    // ---------------- command sequencer -------------------------------------
    localparam C_WAIT_PWR = 3'd0, C_SRESET = 3'd1, C_WAIT_RST = 3'd2, C_MEASURE = 3'd3,
               C_READ_ID  = 3'd4, C_IDLE   = 3'd5, C_READ_XYZ = 3'd6, C_WAIT_X = 3'd7;
    reg [2:0]  cst, after;
    reg [31:0] timer;
    reg        waiting;          // a transfer is in flight

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cst <= C_WAIT_PWR; timer <= 32'd0; xfer_go <= 1'b0; waiting <= 1'b0;
            xfer_len <= 4'd0; after <= C_IDLE;
            acc_x <= 12'sd0; acc_y <= 12'sd0; acc_z <= 12'sd0;
            sample_valid <= 1'b0; id_ok <= 1'b0;
        end else begin
            xfer_go      <= 1'b0;
            sample_valid <= 1'b0;
            case (cst)
                C_WAIT_PWR: if (timer == POWERUP_CLKS) begin timer <= 0; cst <= C_SRESET; end
                            else timer <= timer + 1;
                C_SRESET: begin                                     // write 0x52 -> SOFT_RESET
                    tx_buf[0] <= 8'h0A; tx_buf[1] <= 8'h1F; tx_buf[2] <= 8'h52;
                    xfer_len <= 4'd3; xfer_go <= 1'b1; cst <= C_WAIT_X; after <= C_WAIT_RST;
                end
                C_WAIT_RST: if (timer == RESET_CLKS) begin timer <= 0; cst <= C_MEASURE; end
                            else timer <= timer + 1;
                C_MEASURE: begin                                    // POWER_CTL = measure
                    tx_buf[0] <= 8'h0A; tx_buf[1] <= 8'h2D; tx_buf[2] <= 8'h02;
                    xfer_len <= 4'd3; xfer_go <= 1'b1; cst <= C_WAIT_X; after <= C_READ_ID;
                end
                C_READ_ID: begin                                    // read DEVID_AD
                    tx_buf[0] <= 8'h0B; tx_buf[1] <= 8'h00; tx_buf[2] <= 8'h00;
                    xfer_len <= 4'd3; xfer_go <= 1'b1; cst <= C_WAIT_X; after <= C_IDLE;
                end
                C_IDLE: begin
                    if (timer >= POLL_CLKS) begin timer <= 0; cst <= C_READ_XYZ; end
                    else timer <= timer + 1;
                end
                C_READ_XYZ: begin                                   // burst read 0x0E..0x13
                    tx_buf[0] <= 8'h0B; tx_buf[1] <= 8'h0E;
                    tx_buf[2] <= 8'h00; tx_buf[3] <= 8'h00; tx_buf[4] <= 8'h00;
                    tx_buf[5] <= 8'h00; tx_buf[6] <= 8'h00; tx_buf[7] <= 8'h00;
                    xfer_len <= 4'd8; xfer_go <= 1'b1; cst <= C_WAIT_X; after <= C_IDLE;
                end
                C_WAIT_X: begin
                    if (xfer_go) waiting <= 1'b1;
                    if (waiting && xfer_done) begin
                        waiting <= 1'b0;
                        if (xfer_len == 4'd8) begin
                            acc_x <= {rx_buf[3][3:0], rx_buf[2]};
                            acc_y <= {rx_buf[5][3:0], rx_buf[4]};
                            acc_z <= {rx_buf[7][3:0], rx_buf[6]};
                            sample_valid <= 1'b1;
                        end else if (tx_buf[0] == 8'h0B && tx_buf[1] == 8'h00) begin
                            id_ok <= (rx_buf[2] == 8'hAD);
                        end
                        timer <= 0;
                        cst   <= after;
                    end
                end
            endcase
        end
    end
endmodule

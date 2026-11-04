module rasterizer_top (
    input  wire        clk,      // 100 MHz board oscillator
    input  wire        rst_n,    // Active-low reset button
    output wire [15:0] led
);
    // Async assert, synchronous release.
    reg [1:0] rst_sync = 2'b00;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            rst_sync <= 2'b00;
        else
            rst_sync <= {rst_sync[0], 1'b1};
    end

    wire rst_n_sync = rst_sync[1];

    (* mark_debug = "true" *) wire [77:0] log;

    raster_demo #(
        .SLOW_DIV (100_000_000),
        .GAP      (100_000_000)
    ) u_demo (
        .clk   (clk),
        .rst_n (rst_n_sync),
        .log   (log)
    );

    assign led = {
    log[0],       // LED15: orient_neg
    log[23:21],   // LED14-12: actual FSM state
    log[24],      // LED11: idle_raster
    log[41],      // LED10: pixel_valid
    log[77:68]    // LED9-0: pixel_x
};
endmodule
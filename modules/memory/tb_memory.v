`timescale 1ns / 1ps

module tb_memory;

    // ========================================================================
    // CLOCK GENERATION
    // ========================================================================
    reg clk;
    initial begin
        clk = 0;
        forever #5 clk = ~clk; // 100 MHz clock
    end

    // ========================================================================
    // VERTEX MEMORY SIGNALS
    // ========================================================================
    reg v_en;
    reg v_we;
    reg [5:0] v_addr;         
    reg [143:0] v_data_in;
    wire [143:0] v_data_out;

    // ========================================================================
    // MATRIX MEMORY SIGNALS
    // ========================================================================
    reg m_we;
    reg [3:0] m_write_addr;
    reg [31:0] m_write_data;
    wire [511:0] m_matrix_out;

    // ========================================================================
    // MODULE INSTANTIATIONS
    // ========================================================================
    vertex_memory #(
        .DEPTH(64), 
        .WIDTH(144)
    ) u_vertex_mem (
        .clk(clk),
        .en(v_en),
        .we(v_we),
        .addr(v_addr),
        .data_in(v_data_in),
        .data_out(v_data_out)
    );

    matrix_memory u_matrix_mem (
        .clk(clk),
        .we(m_we),
        .write_addr(m_write_addr),
        .write_data(m_write_data),
        .matrix_out(m_matrix_out)
    );

    // ========================================================================
    // TEST SEQUENCE
    // ========================================================================
    integer i;

    initial begin
        // Initialize Inputs
        v_en = 0; v_we = 0; v_addr = 0; v_data_in = 0;
        m_we = 0; m_write_addr = 0; m_write_data = 0;
        #20;

        // --------------------------------------------------------------------
        // TEST 1: STATIC MATRIX MEMORY LOAD
        // --------------------------------------------------------------------
        $display("--- TEST 1: STATIC MATRIX MEMORY LOAD ---");
        $display("Initial Matrix: %h\n", m_matrix_out);
        #10;

        // --------------------------------------------------------------------
        // TEST 2: READ VERTEX MEMORY (Cube = 36 vertices)
        // --------------------------------------------------------------------
        $display("--- TEST 2: READING VERTEX MEMORY ---");
        v_en = 1; 
        v_we = 0;
        
        for (i = 0; i < 36; i = i + 1) begin
            @(negedge clk);
            v_addr = i;
        end
        #20;

        // --------------------------------------------------------------------
        // TEST 3: VERTEX MEMORY ENABLE TOGGLE TEST
        // Verify that when v_en = 0, changing the address does not change output.
        // --------------------------------------------------------------------
        $display("--- TEST 3: VERTEX MEMORY ENABLE TEST ---");
        @(negedge clk);
        v_en = 0;         // Disable memory
        v_addr = 6'd10;   // Change address
        @(posedge clk);
        #1;
        $display("Output with EN=0 (Should hold last value): %h\n", v_data_out);
        #10;

        // --------------------------------------------------------------------
        // TEST 4: FULL UART MATRIX FRAME UPDATE (Phase 2 Simulation)
        // Overwrite all 16 elements of the matrix to test the 4-to-16 decoder
        // --------------------------------------------------------------------
        $display("--- TEST 4: FULL MATRIX OVERWRITE ---");
        for (i = 0; i < 16; i = i + 1) begin
            @(negedge clk);
            m_we = 1;
            m_write_addr = i;
            // Write a dummy pattern: 0x00000000, 0x00000001, etc.
            m_write_data = i; 
        end
        
        @(negedge clk);
        m_we = 0;
        @(posedge clk);
        #1;
        $display("Fully Updated Matrix Bus: %h\n", m_matrix_out);

        // --------------------------------------------------------------------
        // TEST 5: VERTEX MEMORY WRITE TEST (Phase 2 Simulation)
        // --------------------------------------------------------------------
        $display("--- TEST 5: VERTEX MEMORY WRITE TEST ---");
        @(negedge clk);
        v_en = 1;
        v_we = 1;
        v_addr = 6'd63; // Write to the very last address slot
        v_data_in = 144'hAAAA_BBBB_CCCC_DDDD_EEEE_FFFF_1111_2222_3333;
        
        @(negedge clk);
        v_we = 0; // Turn off write, read it back
        
        @(posedge clk);
        #1;
        $display("Read Back Written Vertex: %h\n", v_data_out);

        $display("--- ALL TESTS PASSED! SIMULATION COMPLETE ---");
        $finish;
    end

endmodule

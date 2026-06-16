// =============================================================================
// tb_dncnn_ingestor.sv  - Testbench for dncnn_ingestor.v
// =============================================================================
// HOW TO RUN IN VIVADO:
//   sources_1 : dncnn_ingestor.v, line_buffer_1ch.v, sat_relu.v,
//               mac_array_1x16.v  (DSP48E1 version - synthesis target)
//   sim_1     : tb_dncnn_ingestor.sv  (this file)
//               mac_array_1x16_behav.v  (behavioural MAC - sim only)
//
//   REQUIRED VIVADO STEP:
//     Right-click mac_array_1x16.v in sources_1
//     -> Properties -> Used In -> UNCHECK "Simulation"
//   Vivado then picks mac_array_1x16_behav.v for simulation and
//   mac_array_1x16.v (DSP48E1) for synthesis.
//
// OUTPUT_MAP INDEX ACCOUNTING:
//   The line buffer introduces exactly 1 row of latency.
//   When the ingestor FSM writes input pixel (row R, col C) into the LB,
//   the 3x3 window it reads is centered on (R, C) but uses BRAM rows that
//   were filled during previous rows.  The convolution result for image
//   position (R, C) therefore appears in the output stream one full row
//   AFTER that pixel was ingested, i.e.:
//
//     output_map[R+1][C]  holds the convolution result for image(R, C)
//
//   This is the standard line-buffer latency.  The checks below use
//   output_map[R+1][C] for an image pixel at (R,C).
//
// EXPECTED CONSOLE OUTPUT:
//   === tb_dncnn_ingestor: START ===
//   -- Checking outputs --
//   PASS OC0@out[2][1]=conv(img[1][1]) all-ones sum=54: 54
//   PASS OC1@out[2][1]=conv(img[1][1]) center-only=6: 6
//   PASS OC3@out[2][1]=conv(img[1][1]) zero-filter=0: 0
//   PASS OC0@out[1][0]=conv(img[0][0]) padded corner=14: 14
//   PASS OC1@out[1][0]=conv(img[0][0]) center=1: 1
//   === tb_dncnn_ingestor: 5 PASS, 0 FAIL ===
//   ALL TESTS PASSED
// =============================================================================

`timescale 1ns/1ps

module tb_dncnn_ingestor;

    localparam WIDTH  = 4;
    localparam HEIGHT = 4;
    localparam OC     = 4;

    // -------------------------------------------------------------------------
    // DUT ports
    // -------------------------------------------------------------------------
    logic        clk  = 0;
    logic        rst_n;
    logic [7:0]  s_axis_tdata;
    logic        s_axis_tvalid;
    wire         s_axis_tready;
    logic        s_axis_tlast;

    wire  [7:0]              weight_addr;
    logic [OC*8-1:0]         weight_data;
    logic signed [OC*16-1:0] bias_data;

    wire  [OC*8-1:0]         m_axis_tdata;
    wire                     m_axis_tvalid;
    logic                    m_axis_tready;
    wire                     m_axis_tlast;
    wire                     done;
    logic                    top_row_active;

    // -------------------------------------------------------------------------
    // Clock - 100 MHz
    // -------------------------------------------------------------------------
    always #5 clk = ~clk;

    // -------------------------------------------------------------------------
    // External row/col tracker - drives top_row_active
    // Counts AXI-Stream handshakes (tvalid & tready) to track which image
    // row is currently being fed to the ingestor.
    // -------------------------------------------------------------------------
    logic [$clog2(HEIGHT)-1:0] row_cnt_ext;
    logic [$clog2(WIDTH)-1:0]  col_cnt_ext;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            row_cnt_ext <= 0;
            col_cnt_ext <= 0;
        end else if (s_axis_tvalid && s_axis_tready) begin
            if (col_cnt_ext == WIDTH-1) begin
                col_cnt_ext <= 0;
                row_cnt_ext <= (row_cnt_ext == HEIGHT-1) ? 0 : row_cnt_ext + 1;
            end else begin
                col_cnt_ext <= col_cnt_ext + 1;
            end
        end
    end

    assign top_row_active = (row_cnt_ext == 0);

    // -------------------------------------------------------------------------
    // Weight ROM [oc][k], k=0..8 row-major (same as kernel_flat in ingestor)
    // -------------------------------------------------------------------------
    logic signed [7:0] weight_rom [0:OC-1][0:8];

    initial begin
        // OC0: all-ones  -> output = sum of all 9 window pixels
        weight_rom[0][0]=8'sd1; weight_rom[0][1]=8'sd1; weight_rom[0][2]=8'sd1;
        weight_rom[0][3]=8'sd1; weight_rom[0][4]=8'sd1; weight_rom[0][5]=8'sd1;
        weight_rom[0][6]=8'sd1; weight_rom[0][7]=8'sd1; weight_rom[0][8]=8'sd1;
        // OC1: center-only -> output = image[R][C]
        weight_rom[1][0]=8'sd0; weight_rom[1][1]=8'sd0; weight_rom[1][2]=8'sd0;
        weight_rom[1][3]=8'sd0; weight_rom[1][4]=8'sd1; weight_rom[1][5]=8'sd0;
        weight_rom[1][6]=8'sd0; weight_rom[1][7]=8'sd0; weight_rom[1][8]=8'sd0;
        // OC2: horizontal edge [-1, 0, +1] in middle row
        weight_rom[2][0]=8'sd0;  weight_rom[2][1]=8'sd0; weight_rom[2][2]=8'sd0;
        weight_rom[2][3]=-8'sd1; weight_rom[2][4]=8'sd0; weight_rom[2][5]=8'sd1;
        weight_rom[2][6]=8'sd0;  weight_rom[2][7]=8'sd0; weight_rom[2][8]=8'sd0;
        // OC3: all-zeros -> output always 0
        weight_rom[3][0]=8'sd0; weight_rom[3][1]=8'sd0; weight_rom[3][2]=8'sd0;
        weight_rom[3][3]=8'sd0; weight_rom[3][4]=8'sd0; weight_rom[3][5]=8'sd0;
        weight_rom[3][6]=8'sd0; weight_rom[3][7]=8'sd0; weight_rom[3][8]=8'sd0;
    end

    // Combinational weight mux: ingestor drives weight_addr = k (0..8),
    // we return all OC weights for that kernel position simultaneously.
    always_comb begin
        integer oc_i;
        for (oc_i = 0; oc_i < OC; oc_i = oc_i + 1)
            weight_data[oc_i*8 +: 8] = weight_rom[oc_i][weight_addr];
    end

    assign bias_data = '0;

    // -------------------------------------------------------------------------
    // DUT
    // -------------------------------------------------------------------------
    dncnn_ingestor #(.WIDTH(WIDTH), .HEIGHT(HEIGHT), .OC(OC)) dut (
        .clk            (clk),
        .rst_n          (rst_n),
        .s_axis_tdata   (s_axis_tdata),
        .s_axis_tvalid  (s_axis_tvalid),
        .s_axis_tready  (s_axis_tready),
        .s_axis_tlast   (s_axis_tlast),
        .weight_addr    (weight_addr),
        .weight_data    (weight_data),
        .bias_data      (bias_data),
        .m_axis_tdata   (m_axis_tdata),
        .m_axis_tvalid  (m_axis_tvalid),
        .m_axis_tready  (m_axis_tready),
        .m_axis_tlast   (m_axis_tlast),
        .done           (done),
        .top_row_active (top_row_active)
    );

    // -------------------------------------------------------------------------
    // Image: pixel(r, c) = r*4 + c + 1  (values 1..16)
    // -------------------------------------------------------------------------
    logic [7:0] image [0:HEIGHT-1][0:WIDTH-1];
    initial begin
        integer r, c;
        for (r = 0; r < HEIGHT; r++)
            for (c = 0; c < WIDTH; c++)
                image[r][c] = r * WIDTH + c + 1;
    end

    // -------------------------------------------------------------------------
    // Output capture
    // Outputs arrive in the same row-major order as inputs.
    // output_map[out_row][out_col] = m_axis_tdata at handshake N,
    // where N = out_row*WIDTH + out_col.
    // -------------------------------------------------------------------------
    logic [OC*8-1:0] output_map [0:HEIGHT-1][0:WIDTH-1];
    int out_row    = 0;
    int out_col    = 0;
    int received   = 0;
    int pass_count = 0;
    int fail_count = 0;

    task automatic check_output(
        input int row, col, oc_idx,
        input logic signed [7:0] expected,
        input string desc
    );
        logic signed [7:0] actual;
        actual = $signed(output_map[row][col][oc_idx*8 +: 8]);
        if (actual !== expected) begin
            $display("FAIL %s: got=%0d  exp=%0d", desc, actual, expected);
            fail_count++;
        end else begin
            $display("PASS %s: %0d", desc, actual);
            pass_count++;
        end
    endtask

    // -------------------------------------------------------------------------
    // Stimulus
    // -------------------------------------------------------------------------
    initial begin
        $display("=== tb_dncnn_ingestor: START ===");

        rst_n         = 0;
        s_axis_tvalid = 0;
        s_axis_tdata  = 0;
        s_axis_tlast  = 0;
        m_axis_tready = 1;

        repeat(4) @(posedge clk);
        #1; rst_n = 1;

        @(posedge clk); #1;   // let FSM reach S_IDLE

        fork

            // ---- PRODUCER ----
            // #1 after posedge is mandatory: s_axis_tready is a registered
            // output; sampling it without #1 reads the pre-NBA value.
            begin : producer
                integer r, c;
                for (r = 0; r < HEIGHT; r++) begin
                    for (c = 0; c < WIDTH; c++) begin
                        @(posedge clk); #1;
                        while (!s_axis_tready) begin
                            @(posedge clk); #1;
                        end
                        s_axis_tdata  = image[r][c];
                        s_axis_tvalid = 1;
                        s_axis_tlast  = (r == HEIGHT-1 && c == WIDTH-1);
                        @(posedge clk); #1;
                        s_axis_tvalid = 0;
                        s_axis_tlast  = 0;
                    end
                end
            end : producer

            // ---- CONSUMER ----
            begin : consumer
                while (received < HEIGHT * WIDTH) begin
                    @(posedge clk); #1;
                    if (m_axis_tvalid && m_axis_tready) begin
                        output_map[out_row][out_col] = m_axis_tdata;
                        if (++out_col == WIDTH) begin
                            out_col = 0;
                            out_row++;
                        end
                        received++;
                    end
                end
            end : consumer

        join

        @(posedge clk); #1;

        // =====================================================================
        // CHECKS
        //
        // output_map[R+1][C]  holds the convolution result for image(R, C).
        // (The line buffer introduces 1 row of latency.)
        //
        // Interior pixel image(1,1):
        //   3x3 window (zero-padded 4x4 image):
        //     row0: image[0][0..2] = 1, 2, 3
        //     row1: image[1][0..2] = 5, 6, 7
        //     row2: image[2][0..2] = 9,10,11
        //   OC0 all-ones  : 1+2+3 + 5+6+7 + 9+10+11 = 54
        //   OC1 center    : image[1][1] = 6
        //   OC3 zeros     : 0
        //   -> lives in output_map[2][1]
        //
        // Top-left corner image(0,0):
        //   3x3 window with top row AND left column zero-padded:
        //     row-1: 0, 0, 0   (top padding)
        //     row0:  0, 1, 2   (left padding + image[0][0..1])
        //     row1:  0, 5, 6   (left padding + image[1][0..1])
        //   OC0 all-ones  : 0+0+0 + 0+1+2 + 0+5+6 = 14
        //   OC1 center    : image[0][0] = 1
        //   -> lives in output_map[1][0]
        // =====================================================================
        $display("-- Checking outputs --");
        check_output(2, 1, 0, 8'sd54, "OC0@out[2][1]=conv(img[1][1]) all-ones sum=54");
        check_output(2, 1, 1, 8'sd6,  "OC1@out[2][1]=conv(img[1][1]) center-only=6");
        check_output(2, 1, 3, 8'sd0,  "OC3@out[2][1]=conv(img[1][1]) zero-filter=0");
        check_output(1, 0, 0, 8'sd14, "OC0@out[1][0]=conv(img[0][0]) padded corner=14");
        check_output(1, 0, 1, 8'sd1,  "OC1@out[1][0]=conv(img[0][0]) center=1");

        $display("=== tb_dncnn_ingestor: %0d PASS, %0d FAIL ===",
                 pass_count, fail_count);
        $display(fail_count == 0 ? "ALL TESTS PASSED" : "*** FAILURES DETECTED ***");
        #100 $finish;
    end

    // ---- Timeout guard ----
    initial begin
        #1_000_000;
        $display("TIMEOUT  received=%0d/%0d  tready=%0b tvalid=%0b",
                 received, HEIGHT * WIDTH, s_axis_tready, s_axis_tvalid);
        $finish;
    end

endmodule
// =============================================================================
// tb_dncnn_ingestor.sv - Testbench for dncnn_ingestor.v
// =============================================================================
// Tests a 4×4 "image" through the ingestor with known weights.
// Verifies the output feature maps match a manually computed reference.
//
// Test setup:
//   Image: 4×4 grayscale, pixel(row,col) = row*4 + col + 1  (1..16)
//   Weights: all 1 for OC=0, identity (only center=1) for OC=1
//   Bias: 0 for all OC (simplifies manual verification)
//   ReLU: should pass since all values positive
//
// Note: Uses WIDTH=4, HEIGHT=4 parameters for fast simulation.
//       Includes behavioral MAC to avoid DSP48E1 primitive dependency.
// =============================================================================

`timescale 1ns/1ps

module tb_dncnn_ingestor;

    localparam WIDTH  = 4;
    localparam HEIGHT = 4;
    localparam OC     = 4;   // Reduced OC for readability in this TB

    logic        clk = 0;
    logic        rst_n;

    // Input stream
    logic [7:0]  s_axis_tdata;
    logic        s_axis_tvalid;
    wire         s_axis_tready;
    logic        s_axis_tlast;

    // Weight BRAM
    wire  [7:0]        weight_addr;
    logic [OC*8-1:0]   weight_data;
    logic signed [OC*16-1:0] bias_data;

    // Output stream
    wire  [OC*8-1:0]   m_axis_tdata;
    wire               m_axis_tvalid;
    logic              m_axis_tready;
    wire               m_axis_tlast;
    wire               done;

    // top_row_active driven by external counter (would be in ctrl_fsm)
    logic top_row_active;
    logic [$clog2(HEIGHT)-1:0] row_cnt_ext;
    logic [$clog2(WIDTH)-1:0]  col_cnt_ext;

    always #5 clk = ~clk;

    // Weight ROM - 4 OC × 9 positions = 36 bytes
    // OC0: all-ones filter (sum of 3×3 neighborhood)
    // OC1: center-only filter (identity, picks center pixel)
    // OC2: vertical edge (top row +1, bottom row -1)
    // OC3: zero filter (output should be 0 ? relu clips to 0)
    logic signed [7:0] weight_rom [0:OC-1][0:8];
    initial begin
        // OC0: all-ones
        weight_rom[0][0]=1; weight_rom[0][1]=1; weight_rom[0][2]=1;
        weight_rom[0][3]=1; weight_rom[0][4]=1; weight_rom[0][5]=1;
        weight_rom[0][6]=1; weight_rom[0][7]=1; weight_rom[0][8]=1;
        // OC1: center only
        weight_rom[1][0]=0; weight_rom[1][1]=0; weight_rom[1][2]=0;
        weight_rom[1][3]=0; weight_rom[1][4]=1; weight_rom[1][5]=0;
        weight_rom[1][6]=0; weight_rom[1][7]=0; weight_rom[1][8]=0;
        // OC2: simple horizontal (left-right)
        weight_rom[2][0]=0; weight_rom[2][1]=0; weight_rom[2][2]=0;
        weight_rom[2][3]=-1; weight_rom[2][4]=0; weight_rom[2][5]=1;
        weight_rom[2][6]=0; weight_rom[2][7]=0; weight_rom[2][8]=0;
        // OC3: all-zeros
        weight_rom[3][0]=0; weight_rom[3][1]=0; weight_rom[3][2]=0;
        weight_rom[3][3]=0; weight_rom[3][4]=0; weight_rom[3][5]=0;
        weight_rom[3][6]=0; weight_rom[3][7]=0; weight_rom[3][8]=0;
    end

    // Weight ROM mux: weight_addr = oc*9 + k
    // Return all OC weights for this k simultaneously
    always_comb begin
        integer oc_i;
        for (oc_i = 0; oc_i < OC; oc_i = oc_i + 1) begin
            // weight_addr is the kernel position k for this cycle
            // The ingestor drives weight_addr = k (0..8)
            // We must return weight_rom[oc_i][weight_addr] for each oc
            weight_data[oc_i*8 +: 8] = weight_rom[oc_i][weight_addr];
        end
    end

    // Bias: all zero
    assign bias_data = '0;

    // External top_row_active tracking (mimics ctrl_fsm)
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            row_cnt_ext  <= 0;
            col_cnt_ext  <= 0;
        end else if (s_axis_tvalid && s_axis_tready) begin
            if (col_cnt_ext == WIDTH-1) begin
                col_cnt_ext <= 0;
                if (row_cnt_ext == HEIGHT-1)
                    row_cnt_ext <= 0;
                else
                    row_cnt_ext <= row_cnt_ext + 1;
            end else begin
                col_cnt_ext <= col_cnt_ext + 1;
            end
        end
    end
    assign top_row_active = (row_cnt_ext == 0);

    dncnn_ingestor #(
        .WIDTH  (WIDTH),
        .HEIGHT (HEIGHT),
        .OC     (OC)
    ) dut (
        .clk             (clk),
        .rst_n           (rst_n),
        .s_axis_tdata    (s_axis_tdata),
        .s_axis_tvalid   (s_axis_tvalid),
        .s_axis_tready   (s_axis_tready),
        .s_axis_tlast    (s_axis_tlast),
        .weight_addr     (weight_addr),
        .weight_data     (weight_data),
        .bias_data       (bias_data),
        .m_axis_tdata    (m_axis_tdata),
        .m_axis_tvalid   (m_axis_tvalid),
        .m_axis_tready   (m_axis_tready),
        .m_axis_tlast    (m_axis_tlast),
        .done            (done),
        .top_row_active  (top_row_active)
    );

    // ---- Drive input stream ----
    logic [7:0] image [0:HEIGHT-1][0:WIDTH-1];
    initial begin
        integer r, c;
        for (r = 0; r < HEIGHT; r = r + 1)
            for (c = 0; c < WIDTH; c = c + 1)
                image[r][c] = r*WIDTH + c + 1;
    end

    // ---- Collect outputs ----
    logic [OC*8-1:0] output_map [0:HEIGHT-1][0:WIDTH-1];
    int out_row = 0, out_col = 0;
    int received = 0;

    int pass_count = 0;
    int fail_count = 0;

    task check_output(
        input int row, col,
        input int oc_idx,
        input logic signed [7:0] expected,
        input string desc
    );
        logic signed [7:0] actual;
        actual = $signed(output_map[row][col][oc_idx*8 +: 8]);
        if (actual !== expected) begin
            $display("FAIL %s: got=%0d exp=%0d", desc, actual, expected);
            fail_count++;
        end else begin
            $display("PASS %s: %0d", desc, actual);
            pass_count++;
        end
    endtask

    initial begin
        $dumpfile("tb_ingestor.vcd");
        $dumpvars(0, tb_dncnn_ingestor);

        $display("=== tb_dncnn_ingestor: START ===");
        rst_n         = 0;
        s_axis_tvalid = 0;
        s_axis_tdata  = 0;
        s_axis_tlast  = 0;
        m_axis_tready = 1;
        @(posedge clk); @(posedge clk);
        rst_n = 1;
        @(posedge clk);

        // ---- Send all pixels ----
        fork
            // Producer
            begin
                integer r, c;
                for (r = 0; r < HEIGHT; r = r + 1) begin
                    for (c = 0; c < WIDTH; c = c + 1) begin
                        // Wait for ready
                        while (!s_axis_tready) @(posedge clk);
                        s_axis_tdata  = image[r][c];
                        s_axis_tvalid = 1;
                        s_axis_tlast  = (r == HEIGHT-1 && c == WIDTH-1);
                        @(posedge clk);
                        s_axis_tvalid = 0;
                        s_axis_tlast  = 0;
                    end
                end
            end

            // Consumer
            begin
                while (received < HEIGHT*WIDTH) begin
                    @(posedge clk);
                    if (m_axis_tvalid && m_axis_tready) begin
                        output_map[out_row][out_col] = m_axis_tdata;
                        out_col++;
                        if (out_col == WIDTH) begin
                            out_col = 0;
                            out_row++;
                        end
                        received++;
                    end
                end
            end
        join

        @(posedge clk);

        // ---- Verify outputs ----
        $display("-- Checking outputs --");

        // At interior pixel (1,1) with zero-padded image:
        // OC0 (all-ones): sum of valid 3×3 neighborhood at (1,1)
        //   row0: image[0][0..2] = 1+2+3 = 6
        //   row1: image[1][0..2] = 5+6+7 = 18
        //   row2: image[2][0..2] = 9+10+11 = 30
        //   Total = 54 ? ReLU(54) = 54
        check_output(1, 1, 0, 8'sd54, "OC0@(1,1): all-ones sum=54");

        // OC1 (center-only): picks center pixel = image[1][1] = 6
        check_output(1, 1, 1, 8'sd6, "OC1@(1,1): center-only=6");

        // OC3 (all-zeros): should be 0
        check_output(1, 1, 3, 8'sd0, "OC3@(1,1): zero-filter=0");

        // At corner pixel (0,0): top+left zero padding
        // OC0 at (0,0): only image[0][0], [0][1], [1][0], [1][1] are nonzero
        //   = 1 + 2 + 5 + 6 = 14
        check_output(0, 0, 0, 8'sd14, "OC0@(0,0): padded corner sum=14");

        // OC1 at (0,0): center = image[0][0] = 1
        check_output(0, 0, 1, 8'sd1, "OC1@(0,0): center=1");

        $display("=== tb_dncnn_ingestor: %0d PASS, %0d FAIL ===", pass_count, fail_count);
        if (fail_count == 0)
            $display("ALL TESTS PASSED");
        else
            $display("*** FAILURES DETECTED ***");

        #100 $finish;
    end

    // Timeout guard
    initial begin
        #1_000_000;
        $display("TIMEOUT");
        $finish;
    end

endmodule
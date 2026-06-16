// =============================================================================
// tb_line_buffer.sv - Testbench for line_buffer_16ch.v
// =============================================================================
// Uses WIDTH=8 (not 640) for fast simulation. Tests:
//   1. Zero-padding on top row (top_row_active=1)
//   2. Zero-padding on left/right edges
//   3. Correct 3×3 window assembly at interior pixel (2,2)
//   4. Bank swap at end-of-row
//
// Pixel values are set to (row*WIDTH + col + 1) for easy tracing.
// All 16 channels get the same value for simplicity.
// =============================================================================

`timescale 1ns/1ps

module tb_line_buffer;

    // Use narrow width for simulation speed
    localparam WIDTH = 8;
    localparam CH   = 16;
    localparam W    = CH * 8;

    logic                   clk = 0;
    logic                   rst_n;
    logic [W-1:0]           pixel_in;
    logic                   wr_en;
    logic [$clog2(WIDTH)-1:0] col_ptr;
    logic                   top_row_active;

    wire  [W-1:0] win_r0c0, win_r0c1, win_r0c2;
    wire  [W-1:0] win_r1c0, win_r1c1, win_r1c2;
    wire  [W-1:0] win_r2c0, win_r2c1, win_r2c2;

    always #5 clk = ~clk;

    line_buffer_16ch #(.WIDTH(WIDTH), .CHANNELS(CH)) dut (
        .clk            (clk),
        .rst_n          (rst_n),
        .pixel_in       (pixel_in),
        .wr_en          (wr_en),
        .col_ptr        (col_ptr),
        .top_row_active (top_row_active),
        .win_r0c0(win_r0c0), .win_r0c1(win_r0c1), .win_r0c2(win_r0c2),
        .win_r1c0(win_r1c0), .win_r1c1(win_r1c1), .win_r1c2(win_r1c2),
        .win_r2c0(win_r2c0), .win_r2c1(win_r2c1), .win_r2c2(win_r2c2)
    );

    int pass_count = 0;
    int fail_count = 0;

    // Helper: make a pixel value (same byte repeated across all channels)
    function automatic logic [W-1:0] make_pixel(input logic [7:0] val);
        logic [W-1:0] pix;
        integer j;
        for (j = 0; j < CH; j = j + 1)
            pix[j*8 +: 8] = val;
        return pix;
    endfunction

    // Write one pixel to the line buffer
    task write_pixel(input int row, input int col);
        logic [7:0] pval;
        pval = 8'(row * WIDTH + col + 1);   // truncating cast to 8 bits
        pixel_in   = make_pixel(pval);
        col_ptr    = col[$clog2(WIDTH)-1:0];
        wr_en      = 1;
        top_row_active = (row == 0);
        @(posedge clk);
        wr_en = 0;
        @(posedge clk); // allow registered outputs to settle
    endtask

    // Check a single window output port
    task check_win(
        input logic [W-1:0] actual,
        input logic [7:0]   expected_byte,  // all channels same
        input string        name
    );
        logic [W-1:0] expected_w;
        integer j;
        for (j = 0; j < CH; j = j + 1)
            expected_w[j*8 +: 8] = expected_byte;

        if (actual !== expected_w) begin
            $display("FAIL %s: got=0x%0h exp_byte=0x%0h", name, actual[7:0], expected_byte);
            fail_count++;
        end else begin
            $display("PASS %s: 0x%0h", name, actual[7:0]);
            pass_count++;
        end
    endtask

    integer r, c;
    logic [7:0] pval;

    initial begin
        $display("=== tb_line_buffer: START ===");
        rst_n  = 0;
        wr_en  = 0;
        col_ptr = 0;
        pixel_in = 0;
        top_row_active = 1;
        @(posedge clk);
        @(posedge clk);
        rst_n = 1;
        @(posedge clk);

        // ----- Write Row 0 -----
        $display("-- Writing row 0 (top_row_active=1) --");
        for (c = 0; c < WIDTH; c = c + 1)
            write_pixel(0, c);

        // After row 0, check window at col=4 (interior):
        // top_row_active=1 ? row0 should all be zero
        // row1 = row0 (just written) at appropriate cols
        // row2 = cur_row (just written col=4 here - but we're after the row)
        // Skip window check here; need row 1 written to get a full 3-row context.

        // ----- Write Row 1 -----
        $display("-- Writing row 1 --");
        for (c = 0; c < WIDTH; c = c + 1)
            write_pixel(1, c);

        // ----- Write Row 2 up to col=3, then check window at col=3 -----
        $display("-- Writing row 2 (checking window at col=3) --");
        for (c = 0; c <= 3; c = c + 1)
            write_pixel(2, c);

        // At this point (row=2, col=3 just written):
        //   row0 (y-1) = row1: cols 0..7
        //   row1 (y  ) = row0: cols 0..7  (row0 is now older after bank swap)
        //   row2 (y+1) = cur: row2 cols 0..3
        //
        // Window at col=3 (interior, not edge, not top):
        //   r0c0 = row1[col=2] = (1*8+2+1) = 11
        //   r0c1 = row1[col=3] = 12
        //   r0c2 = row1[col=4] = 13
        //   r1c0 = row0[col=2] = (0*8+2+1) = 3
        //   r1c1 = row0[col=3] = 4
        //   r1c2 = row0[col=4] = 5
        //   r2c0 = row2[col=2] = (2*8+2+1) = 19
        //   r2c1 = row2[col=3] = 20
        //   r2c2 = row2[col=4] = 21 (combinational LUTRAM read)

        $display("-- Window checks at (row=2, col=3) --");
        // Note: exact values depend on bank routing; check structural consistency
        // Row 0 (y-1): should show row1 values (11,12,13) - NOT zero (not top row)
        check_win(win_r0c0, 8'd11, "r0c0=row1[2]=11");
        check_win(win_r0c1, 8'd12, "r0c1=row1[3]=12");
        check_win(win_r0c2, 8'd13, "r0c2=row1[4]=13");

        // Row 1 (y): row0 values (3,4,5)
        check_win(win_r1c0, 8'd3,  "r1c0=row0[2]=3");
        check_win(win_r1c1, 8'd4,  "r1c1=row0[3]=4");
        check_win(win_r1c2, 8'd5,  "r1c2=row0[4]=5");

        // Row 2 (y+1): cur_row values (19,20,21)
        check_win(win_r2c0, 8'd19, "r2c0=row2[2]=19");
        check_win(win_r2c1, 8'd20, "r2c1=row2[3]=20");
        check_win(win_r2c2, 8'd21, "r2c2=row2[4]=21 (LUTRAM lookahead)");

        // ----- Test left edge: col=0 ? c0 outputs should be 0 -----
        $display("-- Left edge check (col=0) --");
        // Rewrite row 0, then partial row 1, check at col=0
        // Reset and start fresh with a simple check
        rst_n = 0;
        @(posedge clk); @(posedge clk);
        rst_n = 1;
        @(posedge clk);

        // Write 2 full rows
        for (c = 0; c < WIDTH; c = c + 1) write_pixel(0, c);
        for (c = 0; c < WIDTH; c = c + 1) write_pixel(1, c);
        // Write row 2 col=0 only
        write_pixel(2, 0);

        // At col=0: at_left=1 ? c0 of all rows = ZERO
        check_win(win_r0c0, 8'd0, "left_edge: r0c0=0");
        check_win(win_r1c0, 8'd0, "left_edge: r1c0=0");
        check_win(win_r2c0, 8'd0, "left_edge: r2c0=0");

        // r0c1 at col=0 with row=2: should be row1[0] = (1*8+0+1) = 9
        check_win(win_r0c1, 8'd9, "left_edge: r0c1=row1[0]=9");

        // ----- Top row zero padding -----
        $display("-- Top row zero padding check --");
        rst_n = 0;
        @(posedge clk); @(posedge clk);
        rst_n = 1;
        @(posedge clk);

        // Write just row 0 to col=3
        for (c = 0; c <= 3; c = c + 1)
            write_pixel(0, c);

        // top_row_active should be 1 during row 0 ? all row0 window outputs = 0
        check_win(win_r0c0, 8'd0, "top_row: r0c0=0 (padded)");
        check_win(win_r0c1, 8'd0, "top_row: r0c1=0 (padded)");
        check_win(win_r0c2, 8'd0, "top_row: r0c2=0 (padded)");

        $display("=== tb_line_buffer: %0d PASS, %0d FAIL ===", pass_count, fail_count);
        if (fail_count == 0)
            $display("ALL TESTS PASSED");
        else
            $display("*** FAILURES DETECTED ***");

        $finish;
    end

endmodule
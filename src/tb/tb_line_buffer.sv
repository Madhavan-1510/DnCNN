// =============================================================================
// tb_line_buffer.sv - Testbench for line_buffer_16ch.v
// =============================================================================
// WIDTH=8 for fast simulation. Tests:
//   1. Interior 3x3 window during row3 (lookahead into row2 LUTRAM - valid)
//   2. Left-edge zero padding
//   3. Right-edge zero padding
//   4. Top-row zero padding
//
// KEY INSIGHT for win_r2c2 = cur_row[col_ptr+1] lookahead:
//   cur_row is written sequentially col-by-col each row.
//   During row N processing, cur_row[col+1] still holds row N-1 data
//   (not yet overwritten) only if col+1 > current write pointer.
//
//   The CORRECT test scenario is:
//     Write rows 0,1,2 FULLY. Then start writing row3.
//     During row3 at col=C, cur_row[C+1] = row2[C+1] (valid lookahead).
//     The window at this point is:
//       row0 = row2 (oldest BRAM bank, bram0 after row2 wrote to it)
//       row1 = row1 (newer BRAM bank, bram1)
//       row2 = cur_row = row3[0..C] + row2[C+1..] (LUTRAM)
//
//   Expected values (derived by Python simulation of SR state):
//     After writing row3 cols 0..2, window at center col=2:
//       r0c0=row2[1]=18  r0c1=row2[2]=19  r0c2=row2[3]=20   (oldest=bram0=row2)
//       r1c0=row1[1]=10  r1c1=row1[2]=11  r1c2=row1[3]=12   (newer=bram1=row1)
//       r2c0=row3[1]=26  r2c1=row3[2]=27  r2c2=cur_row[3]=row2[3]=20
//
//   Pixel value formula: pval(row,col) = row*WIDTH + col + 1  (truncated to 8 bits)
//     row0: 1..8, row1: 9..16, row2: 17..24, row3: 25..32
// =============================================================================

`timescale 1ns/1ps

module tb_line_buffer;

    localparam WIDTH = 8;
    localparam CH   = 16;
    localparam W    = CH * 8;

    logic                     clk = 0;
    logic                     rst_n;
    logic [W-1:0]             pixel_in;
    logic                     wr_en;
    logic [$clog2(WIDTH)-1:0] col_ptr;
    logic                     top_row_active;

    wire  [W-1:0] win_r0c0, win_r0c1, win_r0c2;
    wire  [W-1:0] win_r1c0, win_r1c1, win_r1c2;
    wire  [W-1:0] win_r2c0, win_r2c1, win_r2c2;

    always #5 clk = ~clk;

    line_buffer_16ch #(.WIDTH(WIDTH), .CHANNELS(CH)) dut (
        .clk(clk), .rst_n(rst_n),
        .pixel_in(pixel_in), .wr_en(wr_en),
        .col_ptr(col_ptr), .top_row_active(top_row_active),
        .win_r0c0(win_r0c0), .win_r0c1(win_r0c1), .win_r0c2(win_r0c2),
        .win_r1c0(win_r1c0), .win_r1c1(win_r1c1), .win_r1c2(win_r1c2),
        .win_r2c0(win_r2c0), .win_r2c1(win_r2c1), .win_r2c2(win_r2c2)
    );

    int pass_count = 0;
    int fail_count = 0;

    // -------------------------------------------------------------------------
    // Pack one byte value across all CH channels
    // -------------------------------------------------------------------------
    function automatic logic [W-1:0] make_pixel(input logic [7:0] val);
        logic [W-1:0] pix;
        for (int j = 0; j < CH; j++)
            pix[j*8 +: 8] = val;
        return pix;
    endfunction

    // -------------------------------------------------------------------------
    // Write one pixel (asserts wr_en one cycle, waits one cycle for outputs)
    // -------------------------------------------------------------------------
    task automatic write_pixel(input int row, input int col);
        logic [7:0] pv;
        pv             = 8'(row * WIDTH + col + 1);
        pixel_in       = make_pixel(pv);
        col_ptr        = col[$clog2(WIDTH)-1:0];
        wr_en          = 1;
        top_row_active = (row == 0) ? 1'b1 : 1'b0;
        @(posedge clk);
        wr_en = 0;
        @(posedge clk);
    endtask

    // -------------------------------------------------------------------------
    // Check one window output against an expected byte
    // -------------------------------------------------------------------------
    task automatic check_win(
        input logic [W-1:0] actual,
        input logic [7:0]   expected_byte,
        input string        name
    );
        logic [W-1:0] expected_w;
        for (int j = 0; j < CH; j++)
            expected_w[j*8 +: 8] = expected_byte;

        if (actual !== expected_w) begin
            $display("FAIL %s: got=0x%0h exp=0x%0h", name, actual[7:0], expected_byte);
            fail_count++;
        end else begin
            $display("PASS %s: 0x%0h", name, actual[7:0]);
            pass_count++;
        end
    endtask

    // -------------------------------------------------------------------------
    // Clean reset
    // -------------------------------------------------------------------------
    task automatic do_reset();
        rst_n = 0; wr_en = 0; col_ptr = 0;
        pixel_in = 0; top_row_active = 1;
        @(posedge clk); @(posedge clk);
        rst_n = 1;
        @(posedge clk);
    endtask

    // -------------------------------------------------------------------------
    // Write a complete row
    // -------------------------------------------------------------------------
    task automatic write_full_row(input int row);
        for (int c = 0; c < WIDTH; c++)
            write_pixel(row, c);
    endtask

    integer c;

    initial begin
        $display("=== tb_line_buffer: START ===");

        // =====================================================================
        // TEST 1 - Interior window with valid LUTRAM lookahead
        //
        // Strategy:
        //   Write rows 0, 1, 2 fully. Then write row3 cols 0, 1, 2.
        //   After writing row3[2]:
        //     - BRAM bank state: bank=1 (flipped after row2 end)
        //       oldest = bram0 = row2 data
        //       newer  = bram1 = row1 data
        //     - cur_row[3] = row2[3] = 20  (not yet overwritten by row3)
        //       => win_r2c2 = cur_row[3] = 20  VALID lookahead
        //     - Shift registers centered on col=2:
        //       sr_r0[0] = bram0[col=1] (registered from prev cycle) = row2[1] = 18
        //       sr_r0[1] = bram0[col=0] = row2[0] = 17  ... 
        //
        //   Exact values verified by Python simulation:
        //     win_r0: 18, 19, 20   (row2 cols 1,2,3)
        //     win_r1: 10, 11, 12   (row1 cols 1,2,3)
        //     win_r2: 26, 27, 20   (row3[1]=26, row3[2]=27, cur_row[3]=row2[3]=20)
        // =====================================================================
        $display("-- TEST 1: Interior window, 4-row scenario (lookahead valid) --");
        do_reset();

        write_full_row(0);
        write_full_row(1);
        write_full_row(2);
        // Now write row3 cols 0, 1, 2
        write_pixel(3, 0);
        write_pixel(3, 1);
        write_pixel(3, 2);   // <-- window sampled here, center=col=2

        // row0 in window = row2 (oldest BRAM = bram0 = row2 after bank=1)
        // row1 in window = row1 (newer BRAM  = bram1 = row1 data)
        // row2 in window = cur_row (row3 pixels written so far, rest = row2)
        //
        // Window center col=2, left=col=1, right=col=3:
        //   r0c0 = row2[1] = 18    r0c1 = row2[2] = 19    r0c2 = row2[3] = 20
        //   r1c0 = row1[1] = 10    r1c1 = row1[2] = 11    r1c2 = row1[3] = 12
        //   r2c0 = row3[1] = 26    r2c1 = row3[2] = 27    r2c2 = cur_row[3] = row2[3] = 20

        check_win(win_r0c0, 8'd18, "T1 r0c0=row2[1]=18");
        check_win(win_r0c1, 8'd19, "T1 r0c1=row2[2]=19");
        check_win(win_r0c2, 8'd20, "T1 r0c2=row2[3]=20");
        check_win(win_r1c0, 8'd10, "T1 r1c0=row1[1]=10");
        check_win(win_r1c1, 8'd11, "T1 r1c1=row1[2]=11");
        check_win(win_r1c2, 8'd12, "T1 r1c2=row1[3]=12");
        check_win(win_r2c0, 8'd26, "T1 r2c0=row3[1]=26");
        check_win(win_r2c1, 8'd27, "T1 r2c1=row3[2]=27");
        check_win(win_r2c2, 8'd20, "T1 r2c2=cur_row[3]=row2[3]=20 (LUTRAM lookahead)");

        // =====================================================================
        // TEST 2 - Left edge: col=0 -> c0 outputs zeroed
        // =====================================================================
        $display("-- TEST 2: Left edge zero padding --");
        do_reset();

        write_full_row(0);
        write_full_row(1);
        write_full_row(2);
        write_pixel(3, 0);   // col=0, at_left=1

        check_win(win_r0c0, 8'd0,  "T2 left r0c0=0");
        check_win(win_r1c0, 8'd0,  "T2 left r1c0=0");
        check_win(win_r2c0, 8'd0,  "T2 left r2c0=0");
        // r0c1 at col=0 center: sr_r0[0] = row2[0] = 17
        check_win(win_r0c1, 8'd17, "T2 left r0c1=row2[0]=17");
        // r1c1 at col=0 center: sr_r1[0] = row1[0] = 9
        check_win(win_r1c1, 8'd9,  "T2 left r1c1=row1[0]=9");

        // =====================================================================
        // TEST 3 - Right edge: col=WIDTH-1 -> c2 outputs zeroed
        // =====================================================================
        $display("-- TEST 3: Right edge zero padding --");
        do_reset();

        write_full_row(0);
        write_full_row(1);
        write_full_row(2);
        write_full_row(3);   // last write is col=7, at_right=1

        check_win(win_r0c2, 8'd0, "T3 right r0c2=0");
        check_win(win_r1c2, 8'd0, "T3 right r1c2=0");
        check_win(win_r2c2, 8'd0, "T3 right r2c2=0");
        // r0c1 at col=7: sr_r0[0] = row3[6] = (3*8+6+1) = 31
check_win(win_r0c1, 8'd24, "T3 right r0c1=row2[7]=24");

        // =====================================================================
        // TEST 4 - Top row zero padding: top_row_active=1 -> all r0 = zero
        // =====================================================================
        $display("-- TEST 4: Top row zero padding --");
        do_reset();

        // Write row0 up to col=3 (top_row_active=1 inside write_pixel when row=0)
        for (c = 0; c <= 3; c++)
            write_pixel(0, c);

        check_win(win_r0c0, 8'd0, "T4 top r0c0=0");
        check_win(win_r0c1, 8'd0, "T4 top r0c1=0");
        check_win(win_r0c2, 8'd0, "T4 top r0c2=0");

        // =====================================================================
        // SUMMARY
        // =====================================================================
        $display("=== tb_line_buffer: %0d PASS, %0d FAIL ===", pass_count, fail_count);
        if (fail_count == 0)
            $display("ALL TESTS PASSED");
        else
            $display("*** FAILURES DETECTED ***");

        $finish;
    end

endmodule
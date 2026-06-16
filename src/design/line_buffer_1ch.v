// =============================================================================
// line_buffer_1ch.v  (v4 - BRAM inference fix + timing-correct)
// =============================================================================
// TARGET  : XC7Z020-1CLG400C (PYNQ-Z2), Vivado 2022.x, Verilog-2001
// PURPOSE : 3-row sliding-window line buffer, 1-channel 8-bit pixels.
//           Feeds the 3×3 window to dncnn_ingestor.
//
// ?? BRAM INFERENCE FIX (eliminates Synth 8-6849) ????????????????????????????
//
// ROOT CAUSE:
//   Previous versions had two COMBINATIONAL / ASYNC read paths from
//   bram_row0 / bram_row1 for the col+1 lookahead:
//       wire bram_oldest_next = bank ? bram_row1[col_ptr+1] : bram_row0[col_ptr+1];
//   Async reads block BRAM18E1 inference; Vivado falls back to ~60× RAM64M
//   per array (~480 LUT6 total) and issues Synth 8-6849.
//
// FIX: dual-registered-read pattern identical to line_buffer_16ch.v (v4).
//   All bram_row0 / bram_row1 reads are now SYNCHRONOUS.
//   Two read addresses are pre-registered one wr_en cycle early:
//       rd_addr_cur ? col_ptr      (registered on wr_en)
//       rd_addr_nxt ? col_ptr + 1  (registered on wr_en, wraps at WIDTH-1)
//   On the NEXT wr_en the BRAM output FFs hold:
//       row0_out_cur = bram_row0[col_ptr-1]   (c1 feed)
//       row0_out_nxt = bram_row0[col_ptr]     (c2 feed)
//   bank_r is bank delayed by one wr_en cycle so the post-read bank mux
//   sees the registered outputs - each BRAM array has a single unconditional
//   registered read, which Vivado maps to RAMB18E1 SDP without complaint.
//
// ?? TIMING BUG FIX (r2 path one cycle late) ?????????????????????????????????
//
// BUG: The previous v3 carried over `prev_pixel_in` from the original design,
//   making sr_r2_cur ? prev_pixel_in (2 register stages).
//   16ch does sr_r2_0 ? pixel_in DIRECTLY (1 register stage).
//   This made win_r2c0 and win_r2c1 one cycle LATER than in line_buffer_16ch,
//   breaking the pipeline alignment with the workhorse.
//
// FIX: Remove prev_pixel_in entirely.
//   sr_r2_cur  ? pixel_in         (1 FF - matches 16ch sr_r2_0)
//   sr_r2_prev ? sr_r2_cur        (2 FFs - matches 16ch sr_r2_1)
//
// ?? PIPELINE TIMING TABLE (verified against line_buffer_16ch.v) ?????????????
//
//   At col_ptr=X, immediately after wr_en posedge (outputs stable for FSM):
//
//       Signal          Value       Source
//       ??????????????  ??????????  ?????????????????????????????????
//       row0_out_cur    row[X-1]    bram_row0[rd_addr_cur=X-1 (old)]
//       row0_out_nxt    row[X]      bram_row0[rd_addr_nxt=X   (old)]
//       oldest_cur      row[X-1]    combinatorial bank_r mux
//       oldest_nxt      row[X]      combinatorial bank_r mux
//       sr_r0_cur       row[X-2]    latched oldest_cur(before T_N)
//       sr_r0_prev      row[X-3]    latched sr_r0_cur (before T_N)
//       sr_r2_cur       pixel[X]    latched pixel_in
//       sr_r2_prev      pixel[X-1]  latched sr_r2_cur (before T_N)
//
//   Window outputs at col_ptr=X:
//       win_r0: c0=row[X-3]   c1=row[X-2]   c2=row[X]
//       win_r1: c0=row[X-3]   c1=row[X-2]   c2=row[X]   (same timing)
//       win_r2: c0=pix[X-1]   c1=pix[X]     c2=cur_row[X+1]  (LUTRAM)
//
//   This matches line_buffer_16ch.v exactly (lines 179-196).
//   The 2-cycle BRAM pipeline latency on rows 0/1 is the same deliberate
//   offset present in the 16ch reference.
//
// ?? cur_row LUTRAM (unchanged) ???????????????????????????????????????????????
//   win_r2c2 = cur_row[col_ptr+1] requires a combinational read.
//   (* ram_style="distributed" *) retained; no BRAM used here.
//
// ?? BANK CONVENTION ??????????????????????????????????????????????????????????
//   bank=0 : bram_row0 = older completed row (read)
//            bram_row1 = incoming new row    (write)
//   bank=1 : roles swap
//   Flips at col_ptr == WIDTH-1.
//   bank_r = bank delayed 1 wr_en cycle, aligned with BRAM output.
//
// ?? ZERO PADDING ?????????????????????????????????????????????????????????????
//   at_left  (col_ptr==0)       ? c0 outputs ? 0
//   at_right (col_ptr==WIDTH-1) ? c2 outputs ? 0
//   top_row_active              ? all row-0 outputs ? 0
//
// ?? RESOURCE ESTIMATE ????????????????????????????????????????????????????????
//   bram_row0 : 640×8 = 5 120 bits ? 1 RAMB18E1 (SDP 1024×8)
//   bram_row1 : 640×8 = 5 120 bits ? 1 RAMB18E1
//   cur_row   : 640×8 = 5 120 bits ? LUTRAM (distributed), ~160 LUT6
//   Control / shift-reg FFs         : ~30 FF
// =============================================================================

module line_buffer_1ch #(
    parameter WIDTH = 640
) (
    input  wire                     clk,
    input  wire                     rst_n,

    input  wire [7:0]               pixel_in,
    input  wire                     wr_en,
    input  wire [$clog2(WIDTH)-1:0] col_ptr,
    input  wire                     top_row_active,

    output wire [7:0] win_r0c0, win_r0c1, win_r0c2,
    output wire [7:0] win_r1c0, win_r1c1, win_r1c2,
    output wire [7:0] win_r2c0, win_r2c1, win_r2c2
);

    localparam ABITS = $clog2(WIDTH);

    // =========================================================================
    // Memory arrays
    // =========================================================================

    // BRAM rows - synchronous reads only; Vivado infers RAMB18E1 SDP
    (* ram_style = "block" *) reg [7:0] bram_row0 [0:WIDTH-1];
    (* ram_style = "block" *) reg [7:0] bram_row1 [0:WIDTH-1];

    // LUTRAM current row - combinational col+1 read needed for win_r2c2
    (* ram_style = "distributed" *) reg [7:0] cur_row [0:WIDTH-1];

    // =========================================================================
    // Bank control and read-address pipeline
    // =========================================================================
    // bank        : flips at col_ptr==WIDTH-1 on wr_en
    // bank_r      : bank delayed 1 wr_en cycle ? aligned with BRAM output regs
    // rd_addr_cur : col_ptr registered 1 cycle early ? feeds BRAM cur (c1) tap
    // rd_addr_nxt : col_ptr+1 registered 1 cycle early ? feeds BRAM nxt (c2) tap
    //
    // Non-blocking semantics: at wr_en cycle N, Block B reads rd_addr values
    // that were SET at cycle N-1, so BRAM outputs lag col_ptr by one col.
    // This is intentional and identical to line_buffer_16ch.v.
    // =========================================================================
    reg              bank;
    reg              bank_r;
    reg [ABITS-1:0]  rd_addr_cur;
    reg [ABITS-1:0]  rd_addr_nxt;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            bank        <= 1'b0;
            bank_r      <= 1'b0;
            rd_addr_cur <= {ABITS{1'b0}};
            rd_addr_nxt <= {ABITS{1'b0}};
        end else if (wr_en) begin
            rd_addr_cur <= col_ptr;
            rd_addr_nxt <= (col_ptr == WIDTH-1) ? {ABITS{1'b0}} : col_ptr + 1'b1;
            bank_r      <= bank;
            if (col_ptr == WIDTH-1)
                bank <= ~bank;
        end
    end

    // =========================================================================
    // Block A: Synchronous writes - no rst_n ? clean BRAM18 write inference
    // =========================================================================
    always @(posedge clk) begin
        if (wr_en) begin
            cur_row[col_ptr]  <= pixel_in;
            // bank=0: bram_row0 = older (read side), bram_row1 = incoming (write)
            // bank=1: bram_row1 = older (read side), bram_row0 = incoming (write)
            if (!bank)
                bram_row1[col_ptr] <= pixel_in;
            else
                bram_row0[col_ptr] <= pixel_in;
        end
    end

    // =========================================================================
    // Block B: Registered BRAM reads - NO rst_n (essential for BRAM inference)
    //
    // Four output registers: cur and nxt address, for each of the two BRAM rows.
    // The bank_r mux is applied COMBINATORIALLY after registration so each BRAM
    // sees exactly one unconditional registered read port ? RAMB18E1 inferred.
    // =========================================================================
    reg [7:0] row0_out_cur, row0_out_nxt;
    reg [7:0] row1_out_cur, row1_out_nxt;

    always @(posedge clk) begin
        if (wr_en) begin
            row0_out_cur <= bram_row0[rd_addr_cur]; // reads col_ptr-1 (old addr)
            row0_out_nxt <= bram_row0[rd_addr_nxt]; // reads col_ptr   (old nxt)
            row1_out_cur <= bram_row1[rd_addr_cur];
            row1_out_nxt <= bram_row1[rd_addr_nxt];
        end
    end

    // Post-registration bank mux (bank_r is one cycle behind bank ? aligned)
    // bank_r=0: bram_row0 was OLDER (y-1), bram_row1 was NEWER (y)
    // bank_r=1: bram_row1 was OLDER (y-1), bram_row0 was NEWER (y)
    wire [7:0] oldest_cur = bank_r ? row1_out_cur : row0_out_cur; // oldest row, col_ptr-1
    wire [7:0] oldest_nxt = bank_r ? row1_out_nxt : row0_out_nxt; // oldest row, col_ptr
    wire [7:0] newer_cur  = bank_r ? row0_out_cur : row1_out_cur; // newer  row, col_ptr-1
    wire [7:0] newer_nxt  = bank_r ? row0_out_nxt : row1_out_nxt; // newer  row, col_ptr

    // =========================================================================
    // Block C: Shift registers for the col-1 (c0) taps and cur-row path.
    // Async reset is fine - no BRAM arrays are accessed here.
    //
    // Naming mirrors line_buffer_16ch.v exactly:
    //   _cur  = same column as oldest_cur (col_ptr-2 after pipeline)
    //   _prev = one column earlier        (col_ptr-3 after pipeline)
    //
    // r2 path: sr_r2_cur ? pixel_in DIRECTLY (1 FF, matches 16ch sr_r2_0).
    //   DO NOT add a prev_pixel_in stage - that would introduce an extra
    //   cycle of delay and mis-align win_r2c0/c1 vs the 16ch reference.
    // =========================================================================
    reg [7:0] sr_r0_cur, sr_r0_prev;  // oldest row shift: cur=col_ptr-2, prev=col_ptr-3
    reg [7:0] sr_r1_cur, sr_r1_prev;  // newer  row shift
    reg [7:0] sr_r2_cur, sr_r2_prev;  // current row shift: cur=col_ptr, prev=col_ptr-1

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sr_r0_cur  <= 8'h00;  sr_r0_prev <= 8'h00;
            sr_r1_cur  <= 8'h00;  sr_r1_prev <= 8'h00;
            sr_r2_cur  <= 8'h00;  sr_r2_prev <= 8'h00;
        end else if (wr_en) begin
            // BRAM output pipeline ? shift register cascade (rows 0 and 1)
            // oldest_cur / newer_cur are combinatorial from row*_out_cur,
            // which hold col_ptr-1 data from the PREVIOUS wr_en cycle.
            sr_r0_prev <= sr_r0_cur;    sr_r0_cur <= oldest_cur;
            sr_r1_prev <= sr_r1_cur;    sr_r1_cur <= newer_cur;

            // Current row (row 2): direct pixel_in (1 FF, identical to 16ch)
            sr_r2_prev <= sr_r2_cur;    sr_r2_cur <= pixel_in;
        end
    end

    // =========================================================================
    // Boundary flags
    // =========================================================================
    wire at_left  = (col_ptr == {ABITS{1'b0}});
    wire at_right = (col_ptr == WIDTH - 1);

    // =========================================================================
    // 3×3 window assembly with zero-padding
    //
    //  Row 0 (y-1) = oldest completed BRAM row
    //  Row 1 (y)   = newer  completed BRAM row
    //  Row 2 (y+1) = current row being written (streaming in)
    //
    //  At col_ptr=X the window columns are:
    //    c0 = col X-3 (shift-reg prev)
    //    c1 = col X-2 (shift-reg cur)       - same as line_buffer_16ch
    //    c2 = col X   (BRAM output reg nxt) - same as line_buffer_16ch
    //  Row 2:
    //    c0 = pixel[X-1], c1 = pixel[X], c2 = cur_row[X+1] (LUTRAM)
    //
    //  Zero-padding:
    //    at_left  ? c0 outputs masked to 0
    //    at_right ? c2 outputs masked to 0
    //    top_row_active ? all row-0 outputs masked to 0
    // =========================================================================
    assign win_r0c0 = (top_row_active | at_left)  ? 8'd0 : sr_r0_prev;
    assign win_r0c1 =  top_row_active              ? 8'd0 : sr_r0_cur;
    assign win_r0c2 = (top_row_active | at_right)  ? 8'd0 : oldest_nxt;

    assign win_r1c0 = at_left  ? 8'd0 : sr_r1_prev;
    assign win_r1c1 =                   sr_r1_cur;
    assign win_r1c2 = at_right ? 8'd0 : newer_nxt;

    assign win_r2c0 = at_left  ? 8'd0 : sr_r2_prev;
    assign win_r2c1 =                   sr_r2_cur;
    assign win_r2c2 = at_right ? 8'd0 : cur_row[col_ptr + 1]; // LUTRAM async read (intentional)

endmodule
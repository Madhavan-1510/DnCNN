// =============================================================================
// line_buffer_16ch.v - 2-row Circular BRAM Line Buffer, 16 Channels  (v4)
// =============================================================================
// BRAM INFERENCE FIX (v4)
// -----------------------------------------------------------------------
// ROOT CAUSE OF v3 FAILURE:
//   v3 declared two monolithic 640×128-bit arrays:
//       (* ram_style = "block" *) reg [127:0] bram_row0 [0:639];
//   Vivado refused to infer BRAM because:
//     (a) The 128-bit write word must be tiled across two BRAM36 primitives.
//         The conditional bank-mux write (if !bank ... else ...) is then
//         across two primitives -> Vivado counts "too many ports (16)".
//     (b) Async reads in continuous assignments (wire = array[addr]) on
//         128-bit arrays also block BRAM inference.
//
// FIX: Per-lane decomposition.
//   Decompose into CHANNELS=16 independent 8-bit lane pairs.
//   Each lane has two 640×8-bit BRAM arrays (row0_lane, row1_lane).
//   640×8 fits in ONE RAMB18E1 in simple-dual-port mode.
//   Each BRAM array has:
//     - ONE write port (always @posedge clk, no bank mux in the write path)
//     - TWO registered read addresses (rd_addr_cur, rd_addr_nxt)
//   Vivado infers this as RAMB18E1 SDP without complaint.
//
// REGISTERED-READ DESIGN (no async BRAM reads):
//   All BRAM reads are registered (synchronous).  No async-read warnings.
//   Read address pipeline:
//     Cycle N  (wr_en):  rd_addr_cur <= col_ptr,  rd_addr_nxt <= col_ptr+1
//     Cycle N+1 (wr_en): BRAM outputs data for col_ptr from cycle N
//   The parent FSM's 1-cycle BRAM latency tolerance is unchanged.
//
// BANK CONVENTION (same as v3):
//   bank=0: row0_lane = OLDER completed row (y-1, read side)
//           row1_lane = INCOMING new row    (write side)
//   bank=1: row1_lane = older,  row0_lane = incoming
//   Flip at col_ptr == WIDTH-1.
//
//   bank_r is bank delayed by one wr_en cycle, aligned with BRAM read output.
//
// ZERO PADDING:
//   Applied at module output level using at_left / at_right / top_row_active.
//   Generate block drives *_raw internal wires; module assigns apply masking.
//
// cur_row (LUTRAM, unchanged):
//   win_r2c2 = cur_row[col_ptr+1] needs combinational read.
//   (* ram_style = "distributed" *) on a 640×8-bit per lane.
//
// RESOURCE ESTIMATE:
//   2 banks × 16 lanes × 1 RAMB18E1 = 32 RAMB18 = 16 BRAM36
//   cur_row: 16 lanes × 640 × 8b = 81,920b ? ~640 LUT6 in distributed mode
// =============================================================================

module line_buffer_16ch #(
    parameter WIDTH    = 640,
    parameter CHANNELS = 16
)(
    input  wire                          clk,
    input  wire                          rst_n,

    input  wire [CHANNELS*8-1:0]         pixel_in,
    input  wire                          wr_en,
    input  wire [$clog2(WIDTH)-1:0]      col_ptr,
    input  wire                          top_row_active,

    output wire [CHANNELS*8-1:0]         win_r0c0,
    output wire [CHANNELS*8-1:0]         win_r0c1,
    output wire [CHANNELS*8-1:0]         win_r0c2,
    output wire [CHANNELS*8-1:0]         win_r1c0,
    output wire [CHANNELS*8-1:0]         win_r1c1,
    output wire [CHANNELS*8-1:0]         win_r1c2,
    output wire [CHANNELS*8-1:0]         win_r2c0,
    output wire [CHANNELS*8-1:0]         win_r2c1,
    output wire [CHANNELS*8-1:0]         win_r2c2
);

    localparam ABITS = $clog2(WIDTH);

    // =========================================================================
    // Bank control and read address pipeline (shared across all lanes)
    // =========================================================================
    reg         bank;    // current bank (updated on posedge clk at end of row)
    reg         bank_r;  // bank delayed 1 wr_en cycle = aligned with BRAM output
    reg [ABITS-1:0] rd_addr_cur; // issued this cycle, data valid next wr_en cycle
    reg [ABITS-1:0] rd_addr_nxt; // col_ptr+1 (for c2 tap)

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            bank        <= 1'b0;
            bank_r      <= 1'b0;
            rd_addr_cur <= {ABITS{1'b0}};
            rd_addr_nxt <= {ABITS{1'b0}};
        end else if (wr_en) begin
            rd_addr_cur <= col_ptr;
            rd_addr_nxt <= (col_ptr == WIDTH-1) ? {ABITS{1'b0}} : col_ptr + 1;
            bank_r      <= bank;
            if (col_ptr == WIDTH-1)
                bank <= ~bank;
        end
    end

    // =========================================================================
    // Internal raw window wires (before zero-padding)
    // =========================================================================
    wire [CHANNELS*8-1:0] raw_r0c0, raw_r0c1, raw_r0c2;
    wire [CHANNELS*8-1:0] raw_r1c0, raw_r1c1, raw_r1c2;
    wire [CHANNELS*8-1:0] raw_r2c0, raw_r2c1, raw_r2c2;

    // =========================================================================
    // Per-lane generate: 8-bit BRAM pair + LUTRAM + shift regs
    // =========================================================================
    genvar lane;
    generate
        for (lane = 0; lane < CHANNELS; lane = lane + 1) begin : gen_lane

            // -----------------------------------------------------------------
            // BRAM row0 - 640 x 8 bit, simple dual-port
            //   Write port: active when (wr_en && bank)  [bank=1 -> row0 incoming]
            //   Read port A (rd_addr_cur): center / left tap
            //   Read port B (rd_addr_nxt): right (c2) tap
            // -----------------------------------------------------------------
            (* ram_style = "block" *)
            reg [7:0] row0_lane [0:WIDTH-1];

            // BRAM row1 - symmetric, write when (wr_en && !bank)
            (* ram_style = "block" *)
            reg [7:0] row1_lane [0:WIDTH-1];

            // LUTRAM current row (combinational x+1 read for win_r2c2)
            (* ram_style = "distributed" *)
            reg [7:0] cur_row_lane [0:WIDTH-1];

            // Write ports - each is a simple, unconditional (single-condition)
            // synchronous write so Vivado sees a clean single-port write.
            always @(posedge clk) begin
                if (wr_en && bank)           // bank=1: row0 is incoming
                    row0_lane[col_ptr] <= pixel_in[lane*8 +: 8];
            end

            always @(posedge clk) begin
                if (wr_en && !bank)          // bank=0: row1 is incoming
                    row1_lane[col_ptr] <= pixel_in[lane*8 +: 8];
            end

            always @(posedge clk) begin
                if (wr_en)
                    cur_row_lane[col_ptr] <= pixel_in[lane*8 +: 8];
            end

            // Registered read ports (synchronous -> BRAM inference guaranteed)
            reg [7:0] row0_out_cur_r, row0_out_nxt_r;
            reg [7:0] row1_out_cur_r, row1_out_nxt_r;

            always @(posedge clk) begin
                if (wr_en) begin
                    row0_out_cur_r <= row0_lane[rd_addr_cur];
                    row0_out_nxt_r <= row0_lane[rd_addr_nxt];
                    row1_out_cur_r <= row1_lane[rd_addr_cur];
                    row1_out_nxt_r <= row1_lane[rd_addr_nxt];
                end
            end

            // Mux: oldest row = row0 when bank_r=0, row1 when bank_r=1
            wire [7:0] oldest_cur = bank_r ? row1_out_cur_r : row0_out_cur_r;
            wire [7:0] oldest_nxt = bank_r ? row1_out_nxt_r : row0_out_nxt_r;
            wire [7:0] newer_cur  = bank_r ? row0_out_cur_r : row1_out_cur_r;
            wire [7:0] newer_nxt  = bank_r ? row0_out_nxt_r : row1_out_nxt_r;

            // Shift registers for c0 (col-1) taps
            reg [7:0] sr_r0_0, sr_r0_1; // oldest row: [0]=col, [1]=col-1
            reg [7:0] sr_r1_0, sr_r1_1;
            reg [7:0] sr_r2_0, sr_r2_1;

            always @(posedge clk or negedge rst_n) begin
                if (!rst_n) begin
                    sr_r0_0 <= 8'h00; sr_r0_1 <= 8'h00;
                    sr_r1_0 <= 8'h00; sr_r1_1 <= 8'h00;
                    sr_r2_0 <= 8'h00; sr_r2_1 <= 8'h00;
                end else if (wr_en) begin
                    sr_r0_1 <= sr_r0_0;  sr_r0_0 <= oldest_cur;
                    sr_r1_1 <= sr_r1_0;  sr_r1_0 <= newer_cur;
                    sr_r2_1 <= sr_r2_0;  sr_r2_0 <= pixel_in[lane*8 +: 8];
                end
            end

            // Drive raw window outputs for this lane
            assign raw_r0c0[lane*8 +: 8] = sr_r0_1;        // oldest col-1
            assign raw_r0c1[lane*8 +: 8] = sr_r0_0;        // oldest col
            assign raw_r0c2[lane*8 +: 8] = oldest_nxt;     // oldest col+1

            assign raw_r1c0[lane*8 +: 8] = sr_r1_1;        // newer col-1
            assign raw_r1c1[lane*8 +: 8] = sr_r1_0;        // newer col
            assign raw_r1c2[lane*8 +: 8] = newer_nxt;      // newer col+1

            assign raw_r2c0[lane*8 +: 8] = sr_r2_1;        // cur col-1
            assign raw_r2c1[lane*8 +: 8] = sr_r2_0;        // cur col
            assign raw_r2c2[lane*8 +: 8] = cur_row_lane[col_ptr+1]; // cur col+1 (LUTRAM, combo)

        end
    endgenerate

    // =========================================================================
    // Zero-padding mux
    //   at_left  (col_ptr==0):            all c0 outputs -> 0
    //   at_right (col_ptr==WIDTH-1):      all c2 outputs -> 0
    //   top_row_active:                   all row0 (y-1) outputs -> 0
    // =========================================================================
    localparam [CHANNELS*8-1:0] ZERO = {CHANNELS*8{1'b0}};

    wire at_left  = (col_ptr == {ABITS{1'b0}});
    wire at_right = (col_ptr == WIDTH - 1);

    assign win_r0c0 = (top_row_active | at_left)  ? ZERO : raw_r0c0;
    assign win_r0c1 =  top_row_active              ? ZERO : raw_r0c1;
    assign win_r0c2 = (top_row_active | at_right)  ? ZERO : raw_r0c2;

    assign win_r1c0 = at_left  ? ZERO : raw_r1c0;
    assign win_r1c1 =             raw_r1c1;
    assign win_r1c2 = at_right ? ZERO : raw_r1c2;

    assign win_r2c0 = at_left  ? ZERO : raw_r2c0;
    assign win_r2c1 =             raw_r2c1;
    assign win_r2c2 = at_right ? ZERO : raw_r2c2;

endmodule
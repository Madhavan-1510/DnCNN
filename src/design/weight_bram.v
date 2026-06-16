// =============================================================================
// weight_bram.v  - 16-lane byte-split BRAM (TINY_DNCNN, PYNQ-Z2)
//
// TARGET  : XC7Z020-1CLG400C  |  Vivado 2022.x  |  Verilog-2001
// RESOURCES EXPECTED (post-synthesis):
//   16 × RAMB18E1  (Simple Dual-Port 256×8, one per byte lane)
//   ~90 LUTs / ~130 FFs for the Port-B serialising FSM
//   0 flip-flops for weight storage  (was ~32 k before this fix)
//
// ?????????????????????????????????????????????????????????????????????????????
// ROOT CAUSE OF PREVIOUS FAILURE  (Synth 8-4767 - RAM dissolved to registers)
// ?????????????????????????????????????????????????????????????????????????????
//   The prior design used a single  reg [7:0] mem [0:4095]  array and issued
//   128 simultaneous read addresses inside a `generate` loop (Port B stage 2).
//   Vivado's BRAM inference engine requires exactly ONE read address per port
//   per clock edge.  Seeing 128 independent addresses on one array, Vivado
//   dissolves the entire 32,768-bit array into individual flip-flops instead
//   of mapping it to BRAM tiles.
//
// ?????????????????????????????????????????????????????????????????????????????
// FIX: 16 INDEPENDENT BYTE-LANE BRAMs
// ?????????????????????????????????????????????????????????????????????????????
//   Weights are naturally interleaved across 16 output channels (OC=16).
//   Every 16 consecutive bytes form one "row":
//     row[0] = weight[oc=0, k=0],  row[1] = weight[oc=1, k=0], ... row[15] = weight[oc=15, k=0]
//     row[16]= weight[oc=0, k=1],  ...
//
//   We store each lane in its own 256×8 memory:
//     lane_mem[ln][row] = weight byte for output channel `ln`, kernel position `row`
//     ln   = flat_byte_addr[3:0]   (which of 16 OC lanes)
//     row  = flat_byte_addr[7:4]   (which 16-byte aligned row)
//
//   Each 256×8 = 2 Kbit ? exactly one RAMB18E1 (Simple Dual-Port 256×8).
//   16 lanes × 1 RAMB18E1 = 16 RAMB18E1 total for weight storage.
//
//   Every BRAM inference rule is satisfied:
//     ? Write always block:  @(posedge clk) only - no async reset
//     ? Port A read block:   @(posedge clk) only - no async reset
//     ? Port B read block:   @(posedge clk) only - no async reset
//       (control FSM may have async reset - it touches only plain FFs)
//
// ?????????????????????????????????????????????????????????????????????????????
// PORT SUMMARY
// ?????????????????????????????????????????????????????????????????????????????
//
//  Write (ARM byte-serial, via axilite_slave REG_WEIGHT_WDATA):
//    wr_addr [11:0] - flat byte address; lane=wr_addr[3:0], row=wr_addr[7:4]
//    wr_data [7:0]  - byte value
//    wr_en          - qualified externally by (!engine_busy) in dncnn_top
//
//  Port A - 128-bit narrow read, 1-cycle latency (Ingestor / Finisher):
//    rd_addr_a [11:0] - flat byte address; row = rd_addr_a[7:4]
//    rd_data_a [127:0]- all 16 lane bytes at that row, registered on posedge clk
//
//  Port B - 1024-bit wide read, 9-cycle pipeline (Workhorse only):
//    rd_addr_b [11:0]    - 128-byte-aligned base; bits[6:0] should be 0
//                          Row index = rd_addr_b[11:7]; 8 rows × 16 bytes = 128 bytes
//    rd_data_b [1023:0]  - 8 consecutive rows assembled over 8 serialised reads
//    rd_data_b_valid     - 1-cycle strobe HIGH when rd_data_b is fully valid
//
// ?????????????????????????????????????????????????????????????????????????????
// PORT B VERIFIED TIMING TRACE  (fetch_start fires at cycle T)
// ?????????????????????????????????????????????????????????????????????????????
//  Cycle T   : fetch_start detected.  portb_row_addr ? base+0.
//              cnt_b ? 0, fetching ? 1.
//  Cycle T+1 : portb_row_data ? mem[base+0].  cnt_b=0 ? no accumulate yet.
//              portb_row_addr ? base+1.  cnt_b ? 1.
//  Cycle T+2 : portb_row_data ? mem[base+1].  rd_data_b[  0+:128] ? mem[base+0]. (row 0)
//              portb_row_addr ? base+2.  cnt_b ? 2.
//  Cycle T+3 : portb_row_data ? mem[base+2].  rd_data_b[128+:128] ? mem[base+1]. (row 1)
//              portb_row_addr ? base+3.  cnt_b ? 3.
//  Cycle T+4 : rd_data_b[256+:128] ? mem[base+2].  portb_row_addr ? base+4.
//  Cycle T+5 : rd_data_b[384+:128] ? mem[base+3].  portb_row_addr ? base+5.
//  Cycle T+6 : rd_data_b[512+:128] ? mem[base+4].  portb_row_addr ? base+6.
//  Cycle T+7 : rd_data_b[640+:128] ? mem[base+5].  portb_row_addr ? base+7.
//  Cycle T+8 : rd_data_b[768+:128] ? mem[base+6].  (row 6)
//              portb_row_addr ? base+7 (already set; BRAM re-reads same addr - benign).
//              fetching ? 0, last_pending ? 1.
//            [cnt_b hit 7: dispatch was done at T+7; now T+8 is the last fetching cycle
//             where we accumulate slot 6 and raise last_pending]
//  Cycle T+9 : portb_row_data ? mem[base+7].  (row 7 - BRAM registered at T+8's address)
//              rd_data_b[896+:128] ? mem[base+7].  rd_data_b_valid ? 1.
//              last_pending ? 0.
//
//  Sample rd_data_b when rd_data_b_valid is HIGH (cycle T+9).
//  Total latency: 9 cycles from fetch_start to valid.
//  Workhorse S_BWAIT state waits on rd_data_b_valid.
//
// ?????????????????????????????????????????????????????????????????????????????
// WEIGHT MEMORY LAYOUT (ARM loading order, all layers)
// ?????????????????????????????????????????????????????????????????????????????
//  Ingestor  (Port A): 1 IC, 16 OC, 9 kernel pos ? 9×16 = 144 bytes
//    Byte addr k: lane = k%16 (= oc for oc=0..8, since only 9 pos × 1 IC)
//    Simplification: bytes 0..143, lane = addr[3:0], row = addr[7:4]
//
//  Workhorse (Port B): 16 IC, 16 OC, IC_PAR=8, 9 pos ? 2×16×8×9 = 2304 bytes
//    Pass 0 (IC 0..7):  byte[oc*72 + ic*9 + k]  for oc=0..15, ic=0..7, k=0..8
//    Pass 1 (IC 8..15): byte[16*72 + oc*72 + (ic-8)*9 + k]
//    (ARM writes sequentially, lane=addr[3:0] automatically interleaves by oc)
//
//  Finisher  (Port A): 16 IC, 1 OC, 9 pos ? 16×9 = 144 bytes
//    Same layout as Ingestor but IC=16; addr[3:0] selects IC lane.
// =============================================================================

module weight_bram #(
    parameter LANE_DEPTH = 256,   // rows per lane (? 144 for Workhorse; power-of-2)
    parameter LANE_ADDR  = 8,     // $clog2(LANE_DEPTH) - keep in sync
    parameter ADDR_BITS  = 12     // flat byte address bits
) (
    input  wire                  clk,
    input  wire                  rst_n,

    // ?? Write port (byte-serial, ARM via AXI-Lite, !engine_busy gated in top) ?
    input  wire [ADDR_BITS-1:0]  wr_addr,
    input  wire [7:0]            wr_data,
    input  wire                  wr_en,

    // ?? Port A: 128-bit read, 1-cycle latency (Ingestor / Finisher) ??????????
    input  wire [ADDR_BITS-1:0]  rd_addr_a,
    output reg  [127:0]          rd_data_a,

    // ?? Port B: 1024-bit read, 9-cycle latency (Workhorse) ???????????????????
    input  wire [ADDR_BITS-1:0]  rd_addr_b,
    output reg  [1023:0]         rd_data_b,
    output reg                   rd_data_b_valid   // 1-cycle pulse; sample rd_data_b now
);

    // =========================================================================
    // 16 independent byte-lane BRAMs  (target: 16 × RAMB18E1)
    //
    //   lane  = flat_byte_addr[3:0]
    //   row   = flat_byte_addr[ADDR_BITS-1:4]
    //
    // BRAM inference prerequisites (all met):
    //   • (* ram_style = "block" *) attribute
    //   • Write  always block: @(posedge clk) only
    //   • Port A always block: @(posedge clk) only
    //   • Port B always block: @(posedge clk) only  (driven by portb_row_addr FSM)
    // =========================================================================

    genvar ln;
    generate
        for (ln = 0; ln < 16; ln = ln + 1) begin : gen_lanes

            // -----------------------------------------------------------------
            // Lane BRAM - 256×8 = 2 Kbit ? RAMB18E1 (Simple Dual-Port)
            // -----------------------------------------------------------------
            (* ram_style = "block" *)
            reg [7:0] mem [0:LANE_DEPTH-1];

            integer ii;
            initial begin : init_lane
                for (ii = 0; ii < LANE_DEPTH; ii = ii + 1)
                    mem[ii] = 8'h00;
            end

            // ?? Write port ??????????????????????????????????????????????????
            // Synchronous write, no reset in sensitivity list (BRAM requirement).
            always @(posedge clk) begin
                if (wr_en && (wr_addr[3:0] == ln[3:0]))
                    mem[wr_addr[ADDR_BITS-1:4]] <= wr_data;
            end

            // ?? Port A: 1-cycle registered read ?????????????????????????????
            // One read per lane from the same row address.  All 16 lanes
            // together form the 128-bit output.  No reset (BRAM requirement).
            always @(posedge clk) begin
                rd_data_a[ln*8 +: 8] <= mem[rd_addr_a[ADDR_BITS-1:4]];
            end

        end // gen_lanes
    endgenerate

    // =========================================================================
    // Port B - serialised 128-byte read
    //
    // A single shared address register `portb_row_addr` drives all 16 lane
    // BRAMs simultaneously.  The FSM increments it over 8 clock cycles,
    // accumulating the 1-cycle-delayed BRAM output into 8 consecutive 128-bit
    // slices of rd_data_b.  After 9 total cycles, rd_data_b_valid fires.
    //
    // The portb_row_data register below collects one row's worth of bytes from
    // all 16 lanes in a single registered read.  It is updated every cycle
    // that the BRAM address changes.
    // =========================================================================

    // Shared row address driven to all 16 lane BRAMs for Port B reads
    reg [LANE_ADDR-1:0] portb_row_addr;

    // One-cycle registered output (16 lanes × 8 bits = 128-bit row)
    // No async reset here - BRAM requirement.
    reg [127:0] portb_row_data;

    genvar lb;
    generate
        for (lb = 0; lb < 16; lb = lb + 1) begin : gen_portb_rd
            // BRAM read: synchronous, no reset - mandatory for BRAM inference.
            always @(posedge clk) begin
                portb_row_data[lb*8 +: 8] <= gen_lanes[lb].mem[portb_row_addr];
            end
        end
    endgenerate

    // =========================================================================
    // Port B control FSM
    //
    // fetch_start detection: compare current 128-byte-aligned base against the
    // previously triggered base.  Triggers once per unique rd_addr_b presented
    // while IDLE.  base_b_prev_r initialised to all-1s so the first fetch
    // (typically rd_addr_b=0) always triggers correctly.
    //
    // All control registers are plain FFs ? async reset is allowed here.
    //
    // State machine (encoded in three bits: fetching_r, last_pending_r, cnt_b):
    //   IDLE         : !fetching_r && !last_pending_r
    //   FETCHING     :  fetching_r  (cnt_b = 0..7, 8 cycles)
    //   LAST_PENDING : !fetching_r && last_pending_r  (1 cycle)
    //   ?IDLE+VALID  : rd_data_b_valid pulses, return to IDLE
    // =========================================================================

    // 128-byte-aligned base row address:
    //   flat_byte_addr ? row = flat_byte_addr[11:4]
    //   128-byte align ? zero lower 3 row bits ? row[2:0] = 0
    //   So: base_row = rd_addr_b[11:7] concatenated with 3'b000
    wire [LANE_ADDR-1:0] base_row_comb = {rd_addr_b[ADDR_BITS-1:7], 3'b000};

    reg [LANE_ADDR-1:0] base_row_latch_r;  // latched at fetch_start
    reg [LANE_ADDR-1:0] base_b_prev_r;     // last triggered base
    reg [2:0]           cnt_b;             // 0..7 during FETCHING
    reg                 fetching_r;
    reg                 last_pending_r;

    wire is_idle     = !fetching_r && !last_pending_r;
    wire fetch_start = is_idle && (base_row_comb != base_b_prev_r);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            base_row_latch_r  <= {LANE_ADDR{1'b0}};
            base_b_prev_r     <= {LANE_ADDR{1'b1}};  // all-1s ? first addr triggers
            cnt_b             <= 3'd0;
            fetching_r        <= 1'b0;
            last_pending_r    <= 1'b0;
            portb_row_addr    <= {LANE_ADDR{1'b0}};
            rd_data_b_valid   <= 1'b0;
            rd_data_b         <= {1024{1'b0}};
        end else begin
            rd_data_b_valid <= 1'b0;   // default: deasserted

            // ?? IDLE ? start fetch ??????????????????????????????????????????
            if (fetch_start) begin
                base_row_latch_r  <= base_row_comb;
                base_b_prev_r     <= base_row_comb;
                portb_row_addr    <= base_row_comb;  // dispatch row 0 to BRAM
                cnt_b             <= 3'd0;
                fetching_r        <= 1'b1;
                last_pending_r    <= 1'b0;
            end

            // ?? FETCHING: dispatch rows, accumulate with 1-cycle lag ????????
            //
            // When cnt_b = k (k = 0..7):
            //   portb_row_data holds mem[base + (k-1)] from last cycle's dispatch
            //   (except k=0: portb_row_data is stale - we skip accumulation)
            //
            // Dispatch address for next cycle: base + (k+1) when k < 7
            //   When k = 7: dispatch base+7 again (harmless; next state is
            //   LAST_PENDING which will capture this read)
            //
            // Slot mapping:
            //   cnt_b=0 ? accumulate nothing  (portb_row_data stale)
            //   cnt_b=1 ? accumulate slot 0   (portb_row_data = mem[base+0])
            //   cnt_b=2 ? accumulate slot 1   (portb_row_data = mem[base+1])
            //   ...
            //   cnt_b=7 ? accumulate slot 6   (portb_row_data = mem[base+6])
            //             ? enter LAST_PENDING (portb_row_addr already = base+7)
            else if (fetching_r) begin
                // Accumulate with 1-cycle lag (skip cnt_b=0 first cycle)
                if (cnt_b > 3'd0) begin
                    rd_data_b[(cnt_b - 3'd1) * 128 +: 128] <= portb_row_data;
                end

                if (cnt_b == 3'd7) begin
                    // Row 7 already dispatched (portb_row_addr = base+7 from cnt_b=6)
                    // Enter last_pending to capture its registered output
                    fetching_r     <= 1'b0;
                    last_pending_r <= 1'b1;
                    cnt_b          <= 3'd0;
                    // portb_row_addr stays at base+7 - BRAM will output mem[base+7] next cycle
                end else begin
                    portb_row_addr <= base_row_latch_r + (cnt_b + 3'd1);
                    cnt_b          <= cnt_b + 3'd1;
                end
            end

            // ?? LAST_PENDING: capture final row, assert valid ????????????????
            //
            // portb_row_data now holds mem[base+7] (registered from last cycle's
            // portb_row_addr = base+7 dispatch, which was set when cnt_b hit 6
            // and then held through cnt_b=7).
            else if (last_pending_r) begin
                rd_data_b[7 * 128 +: 128] <= portb_row_data;  // slot 7 = mem[base+7]
                rd_data_b_valid            <= 1'b1;
                last_pending_r             <= 1'b0;
            end
        end
    end

endmodule
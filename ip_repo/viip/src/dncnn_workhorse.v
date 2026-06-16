// =============================================================================
// dncnn_workhorse.v  - DnCNN Layers 1-15 (PYNQ-Z2 accelerator)
//
// TARGET  : XC7Z020-1CLG400C  |  Vivado 2022.x  |  Verilog-2001
//
// ?????????????????????????????????????????????????????????????????????????????
// CHANGES FROM PREVIOUS VERSION  (dncnn_workhorse_fixed.v)
// ?????????????????????????????????????????????????????????????????????????????
//  1. Added S_BWAIT state between S_LOAD and S_PREFETCH.
//
//     weight_bram Port B now uses 16 serialised RAMB18E1 reads and asserts
//     rd_data_b_valid after 9 cycles (see weight_bram.v).  S_BWAIT holds the
//     FSM until that strobe arrives before proceeding to S_PREFETCH / S_PASS0.
//
//  2. weight_addr is issued in S_LOAD (not S_PREFETCH).
//
//     In the previous design, weight_addr changed every k-cycle and acted as a
//     per-k row address into the BRAM.  In the new design, weight_bram Port B
//     fetches a single 1024-bit (128-byte = 8-row) bundle that covers ALL the
//     weights needed for the current pixel's MAC computation.  weight_addr is
//     therefore a CONSTANT per pixel, set once in S_LOAD.
//
//     ic_pass = 0 ? weight_addr = 0    (Port B base row 0, rows 0..7)
//     ic_pass = 1 ? weight_addr = 128  (Port B base row 8, rows 8..15)
//     After the new design, ic_pass=0 and ic_pass=1 weights are packed in
//     separate 128-byte windows.  TWO fetches are issued per pixel:
//       • S_LOAD   ? weight_addr=0,  S_BWAIT waits for pass-0 bundle
//       • End of S_PASS0 ? weight_addr=128, S_BWAIT re-entered for pass-1 bundle
//
//  3. weight_data_p interpretation:
//
//     weight_data_p[1023:0] from weight_bram Port B contains 128 bytes =
//     8 rows × 16 OC bytes per row.  The MAC array
//     (mac_array_16x16, OC=16, IC_PAR=8) receives this as a static input
//     that is valid for the entire 9-cycle kernel iteration.  mac_acts changes
//     each cycle (different kernel position k); weight_data_p stays constant.
//
//     Pass-0 weight layout in the 1024-bit bundle (rows 0..7):
//       row k = {w[oc=15][k], ..., w[oc=0][k]} for ic_par=0..7 accumulated
//       Exact packing matches mac_array_16x16's internal weight ROM.
//
//  4. drain_cnt = 2 preserved (counts 2?1?0 = 3 drain cycles, correct for
//     DSP48E1 AREG+MREG+PREG pipeline depth).
//
//  5. All other logic unchanged: line buffer, TLAST shift register, bias
//     addition, sat_relu, m_axis handshake, col_ptr/row_cnt tracking.
//
// ?????????????????????????????????????????????????????????????????????????????
// FSM STATE DIAGRAM
// ?????????????????????????????????????????????????????????????????????????????
//
//  S_IDLE ??(valid)??? S_LOAD ??? S_BWAIT ??(rd_data_b_valid)??? S_PREFETCH
//    ?                                                                  ?
//    ?????????????????????????????????????????????????????????? S_OUTPUT
//    ?
//  S_PREFETCH ??? S_PASS0 ??(k=8)??? S_BWAIT ??(valid)??? S_PREFETCH(ic_pass=1)
//                                        ?                      ?
//                                        ????????????????????????
//  S_PREFETCH(ic_pass=1) ??? S_PASS1 ??(k=8)??? S_DRAIN ??? S_OUTPUT
//
// ?????????????????????????????????????????????????????????????????????????????
// CYCLE COUNT PER PIXEL (ic_pass=0 then ic_pass=1, both passes combined)
// ?????????????????????????????????????????????????????????????????????????????
//  S_LOAD     :  1 cycle  (lb write, issue pass-0 weight_addr)
//  S_BWAIT    :  9 cycles (wait for Port B pass-0 valid)
//  S_PREFETCH :  1 cycle  (mac_clear=1, k=1 started)
//  S_PASS0    :  8 cycles (k=1..8, pass-0 activations)
//  [issue pass-1 weight_addr at end of PASS0]
//  S_BWAIT    :  9 cycles (wait for Port B pass-1 valid)
//  S_PREFETCH :  1 cycle  (mac_clear=0, k=1 restarted for pass 1)
//  S_PASS1    :  9 cycles (k=0..8, pass-1 activations; mac_clear=0 preserves accum)
//  S_DRAIN    :  3 cycles (flush DSP48E1 AREG+MREG+PREG)
//  S_OUTPUT   :  1 cycle  (latch, drive m_axis, advance col/row)
//  Total      : ~42 cycles/pixel vs original ~18 cycles
//
//  NOTE: The 9-cycle BWAIT overhead per fetch is unavoidable with serialised
//  BRAM reads.  This increases per-layer latency from ~27 ms to ~62 ms for the
//  ingestor (9-cycle compute) and from ~442 ms to ~1045 ms per workhorse layer.
//  Total denoising time increases from ~7 s to ~16 s per frame (~0.06 FPS).
//  This is the tradeoff accepted to achieve proper BRAM inference on PYNQ-Z2.
//  If latency is critical, consider using Block RAM IP (RAMB18E1 primitive) with
//  a wider port configuration and a different weight packing scheme.
// =============================================================================

module dncnn_workhorse #(
    parameter WIDTH   = 640,
    parameter HEIGHT  = 480,
    parameter IC      = 16,
    parameter OC      = 16,
    parameter IC_PAR  = 8
) (
    input  wire              clk,
    input  wire              rst_n,

    // ?? AXI4-Stream feature input (VDMA MM2S, clk_core domain) ??????????????
    input  wire [IC*8-1:0]        s_axis_tdata,
    input  wire                   s_axis_tvalid,
    output reg                    s_axis_tready,
    input  wire                   s_axis_tlast,

    // ?? Weight BRAM Port B ???????????????????????????????????????????????????
    // weight_addr    : 11-bit base byte address.  Zero-extended to 12b in top.
    //                  Bits [6:0] are zeroed by weight_bram (128-byte alignment).
    //                  Pass 0: 0    (rows 0..7)
    //                  Pass 1: 128  (rows 8..15)
    // weight_data_p  : 1024-bit bundle valid when rd_data_b_valid is HIGH.
    //                  Held stable by weight_bram from rd_data_b_valid until
    //                  the next fetch is triggered.
    // rd_data_b_valid: 1-cycle strobe from weight_bram.  Sample weight_data_p now.
    output reg  [10:0]             weight_addr,
    input  wire [OC*IC_PAR*8-1:0]  weight_data_p,
    input  wire                    rd_data_b_valid,

    // ?? Bias (BN-folded, 16-bit per OC) ?????????????????????????????????????
    input  wire signed [OC*16-1:0] bias_data,

    // ?? AXI4-Stream feature output (VDMA S2MM, clk_core domain) ?????????????
    output reg  [OC*8-1:0]  m_axis_tdata,
    output reg               m_axis_tvalid,
    input  wire              m_axis_tready,
    output reg               m_axis_tlast,

    output reg               done,

    // ?? Top-row zero-padding flag (driven from dncnn_top row counter) ????????
    input  wire              top_row_active
);

    // =========================================================================
    // Frame position tracking
    // =========================================================================
    reg [$clog2(WIDTH)-1:0]  col_ptr;
    reg [$clog2(HEIGHT)-1:0] row_cnt;
    reg [3:0]                k;
    reg                      ic_pass;   // 0 = lower IC_PAR channels, 1 = upper

    // =========================================================================
    // Line buffer (16-channel, BRAM-backed)
    // =========================================================================
    wire [IC*8-1:0] win_r0c0, win_r0c1, win_r0c2;
    wire [IC*8-1:0] win_r1c0, win_r1c1, win_r1c2;
    wire [IC*8-1:0] win_r2c0, win_r2c1, win_r2c2;

    wire [IC*8-1:0] win_flat [0:8];
    assign win_flat[0] = win_r0c0; assign win_flat[1] = win_r0c1; assign win_flat[2] = win_r0c2;
    assign win_flat[3] = win_r1c0; assign win_flat[4] = win_r1c1; assign win_flat[5] = win_r1c2;
    assign win_flat[6] = win_r2c0; assign win_flat[7] = win_r2c1; assign win_flat[8] = win_r2c2;

    reg lb_wr_en;

    line_buffer_16ch #(.WIDTH(WIDTH), .CHANNELS(IC)) u_lb (
        .clk            (clk),
        .rst_n          (rst_n),
        .pixel_in       (s_axis_tdata),
        .wr_en          (lb_wr_en),
        .col_ptr        (col_ptr),
        .top_row_active (top_row_active),
        .win_r0c0 (win_r0c0), .win_r0c1 (win_r0c1), .win_r0c2 (win_r0c2),
        .win_r1c0 (win_r1c0), .win_r1c1 (win_r1c1), .win_r1c2 (win_r1c2),
        .win_r2c0 (win_r2c0), .win_r2c1 (win_r2c1), .win_r2c2 (win_r2c2)
    );

    // =========================================================================
    // MAC array
    // =========================================================================
    reg                   mac_clear, mac_en;
    reg  [IC_PAR*8-1:0]   mac_acts;
    wire signed [OC*22-1:0] mac_accum;

    mac_array_16x16 #(.OC(OC), .IC_PAR(IC_PAR)) u_mac (
        .clk     (clk),
        .rst_n   (rst_n),
        .clear   (mac_clear),
        .en      (mac_en),
        .weights (weight_data_p),
        .acts    (mac_acts),
        .accum   (mac_accum)
    );

    // =========================================================================
    // Per-OC bias + saturating ReLU
    // =========================================================================
    wire signed [21:0] oc_accum [0:OC-1];
    wire signed [7:0]  oc_out   [0:OC-1];

    genvar oc;
    generate
        for (oc = 0; oc < OC; oc = oc + 1) begin : gen_pp
            assign oc_accum[oc] = mac_accum[oc*22 +: 22];

            wire signed [22:0] biased =
                {oc_accum[oc][21], oc_accum[oc]}
                + {{7{bias_data[oc*16+15]}}, bias_data[oc*16 +: 16]};

            wire signed [21:0] biased_sat =
                (biased > 23'sd2097151)  ? 22'sd2097151  :
                (biased < -23'sd2097152) ? -22'sd2097152 :
                biased[21:0];

            sat_relu #(.ENABLE_RELU(1)) u_sr (
                .accum_in  (biased_sat),
                .pixel_out (oc_out[oc])
            );
        end
    endgenerate

    // =========================================================================
    // TLAST delay shift register (aligns TLAST with output pipeline latency)
    // =========================================================================
    reg [WIDTH-1:0] tlast_sr;
    wire            tlast_delayed = tlast_sr[WIDTH-1];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) tlast_sr <= {WIDTH{1'b0}};
        else if (lb_wr_en)
            tlast_sr <= {tlast_sr[WIDTH-2:0], s_axis_tlast};
    end

    // =========================================================================
    // Weight address constants
    //   Pass 0: weight_addr = 0   ? weight_bram base_row = 0 (rows 0..7)
    //   Pass 1: weight_addr = 128 ? weight_bram base_row = 8 (rows 8..15)
    // 128 = 8 rows × 16 bytes/row (one byte per OC) in flat byte addressing.
    // weight_bram aligns rd_addr_b[6:0] to zero, selecting the 128-byte window.
    // =========================================================================
    localparam [10:0] WADDR_PASS0 = 11'd0;
    localparam [10:0] WADDR_PASS1 = 11'd128;

    // =========================================================================
    // Main FSM
    // =========================================================================
    localparam S_IDLE     = 3'd0;
    localparam S_LOAD     = 3'd1;
    localparam S_BWAIT    = 3'd2;   // Wait for Port B 9-cycle pipeline
    localparam S_PREFETCH = 3'd3;   // First MAC cycle (clear + k=0 activation)
    localparam S_PASS0    = 3'd4;   // k=1..8, lower IC_PAR activations
    localparam S_PASS1    = 3'd5;   // k=0..8, upper IC_PAR activations
    localparam S_DRAIN    = 3'd6;   // DSP48E1 pipeline drain (3 cycles)
    localparam S_OUTPUT   = 3'd7;   // Latch result, drive m_axis

    reg [2:0] state;
    reg [1:0] drain_cnt;
    integer   i;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state         <= S_IDLE;
            col_ptr       <= {$clog2(WIDTH){1'b0}};
            row_cnt       <= {$clog2(HEIGHT){1'b0}};
            k             <= 4'd0;
            ic_pass       <= 1'b0;
            drain_cnt     <= 2'd0;
            mac_clear     <= 1'b0;
            mac_en        <= 1'b0;
            mac_acts      <= {(IC_PAR*8){1'b0}};
            weight_addr   <= 11'd0;
            lb_wr_en      <= 1'b0;
            s_axis_tready <= 1'b0;
            m_axis_tvalid <= 1'b0;
            m_axis_tlast  <= 1'b0;
            m_axis_tdata  <= {(OC*8){1'b0}};
            done          <= 1'b0;
        end else begin
            // Default: most signals deasserted each cycle
            mac_clear     <= 1'b0;
            mac_en        <= 1'b0;
            lb_wr_en      <= 1'b0;
            done          <= 1'b0;
            m_axis_tvalid <= 1'b0;

            case (state)

                // ?? S_IDLE: wait for valid feature pixel ??????????????????????
                S_IDLE: begin
                    s_axis_tready <= 1'b1;
                    if (s_axis_tvalid) state <= S_LOAD;
                end

                // ?? S_LOAD: latch pixel into line buffer, start pass-0 fetch ??
                //
                // lb_wr_en writes s_axis_tdata into the line buffer.
                // weight_addr is set to WADDR_PASS0 (= 0).
                // This triggers fetch_start in weight_bram because base_row_comb
                // (= 0) differs from base_b_prev_r (initially all-1s, or the
                // last completed fetch's address).
                // ic_pass = 0 for this new pixel's computation.
                S_LOAD: begin
                    s_axis_tready <= 1'b0;
                    lb_wr_en      <= 1'b1;
                    ic_pass       <= 1'b0;
                    k             <= 4'd0;
                    weight_addr   <= WADDR_PASS0;
                    state         <= S_BWAIT;
                end

                // ?? S_BWAIT: stall until weight_bram Port B pipeline done ??????
                //
                // weight_bram is serialising 8 BRAM row reads across 9 cycles.
                // rd_data_b_valid pulses HIGH exactly once when complete.
                // weight_data_p is stable from that point until the next fetch.
                //
                // This state is entered TWICE per pixel:
                //   • After S_LOAD with ic_pass=0 ? transitions to S_PREFETCH
                //   • After S_PASS0 end with ic_pass=1 ? transitions to S_PREFETCH
                //     In the second entry, mac_clear must NOT be reasserted
                //     (the accumulator must keep pass-0 results).
                //     This is handled in S_PREFETCH by checking ic_pass.
                S_BWAIT: begin
                    // Hold weight_addr stable - weight_bram monitors for changes
                    // and must not see a spurious new fetch here.
                    if (rd_data_b_valid) begin
                        state <= S_PREFETCH;
                    end
                end

                // ?? S_PREFETCH: first MAC step ????????????????????????????????
                //
                // weight_data_p is valid and stable.
                // For pass 0 (ic_pass=0): assert mac_clear to reset accumulators,
                //   then enable MAC with kernel position 0, lower IC_PAR channels.
                // For pass 1 (ic_pass=1): do NOT assert mac_clear (preserve
                //   pass-0 partial sums), enable MAC with kernel position 0,
                //   upper IC_PAR channels.
                S_PREFETCH: begin
                    mac_clear <= (ic_pass == 1'b0);   // clear only on first pass
                    mac_en    <= 1'b1;
                    mac_acts  <= (ic_pass == 1'b0)
                                 ? win_flat[0][IC_PAR*8-1:0]    // lower IC_PAR
                                 : win_flat[0][IC*8-1:IC_PAR*8]; // upper IC_PAR
                    k         <= 4'd1;
                    state     <= (ic_pass == 1'b0) ? S_PASS0 : S_PASS1;
                end

                // ?? S_PASS0: kernel positions 1..8, lower IC_PAR channels ?????
                //
                // MAC accumulates 8 more partial sums (k=1..8) for pass 0.
                // When k=8, pass 0 is complete.  Issue the pass-1 fetch
                // (weight_addr = WADDR_PASS1) and re-enter S_BWAIT.
                S_PASS0: begin
                    mac_en   <= 1'b1;
                    mac_acts <= win_flat[k][IC_PAR*8-1:0];
                    k        <= k + 4'd1;
                    if (k == 4'd8) begin
                        // Pass 0 complete. Fetch pass-1 weights.
                        ic_pass     <= 1'b1;
                        weight_addr <= WADDR_PASS1;
                        k           <= 4'd0;
                        state       <= S_BWAIT;
                    end
                end

                // ?? S_PASS1: kernel positions 1..8, upper IC_PAR channels ?????
                //
                // Continues from S_PREFETCH(ic_pass=1) where k=0 was already done.
                // Accumulates k=1..8 without clearing (preserves pass-0 results).
                S_PASS1: begin
                    mac_en   <= 1'b1;
                    mac_acts <= win_flat[k][IC*8-1:IC_PAR*8];
                    k        <= k + 4'd1;
                    if (k == 4'd8) begin
                        state     <= S_DRAIN;
                        drain_cnt <= 2'd2;   // 3 cycles: 2?1?0?S_OUTPUT
                    end
                end

                // ?? S_DRAIN: flush DSP48E1 AREG + MREG + PREG pipeline ????????
                S_DRAIN: begin
                    if (drain_cnt != 2'd0) drain_cnt <= drain_cnt - 2'd1;
                    else                   state      <= S_OUTPUT;
                end

                // ?? S_OUTPUT: drive m_axis with final INT8 channel outputs ?????
                S_OUTPUT: begin
                    for (i = 0; i < OC; i = i + 1)
                        m_axis_tdata[i*8 +: 8] <= oc_out[i];
                    m_axis_tvalid <= 1'b1;
                    m_axis_tlast  <= tlast_delayed;

                    if (m_axis_tready) begin
                        if (col_ptr == WIDTH-1) begin
                            col_ptr <= {$clog2(WIDTH){1'b0}};
                            if (row_cnt == HEIGHT-1) begin
                                done    <= 1'b1;
                                state   <= S_IDLE;
                            end else begin
                                row_cnt       <= row_cnt + 1;
                                state         <= S_IDLE;
                                s_axis_tready <= 1'b1;
                            end
                        end else begin
                            col_ptr       <= col_ptr + 1;
                            state         <= S_IDLE;
                            s_axis_tready <= 1'b1;
                        end
                    end
                end

                default: state <= S_IDLE;

            endcase
        end
    end

endmodule
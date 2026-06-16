// =============================================================================
// mac_array_16x1.v - MAC Array for Finisher (Layer 16): 16 IC x 1 OC
// =============================================================================
// TIMING FIX (v2):
//   Root cause: the combinational adder tree from 16 DSP P outputs to the
//   output wire 'accum' contained 22 CARRY4 + LUT levels in one cycle,
//   producing an 18.2 ns data path at 100 MHz (budget = 10 ns). WNS = -8.458 ns.
//
//   Fix: split the 4-level binary reduction tree into two registered pipeline
//   stages inserted between the DSP outputs and the saturation output:
//
//     Stage 1 (reg): 16 -> 8 pairwise sums   (24-bit, registered)
//     Stage 2 (reg): 8  -> 1 final sum        (27-bit, registered, saturated)
//
//   This adds 2 cycles of latency to the 'accum' output relative to the
//   last mac_en pulse.  The finisher FSM drain_cnt is increased from 2 to 4
//   to compensate (3 DSP pipeline cycles + 2 adder pipeline cycles = 5 total;
//   drain_cnt=4 counts 4->3->2->1->0->exit = 5 cycles).
//
//   partial[] widened from [21:0] to [22:0] to match mac_unit's corrected
//   23-bit accum_out (mac_unit.v was already fixed to output p_out[22:0]).
//
//   Output port 'accum' remains [21:0] saturated - no change to finisher
//   port connections.
//
// Resource: 16 DSP48E1 (unchanged).
// =============================================================================

module mac_array_16x1 #(
    parameter IC = 16
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        clear,
    input  wire        en,
    input  wire signed [IC*8-1:0] weights,
    input  wire signed [IC*8-1:0] acts,
    output wire signed [21:0]     accum   // saturated 22-bit, 2 cycles after last DSP output
);

    // -------------------------------------------------------------------------
    // 16 MAC units - one per IC.  Each accumulates over 9 kernel cycles.
    // partial[] is 23-bit to match mac_unit's corrected accum_out width.
    // -------------------------------------------------------------------------
    wire signed [22:0] partial [0:IC-1];

    genvar ic;
    generate
        for (ic = 0; ic < IC; ic = ic + 1) begin : gen_mac
            mac_unit u_mac (
                .clk      (clk),
                .rst_n    (rst_n),
                .clear    (clear),
                .en       (en),
                .weight   (weights[ic*8 +: 8]),
                .act_in   (acts[ic*8 +: 8]),
                .accum_out(partial[ic])
            );
        end
    endgenerate

    // -------------------------------------------------------------------------
    // Pipeline Stage 1: 16 -> 8 pairwise sums, registered.
    //
    // Each partial is 23-bit signed (max 145,161 per DSP after 9 kernel cycles).
    // Pair sum: 2 x 145,161 = 290,322, fits in 24-bit signed (max 8,388,607).
    // -------------------------------------------------------------------------
    reg signed [23:0] ps [0:7];   // 8 pairwise sums, 24-bit

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ps[0] <= 24'sd0; ps[1] <= 24'sd0; ps[2] <= 24'sd0; ps[3] <= 24'sd0;
            ps[4] <= 24'sd0; ps[5] <= 24'sd0; ps[6] <= 24'sd0; ps[7] <= 24'sd0;
        end else begin
            ps[0] <= $signed(partial[0])  + $signed(partial[1]);
            ps[1] <= $signed(partial[2])  + $signed(partial[3]);
            ps[2] <= $signed(partial[4])  + $signed(partial[5]);
            ps[3] <= $signed(partial[6])  + $signed(partial[7]);
            ps[4] <= $signed(partial[8])  + $signed(partial[9]);
            ps[5] <= $signed(partial[10]) + $signed(partial[11]);
            ps[6] <= $signed(partial[12]) + $signed(partial[13]);
            ps[7] <= $signed(partial[14]) + $signed(partial[15]);
        end
    end

    // -------------------------------------------------------------------------
    // Pipeline Stage 2: 8 -> 1 final sum, registered then saturated.
    //
    // Max of 8 x 290,322 = 2,322,576.  Needs at least 22-bit signed
    // (2^21 = 2,097,152 - just under, so use 27-bit intermediate to be safe).
    //
    // The three-level reduction (4+2+1) is all in combinational logic between
    // the ps[] registers and the total_r register, which is a single LUT level
    // per stage - well within one cycle budget.
    // -------------------------------------------------------------------------
    wire signed [24:0] q0  = $signed(ps[0]) + $signed(ps[1]);
    wire signed [24:0] q1  = $signed(ps[2]) + $signed(ps[3]);
    wire signed [24:0] q2  = $signed(ps[4]) + $signed(ps[5]);
    wire signed [24:0] q3  = $signed(ps[6]) + $signed(ps[7]);
    wire signed [25:0] r0  = $signed(q0) + $signed(q1);
    wire signed [25:0] r1  = $signed(q2) + $signed(q3);
    wire signed [26:0] total_comb = $signed(r0) + $signed(r1);

    reg signed [26:0] total_r;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            total_r <= 27'sd0;
        else
            total_r <= total_comb;
    end

    // -------------------------------------------------------------------------
    // Saturate to 22-bit signed output (matches downstream finisher expectation)
    // -------------------------------------------------------------------------
    assign accum =
        (total_r > 27'sd2097151)  ?  22'sd2097151  :
        (total_r < -27'sd2097152) ? -22'sd2097152  :
        total_r[21:0];

endmodule
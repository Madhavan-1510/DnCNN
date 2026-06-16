// =============================================================================
// mac_array_16x16.v  (FIXED)
// FIX: partial[oc][ip] widened from [21:0] to [22:0] (23-bit) to match
// mac_unit's corrected 23-bit output. Adder tree intermediate widths updated.
// sat thresholds corrected to guard against 25-bit sum_tree overflow.
// =============================================================================
module mac_array_16x16 #(
    parameter OC      = 16,
    parameter IC_PAR  = 8
) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        clear,
    input  wire        en,
    input  wire signed [OC*IC_PAR*8-1:0] weights,   // 128 bytes
    input  wire signed [IC_PAR*8-1:0]    acts,       // 8 bytes
    output wire signed [OC*22-1:0]       accum       // 16 x 22-bit
);
    wire signed [22:0] partial [0:OC-1][0:IC_PAR-1];  // FIX: was [21:0]

    genvar oc, ip;
    generate
        for (oc = 0; oc < OC; oc = oc + 1) begin : gen_oc
            for (ip = 0; ip < IC_PAR; ip = ip + 1) begin : gen_ip
                mac_unit u_mac (
                    .clk      (clk),
                    .rst_n    (rst_n),
                    .clear    (clear),
                    .en       (en),
                    .weight   (weights[(oc*IC_PAR + ip)*8 +: 8]),
                    .act_in   (acts[ip*8 +: 8]),
                    .accum_out(partial[oc][ip])
                );
            end
        end
    endgenerate

    // -------------------------------------------------------------------------
    // Sum tree: reduce IC_PAR=8 partial accumulators per OC.
    //
    // Each partial[oc][ip] holds sum across 9 kernel cycles (one IC pass) plus
    // accumulation across both passes:
    //   Pass 0: 9 x 127 x 127 = 145,161 per partial
    //   8 partials x 2 passes = worst case sum per OC = 16 x 145,161 = 2,322,576
    //
    // BUT: each DSP mac_unit accumulates across BOTH passes (clear=0 on pass 1).
    // So after 18 MAC cycles the partials already contain the full cross-pass sum.
    // 8 partials summed = 8 x 2 x 145,161 = 2,322,576.
    // Fits in 22-bit signed (max ±4,194,303). Tree uses 25-bit intermediate to be safe.
    // -------------------------------------------------------------------------
    wire signed [24:0] sum_tree [0:OC-1];

    generate
        for (oc = 0; oc < OC; oc = oc + 1) begin : gen_sum
            wire signed [23:0] s0 = $signed(partial[oc][0]) + $signed(partial[oc][1]);
            wire signed [23:0] s1 = $signed(partial[oc][2]) + $signed(partial[oc][3]);
            wire signed [23:0] s2 = $signed(partial[oc][4]) + $signed(partial[oc][5]);
            wire signed [23:0] s3 = $signed(partial[oc][6]) + $signed(partial[oc][7]);
            wire signed [24:0] s4 = $signed(s0) + $signed(s1);
            wire signed [24:0] s5 = $signed(s2) + $signed(s3);
            assign sum_tree[oc]   = $signed(s4) + $signed(s5);

            // Saturate: sum_tree is 25-bit signed, output is 22-bit signed
            assign accum[oc*22 +: 22] =
                (sum_tree[oc] > 25'sd2097151)  ?  22'sd2097151  :
                (sum_tree[oc] < -25'sd2097152) ? -22'sd2097152  :
                sum_tree[oc][21:0];
        end
    endgenerate

endmodule
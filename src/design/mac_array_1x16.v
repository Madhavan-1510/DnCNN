// =============================================================================
// mac_array_1x16.v  (FIXED)
// FIX: accum output bus is [OC*23-1:0] and slices are 23-bit, matching
// mac_unit's corrected 23-bit accum_out port. The previous mismatch
// generated 16x "width (23) does not match port width (22)" warnings.
// No logic change; only port-width alignment.
// =============================================================================
module mac_array_1x16 #(
    parameter OC = 16
) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        clear,
    input  wire        en,
    input  wire signed [OC*8-1:0]  weights,
    input  wire signed [7:0]       act_in,
    output wire signed [OC*23-1:0] accum    // 23-bit per OC, matches mac_unit
);
    genvar oc;
    generate
        for (oc = 0; oc < OC; oc = oc + 1) begin : gen_mac
            mac_unit u_mac (
                .clk      (clk),
                .rst_n    (rst_n),
                .clear    (clear),
                .en       (en),
                .weight   (weights[oc*8 +: 8]),
                .act_in   (act_in),
                .accum_out(accum[oc*23 +: 23])
            );
        end
    endgenerate

endmodule
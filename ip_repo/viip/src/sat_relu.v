// =============================================================================
// sat_relu.v - Saturation + Optional ReLU (Combinational)
// =============================================================================
// WHAT THIS DOES:
//   Takes the 22-bit signed accumulator output from the MAC array and squashes
//   it into a signed 8-bit INT8 pixel.
//
//   ENABLE_RELU=1 (Ingestor, Workhorse):
//     - Negative  ? clamp to 0        (ReLU)
//     - > +127    ? clamp to +127     (positive saturation)
//     - else      ? passthrough [7:0]
//
//   ENABLE_RELU=0 (Finisher - output can be legitimately negative before
//     residual subtraction):
//     - < -128    ? clamp to -128
//     - > +127    ? clamp to +127
//     - else      ? passthrough [7:0]
//
// SYNTHESIS NOTE:
//   Pure combinational; no registers. Vivado infers LUT logic only (~6 LUTs).
//   Used 16? in parallel (one per output channel).
// =============================================================================

module sat_relu #(
    parameter ENABLE_RELU = 1   // 1 = ReLU+sat  (layers 0..15)
                                // 0 = sat only   (layer 16 / finisher)
)(
    input  wire signed [21:0] accum_in,
    output reg  signed [7:0]  pixel_out
);

    always @(*) begin
        if (ENABLE_RELU && accum_in[21]) begin
            // Sign bit set ? negative after BN fold ? ReLU ? zero
            pixel_out = 8'sd0;
        end else if (accum_in > 22'sd127) begin
            // Overflow positive ? saturate at INT8 max
            pixel_out = 8'sd127;
        end else if (!ENABLE_RELU && (accum_in < -22'sd128)) begin
            // Finisher only: underflow negative ? saturate at INT8 min
            pixel_out = -8'sd128;
        end else begin
            // Normal range [-128..+127]: direct truncation is safe
            pixel_out = accum_in[7:0];
        end
    end

endmodule
// =============================================================================
// residual_sub.v - Residual Subtraction (DnCNN final output stage)
// =============================================================================
// WHAT THIS DOES:
//   DnCNN predicts NOISE, not the clean image. This module performs:
//
//       clean = clamp(raw_pixel - noise_pred, 0, 255)
//
//   raw_pixel  : original noisy input  [7:0] unsigned (0..255 from camera/VDMA)
//   noise_pred : INT8 signed [-128..127] - the predicted noise from the finisher
//                conv layer. Positive noise_pred means the pixel was pushed UP
//                by noise, so we subtract it back down.
//
//   The subtraction is done in 9-bit signed space to catch over/underflow
//   correctly before clamping back to uint8.
//
//   Output is UNSIGNED [7:0] for display (HDMI RGB).
//
// SYNTHESIS NOTE:
//   Combinational. One 9-bit subtractor + two comparators ? ~8 LUTs.
// =============================================================================

module residual_sub (
    input  wire [7:0]        raw_pixel,
    input  wire signed [7:0] noise_pred,
    output reg  [7:0]        clean_out
);
    wire signed [9:0] diff;

    // {1'b0, raw_pixel} zero-extends raw to 9-bit unsigned, then to 10-bit signed
    // $signed(noise_pred) sign-extends INT8 ? 10-bit signed correctly
    assign diff = $signed({1'b0, raw_pixel}) - $signed(noise_pred);

    always @(*) begin
        if (diff < $signed(10'sd0))
            clean_out = 8'd0;
        else if (diff > $signed(10'sd255))
            clean_out = 8'd255;
        else
            clean_out = diff[7:0];
    end
endmodule
// =============================================================================
// mac_array_1x16_behav.v  -- SIMULATION ONLY, DO NOT ADD TO sources_1
// =============================================================================
// PURPOSE:
//   mac_array_1x16.v instantiates mac_unit.v which instantiates DSP48E1.
//   The Xilinx unisim DSP48E1 model prints "OPMODE Input ERROR" when en=0
//   and drives P = X, corrupting all downstream outputs in simulation.
//   This behavioural substitute avoids the DSP48E1 primitive entirely so
//   the ingestor testbench can verify convolution correctness.
//
// HOW TO USE IN VIVADO:
//   1. Add THIS file to sim_1 fileset only (not sources_1).
//   2. In sources_1: right-click mac_array_1x16.v -> Properties ->
//      Used In -> UNCHECK "Simulation".
//   Vivado resolves the duplicate module name: this file wins in sim,
//   the DSP48E1 version wins in synthesis.
//
// PORT CONTRACT (must match mac_array_1x16.v exactly):
//   accum port width = OC * 22 bits.
//   dncnn_ingestor reads: mac_accum[oc*22 +: 22] for each OC.
//
// ARITHMETIC:
//   1-cycle registered latency (not 3-cycle DSP pipeline).
//   This does NOT affect correctness: the ingestor FSM has a 3-cycle S_DRAIN
//   state that waits for the pipeline to flush.  With 1-cycle behavioural MAC
//   the drain cycles are wasted but the final value is still correct.
//
//   Sign extension: weight is 8-bit signed, extended to 16-bit for multiply.
//   act_in is 8-bit signed, extended to 16-bit.
//   Product is 16-bit signed.  Accumulator is 22-bit signed.
//   Max accumulation: 9 * 127 * 127 = 145,161 << 2^21 = 2,097,152.  Safe.
// =============================================================================

module mac_array_1x16 #(
    parameter OC = 16
)(
    input  wire                    clk,
    input  wire                    rst_n,
    input  wire                    clear,
    input  wire                    en,
    input  wire [OC*8-1:0]         weights,   // OC x INT8, packed
    input  wire signed [7:0]       act_in,    // single activation tap
    output reg  signed [OC*22-1:0] accum      // OC x 22-bit accumulators
);

    integer oc_i;
    // Intermediate: product per OC (16-bit signed is enough: 127*127=16129)
    reg signed [15:0] product [0:OC-1];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            accum <= {(OC*22){1'b0}};
        end else if (en) begin
            for (oc_i = 0; oc_i < OC; oc_i = oc_i + 1) begin
                // Sign-extend 8-bit weight to 16-bit, multiply by 8-bit act_in
                product[oc_i] = $signed(weights[oc_i*8 +: 8]) * $signed(act_in);

                if (clear)
                    // S_PREFETCH: reset accumulator, load first product
                    accum[oc_i*22 +: 22] <= {{6{product[oc_i][15]}}, product[oc_i]};
                else
                    // S_COMPUTE k=1..8: accumulate
                    accum[oc_i*22 +: 22] <= accum[oc_i*22 +: 22] +
                                            {{6{product[oc_i][15]}}, product[oc_i]};
            end
        end
        // en=0: hold value (S_DRAIN wait cycles - no action needed)
    end

endmodule
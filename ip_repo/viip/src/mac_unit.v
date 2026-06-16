// =============================================================================
// mac_unit.v  (FIXED - DSP48E1 unconnected port warnings resolved)
//
// FIX 1: accum_out widened from [21:0] to [22:0] (23-bit).
//
// WHY: worst-case accumulation across 9 kernel positions:
//   9 x 127 x 127 = 145,161  (single mac_unit, 1 IC)
//   mac_array_16x1 sums 16 of these: 16 x 145,161 = 2,322,576
//   2,322,576 > 2^21 (2,097,151) but < 2^22 (4,194,303)
//   => 22-bit signed overflows; 23-bit signed is safe.
//
// mac_array_1x16.v was already slicing [oc*23 +: 23] (correct),
// but mac_unit declared [21:0] - Vivado silently zero-padded and
// emitted "width (23) does not match port width (22)" 16x per instance.
// Now both sides match.
//
// mac_array_16x16 uses its own adder tree that reduces 8 partials per OC;
// each partial comes from this mac_unit (23-bit). The tree widens to 25-bit
// before saturating back to 22-bit output - no change needed there.
//
// FIX 2: Synth 8-7071 - Explicitly tie all 7 unconnected DSP48E1 input ports:
//
//   Port           Tied to     Reason
//   -------------- ----------- -----------------------------------------------
//   CARRYCASCIN    1'b0        Cascade carry not used; tie low (safe default)
//   CARRYINSEL     3'b000      Select CARRYIN pin (not cascade); harmless tie
//   CEC            1'b1        Clock-enable for C register; C=48'd0 constant,
//                              so always-enabled is harmless and avoids glitch
//   CEINMODE       1'b1        Clock-enable for INMODE register; INMODE=5'b00000
//                              is fixed, always-enabled is harmless
//   MULTSIGNIN     1'b0        Cascade multiplier sign not used; tie low
//   RSTC           1'b0        Active-high reset for C register; tie low
//                              (do NOT assert; C=0 is the right constant)
//   RSTINMODE      1'b0        Active-high reset for INMODE register; tie low
//                              (INMODE driven combinatorially, no reset needed)
// =============================================================================

(* use_dsp = "yes" *)
module mac_unit (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        clear,
    input  wire        en,
    input  wire signed [7:0]  weight,
    input  wire signed [7:0]  act_in,
    output wire signed [22:0] accum_out   // FIX 1: was [21:0]
);
    wire [47:0] p_out;

    // OPMODE:
    //   en=0          : 7'b000_00_00  no-op (CEP=0 anyway, prevents unisim warning)
    //   en=1, clear=1 : 7'b000_01_01  P = 0 + M  (load first product, reset accum)
    //   en=1, clear=0 : 7'b010_01_01  P = P + M  (accumulate)
    wire [6:0] opmode = !en    ? 7'b000_00_00 :
                         clear ? 7'b000_01_01 :
                                 7'b010_01_01;

    DSP48E1 #(
        .AREG             (1),
        .BREG             (1),
        .ADREG            (1),
        .DREG             (1),
        .MREG             (1),
        .PREG             (1),
        .ACASCREG         (1),
        .BCASCREG         (1),
        .A_INPUT          ("DIRECT"),
        .B_INPUT          ("DIRECT"),
        .USE_MULT         ("MULTIPLY"),
        .USE_SIMD         ("ONE48"),
        .USE_DPORT        ("FALSE"),
        .MASK             (48'h3FFFFFFFFFFF),
        .PATTERN          (48'h000000000000),
        .SEL_MASK         ("MASK"),
        .SEL_PATTERN      ("PATTERN"),
        .AUTORESET_PATDET ("NO_RESET")
    ) dsp_inst (
        // ----------------------------------------------------------------
        // Clock
        // ----------------------------------------------------------------
        .CLK          (clk),

        // ----------------------------------------------------------------
        // Data inputs
        // ----------------------------------------------------------------
        .A            ({{22{act_in[7]}}, act_in}),   // 30-bit sign-extended activation
        .B            ({{10{weight[7]}}, weight}),   // 18-bit sign-extended weight
        .C            (48'd0),
        .D            (25'd0),

        // ----------------------------------------------------------------
        // Data output
        // ----------------------------------------------------------------
        .P            (p_out),

        // ----------------------------------------------------------------
        // Control inputs
        // ----------------------------------------------------------------
        .OPMODE       (opmode),
        .ALUMODE      (4'b0000),
        .INMODE       (5'b00000),
        .CARRYIN      (1'b0),

        // FIX 2a: CARRYINSEL - select CARRYIN pin (3'b000), cascade carry unused
        .CARRYINSEL   (3'b000),

        // FIX 2b: CARRYCASCIN - carry cascade input, not used; tie low
        .CARRYCASCIN  (1'b0),

        // FIX 2c: MULTSIGNIN - multiplier sign cascade input, not used; tie low
        .MULTSIGNIN   (1'b0),

        // ----------------------------------------------------------------
        // Clock enables
        // ----------------------------------------------------------------
        .CEA1         (1'b0),
        .CEA2         (en),
        .CEB1         (1'b0),
        .CEB2         (en),
        .CED          (1'b0),
        .CEAD         (1'b0),
        .CEM          (en),
        .CEP          (en),
        .CECTRL       (en),
        .CECARRYIN    (1'b0),
        .CEALUMODE    (1'b0),

        // FIX 2d: CEC - clock-enable for C register; C=48'd0 constant, always-enable is safe
        .CEC          (1'b1),

        // FIX 2e: CEINMODE - clock-enable for INMODE register; INMODE fixed at 0, always-enable is safe
        .CEINMODE     (1'b1),

        // ----------------------------------------------------------------
        // Resets (active-high)
        // ----------------------------------------------------------------
        .RSTA         (!rst_n),
        .RSTB         (!rst_n),
        .RSTD         (1'b0),
        .RSTM         (!rst_n),
        .RSTP         (!rst_n),
        .RSTCTRL      (!rst_n),
        .RSTALLCARRYIN(1'b0),
        .RSTALUMODE   (1'b0),

        // FIX 2f: RSTC - active-high reset for C register; tie low (C=0 is the desired constant)
        .RSTC         (1'b0),

        // FIX 2g: RSTINMODE - active-high reset for INMODE register; tie low (INMODE driven combinatorially)
        .RSTINMODE    (1'b0),

        // ----------------------------------------------------------------
        // Cascade inputs (unused - tied to safe defaults)
        // ----------------------------------------------------------------
        .ACIN         (30'd0),
        .BCIN         (18'd0),
        .PCIN         (48'd0),

        // ----------------------------------------------------------------
        // Cascade / status outputs (undriven - left open)
        // ----------------------------------------------------------------
        .ACOUT        (),
        .BCOUT        (),
        .PCOUT        (),
        .OVERFLOW     (),
        .UNDERFLOW    (),
        .PATTERNDETECT(),
        .PATTERNBDETECT(),
        .CARRYCASCOUT (),
        .MULTSIGNOUT  (),
        .CARRYOUT     ()
    );

    // 23-bit signed output: p_out[22:0]
    // Adder trees in mac_array_16x1 / mac_array_16x16 widen further before saturating.
    assign accum_out = $signed(p_out[22:0]);

endmodule
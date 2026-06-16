`timescale 1ns/1ps

// =============================================================================
// mac_unit_behav: mirrors DSP48E1 pipeline exactly
// Pipeline: A/B regs (stage1) ? M reg (stage2) ? P reg (stage3)
// CRITICAL: all 3 stages clock independently; drain needs en=1 to flush
// =============================================================================
module mac_unit_behav (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        clear,
    input  wire        en,
    input  wire signed [7:0]  weight,
    input  wire signed [7:0]  act_in,
    output wire signed [22:0] accum_out
);
    // Stage 1: A/B input registers (clocked with en)
    logic signed [7:0]  a_reg, b_reg;
    logic               clear_s1;

    // Stage 2: M register = product (clocked with en)
    logic signed [15:0] m_reg;
    logic               clear_s2;

    // Stage 3: P register = accumulator (clocked with en)
    logic signed [21:0] p_reg;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            a_reg    <= '0;
            b_reg    <= '0;
            clear_s1 <= '0;
            m_reg    <= '0;
            clear_s2 <= '0;
            p_reg    <= '0;
        end else if (en) begin
            // Stage 1: latch inputs
            a_reg    <= act_in;
            b_reg    <= weight;
            clear_s1 <= clear;

            // Stage 2: multiply
            m_reg    <= a_reg * b_reg;
            clear_s2 <= clear_s1;

            // Stage 3: accumulate or load
            p_reg    <= clear_s2 ? m_reg : p_reg + m_reg;
        end
    end

    assign accum_out = p_reg;
endmodule


// =============================================================================
// Testbench
// =============================================================================
module tb_mac_unit;

    logic        clk = 1'b0;
    logic        rst_n;
    logic        clear;
    logic        en;
    logic signed [7:0]  weight;
    logic signed [7:0]  act_in;
    logic signed [21:0] accum_out;

    mac_unit_behav dut (.*);

    always #5 clk = ~clk;  // 100 MHz

    int pass_count = 0;
    int fail_count = 0;

    // DSP48E1 latency: A/B reg ? M reg ? P reg = 3 stages
    // drain() must keep en=1 for LAT cycles to flush the pipeline
    localparam int LAT = 3;

    // ------------------------------------------------------------------
    // drive_in: apply inputs for ONE cycle (#1 after posedge avoids races)
    // ------------------------------------------------------------------
    task automatic drive_in(
        input logic signed [7:0] w,
        input logic signed [7:0] a,
        input logic              clr
    );
        @(posedge clk); #1;
        weight = w;
        act_in = a;
        clear  = clr;
        en     = 1'b1;
    endtask

    // ------------------------------------------------------------------
    // drain: inject LAT bubble cycles with en=1, weight/act=0, clear=0
    // This FLUSHES the pipeline - en must stay 1 so stages keep clocking
    // ------------------------------------------------------------------
    task automatic drain();
        repeat (LAT) begin
            @(posedge clk); #1;
            weight = 8'sd0;
            act_in = 8'sd0;
            clear  = 1'b0;
            en     = 1'b1;   // KEEP en=1 to clock pipeline stages forward
        end
        @(posedge clk); #1;
        en = 1'b0;           // now safe to deassert
    endtask

    // ------------------------------------------------------------------
    // check: sample on negedge after drain completes
    // ------------------------------------------------------------------
    task automatic check(
        input logic signed [21:0] exp,
        input string              tag
    );
        @(negedge clk);
        if (accum_out !== exp) begin
            $display("FAIL %-35s got=%0d  exp=%0d",
                     tag, $signed(accum_out), $signed(exp));
            fail_count++;
        end else begin
            $display("PASS %-35s accum=%0d", tag, $signed(accum_out));
            pass_count++;
        end
    endtask

    // ------------------------------------------------------------------
    // Stimulus
    // ------------------------------------------------------------------
    initial begin
        $display("=== tb_mac_unit START ===");

        rst_n  = 1'b0;
        clear  = 1'b0;
        en     = 1'b0;
        weight = '0;
        act_in = '0;
        repeat(2) @(posedge clk); #1;
        rst_n = 1'b1;
        @(posedge clk);

        // ----------------------------------------------------------------
        // T1: single product  3×4 = 12
        // Pipeline sees: [3×4, clear=1] ? [0×0] ? [0×0] ? result
        // ----------------------------------------------------------------
        drive_in(8'sd3, 8'sd4, 1'b1);
        drain();
        check(22'sd12, "T1: 3x4=12");

        // ----------------------------------------------------------------
        // T2: 1×1 + 2×2 + 3×3 = 14
        // ----------------------------------------------------------------
        drive_in(8'sd1, 8'sd1, 1'b1);
        @(posedge clk); #1; weight=8'sd2; act_in=8'sd2; clear=0; en=1;
        @(posedge clk); #1; weight=8'sd3; act_in=8'sd3; clear=0; en=1;
        drain();
        check(22'sd14, "T2: 1+4+9=14");

        // ----------------------------------------------------------------
        // T3: (-5)×(-4) = 20
        // ----------------------------------------------------------------
        drive_in(-8'sd5, -8'sd4, 1'b1);
        drain();
        check(22'sd20, "T3: -5x-4=20");

        // ----------------------------------------------------------------
        // T4: 10×(-3) = -30
        // ----------------------------------------------------------------
        drive_in(8'sd10, -8'sd3, 1'b1);
        drain();
        check(-22'sd30, "T4: 10x-3=-30");

        // ----------------------------------------------------------------
        // T5: 9×(127×127) = 145161 - fits in 22-bit signed (max=2,097,151)
        // ----------------------------------------------------------------
        drive_in(8'sd127, 8'sd127, 1'b1);
        for (int i = 1; i < 9; i++) begin
            @(posedge clk); #1;
            weight=8'sd127; act_in=8'sd127; clear=0; en=1;
        end
        drain();
        check(22'sd145161, "T5: 9x127x127=145161");

        // ----------------------------------------------------------------
        // T6: 144 products - will overflow 22-bit, just check no X/Z
        // ----------------------------------------------------------------
        drive_in(8'sd127, 8'sd127, 1'b1);
        for (int i = 1; i < 144; i++) begin
            @(posedge clk); #1;
            weight=8'sd127; act_in=8'sd127; clear=0; en=1;
        end
        drain();
        if (accum_out === 22'bx || accum_out === 22'bz) begin
            $display("FAIL T6: X/Z on accum_out after 144 products");
            fail_count++;
        end else begin
            $display("PASS T6: 144-product sim stable  accum=%0d (overflow expected)",
                     $signed(accum_out));
            pass_count++;
        end

        $display("=== tb_mac_unit: %0d PASS, %0d FAIL ===", pass_count, fail_count);
        $display(fail_count == 0 ? "ALL TESTS PASSED" : "*** FAILURES DETECTED ***");
        $finish;
    end

endmodule
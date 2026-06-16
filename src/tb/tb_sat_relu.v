// =============================================================================
// tb_sat_relu.sv - Testbench for sat_relu.v
// =============================================================================
// Verifies:
//   RELU=1 mode: negative?0, overflow positive?127, normal passthrough
//   RELU=0 mode: underflow negative?-128, overflow positive?127, normal pass
// No clock needed - sat_relu is combinational.
// =============================================================================

`timescale 1ns/1ps

module tb_sat_relu;

    // ---- DUT signals ----
    logic signed [21:0] accum_in;
    logic signed [7:0]  out_relu1;   // ENABLE_RELU=1
    logic signed [7:0]  out_relu0;   // ENABLE_RELU=0

    // ---- DUT instances ----
    sat_relu #(.ENABLE_RELU(1)) dut_relu (
        .accum_in  (accum_in),
        .pixel_out (out_relu1)
    );

    sat_relu #(.ENABLE_RELU(0)) dut_nrelu (
        .accum_in  (accum_in),
        .pixel_out (out_relu0)
    );

    // ---- Test task ----
    int pass_count = 0;
    int fail_count = 0;

    task check(
        input logic signed [21:0] in,
        input logic signed [7:0]  expected_relu,
        input logic signed [7:0]  expected_nrelu,
        input string desc
    );
        accum_in = in;
        #1;
        if (out_relu1 !== expected_relu) begin
            $display("FAIL [RELU=1] %s: in=%0d got=%0d exp=%0d",
                     desc, $signed(in), $signed(out_relu1), $signed(expected_relu));
            fail_count++;
        end else begin
            $display("PASS [RELU=1] %s: in=%0d ? %0d", desc, $signed(in), $signed(out_relu1));
            pass_count++;
        end

        if (out_relu0 !== expected_nrelu) begin
            $display("FAIL [RELU=0] %s: in=%0d got=%0d exp=%0d",
                     desc, $signed(in), $signed(out_relu0), $signed(expected_nrelu));
            fail_count++;
        end else begin
            $display("PASS [RELU=0] %s: in=%0d ? %0d", desc, $signed(in), $signed(out_relu0));
            pass_count++;
        end
    endtask

    initial begin
        $display("=== tb_sat_relu: START ===");

        // ----- Normal range -----
        check(22'sd0,    8'sd0,    8'sd0,   "zero");
        check(22'sd1,    8'sd1,    8'sd1,   "one");
        check(22'sd127,  8'sd127,  8'sd127, "max_int8");
        check(-22'sd1,   8'sd0,   -8'sd1,   "minus_one");
        check(-22'sd128, 8'sd0,  -8'sd128,  "min_int8");

        // ----- Positive overflow ? clamp +127 -----
        check(22'sd128,  8'sd127,  8'sd127, "128_overflow");
        check(22'sd255,  8'sd127,  8'sd127, "255_overflow");
        check(22'sd2097151, 8'sd127, 8'sd127, "max_22bit");  // 2^21 - 1

        // ----- Negative: ReLU clamps to 0, no-ReLU clamps to -128 -----
        check(-22'sd129, 8'sd0,  -8'sd128, "neg_129");
        check(-22'sd2097152, 8'sd0, -8'sd128, "min_22bit");  // -2^21

        // ----- Boundary: exactly at 127 and -128 -----
        check(22'sd127,  8'sd127,  8'sd127, "exact_pos_max");
        check(-22'sd128, 8'sd0,   -8'sd128, "exact_neg_min");

        $display("=== tb_sat_relu: %0d PASS, %0d FAIL ===", pass_count, fail_count);
        if (fail_count == 0)
            $display("ALL TESTS PASSED");
        else
            $display("*** FAILURES DETECTED ***");

        $finish;
    end

endmodule
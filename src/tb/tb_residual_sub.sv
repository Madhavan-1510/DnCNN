// =============================================================================
// tb_residual_sub.sv - Testbench for residual_sub.v
// =============================================================================
// Verifies: clean = clamp(raw - noise, 0, 255) for edge and normal cases
// =============================================================================

`timescale 1ns/1ps

module tb_residual_sub;

    logic [7:0]       raw_pixel;
    logic signed [7:0] noise_pred;
    logic [7:0]       clean_out;

    residual_sub dut (
        .raw_pixel  (raw_pixel),
        .noise_pred (noise_pred),
        .clean_out  (clean_out)
    );

    int pass_count = 0;
    int fail_count = 0;

    task check(
        input logic [7:0]        raw,
        input logic signed [7:0] noise,
        input logic [7:0]        expected,
        input string             desc
    );
        raw_pixel  = raw;
        noise_pred = noise;
        #1;
        if (clean_out !== expected) begin
            $display("FAIL %s: raw=%0d noise=%0d got=%0d exp=%0d",
                desc, raw, $signed(noise), clean_out, expected);
            fail_count++;
        end else begin
            $display("PASS %s: raw=%0d - noise=%0d = %0d",
                desc, raw, $signed(noise), clean_out);
            pass_count++;
        end
    endtask

    initial begin
        $display("=== tb_residual_sub: START ===");

        // Normal subtraction
        check(8'd100,  8'sd20,   8'd80,  "normal: 100-20=80");
        check(8'd200,  8'sd100,  8'd100, "normal: 200-100=100");
        check(8'd128,  8'sd0,    8'd128, "zero noise: passthrough");

        // Negative noise (noise_pred < 0 means pixel was pushed DOWN by noise)
        // clean = raw - (-50) = raw + 50
        check(8'd100, -8'sd50,   8'd150, "neg noise: 100+50=150");
        check(8'd200, -8'sd55,   8'd255, "neg noise saturate high: 200+55=255 clamp");
        check(8'd10,  -8'sd128,  8'd138, "neg noise max: 10+128=138");

        // Underflow: result < 0 ? clamp to 0
        check(8'd10,   8'sd50,   8'd0,   "underflow: 10-50=-40 ? 0");
        check(8'd0,    8'sd1,    8'd0,   "underflow: 0-1=-1 ? 0");
        check(8'd0,    8'sd127,  8'd0,   "underflow: 0-127 ? 0");

        // Overflow: result > 255 ? clamp to 255
        check(8'd255, -8'sd1,    8'd255, "overflow: 255+1=256 ? 255");
        check(8'd255, -8'sd127,  8'd255, "overflow: 255+127 ? 255");

        // Boundaries
        check(8'd255,  8'sd0,    8'd255, "max raw, zero noise");
        check(8'd0,    8'sd0,    8'd0,   "zero raw, zero noise");
        check(8'd128,  8'sd128,  8'd0,   "128-128=0");
        check(8'd127,  8'sd127,  8'd0,   "127-127=0");

        $display("=== tb_residual_sub: %0d PASS, %0d FAIL ===", pass_count, fail_count);
        if (fail_count == 0)
            $display("ALL TESTS PASSED");
        else
            $display("*** FAILURES DETECTED ***");

        $finish;
    end

endmodule
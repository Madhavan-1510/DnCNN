`timescale 1ns/1ps

module mac_unit_behav (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        clear,
    input  wire        en,
    input  wire signed [7:0]  weight,
    input  wire signed [7:0]  act_in,
    output wire signed [21:0] accum_out
);
    reg signed [15:0] pipe_m1;
    reg signed [15:0] pipe_m2;
    reg signed [21:0] pipe_p;
    reg               clear_m1, clear_m2;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pipe_m1  <= 0; pipe_m2 <= 0; pipe_p <= 0;
            clear_m1 <= 0; clear_m2 <= 0;
        end else if (en) begin
            pipe_m1  <= weight * act_in;
            clear_m1 <= clear;
            pipe_m2  <= pipe_m1;
            clear_m2 <= clear_m1;
            if (clear_m2)
                pipe_p <= pipe_m2;
            else
                pipe_p <= pipe_p + pipe_m2;
        end
    end

    assign accum_out = pipe_p;
endmodule


module tb_mac_unit;

    reg         clk;
    reg         rst_n;
    reg         clear;
    reg         en;
    reg  signed [7:0]  weight;
    reg  signed [7:0]  act_in;
    wire signed [21:0] accum_out;

    mac_unit_behav dut (
        .clk      (clk),
        .rst_n    (rst_n),
        .clear    (clear),
        .en       (en),
        .weight   (weight),
        .act_in   (act_in),
        .accum_out(accum_out)
    );

    initial clk = 0;
    always #5 clk = ~clk;

    integer pass_count;
    integer fail_count;
    integer i;

    localparam LAT = 3;

    // ----------------------------------------------------------------
    // Tasks
    // ----------------------------------------------------------------
    task apply_mac;
        input signed [7:0] w;
        input signed [7:0] a;
        input              clr;
        begin
            @(posedge clk); #1;
            weight = w;
            act_in = a;
            clear  = clr;
            en     = 1;
        end
    endtask

    task drain;
        begin
            @(posedge clk); #1;
            en    = 0;
            clear = 0;
            repeat (LAT) @(posedge clk);
        end
    endtask

    task check_accum;
        input signed [21:0] expected;
        input [127:0]       desc;     // fixed-width string for Verilog-2001
        begin
            @(negedge clk);
            if (accum_out !== expected) begin
                $display("FAIL %s: got=%0d exp=%0d",
                         desc, $signed(accum_out), $signed(expected));
                fail_count = fail_count + 1;
            end else begin
                $display("PASS %s: accum=%0d", desc, $signed(accum_out));
                pass_count = pass_count + 1;
            end
        end
    endtask

    // ----------------------------------------------------------------
    // Stimulus
    // ----------------------------------------------------------------
    initial begin
        $display("=== tb_mac_unit: START ===");
        pass_count = 0;
        fail_count = 0;
        rst_n  = 0;
        clear  = 0;
        en     = 0;
        weight = 0;
        act_in = 0;
        repeat (2) @(posedge clk);
        #1; rst_n = 1;
        @(posedge clk);

        // --- T1: single product ---
        apply_mac(8'sd3, 8'sd4, 1'b1);
        drain();
        check_accum(22'sd12, "T1: 3x4=12");

        // --- T2: accumulate 1+4+9=14 ---
        apply_mac(8'sd1, 8'sd1, 1'b1);
        @(posedge clk); #1; weight = 8'sd2; act_in = 8'sd2; clear = 0; en = 1;
        @(posedge clk); #1; weight = 8'sd3; act_in = 8'sd3; clear = 0; en = 1;
        drain();
        check_accum(22'sd14, "T2: 1+4+9=14");

        // --- T3: neg x neg ---
        apply_mac(-8'sd5, -8'sd4, 1'b1);
        drain();
        check_accum(22'sd20, "T3: -5x-4=20");

        // --- T4: pos x neg ---
        apply_mac(8'sd10, -8'sd3, 1'b1);
        drain();
        check_accum(-22'sd30, "T4: 10x-3=-30");

        // --- T5: 9 products ---
        apply_mac(8'sd127, 8'sd127, 1'b1);
        for (i = 1; i < 9; i = i + 1) begin
            @(posedge clk); #1;
            weight = 8'sd127; act_in = 8'sd127; clear = 0; en = 1;
        end
        drain();
        check_accum(22'sd145161, "T5: 9x127x127=145161");

        // --- T6: 144 products overflow ---
        apply_mac(8'sd127, 8'sd127, 1'b1);
        for (i = 1; i < 144; i = i + 1) begin
            @(posedge clk); #1;
            weight = 8'sd127; act_in = 8'sd127; clear = 0; en = 1;
        end
        drain();
        if (accum_out === 22'bx || accum_out === 22'bz) begin
            $display("FAIL T6: X/Z on accumulator");
            fail_count = fail_count + 1;
        end else begin
            $display("PASS T6: overflow stable accum=%0d", $signed(accum_out));
            pass_count = pass_count + 1;
        end

        $display("=== tb_mac_unit: %0d PASS, %0d FAIL ===",
                 pass_count, fail_count);
        if (fail_count == 0)
            $display("ALL TESTS PASSED");
        else
            $display("*** FAILURES DETECTED ***");
        $finish;
    end

endmodule
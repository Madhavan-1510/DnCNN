// =============================================================================
// dncnn_finisher.v  - DnCNN Layer 16: 16-channel -> 1-channel, Conv, no ReLU
// =============================================================================
// TIMING FIXES applied in this version (v3):
//
//  FIX 1 - drain_cnt widened [1:0] -> [2:0], init value 2 -> 4
//    mac_array_16x1 now has a 2-stage registered adder tree (pipeline Fix in
//    that module).  Total pipeline depth after last mac_en:
//      3 cycles  DSP48E1 (AREG + MREG + PREG)
//      2 cycles  registered adder tree (stage1 + stage2)
//      ------
//      5 cycles total drain needed
//    drain_cnt = N gives N+1 drain cycles (counts N->...->0 then exits).
//    drain_cnt = 4 => 5 drain cycles. Requires 3-bit counter.
//
//  FIX 2 - tlast_sr: add (* shreg_extract = "yes" *) attribute
//    The 640-bit tlast_sr shift register with a single lb_wr_en enable was
//    causing 407 large-setup-violation paths (TIMING-16) because one FF
//    fanned out to 640 CE pins across distant slices.  The shreg_extract
//    attribute forces Vivado to use SRL32 primitives which have a built-in
//    CE and no fan-out problem.
//
// Previously applied fixes (retained):
//  - Port A 1-cycle weight BRAM latency compensation (S_LOAD/S_PREFETCH)
//  - TLAST delay shift register (line buffer 1-row latency)
//  - S_OUTPUT does not advance counters without m_axis_tready
// =============================================================================

module dncnn_finisher #(
    parameter WIDTH  = 640,
    parameter HEIGHT = 480,
    parameter IC     = 16
)(
    input  wire        clk,
    input  wire        rst_n,

    // AXI4-Stream slave: feature maps (noise prediction input)
    input  wire [IC*8-1:0] s_axis_feat_tdata,
    input  wire             s_axis_feat_tvalid,
    output reg              s_axis_feat_tready,
    input  wire             s_axis_feat_tlast,

    // AXI4-Stream slave: original raw frame (for residual subtraction)
    input  wire [7:0]  s_axis_raw_tdata,
    input  wire        s_axis_raw_tvalid,
    output reg         s_axis_raw_tready,
    input  wire        s_axis_raw_tlast,

    // Weight BRAM port A (1-cycle registered read latency)
    output reg  [7:0]        weight_addr,
    input  wire [IC*8-1:0]   weight_data,

    // BN-folded bias (signed 16-bit, single OC)
    input  wire signed [15:0] bias_data,

    // AXI4-Stream master: clean pixel output
    output reg  [7:0]  m_axis_tdata,
    output reg         m_axis_tvalid,
    input  wire        m_axis_tready,
    output reg         m_axis_tlast,

    output reg  done,
    input  wire top_row_active
);

    // -------------------------------------------------------------------------
    // Counters
    // -------------------------------------------------------------------------
    reg [$clog2(WIDTH)-1:0]  col_ptr;
    reg [$clog2(HEIGHT)-1:0] row_cnt;
    reg [3:0] k;
    reg [7:0] raw_pixel_buf;

    // -------------------------------------------------------------------------
    // Line buffer (16 channels)
    // -------------------------------------------------------------------------
    wire [IC*8-1:0] win_r0c0, win_r0c1, win_r0c2;
    wire [IC*8-1:0] win_r1c0, win_r1c1, win_r1c2;
    wire [IC*8-1:0] win_r2c0, win_r2c1, win_r2c2;
    wire [IC*8-1:0] win_flat [0:8];
    assign win_flat[0]=win_r0c0; assign win_flat[1]=win_r0c1; assign win_flat[2]=win_r0c2;
    assign win_flat[3]=win_r1c0; assign win_flat[4]=win_r1c1; assign win_flat[5]=win_r1c2;
    assign win_flat[6]=win_r2c0; assign win_flat[7]=win_r2c1; assign win_flat[8]=win_r2c2;

    reg lb_wr_en;
    line_buffer_16ch #(.WIDTH(WIDTH), .CHANNELS(IC)) u_lb (
        .clk            (clk),
        .rst_n          (rst_n),
        .pixel_in       (s_axis_feat_tdata),
        .wr_en          (lb_wr_en),
        .col_ptr        (col_ptr),
        .top_row_active (top_row_active),
        .win_r0c0(win_r0c0), .win_r0c1(win_r0c1), .win_r0c2(win_r0c2),
        .win_r1c0(win_r1c0), .win_r1c1(win_r1c1), .win_r1c2(win_r1c2),
        .win_r2c0(win_r2c0), .win_r2c1(win_r2c1), .win_r2c2(win_r2c2)
    );

    // -------------------------------------------------------------------------
    // MAC array: 16 IC x 1 OC
    // NOTE: mac_array_16x1 now has a 2-stage registered adder tree.
    //       mac_accum is valid 2 extra cycles after the DSP pipeline flushes.
    //       The FSM drain_cnt accounts for this (see below).
    // -------------------------------------------------------------------------
    reg  mac_clear;
    reg  mac_en;
    reg  [IC*8-1:0] mac_acts;
    wire signed [21:0] mac_accum;

    mac_array_16x1 #(.IC(IC)) u_mac (
        .clk     (clk),
        .rst_n   (rst_n),
        .clear   (mac_clear),
        .en      (mac_en),
        .weights (weight_data),
        .acts    (mac_acts),
        .accum   (mac_accum)
    );

    // -------------------------------------------------------------------------
    // Post-processing: bias + sat (no ReLU) + residual subtraction
    // All combinational - mac_accum feeds these directly after drain.
    // -------------------------------------------------------------------------
    wire signed [22:0] biased =
        {mac_accum[21], mac_accum} + {{7{bias_data[15]}}, bias_data};
    wire signed [21:0] biased_sat =
        (biased > 23'sd2097151)  ? 22'sd2097151  :
        (biased < -23'sd2097152) ? -22'sd2097152 :
        biased[21:0];
    wire signed [7:0] noise_pred;
    sat_relu #(.ENABLE_RELU(0)) u_sr (
        .accum_in  (biased_sat),
        .pixel_out (noise_pred)
    );
    wire [7:0] clean_pixel;
    residual_sub u_res (
        .raw_pixel  (raw_pixel_buf),
        .noise_pred (noise_pred),
        .clean_out  (clean_pixel)
    );

    // -------------------------------------------------------------------------
    // TLAST delay shift register - WIDTH bits deep, advances on lb_wr_en.
    //
    // FIX 2: (* shreg_extract = "yes" *) forces SRL32 inference.
    // SRL32 has a built-in CE pin and is co-located in a single slice column,
    // eliminating the 640-CE-fan-out timing problem (407 TIMING-16 violations).
    // -------------------------------------------------------------------------
    (* shreg_extract = "yes" *)
    reg [WIDTH-1:0] tlast_sr;
    wire tlast_delayed = tlast_sr[WIDTH-1];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            tlast_sr <= {WIDTH{1'b0}};
        else if (lb_wr_en)
            tlast_sr <= {tlast_sr[WIDTH-2:0], s_axis_feat_tlast};
    end

    // -------------------------------------------------------------------------
    // FSM
    // -------------------------------------------------------------------------
    localparam S_IDLE     = 3'd0;
    localparam S_LOAD     = 3'd1;   // LB write + issue weight_addr=0; no mac_en
    localparam S_PREFETCH = 3'd2;   // weight[0] valid; fire mac k=0, issue addr=1
    localparam S_COMPUTE  = 3'd3;   // k=1..8
    localparam S_DRAIN    = 3'd4;   // flush DSP + adder tree pipeline
    localparam S_OUTPUT   = 3'd5;   // latch clean_pixel, drive m_axis

    reg [2:0] state;

    // FIX 1: drain_cnt widened to 3 bits to hold value 4.
    // drain_cnt = 4 => 5 drain cycles: 4->3->2->1->0->exit
    // (3 DSP cycles + 2 adder-tree pipeline cycles)
    reg [2:0] drain_cnt;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state              <= S_IDLE;
            col_ptr            <= 0;
            row_cnt            <= 0;
            k                  <= 0;
            drain_cnt          <= 0;
            mac_clear          <= 0;
            mac_en             <= 0;
            mac_acts           <= 0;
            weight_addr        <= 0;
            lb_wr_en           <= 0;
            s_axis_feat_tready <= 0;
            s_axis_raw_tready  <= 0;
            m_axis_tvalid      <= 0;
            m_axis_tlast       <= 0;
            done               <= 0;
            m_axis_tdata       <= 0;
            raw_pixel_buf      <= 0;
        end else begin
            // Default de-assert each cycle
            mac_clear     <= 0;
            mac_en        <= 0;
            lb_wr_en      <= 0;
            done          <= 0;
            m_axis_tvalid <= 0;

            case (state)

                // --------------------------------------------------------------
                // S_IDLE: assert both tready signals, wait for both valid
                // --------------------------------------------------------------
                S_IDLE: begin
                    s_axis_feat_tready <= 1;
                    s_axis_raw_tready  <= 1;
                    if (s_axis_feat_tvalid && s_axis_raw_tvalid) begin
                        raw_pixel_buf <= s_axis_raw_tdata;
                        state         <= S_LOAD;
                    end
                end

                // --------------------------------------------------------------
                // S_LOAD: write feature pixel to line buffer, issue weight addr=0.
                // Do NOT fire mac_en - weight_data is registered and arrives
                // one cycle later in S_PREFETCH.
                // --------------------------------------------------------------
                S_LOAD: begin
                    s_axis_feat_tready <= 0;
                    s_axis_raw_tready  <= 0;
                    lb_wr_en    <= 1;
                    weight_addr <= 8'd0;
                    state       <= S_PREFETCH;
                end

                // --------------------------------------------------------------
                // S_PREFETCH: weight_data[0] is valid NOW.
                // Fire first MAC (k=0): clear accumulator and load first product.
                // Issue weight_addr=1 for next cycle.
                // --------------------------------------------------------------
                S_PREFETCH: begin
                    mac_clear   <= 1;
                    mac_en      <= 1;
                    mac_acts    <= win_flat[0];
                    weight_addr <= 8'd1;
                    k           <= 4'd1;
                    state       <= S_COMPUTE;
                end

                // --------------------------------------------------------------
                // S_COMPUTE: k=1..8.
                // mac_en stays 1 for k=8 (last product); only state transitions.
                // weight_addr look-ahead stops at k=7 (addr=8 was issued at k=7
                // cycle; k=8 cycle doesn't need a new address).
                // --------------------------------------------------------------
                S_COMPUTE: begin
                    mac_en   <= 1;
                    mac_acts <= win_flat[k];
                    if (k < 4'd8)
                        weight_addr <= k + 4'd1;
                    k <= k + 1;
                    if (k == 4'd8) begin
                        state     <= S_DRAIN;
                        // FIX 1: start at 4 for 5 drain cycles total
                        drain_cnt <= 3'd4;
                    end
                end

                // --------------------------------------------------------------
                // S_DRAIN: flush the 3-stage DSP pipeline AND 2-stage adder tree.
                // Total: 5 cycles. drain_cnt 4->3->2->1->0 then exit.
                // mac_accum is valid when we enter S_OUTPUT.
                // --------------------------------------------------------------
                S_DRAIN: begin
                    if (drain_cnt != 3'd0)
                        drain_cnt <= drain_cnt - 1;
                    else
                        state <= S_OUTPUT;
                end

                // --------------------------------------------------------------
                // S_OUTPUT: latch post-processed result and drive AXI4-S output.
                // Hold until m_axis_tready (proper AXI-S handshake).
                // Advance col/row counters only on completed handshake.
                // --------------------------------------------------------------
                S_OUTPUT: begin
                    m_axis_tdata  <= clean_pixel;
                    m_axis_tvalid <= 1;
                    m_axis_tlast  <= tlast_delayed;

                    if (m_axis_tready) begin
                        if (col_ptr == WIDTH-1) begin
                            col_ptr <= 0;
                            if (row_cnt == HEIGHT-1) begin
                                done    <= 1;
                                state   <= S_IDLE;
                            end else begin
                                row_cnt            <= row_cnt + 1;
                                state              <= S_IDLE;
                                s_axis_feat_tready <= 1;
                                s_axis_raw_tready  <= 1;
                            end
                        end else begin
                            col_ptr            <= col_ptr + 1;
                            state              <= S_IDLE;
                            s_axis_feat_tready <= 1;
                            s_axis_raw_tready  <= 1;
                        end
                    end
                    // If !m_axis_tready: hold state, keep tvalid=1, do not advance
                end

            endcase
        end
    end

endmodule
// =============================================================================
// dncnn_ingestor.v  (FIXED)
//
// FIX: mac_accum slice width updated from 22 to 23 bits to match the
// corrected mac_array_1x16 output bus [OC*23-1:0].
// oc_accum[oc] is now [22:0] and bias addition is widened to 24-bit signed.
// Saturation thresholds updated accordingly.
//
// All FSM logic (S_LOAD/PREFETCH/COMPUTE/DRAIN/OUTPUT, TLAST shift register,
// drain_cnt=2, Bug-1/2/3 fixes) is unchanged.
// =============================================================================
module dncnn_ingestor #(
    parameter WIDTH    = 640,
    parameter HEIGHT   = 480,
    parameter OC       = 16
) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire [7:0]  s_axis_tdata,
    input  wire        s_axis_tvalid,
    output reg         s_axis_tready,
    input  wire        s_axis_tlast,
    output reg  [7:0]               weight_addr,
    input  wire [OC*8-1:0]          weight_data,
    input  wire signed [OC*16-1:0]  bias_data,
    output reg  [OC*8-1:0]          m_axis_tdata,
    output reg                       m_axis_tvalid,
    input  wire                      m_axis_tready,
    output reg                       m_axis_tlast,
    output reg  done,
    input  wire top_row_active
);

    reg [$clog2(WIDTH)-1:0]  col_ptr;
    reg [$clog2(HEIGHT)-1:0] row_cnt;

    // Line buffer
    wire [7:0] w_r0c0, w_r0c1, w_r0c2;
    wire [7:0] w_r1c0, w_r1c1, w_r1c2;
    wire [7:0] w_r2c0, w_r2c1, w_r2c2;
    reg        lb_wr_en;

    line_buffer_1ch #(.WIDTH(WIDTH)) u_lb (
        .clk            (clk), .rst_n          (rst_n),
        .pixel_in       (s_axis_tdata), .wr_en (lb_wr_en),
        .col_ptr        (col_ptr), .top_row_active (top_row_active),
        .win_r0c0(w_r0c0), .win_r0c1(w_r0c1), .win_r0c2(w_r0c2),
        .win_r1c0(w_r1c0), .win_r1c1(w_r1c1), .win_r1c2(w_r1c2),
        .win_r2c0(w_r2c0), .win_r2c1(w_r2c1), .win_r2c2(w_r2c2)
    );

    wire [7:0] kernel_flat [0:8];
    assign kernel_flat[0]=w_r0c0; assign kernel_flat[1]=w_r0c1; assign kernel_flat[2]=w_r0c2;
    assign kernel_flat[3]=w_r1c0; assign kernel_flat[4]=w_r1c1; assign kernel_flat[5]=w_r1c2;
    assign kernel_flat[6]=w_r2c0; assign kernel_flat[7]=w_r2c1; assign kernel_flat[8]=w_r2c2;

    // MAC array - output is now OC*23 bits
    reg        mac_clear, mac_en;
    reg  [7:0] mac_act;
    wire signed [OC*23-1:0] mac_accum;   // FIX: was OC*22

    mac_array_1x16 #(.OC(OC)) u_mac (
        .clk     (clk), .rst_n (rst_n),
        .clear   (mac_clear), .en (mac_en),
        .weights (weight_data),
        .act_in  ($signed(mac_act)),
        .accum   (mac_accum)
    );

    // Bias + ReLU per OC
    wire signed [22:0] oc_accum [0:OC-1];   // FIX: was [21:0]
    wire signed [7:0]  oc_relu  [0:OC-1];
    genvar oc;
    generate
        for (oc = 0; oc < OC; oc = oc + 1) begin : gen_postproc
            assign oc_accum[oc] = mac_accum[oc*23 +: 23];   // FIX: 23-bit slice
            // Sign-extend 23-bit accum and 16-bit bias to 24-bit for safe add
            wire signed [23:0] biased =
                $signed({oc_accum[oc][22], oc_accum[oc]})
                + {{8{bias_data[oc*16+15]}}, bias_data[oc*16 +: 16]};
            // Saturate to 22-bit signed before ReLU
            wire signed [21:0] biased_sat =
                (biased > 24'sd2097151)  ? 22'sd2097151  :
                (biased < -24'sd2097152) ? -22'sd2097152 :
                biased[21:0];
            sat_relu #(.ENABLE_RELU(1)) u_sr (
                .accum_in  (biased_sat),
                .pixel_out (oc_relu[oc])
            );
        end
    endgenerate

    // TLAST shift register
    reg [WIDTH-1:0] tlast_sr;
    wire tlast_delayed = tlast_sr[WIDTH-1];
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) tlast_sr <= {WIDTH{1'b0}};
        else if (lb_wr_en)
            tlast_sr <= {tlast_sr[WIDTH-2:0], s_axis_tlast};
    end

    // FSM
    localparam S_IDLE     = 3'd0;
    localparam S_LOAD     = 3'd1;
    localparam S_PREFETCH = 3'd2;
    localparam S_COMPUTE  = 3'd3;
    localparam S_DRAIN    = 3'd4;
    localparam S_OUTPUT   = 3'd5;

    reg [2:0] state;
    reg [3:0] k;
    reg [1:0] drain_cnt;
    integer   i;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE; col_ptr <= 0; row_cnt <= 0;
            k <= 0; drain_cnt <= 0;
            mac_clear <= 0; mac_en <= 0; mac_act <= 0;
            weight_addr <= 0; lb_wr_en <= 0;
            s_axis_tready <= 0; m_axis_tvalid <= 0;
            m_axis_tlast <= 0; m_axis_tdata <= 0; done <= 0;
        end else begin
            mac_clear <= 0; mac_en <= 0; lb_wr_en <= 0;
            done <= 0; m_axis_tvalid <= 0;

            case (state)
                S_IDLE: begin
                    s_axis_tready <= 1;
                    if (s_axis_tvalid) state <= S_LOAD;
                end
                S_LOAD: begin
                    s_axis_tready <= 0;
                    lb_wr_en    <= 1;
                    weight_addr <= 8'd0;
                    state       <= S_PREFETCH;
                end
                S_PREFETCH: begin
                    mac_clear   <= 1; mac_en <= 1;
                    mac_act     <= kernel_flat[0];
                    weight_addr <= 8'd1;
                    k           <= 4'd1;
                    state       <= S_COMPUTE;
                end
                S_COMPUTE: begin
                    mac_en  <= 1;
                    mac_act <= kernel_flat[k];
                    if (k < 4'd8) weight_addr <= k + 1;
                    if (k == 4'd8) begin
                        state     <= S_DRAIN;
                        drain_cnt <= 2'd2;   // 3 drain cycles: 2?1?0
                    end else begin
                        k <= k + 1;
                    end
                end
                S_DRAIN: begin
                    if (drain_cnt != 2'd0) drain_cnt <= drain_cnt - 1;
                    else                   state <= S_OUTPUT;
                end
                S_OUTPUT: begin
                    for (i = 0; i < OC; i = i + 1)
                        m_axis_tdata[i*8 +: 8] <= oc_relu[i];
                    m_axis_tvalid <= 1;
                    m_axis_tlast  <= tlast_delayed;
                    if (m_axis_tready) begin
                        if (col_ptr == WIDTH-1) begin
                            col_ptr <= 0;
                            if (row_cnt == HEIGHT-1) begin
                                row_cnt <= 0; done <= 1; state <= S_IDLE;
                            end else begin
                                row_cnt <= row_cnt + 1; state <= S_IDLE;
                                s_axis_tready <= 1;
                            end
                        end else begin
                            col_ptr <= col_ptr + 1; state <= S_IDLE;
                            s_axis_tready <= 1;
                        end
                    end
                end
            endcase
        end
    end

endmodule
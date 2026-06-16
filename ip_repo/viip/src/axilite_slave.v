// =============================================================================
// axilite_slave.v  (FIXED)
//
// FIX: 3x CRITICAL WARNING [Synth 8-6859] multi-driven net on reg_irq_status
//
// ROOT CAUSE:
//   reg_irq_status was driven by TWO separate always blocks:
//     1. Write-register block: W1C clear (write 1 to bit[0] clears it)
//     2. IRQ-capture block:    set when engine_done pulses
//   Two always blocks writing the same reg = multi-driver. Vivado's
//   constant-propagation merged one driver with GND and silently ignored it,
//   meaning engine_done could NEVER set the IRQ bit at runtime.
//
// FIX APPLIED:
//   Merged both write paths into a single always block (the IRQ block).
//   Priority: engine_done SET takes precedence over W1C CLEAR on the same
//   cycle (if both happen simultaneously, set wins - correct behaviour since
//   a new done event arriving the same cycle as a clear should re-arm the IRQ).
//   The write-register block no longer touches reg_irq_status.
//
// All other logic is unchanged.
// =============================================================================
module axilite_slave #(
    parameter DATA_WIDTH = 32,
    parameter ADDR_WIDTH = 7
) (
    input  wire        clk,
    input  wire        rst_n,

    // AXI4-Lite Slave ports
    input  wire [ADDR_WIDTH-1:0] s_axil_awaddr,
    input  wire                  s_axil_awvalid,
    output reg                   s_axil_awready,
    input  wire [DATA_WIDTH-1:0] s_axil_wdata,
    input  wire [3:0]            s_axil_wstrb,
    input  wire                  s_axil_wvalid,
    output reg                   s_axil_wready,
    output reg  [1:0]            s_axil_bresp,
    output reg                   s_axil_bvalid,
    input  wire                  s_axil_bready,
    input  wire [ADDR_WIDTH-1:0] s_axil_araddr,
    input  wire                  s_axil_arvalid,
    output reg                   s_axil_arready,
    output reg  [DATA_WIDTH-1:0] s_axil_rdata,
    output reg  [1:0]            s_axil_rresp,
    output reg                   s_axil_rvalid,
    input  wire                  s_axil_rready,

    // Outputs to engine
    output reg         start_pulse,
    output reg  [1:0]  layer_type,
    output reg  [10:0] img_width,
    output reg  [9:0]  img_height,
    output reg         irq_enable,
    output reg  [11:0] weight_wr_addr,
    output reg  [7:0]  weight_wr_data,
    output reg         weight_wr_en,
    output reg  [3:0]  bias_wr_addr,
    output reg  [15:0] bias_wr_data,
    output reg         bias_wr_en,

    // Inputs from engine
    input  wire        engine_busy,
    input  wire        engine_done,
    input  wire        engine_error,
    input  wire [31:0] pixel_cnt,

    // IRQ output
    output reg         done_irq
);

    // Internal register file
    reg        reg_start;
    reg [1:0]  reg_layer_type;
    reg [10:0] reg_img_width;
    reg [9:0]  reg_img_height;
    reg [11:0] reg_weight_addr;
    reg        reg_irq_status;      // FIX: driven by ONE block only
    reg        reg_irq_enable;
    reg [3:0]  reg_bias_addr;

    // W1C strobe - set when the write block decodes ADDR_IRQ_STATUS with bit[0]=1
    // This is a combinational signal from the write block, consumed by the IRQ block.
    reg w1c_irq_clear;

    // Address constants
    localparam ADDR_CR_CTRL        = 7'h00;
    localparam ADDR_SR_STATUS      = 7'h04;
    localparam ADDR_LAYER_TYPE     = 7'h08;
    localparam ADDR_IMG_WIDTH      = 7'h0C;
    localparam ADDR_IMG_HEIGHT     = 7'h10;
    localparam ADDR_WEIGHT_ADDR    = 7'h14;
    localparam ADDR_WEIGHT_WDATA   = 7'h18;
    localparam ADDR_IRQ_STATUS     = 7'h1C;
    localparam ADDR_IRQ_ENABLE     = 7'h20;
    localparam ADDR_DEBUG_PIXEL    = 7'h24;
    localparam ADDR_BIAS_ADDR      = 7'h28;
    localparam ADDR_BIAS_WDATA     = 7'h2C;

    // Write channel buffers
    reg [ADDR_WIDTH-1:0] aw_addr_buf;
    reg                  aw_valid_buf;
    reg [DATA_WIDTH-1:0] w_data_buf;
    reg [3:0]            w_strb_buf;
    reg                  w_valid_buf;

    // -------------------------------------------------------------------------
    // AW channel
    // -------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s_axil_awready <= 1'b0;
            aw_valid_buf   <= 1'b0;
            aw_addr_buf    <= {ADDR_WIDTH{1'b0}};
        end else begin
            if (s_axil_awvalid && s_axil_awready) begin
                aw_addr_buf    <= s_axil_awaddr;
                aw_valid_buf   <= 1'b1;
                s_axil_awready <= 1'b0;
            end else if (aw_valid_buf && w_valid_buf) begin
                aw_valid_buf   <= 1'b0;
                s_axil_awready <= 1'b1;
            end else begin
                s_axil_awready <= !aw_valid_buf;
            end
        end
    end

    // -------------------------------------------------------------------------
    // W channel
    // -------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s_axil_wready <= 1'b0;
            w_valid_buf   <= 1'b0;
            w_data_buf    <= {DATA_WIDTH{1'b0}};
            w_strb_buf    <= 4'b0000;
        end else begin
            if (s_axil_wvalid && s_axil_wready) begin
                w_data_buf    <= s_axil_wdata;
                w_strb_buf    <= s_axil_wstrb;
                w_valid_buf   <= 1'b1;
                s_axil_wready <= 1'b0;
            end else if (aw_valid_buf && w_valid_buf) begin
                w_valid_buf   <= 1'b0;
                s_axil_wready <= 1'b1;
            end else begin
                s_axil_wready <= !w_valid_buf;
            end
        end
    end

    // -------------------------------------------------------------------------
    // Write register file + B response
    // NOTE: reg_irq_status is NOT touched here (FIX). Instead, a combinational
    // w1c_irq_clear signal is derived and consumed by the IRQ block below.
    // -------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s_axil_bvalid    <= 1'b0;
            s_axil_bresp     <= 2'b00;
            start_pulse      <= 1'b0;
            reg_layer_type   <= 2'd1;
            reg_img_width    <= 11'd640;
            reg_img_height   <= 10'd480;
            reg_weight_addr  <= 12'd0;
            reg_irq_enable   <= 1'b0;
            reg_bias_addr    <= 4'd0;
            weight_wr_en     <= 1'b0;
            weight_wr_addr   <= 12'd0;
            weight_wr_data   <= 8'h00;
            bias_wr_en       <= 1'b0;
            bias_wr_addr     <= 4'd0;
            bias_wr_data     <= 16'h0000;
            w1c_irq_clear    <= 1'b0;
        end else begin
            start_pulse   <= 1'b0;
            weight_wr_en  <= 1'b0;
            bias_wr_en    <= 1'b0;
            w1c_irq_clear <= 1'b0;  // default: no clear request

            if (s_axil_bvalid && s_axil_bready)
                s_axil_bvalid <= 1'b0;

            if (aw_valid_buf && w_valid_buf && !s_axil_bvalid) begin
                s_axil_bvalid <= 1'b1;
                s_axil_bresp  <= 2'b00;
                case (aw_addr_buf)
                    ADDR_CR_CTRL: begin
                        if (w_data_buf[0] && !engine_busy)
                            start_pulse <= 1'b1;
                    end
                    ADDR_LAYER_TYPE: begin
                        if (w_strb_buf[0]) reg_layer_type <= w_data_buf[1:0];
                    end
                    ADDR_IMG_WIDTH: begin
                        if (w_strb_buf[0]) reg_img_width[7:0]  <= w_data_buf[7:0];
                        if (w_strb_buf[1]) reg_img_width[10:8] <= w_data_buf[10:8];
                    end
                    ADDR_IMG_HEIGHT: begin
                        if (w_strb_buf[0]) reg_img_height[7:0] <= w_data_buf[7:0];
                        if (w_strb_buf[1]) reg_img_height[9:8] <= w_data_buf[9:8];
                    end
                    ADDR_WEIGHT_ADDR: begin
                        if (w_strb_buf[0]) reg_weight_addr[7:0]  <= w_data_buf[7:0];
                        if (w_strb_buf[1]) reg_weight_addr[11:8] <= w_data_buf[11:8];
                    end
                    ADDR_WEIGHT_WDATA: begin
                        if (w_strb_buf[0]) begin
                            weight_wr_addr  <= reg_weight_addr;
                            weight_wr_data  <= w_data_buf[7:0];
                            weight_wr_en    <= 1'b1;
                            reg_weight_addr <= reg_weight_addr + 1;
                        end
                    end
                    ADDR_IRQ_STATUS: begin
                        // FIX: signal the IRQ block to clear; do NOT write
                        // reg_irq_status here to avoid multi-driver.
                        if (w_strb_buf[0] && w_data_buf[0])
                            w1c_irq_clear <= 1'b1;
                    end
                    ADDR_IRQ_ENABLE: begin
                        if (w_strb_buf[0]) reg_irq_enable <= w_data_buf[0];
                    end
                    ADDR_BIAS_ADDR: begin
                        if (w_strb_buf[0]) reg_bias_addr <= w_data_buf[3:0];
                    end
                    ADDR_BIAS_WDATA: begin
                        if (w_strb_buf[0]) bias_wr_data[7:0]  <= w_data_buf[7:0];
                        if (w_strb_buf[1]) bias_wr_data[15:8] <= w_data_buf[15:8];
                        bias_wr_addr  <= reg_bias_addr;
                        bias_wr_en    <= 1'b1;
                        reg_bias_addr <= reg_bias_addr + 1;
                    end
                    default: ;
                endcase
            end
        end
    end

    // -------------------------------------------------------------------------
    // Output register mirrors
    // -------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            layer_type  <= 2'd1;
            img_width   <= 11'd640;
            img_height  <= 10'd480;
            irq_enable  <= 1'b0;
        end else begin
            layer_type  <= reg_layer_type;
            img_width   <= reg_img_width;
            img_height  <= reg_img_height;
            irq_enable  <= reg_irq_enable;
        end
    end

    // -------------------------------------------------------------------------
    // IRQ generation - SINGLE block drives reg_irq_status (FIX)
    //
    // Priority: engine_done SET > W1C CLEAR
    // If engine_done and w1c_irq_clear arrive simultaneously, SET wins
    // (new event should re-arm the IRQ even if SW just cleared it).
    // -------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            reg_irq_status <= 1'b0;
            done_irq       <= 1'b0;
        end else begin
            if (engine_done)
                reg_irq_status <= 1'b1;          // SET: new done event
            else if (w1c_irq_clear)
                reg_irq_status <= 1'b0;          // CLEAR: SW wrote 1 to W1C bit
            // else: hold

            done_irq <= reg_irq_status && reg_irq_enable;
        end
    end

    // -------------------------------------------------------------------------
    // Read channel
    // -------------------------------------------------------------------------
    wire [1:0] status_code = engine_error ? 2'd3 :
                             engine_done  ? 2'd2 :
                             engine_busy  ? 2'd1 : 2'd0;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s_axil_arready <= 1'b1;
            s_axil_rvalid  <= 1'b0;
            s_axil_rdata   <= {DATA_WIDTH{1'b0}};
            s_axil_rresp   <= 2'b00;
        end else begin
            if (s_axil_rvalid && s_axil_rready)
                s_axil_rvalid <= 1'b0;
            if (s_axil_arvalid && s_axil_arready) begin
                s_axil_arready <= 1'b0;
                s_axil_rvalid  <= 1'b1;
                s_axil_rresp   <= 2'b00;
                case (s_axil_araddr)
                    ADDR_CR_CTRL:      s_axil_rdata <= {DATA_WIDTH{1'b0}};
                    ADDR_SR_STATUS:    s_axil_rdata <= {{30{1'b0}}, status_code};
                    ADDR_LAYER_TYPE:   s_axil_rdata <= {{30{1'b0}}, reg_layer_type};
                    ADDR_IMG_WIDTH:    s_axil_rdata <= {{21{1'b0}}, reg_img_width};
                    ADDR_IMG_HEIGHT:   s_axil_rdata <= {{22{1'b0}}, reg_img_height};
                    ADDR_WEIGHT_ADDR:  s_axil_rdata <= {{20{1'b0}}, reg_weight_addr};
                    ADDR_WEIGHT_WDATA: s_axil_rdata <= {DATA_WIDTH{1'b0}};
                    ADDR_IRQ_STATUS:   s_axil_rdata <= {{31{1'b0}}, reg_irq_status};
                    ADDR_IRQ_ENABLE:   s_axil_rdata <= {{31{1'b0}}, reg_irq_enable};
                    ADDR_DEBUG_PIXEL:  s_axil_rdata <= pixel_cnt;
                    ADDR_BIAS_ADDR:    s_axil_rdata <= {{28{1'b0}}, reg_bias_addr};
                    ADDR_BIAS_WDATA:   s_axil_rdata <= {DATA_WIDTH{1'b0}};
                    default:           s_axil_rdata <= {DATA_WIDTH{1'b0}};
                endcase
            end else if (!s_axil_rvalid) begin
                s_axil_arready <= 1'b1;
            end
        end
    end

endmodule
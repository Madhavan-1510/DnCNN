// =============================================================================
// async_fifo_eject.v - CDC FIFO: DnCNN core clock ? HDMI TX pixel clock
// =============================================================================
// WHAT THIS DOES:
//   Clock-domain crossing for the clean pixel output stream. Denoised pixels
//   are produced at 100 MHz core clock and must be delivered to the rgb2dvi
//   transmitter at 74.25 MHz.
//
//   This module is a WRAPPER around a Xilinx FIFO Generator IP instance.
//   If the IP is not yet generated, a behavioural model is provided for
//   simulation, gated by `BEHAVIOURAL_MODEL.
//
// FIFO SPEC:
//   Write clock:  clk_core      (100 MHz, from DnCNN finisher output)
//   Read clock:   clk_pixel_out (74.25 MHz, to rgb2dvi)
//   Data width:   8 bits (one clean pixel; upstream pads 8b?24b RGB if needed)
//   Depth:        1024 entries
//   Mode:         First-Word-Fall-Through (FWFT)
//   BRAM backing: yes
//
// NOTE ON DATA WIDTH:
//   The finisher outputs 8-bit clean luma pixels. If your display path needs
//   24-bit RGB (e.g. rgb2dvi expects 3×8b), the upstream logic should
//   replicate Y?RGB before writing to this FIFO, OR use a 24-bit wide FIFO.
//   This wrapper is parameterised to DATA_WIDTH (default 8) so the top can
//   pass 24 if desired.
//
// XILINX IP GENERATION SETTINGS (FIFO Generator 13.x):
//   Component Name:      fifo_async_8b_1k_eject   (or 24b variant)
//   FIFO Implementation: Independent Clocks Block RAM
//   Read Mode:           First Word Fall Through
//   Write Width:         8 (or 24)
//   Write Depth:         1024
//   Read Width:          8 (or 24)
//   Reset Type:          Asynchronous Reset
//   Almost Full:         threshold = 960
//   Almost Empty:        threshold = 8
//
// =============================================================================

`ifndef USE_XILINX_IP
`define BEHAVIOURAL_MODEL
`endif

module async_fifo_eject #(
    parameter DATA_WIDTH = 8    // set to 24 if upstream produces RGB
)(
    // Write side (core clock domain)
    input  wire                  wr_clk,          // 100 MHz
    input  wire                  wr_rst_n,
    input  wire                  wr_en,
    input  wire [DATA_WIDTH-1:0] wr_din,

    output wire                  wr_full,
    output wire                  wr_almost_full,

    // Read side (pixel output clock domain)
    input  wire                  rd_clk,          // 74.25 MHz
    input  wire                  rd_rst_n,
    input  wire                  rd_en,
    output wire [DATA_WIDTH-1:0] rd_dout,

    output wire                  rd_empty,
    output wire                  rd_almost_empty
);

`ifdef BEHAVIOURAL_MODEL
    // -------------------------------------------------------------------------
    // Behavioural model (simulation only). Gray-code pointer CDC.
    // -------------------------------------------------------------------------
    localparam DEPTH    = 1024;
    localparam PTR_BITS = 10;

    reg [DATA_WIDTH-1:0] fifo_mem [0:DEPTH-1];

    reg [PTR_BITS:0] wr_ptr_bin;
    reg [PTR_BITS:0] rd_ptr_bin;

    wire [PTR_BITS:0] wr_ptr_gray = wr_ptr_bin ^ (wr_ptr_bin >> 1);
    wire [PTR_BITS:0] rd_ptr_gray = rd_ptr_bin ^ (rd_ptr_bin >> 1);

    reg [PTR_BITS:0] rd_ptr_gray_s1_wclk, rd_ptr_gray_sync_wclk;
    reg [PTR_BITS:0] wr_ptr_gray_s1_rclk, wr_ptr_gray_sync_rclk;

    // Write domain: RAM write (no async reset - required for BRAM inference)
    always @(posedge wr_clk) begin
        if (wr_en && !wr_full)
            fifo_mem[wr_ptr_bin[PTR_BITS-1:0]] <= wr_din;
    end

    // Write domain: control FFs (async reset OK - no RAM arrays)
    always @(posedge wr_clk or negedge wr_rst_n) begin
        if (!wr_rst_n) begin
            wr_ptr_bin            <= {(PTR_BITS+1){1'b0}};
            rd_ptr_gray_s1_wclk   <= {(PTR_BITS+1){1'b0}};
            rd_ptr_gray_sync_wclk <= {(PTR_BITS+1){1'b0}};
        end else begin
            rd_ptr_gray_s1_wclk   <= rd_ptr_gray;
            rd_ptr_gray_sync_wclk <= rd_ptr_gray_s1_wclk;
            if (wr_en && !wr_full)
                wr_ptr_bin <= wr_ptr_bin + 1;
        end
    end

    integer gi;
    reg [PTR_BITS:0] rd_ptr_bin_wclk;
    always @(*) begin
        rd_ptr_bin_wclk[PTR_BITS] = rd_ptr_gray_sync_wclk[PTR_BITS];
        for (gi = PTR_BITS-1; gi >= 0; gi = gi - 1)
            rd_ptr_bin_wclk[gi] = rd_ptr_bin_wclk[gi+1] ^ rd_ptr_gray_sync_wclk[gi];
    end

    wire [PTR_BITS:0] occ_wclk = wr_ptr_bin - rd_ptr_bin_wclk;
    assign wr_full        = (occ_wclk == DEPTH);
    assign wr_almost_full = (occ_wclk >= (DEPTH - 64));

    // Read domain
    always @(posedge rd_clk or negedge rd_rst_n) begin
        if (!rd_rst_n) begin
            rd_ptr_bin            <= {(PTR_BITS+1){1'b0}};
            wr_ptr_gray_s1_rclk   <= {(PTR_BITS+1){1'b0}};
            wr_ptr_gray_sync_rclk <= {(PTR_BITS+1){1'b0}};
        end else begin
            wr_ptr_gray_s1_rclk   <= wr_ptr_gray;
            wr_ptr_gray_sync_rclk <= wr_ptr_gray_s1_rclk;
            if (rd_en && !rd_empty)
                rd_ptr_bin <= rd_ptr_bin + 1;
        end
    end

    reg [PTR_BITS:0] wr_ptr_bin_rclk;
    always @(*) begin
        wr_ptr_bin_rclk[PTR_BITS] = wr_ptr_gray_sync_rclk[PTR_BITS];
        for (gi = PTR_BITS-1; gi >= 0; gi = gi - 1)
            wr_ptr_bin_rclk[gi] = wr_ptr_bin_rclk[gi+1] ^ wr_ptr_gray_sync_rclk[gi];
    end

    wire [PTR_BITS:0] occ_rclk = wr_ptr_bin_rclk - rd_ptr_bin;
    assign rd_empty        = (occ_rclk == 0);
    assign rd_almost_empty = (occ_rclk <= 8);

    assign rd_dout = fifo_mem[rd_ptr_bin[PTR_BITS-1:0]];

`else
    // -------------------------------------------------------------------------
    // Xilinx FIFO Generator instance (for implementation).
    // Generate IP named "fifo_async_8b_1k_eject" (or adjust for DATA_WIDTH=24).
    // -------------------------------------------------------------------------
    fifo_async_8b_1k_eject u_fifo (
        .wr_clk       (wr_clk),
        .rst          (~wr_rst_n),
        .wr_en        (wr_en && !wr_full),
        .din          (wr_din),
        .full         (wr_full),
        .almost_full  (wr_almost_full),

        .rd_clk       (rd_clk),
        .rd_en        (rd_en && !rd_empty),
        .dout         (rd_dout),
        .empty        (rd_empty),
        .almost_empty (rd_almost_empty)
    );
`endif

endmodule
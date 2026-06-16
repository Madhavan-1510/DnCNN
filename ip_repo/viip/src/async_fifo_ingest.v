// =============================================================================
// async_fifo_ingest.v - CDC FIFO: HDMI RX pixel clock ? DnCNN core clock
// =============================================================================
// WHAT THIS DOES:
//   Clock-domain crossing for the raw pixel stream. DVI/HDMI input arrives
//   at the pixel clock (74.25 MHz for 1280x720 or 640x480 scaled) and must
//   be transferred to the DnCNN core at 100 MHz.
//
//   This module is a WRAPPER around a Xilinx FIFO Generator IP instance.
//   If the IP is not yet generated, a behavioural model is included below,
//   gated by `BEHAVIOURAL_MODEL. Switch to the Xilinx instance for
//   implementation by defining USE_XILNX_IP and generating the IP first.
//
// FIFO SPEC:
//   Write clock:  clk_pixel_in  (74.25 MHz, from dvi2rgb)
//   Read clock:   clk_core      (100 MHz, DnCNN engine)
//   Data width:   8 bits (one grayscale pixel)
//   Depth:        1024 entries (enough for ~1.5 lines of 640-pixel row)
//   Mode:         First-Word-Fall-Through (FWFT) - data valid on read port
//                 without requiring an extra read-enable pulse
//   BRAM backing: yes (INDEPENDENT_CLOCKS, BLOCK_RAM)
//
// BACKPRESSURE:
//   almost_full asserts when fewer than 64 entries remain.
//   Drive upstream (HDMI) to stall or drop if needed. In typical
//   operation the core reads faster than the pixel clock writes, so
//   the FIFO operates at low occupancy.
//
// USAGE IN dncnn_top:
//   - wr_clk = clk_pixel_in, wr_en = pixel_valid from dvi2rgb
//   - wr_din = raw 8-bit pixel
//   - rd_clk = clk_core, rd_en = s_axis_tready of dncnn_ingestor
//   - rd_dout ? s_axis_tdata of dncnn_ingestor
//   - rd_empty ? ~s_axis_tvalid of dncnn_ingestor
//
// XILINX IP GENERATION SETTINGS (FIFO Generator 13.x):
//   Component Name:      fifo_async_8b_1k
//   Interface Type:      AXI Stream (or Native - both work)
//   FIFO Implementation: Independent Clocks Block RAM
//   Read Mode:           First Word Fall Through
//   Write Width:         8
//   Write Depth:         1024
//   Read Width:          8
//   Reset Type:          Asynchronous Reset
//   Almost Full Flag:    YES, threshold = 960 (64 from full)
//   Almost Empty Flag:   YES, threshold = 8
//   Valid Flag:          NO (use empty inversion)
//   Overflow/Underflow:  NO (gate externally)
//
// =============================================================================

`ifndef USE_XILINX_IP
`define BEHAVIOURAL_MODEL
`endif

module async_fifo_ingest (
    // Write side (pixel clock domain)
    input  wire       wr_clk,       // 74.25 MHz pixel clock
    input  wire       wr_rst_n,     // async reset (active low)
    input  wire       wr_en,        // write enable
    input  wire [7:0] wr_din,       // 8-bit pixel data

    // Status (write clock domain)
    output wire       wr_full,      // FIFO full - do not write
    output wire       wr_almost_full, // high when <= 64 slots remain

    // Read side (core clock domain)
    input  wire       rd_clk,       // 100 MHz core clock
    input  wire       rd_rst_n,     // async reset (active low)
    input  wire       rd_en,        // read enable (FWFT: advances to next word)
    output wire [7:0] rd_dout,      // 8-bit pixel data (valid when !rd_empty)

    // Status (read clock domain)
    output wire       rd_empty,     // FIFO empty - rd_dout not valid
    output wire       rd_almost_empty // high when <= 8 entries remain
);

`ifdef BEHAVIOURAL_MODEL
    // -------------------------------------------------------------------------
    // Behavioural model: simple async FIFO for simulation only.
    // Uses gray-code pointers for CDC correctness.
    // NOT for implementation - replace with Xilinx IP for synthesis.
    // -------------------------------------------------------------------------
    localparam DEPTH     = 1024;
    localparam PTR_BITS  = 10;  // log2(1024)

    reg [7:0] fifo_mem [0:DEPTH-1];

    // Binary write/read pointers (write-domain and read-domain)
    reg [PTR_BITS:0] wr_ptr_bin;
    reg [PTR_BITS:0] rd_ptr_bin;

    // Gray-code versions for CDC
    wire [PTR_BITS:0] wr_ptr_gray = wr_ptr_bin ^ (wr_ptr_bin >> 1);
    wire [PTR_BITS:0] rd_ptr_gray = rd_ptr_bin ^ (rd_ptr_bin >> 1);

    // Two-FF synchronisers: read pointer into write domain, write pointer into read domain
    reg [PTR_BITS:0] rd_ptr_gray_s1_wclk, rd_ptr_gray_sync_wclk;
    reg [PTR_BITS:0] wr_ptr_gray_s1_rclk, wr_ptr_gray_sync_rclk;

    // ---- Write domain: RAM write (no async reset - required for BRAM inference) ----
    always @(posedge wr_clk) begin
        if (wr_en && !wr_full)
            fifo_mem[wr_ptr_bin[PTR_BITS-1:0]] <= wr_din;
    end

    // ---- Write domain: control FFs (async reset OK - no RAM arrays) ----
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

    // Convert synced read gray ptr back to binary for occupancy
    reg [PTR_BITS:0] rd_ptr_bin_wclk;
    integer gi;
    always @(*) begin
        rd_ptr_bin_wclk[PTR_BITS] = rd_ptr_gray_sync_wclk[PTR_BITS];
        for (gi = PTR_BITS-1; gi >= 0; gi = gi - 1)
            rd_ptr_bin_wclk[gi] = rd_ptr_bin_wclk[gi+1] ^ rd_ptr_gray_sync_wclk[gi];
    end

    wire [PTR_BITS:0] occupancy_wclk = wr_ptr_bin - rd_ptr_bin_wclk;
    assign wr_full        = (occupancy_wclk == DEPTH);
    assign wr_almost_full = (occupancy_wclk >= (DEPTH - 64));

    // ---- Read domain ----
    always @(posedge rd_clk or negedge rd_rst_n) begin
        if (!rd_rst_n) begin
            rd_ptr_bin <= {(PTR_BITS+1){1'b0}};
            wr_ptr_gray_s1_rclk   <= {(PTR_BITS+1){1'b0}};
            wr_ptr_gray_sync_rclk <= {(PTR_BITS+1){1'b0}};
        end else begin
            // Synchronise write pointer into read domain
            wr_ptr_gray_s1_rclk   <= wr_ptr_gray;
            wr_ptr_gray_sync_rclk <= wr_ptr_gray_s1_rclk;

            if (rd_en && !rd_empty)
                rd_ptr_bin <= rd_ptr_bin + 1;
        end
    end

    // Convert synced write gray ptr back to binary
    reg [PTR_BITS:0] wr_ptr_bin_rclk;
    always @(*) begin
        wr_ptr_bin_rclk[PTR_BITS] = wr_ptr_gray_sync_rclk[PTR_BITS];
        for (gi = PTR_BITS-1; gi >= 0; gi = gi - 1)
            wr_ptr_bin_rclk[gi] = wr_ptr_bin_rclk[gi+1] ^ wr_ptr_gray_sync_rclk[gi];
    end

    wire [PTR_BITS:0] occupancy_rclk = wr_ptr_bin_rclk - rd_ptr_bin;
    assign rd_empty        = (occupancy_rclk == 0);
    assign rd_almost_empty = (occupancy_rclk <= 8);

    // FWFT: output data is always at current rd_ptr (no extra read cycle needed)
    assign rd_dout = fifo_mem[rd_ptr_bin[PTR_BITS-1:0]];

`else
    // -------------------------------------------------------------------------
    // Xilinx FIFO Generator instance.
    // Generate IP with name "fifo_async_8b_1k" in Vivado IP Catalog.
    // Settings: INDEPENDENT_CLOCKS, BLOCK_RAM, FWFT, 8b, 1024 deep.
    // -------------------------------------------------------------------------
    wire wr_rst_busy, rd_rst_busy;  // reset-in-progress flags from IP

    fifo_async_8b_1k u_fifo (
        // Write side
        .wr_clk        (wr_clk),
        .rst           (~wr_rst_n),     // IP uses active-high reset
        .wr_en         (wr_en && !wr_full),
        .din           (wr_din),
        .full          (wr_full),
        .almost_full   (wr_almost_full),
        .wr_rst_busy   (wr_rst_busy),

        // Read side
        .rd_clk        (rd_clk),
        .rd_en         (rd_en && !rd_empty),
        .dout          (rd_dout),
        .empty         (rd_empty),
        .almost_empty  (rd_almost_empty),
        .rd_rst_busy   (rd_rst_busy)
    );
`endif

endmodule
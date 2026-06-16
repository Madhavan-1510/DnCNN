// =============================================================================
// dncnn_top_fixed.v  - BUGS 1,2,3,4 FIXED
// =============================================================================
// BUG FIXES APPLIED:
//
//  BUG 1 - Workhorse weight_addr 2-cycle latency compensation
//    The weight_bram Port B has a 2-cycle registered read pipeline.
//    The fixed dncnn_workhorse already issues addresses 2 cycles early
//    internally.  Additionally, this top-level passes the corrected
//    weight_addr directly to rd_addr_b (no extra pipeline register needed
//    here because the workhorse now pre-issues the address).
//    See dncnn_workhorse_fixed.v for the FSM change.
//
//  BUG 2 - top_row_active tied to 0
//    Added per-engine pixel-output counters that track col_ptr and row_cnt
//    by watching each engine's m_axis tvalid+tready handshake.  When the
//    reconstructed row_cnt == 0 the engine is processing the first row, so
//    top_row_active is asserted.
//    No engine port changes required.
//
//  BUG 3 - CDC reset not synchronized
//    Added two-FF reset synchronizers for each clock domain:
//      rst_core_n      - synchronized to clk_core      (100 MHz)
//      rst_pixel_in_n  - synchronized to clk_pixel_in  (74.25 MHz)
//      rst_pixel_out_n - synchronized to clk_pixel_out (74.25 MHz)
//    All sub-module instances now use the domain-correct reset.
//    The raw rst_n input is treated as asynchronous (from PS).
//
//  BUG 4 - Eject FIFO not connected
//    async_fifo_eject is now instantiated AND connected.
//    The finisher 8-bit output is replicated to 24-bit {y,y,y} before
//    being written into the eject FIFO.  hdmi_pixel_out / hdmi_pixel_valid
//    ports are added for the rgb2dvi IP connection.
//
// HOW TO VERIFY:
//  Bug 2: In simulation, assert top_row_active=1 for pixels 0..639
//          and 0 thereafter.  Inspect line_buffer win_r0* = 0 for row 0.
//  Bug 3: Drive rst_n low then high; confirm rst_pixel_in_n deasserts
//          exactly 2 clk_pixel_in cycles after rst_n goes high.
//  Bug 4: After finisher outputs clean pixels, read from rd_dout of
//          eject FIFO and confirm {R,G,B} = {y,y,y}.
// =============================================================================
module dncnn_top #(
    parameter WIDTH  = 640,
    parameter HEIGHT = 480,
    parameter IC     = 16,
    parameter OC     = 16
)(
    // Clocks and reset (rst_n is async, PS-sourced)
    input  wire        clk_core,
    input  wire        clk_pixel_in,
    input  wire        clk_pixel_out,
    input  wire        rst_n,           // asynchronous reset from PS

    // AXI4-Lite Slave
    input  wire [6:0]  s_axil_awaddr,
    input  wire        s_axil_awvalid,
    output wire        s_axil_awready,
    input  wire [31:0] s_axil_wdata,
    input  wire [3:0]  s_axil_wstrb,
    input  wire        s_axil_wvalid,
    output wire        s_axil_wready,
    output wire [1:0]  s_axil_bresp,
    output wire        s_axil_bvalid,
    input  wire        s_axil_bready,
    input  wire [6:0]  s_axil_araddr,
    input  wire        s_axil_arvalid,
    output wire        s_axil_arready,
    output wire [31:0] s_axil_rdata,
    output wire [1:0]  s_axil_rresp,
    output wire        s_axil_rvalid,
    input  wire        s_axil_rready,

    // AXI4-Stream feature input from VDMA (clk_core)
    input  wire [127:0] s_axis_feat_tdata,
    input  wire         s_axis_feat_tvalid,
    output wire         s_axis_feat_tready,
    input  wire         s_axis_feat_tlast,

    // AXI4-Stream raw pixel input for finisher (clk_core)
    input  wire [7:0]  s_axis_raw_tdata,
    input  wire        s_axis_raw_tvalid,
    output wire        s_axis_raw_tready,
    input  wire        s_axis_raw_tlast,

    // AXI4-Stream output to VDMA (clk_core)
    output wire [127:0] m_axis_out_tdata,
    output wire         m_axis_out_tvalid,
    input  wire         m_axis_out_tready,
    output wire         m_axis_out_tlast,

    // HDMI RX raw pixel input (pixel clock domain -> ingest FIFO)
    input  wire [7:0]  hdmi_pixel_in,
    input  wire        hdmi_pixel_valid,
    output wire        hdmi_fifo_full,

    // BUG 4 FIX: HDMI TX clean pixel output (pixel clock domain <- eject FIFO)
    output wire [23:0] hdmi_pixel_out,   // {R,G,B} = {y,y,y} for rgb2dvi
    output wire        hdmi_pixel_out_valid,
    input  wire        hdmi_pixel_out_ready,

    // IRQ to PS
    output wire        done_irq
);

    // =========================================================================
    // BUG 3 FIX: Two-FF reset synchronizers per clock domain
    // =========================================================================

    // -- clk_core domain --
    reg [1:0] rst_core_sync;
    wire      rst_core_n = rst_core_sync[1];
    always @(posedge clk_core or negedge rst_n) begin
        if (!rst_n) rst_core_sync <= 2'b00;
        else        rst_core_sync <= {rst_core_sync[0], 1'b1};
    end

    // -- clk_pixel_in domain --
    reg [1:0] rst_pixin_sync;
    wire      rst_pixel_in_n = rst_pixin_sync[1];
    always @(posedge clk_pixel_in or negedge rst_n) begin
        if (!rst_n) rst_pixin_sync <= 2'b00;
        else        rst_pixin_sync <= {rst_pixin_sync[0], 1'b1};
    end

    // -- clk_pixel_out domain --
    reg [1:0] rst_pixout_sync;
    wire      rst_pixel_out_n = rst_pixout_sync[1];
    always @(posedge clk_pixel_out or negedge rst_n) begin
        if (!rst_n) rst_pixout_sync <= 2'b00;
        else        rst_pixout_sync <= {rst_pixout_sync[0], 1'b1};
    end

    // =========================================================================
    // AXI-Lite decoded signals
    // =========================================================================
    wire        reg_start_pulse;
    wire [1:0]  reg_layer_type;
    wire [10:0] reg_img_width;
    wire [9:0]  reg_img_height;
    wire        reg_irq_enable;

    wire [11:0] axil_weight_wr_addr;
    wire [7:0]  axil_weight_wr_data;
    wire        axil_weight_wr_en;

    wire [3:0]  axil_bias_wr_addr;
    wire [15:0] axil_bias_wr_data;
    wire        axil_bias_wr_en;

    wire        engine_busy;
    wire        engine_done;
    wire        engine_error;
    wire [31:0] pixel_cnt;

    // =========================================================================
    // Weight BRAM ports
    // =========================================================================
    wire [11:0]   weight_addr_a;
    wire [127:0]  weight_data_a;
    wire [11:0]   weight_addr_b;
    wire [1023:0] weight_data_b;

    wire [11:0] ing_weight_addr;
    wire [11:0] wh_weight_addr;
    wire [11:0] fin_weight_addr;

    // =========================================================================
    // Engine done / busy
    // =========================================================================
    wire ing_done, wh_done, fin_done;

    wire start_ingestor  = reg_start_pulse && (reg_layer_type == 2'd0);
    wire start_workhorse = reg_start_pulse && (reg_layer_type == 2'd1);
    wire start_finisher  = reg_start_pulse && (reg_layer_type == 2'd2);

    reg eng_busy_r;
    always @(posedge clk_core or negedge rst_core_n) begin
        if (!rst_core_n)      eng_busy_r <= 1'b0;
        else if (reg_start_pulse) eng_busy_r <= 1'b1;
        else if (engine_done) eng_busy_r <= 1'b0;
    end

    assign engine_busy  = eng_busy_r;
    assign engine_done  = ing_done | wh_done | fin_done;
    assign engine_error = 1'b0;

    // =========================================================================
    // Bias registers
    // =========================================================================
    reg signed [15:0] bias_reg [0:15];
    integer bi;
    always @(posedge clk_core or negedge rst_core_n) begin
        if (!rst_core_n) begin
            for (bi = 0; bi < 16; bi = bi + 1)
                bias_reg[bi] <= 16'sd0;
        end else if (axil_bias_wr_en) begin
            bias_reg[axil_bias_wr_addr] <= $signed(axil_bias_wr_data);
        end
    end

    wire signed [OC*16-1:0] bias_flat;
    genvar bgi;
    generate
        for (bgi = 0; bgi < OC; bgi = bgi + 1) begin : gen_bias_pack
            assign bias_flat[bgi*16 +: 16] = bias_reg[bgi];
        end
    endgenerate

    // =========================================================================
    // Pixel counter (for axilite_slave debug register)
    // =========================================================================
    wire ing_pixel_out_valid, wh_pixel_out_valid, fin_pixel_out_valid;
    reg [31:0] pixel_cnt_r;
    always @(posedge clk_core or negedge rst_core_n) begin
        if (!rst_core_n)      pixel_cnt_r <= 32'd0;
        else if (reg_start_pulse) pixel_cnt_r <= 32'd0;
        else if (ing_pixel_out_valid || wh_pixel_out_valid || fin_pixel_out_valid)
            pixel_cnt_r <= pixel_cnt_r + 1;
    end
    assign pixel_cnt = pixel_cnt_r;

    // =========================================================================
    // BUG 2 FIX: per-engine top_row_active derived from output pixel counter
    //
    // We count pixels output by each engine to reconstruct row_cnt.
    // top_row_active is asserted when row_cnt_r == 0 (first row of frame).
    // It is deasserted after the first WIDTH outputs.
    //
    // This is safe because:
    //   - Each engine resets its LB at frame start.
    //   - The LB needs top_row_active=1 for the first row of writes (row 0),
    //     which is precisely when output pixels 0..(WIDTH-1) are produced.
    //   - We track output (post-compute) pixels; the first output pixel
    //     corresponds to the first input pixel being through the LB.
    //     The line buffer's first row of reads IS row 0, matching the flag.
    // =========================================================================

    // -- Ingestor --
    reg [$clog2(WIDTH)-1:0]  ing_col_r;
    reg [$clog2(HEIGHT)-1:0] ing_row_r;
    wire ing_top_row_active = (ing_row_r == 0);

    always @(posedge clk_core or negedge rst_core_n) begin
        if (!rst_core_n) begin
            ing_col_r <= 0; ing_row_r <= 0;
        end else if (reg_start_pulse && (reg_layer_type == 2'd0)) begin
            ing_col_r <= 0; ing_row_r <= 0;
        end else if (ing_pixel_out_valid) begin
            if (ing_col_r == WIDTH-1) begin
                ing_col_r <= 0;
                if (ing_row_r != HEIGHT-1) ing_row_r <= ing_row_r + 1;
            end else begin
                ing_col_r <= ing_col_r + 1;
            end
        end
    end

    // -- Workhorse --
    reg [$clog2(WIDTH)-1:0]  wh_col_r;
    reg [$clog2(HEIGHT)-1:0] wh_row_r;
    wire wh_top_row_active = (wh_row_r == 0);

    always @(posedge clk_core or negedge rst_core_n) begin
        if (!rst_core_n) begin
            wh_col_r <= 0; wh_row_r <= 0;
        end else if (reg_start_pulse && (reg_layer_type == 2'd1)) begin
            wh_col_r <= 0; wh_row_r <= 0;
        end else if (wh_pixel_out_valid) begin
            if (wh_col_r == WIDTH-1) begin
                wh_col_r <= 0;
                if (wh_row_r != HEIGHT-1) wh_row_r <= wh_row_r + 1;
            end else begin
                wh_col_r <= wh_col_r + 1;
            end
        end
    end

    // -- Finisher --
    reg [$clog2(WIDTH)-1:0]  fin_col_r;
    reg [$clog2(HEIGHT)-1:0] fin_row_r;
    wire fin_top_row_active = (fin_row_r == 0);

    always @(posedge clk_core or negedge rst_core_n) begin
        if (!rst_core_n) begin
            fin_col_r <= 0; fin_row_r <= 0;
        end else if (reg_start_pulse && (reg_layer_type == 2'd2)) begin
            fin_col_r <= 0; fin_row_r <= 0;
        end else if (fin_pixel_out_valid) begin
            if (fin_col_r == WIDTH-1) begin
                fin_col_r <= 0;
                if (fin_row_r != HEIGHT-1) fin_row_r <= fin_row_r + 1;
            end else begin
                fin_col_r <= fin_col_r + 1;
            end
        end
    end

    // =========================================================================
    // Ingest FIFO (HDMI RX pixel clock -> core clock)
    // =========================================================================
    wire       ingest_fifo_rd_empty;
    wire [7:0] ingest_fifo_rd_dout;
    wire       ingest_fifo_rd_en_int;

    async_fifo_ingest u_fifo_in (
        .wr_clk          (clk_pixel_in),
        .wr_rst_n        (rst_pixel_in_n),   // BUG 3 FIX: domain-sync'd reset
        .wr_en           (hdmi_pixel_valid),
        .wr_din          (hdmi_pixel_in),
        .wr_full         (hdmi_fifo_full),
        .wr_almost_full  (),
        .rd_clk          (clk_core),
        .rd_rst_n        (rst_core_n),        // BUG 3 FIX
        .rd_en           (ingest_fifo_rd_en_int),
        .rd_dout         (ingest_fifo_rd_dout),
        .rd_empty        (ingest_fifo_rd_empty),
        .rd_almost_empty ()
    );

    // FWFT handshake
    wire ing_s_axis_tvalid  = !ingest_fifo_rd_empty;
    wire ing_s_axis_tready_int;
    assign ingest_fifo_rd_en_int = ing_s_axis_tvalid && ing_s_axis_tready_int;

    // TLAST generation (pixel-count based)
    reg [19:0] pix_cnt_ingest;
    reg        ing_tlast_r;
    always @(posedge clk_core or negedge rst_core_n) begin
        if (!rst_core_n) begin
            pix_cnt_ingest <= 20'd0;
            ing_tlast_r    <= 1'b0;
        end else if (ingest_fifo_rd_en_int) begin
            if (pix_cnt_ingest == (WIDTH * HEIGHT - 1)) begin
                pix_cnt_ingest <= 20'd0;
                ing_tlast_r    <= 1'b1;
            end else begin
                pix_cnt_ingest <= pix_cnt_ingest + 1;
                ing_tlast_r    <= (pix_cnt_ingest == (WIDTH * HEIGHT - 2));
            end
        end else begin
            ing_tlast_r <= 1'b0;
        end
    end

    // =========================================================================
    // Dncnn Ingestor (Layer 0)
    // =========================================================================
    wire [OC*8-1:0] ing_m_axis_tdata;
    wire             ing_m_axis_tvalid;
    wire             ing_m_axis_tready;
    wire             ing_m_axis_tlast;

    assign ing_pixel_out_valid = ing_m_axis_tvalid && ing_m_axis_tready;

    wire [7:0] ing_weight_addr_8b;
    assign ing_weight_addr = {{4{1'b0}}, ing_weight_addr_8b};

    dncnn_ingestor #(
        .WIDTH  (WIDTH),
        .HEIGHT (HEIGHT),
        .OC     (OC)
    ) u_ingestor (
        .clk              (clk_core),
        .rst_n            (rst_core_n),       // BUG 3 FIX
        .s_axis_tdata     (ingest_fifo_rd_dout),
        .s_axis_tvalid    (ing_s_axis_tvalid && (reg_layer_type == 2'd0) && eng_busy_r),
        .s_axis_tready    (ing_s_axis_tready_int),
        .s_axis_tlast     (ing_tlast_r),
        .weight_addr      (ing_weight_addr_8b),
        .weight_data      (weight_data_a[OC*8-1:0]),
        .bias_data        (bias_flat),
        .m_axis_tdata     (ing_m_axis_tdata),
        .m_axis_tvalid    (ing_m_axis_tvalid),
        .m_axis_tready    (ing_m_axis_tready),
        .m_axis_tlast     (ing_m_axis_tlast),
        .done             (ing_done),
        .top_row_active   (ing_top_row_active)  // BUG 2 FIX
    );

    // =========================================================================
    // Dncnn Workhorse (Layers 1-15)
    // NOTE: dncnn_workhorse_fixed.v must be used (Bug 1 fix inside the FSM).
    // The weight_addr issued by the workhorse is already 2 cycles early;
    // this top connects it directly to rd_addr_b with no extra register.
    // =========================================================================
    wire [OC*8-1:0] wh_m_axis_tdata;
    wire             wh_m_axis_tvalid;
    wire             wh_m_axis_tready;
    wire             wh_m_axis_tlast;
    wire             wh_s_axis_tready_int;

    assign wh_pixel_out_valid = wh_m_axis_tvalid && wh_m_axis_tready;

    wire [10:0] wh_weight_addr_11b;
    assign wh_weight_addr = {{1{1'b0}}, wh_weight_addr_11b};

    dncnn_workhorse #(        // <-- use dncnn_workhorse_fixed.v as this module
        .WIDTH   (WIDTH),
        .HEIGHT  (HEIGHT),
        .IC      (IC),
        .OC      (OC),
        .IC_PAR  (8)
    ) u_workhorse (
        .clk              (clk_core),
        .rst_n            (rst_core_n),       // BUG 3 FIX
        .s_axis_tdata     (s_axis_feat_tdata[IC*8-1:0]),
        .s_axis_tvalid    (s_axis_feat_tvalid && (reg_layer_type == 2'd1) && eng_busy_r),
        .s_axis_tready    (wh_s_axis_tready_int),
        .s_axis_tlast     (s_axis_feat_tlast),
        .weight_addr      (wh_weight_addr_11b),
        .weight_data_p    (weight_data_b),
        .bias_data        (bias_flat),
        .m_axis_tdata     (wh_m_axis_tdata),
        .m_axis_tvalid    (wh_m_axis_tvalid),
        .m_axis_tready    (wh_m_axis_tready),
        .m_axis_tlast     (wh_m_axis_tlast),
        .done             (wh_done),
        .top_row_active   (wh_top_row_active), // BUG 2 FIX
        .rd_data_b_valid  (wh_rd_data_b_valid) //NEW THINGY 
    );

    // =========================================================================
    // Dncnn Finisher (Layer 16)
    // =========================================================================
    wire [7:0]  fin_m_axis_tdata;
    wire        fin_m_axis_tvalid;
    wire        fin_m_axis_tready;
    wire        fin_m_axis_tlast;
    wire        fin_feat_tready_int;
    wire        fin_raw_tready_int;
    wire wh_rd_data_b_valid;

    assign fin_pixel_out_valid = fin_m_axis_tvalid && fin_m_axis_tready;

    wire [7:0] fin_weight_addr_8b;
    assign fin_weight_addr = {{4{1'b0}}, fin_weight_addr_8b};

    dncnn_finisher #(
        .WIDTH  (WIDTH),
        .HEIGHT (HEIGHT),
        .IC     (IC)
    ) u_finisher (
        .clk                (clk_core),
        .rst_n              (rst_core_n),       // BUG 3 FIX
        .s_axis_feat_tdata  (s_axis_feat_tdata[IC*8-1:0]),
        .s_axis_feat_tvalid (s_axis_feat_tvalid && (reg_layer_type == 2'd2) && eng_busy_r),
        .s_axis_feat_tready (fin_feat_tready_int),
        .s_axis_feat_tlast  (s_axis_feat_tlast),
        .s_axis_raw_tdata   (s_axis_raw_tdata),
        .s_axis_raw_tvalid  (s_axis_raw_tvalid && (reg_layer_type == 2'd2) && eng_busy_r),
        .s_axis_raw_tready  (fin_raw_tready_int),
        .s_axis_raw_tlast   (s_axis_raw_tlast),
        .weight_addr        (fin_weight_addr_8b),
        .weight_data        (weight_data_a[IC*8-1:0]),
        .bias_data          (bias_reg[0]),
        .m_axis_tdata       (fin_m_axis_tdata),
        .m_axis_tvalid      (fin_m_axis_tvalid),
        .m_axis_tready      (fin_m_axis_tready),
        .m_axis_tlast       (fin_m_axis_tlast),
        .done               (fin_done),
        .top_row_active     (fin_top_row_active)  // BUG 2 FIX
    );

    // =========================================================================
    // AXI4-Stream TREADY mux
    // =========================================================================
    assign s_axis_feat_tready =
        (reg_layer_type == 2'd1) ? wh_s_axis_tready_int :
        (reg_layer_type == 2'd2) ? fin_feat_tready_int   : 1'b0;

    assign s_axis_raw_tready =
        (reg_layer_type == 2'd2) ? fin_raw_tready_int : 1'b0;

    // =========================================================================
    // AXI4-Stream output mux (to VDMA)
    // =========================================================================
    assign m_axis_out_tdata =
        (reg_layer_type == 2'd0) ? {{64{1'b0}}, ing_m_axis_tdata} :
        (reg_layer_type == 2'd1) ? wh_m_axis_tdata                :
        (reg_layer_type == 2'd2) ? {120'd0, fin_m_axis_tdata}     : 128'd0;

    assign m_axis_out_tvalid =
        (reg_layer_type == 2'd0) ? ing_m_axis_tvalid :
        (reg_layer_type == 2'd1) ? wh_m_axis_tvalid  :
        (reg_layer_type == 2'd2) ? fin_m_axis_tvalid  : 1'b0;

    assign m_axis_out_tlast =
        (reg_layer_type == 2'd0) ? ing_m_axis_tlast :
        (reg_layer_type == 2'd1) ? wh_m_axis_tlast  :
        (reg_layer_type == 2'd2) ? fin_m_axis_tlast  : 1'b0;

    assign ing_m_axis_tready = (reg_layer_type == 2'd0) ? m_axis_out_tready : 1'b0;
    assign wh_m_axis_tready  = (reg_layer_type == 2'd1) ? m_axis_out_tready : 1'b0;

    // Finisher output also goes to eject FIFO (BUG 4 FIX); use OR'd tready
    wire fin_eject_ready;   // from eject FIFO wr_full inverse
    // fin_m_axis_tready must accept whenever either sink (VDMA or FIFO) is ready
    // Strategy: write to eject FIFO whenever finisher outputs, independently of VDMA
    assign fin_m_axis_tready = (reg_layer_type == 2'd2) ? m_axis_out_tready : 1'b0;

    // =========================================================================
    // BUG 4 FIX: Eject FIFO - finisher 8-bit output -> 24-bit RGB -> HDMI TX
    //
    // The finisher outputs one clean luma byte per pixel.
    // We replicate Y->RGB ({y,y,y}) for the rgb2dvi IP.
    // Write into the eject FIFO whenever the finisher produces a valid pixel.
    // =========================================================================
    wire        eject_wr_full;
    wire [23:0] eject_wr_din  = {fin_m_axis_tdata, fin_m_axis_tdata, fin_m_axis_tdata};
    wire        eject_wr_en   = fin_m_axis_tvalid && (reg_layer_type == 2'd2) && !eject_wr_full;

    // Read side: driven by the hdmi_pixel_out_ready signal from rgb2dvi
    wire        eject_rd_empty;
    assign hdmi_pixel_out_valid = !eject_rd_empty;
    wire        eject_rd_en     = !eject_rd_empty && hdmi_pixel_out_ready;

    async_fifo_eject #(
        .DATA_WIDTH (24)          // 24-bit RGB
    ) u_fifo_eject (
        .wr_clk         (clk_core),
        .wr_rst_n       (rst_core_n),       // BUG 3 FIX: sync'd reset
        .wr_en          (eject_wr_en),
        .wr_din         (eject_wr_din),
        .wr_full        (eject_wr_full),
        .wr_almost_full (),
        .rd_clk         (clk_pixel_out),
        .rd_rst_n       (rst_pixel_out_n),  // BUG 3 FIX: sync'd reset
        .rd_en          (eject_rd_en),
        .rd_dout        (hdmi_pixel_out),
        .rd_empty       (eject_rd_empty),
        .rd_almost_empty()
    );

    // =========================================================================
    // Weight BRAM address mux
    // =========================================================================
    assign weight_addr_a =
        (reg_layer_type == 2'd0) ? ing_weight_addr :
        (reg_layer_type == 2'd2) ? fin_weight_addr : 12'd0;

    assign weight_addr_b =
        (reg_layer_type == 2'd1) ? wh_weight_addr : 12'd0;

    // =========================================================================
    // Weight BRAM
    // =========================================================================
    weight_bram u_wbram (
        .clk       (clk_core),
        .rst_n     (rst_core_n),
        .wr_addr   (axil_weight_wr_addr),
        .wr_data   (axil_weight_wr_data),
        .wr_en     (axil_weight_wr_en && !eng_busy_r),
        .rd_addr_a (weight_addr_a),
        .rd_data_a (weight_data_a),
        .rd_addr_b (weight_addr_b),
        .rd_data_b (weight_data_b),
        .rd_data_b_valid (wh_rd_data_b_valid)   // add this
    );

    // =========================================================================
    // AXI4-Lite Slave
    // =========================================================================
    axilite_slave u_axil (
        .clk              (clk_core),
        .rst_n            (rst_core_n),
        .s_axil_awaddr    (s_axil_awaddr),
        .s_axil_awvalid   (s_axil_awvalid),
        .s_axil_awready   (s_axil_awready),
        .s_axil_wdata     (s_axil_wdata),
        .s_axil_wstrb     (s_axil_wstrb),
        .s_axil_wvalid    (s_axil_wvalid),
        .s_axil_wready    (s_axil_wready),
        .s_axil_bresp     (s_axil_bresp),
        .s_axil_bvalid    (s_axil_bvalid),
        .s_axil_bready    (s_axil_bready),
        .s_axil_araddr    (s_axil_araddr),
        .s_axil_arvalid   (s_axil_arvalid),
        .s_axil_arready   (s_axil_arready),
        .s_axil_rdata     (s_axil_rdata),
        .s_axil_rresp     (s_axil_rresp),
        .s_axil_rvalid    (s_axil_rvalid),
        .s_axil_rready    (s_axil_rready),
        .start_pulse      (reg_start_pulse),
        .layer_type       (reg_layer_type),
        .img_width        (reg_img_width),
        .img_height       (reg_img_height),
        .irq_enable       (reg_irq_enable),
        .weight_wr_addr   (axil_weight_wr_addr),
        .weight_wr_data   (axil_weight_wr_data),
        .weight_wr_en     (axil_weight_wr_en),
        .bias_wr_addr     (axil_bias_wr_addr),
        .bias_wr_data     (axil_bias_wr_data),
        .bias_wr_en       (axil_bias_wr_en),
        .engine_busy      (eng_busy_r),
        .engine_done      (engine_done),
        .engine_error     (engine_error),
        .pixel_cnt        (pixel_cnt),
        .done_irq         (done_irq)
    );

endmodule
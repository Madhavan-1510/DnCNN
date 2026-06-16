// tlast_gen.v
// Generates AXI4-Stream TLAST at end of each line (col = WIDTH-1)
// Add as RTL module reference in block design

module tlast_gen #(
    parameter WIDTH  = 640,
    parameter HEIGHT = 480
)(
    input  wire clk,
    input  wire rst_n,
    input  wire pix_valid,    // connect to DVI2RGB vid_pVDE
    output wire pix_tlast,    // TLAST to Broadcaster and VDMA_2
    output wire frame_start   // pulses high on pixel 0,0 of each frame
);
    reg [9:0] col_cnt;
    reg [8:0] row_cnt;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            col_cnt <= 0;
            row_cnt <= 0;
        end else if (pix_valid) begin
            if (col_cnt == WIDTH - 1) begin
                col_cnt <= 0;
                if (row_cnt == HEIGHT - 1) row_cnt <= 0;
                else row_cnt <= row_cnt + 1;
            end else begin
                col_cnt <= col_cnt + 1;
            end
        end
    end

    assign pix_tlast   = pix_valid && (col_cnt == WIDTH - 1);
    assign frame_start = pix_valid && (col_cnt == 0) && (row_cnt == 0);
endmodule
// True dual-port RAM for SPI buffer, Xilinx BRAM-inference friendly.
// 512 x 32-bit (2048 bytes) with per-byte write enables.
// Port A: CPU access via TL-UL adapter
// Port B: SPI core access (TX read / RX write)

module spi_buf_ram #(
  parameter int unsigned Depth = 512,
  localparam int unsigned Aw = $clog2(Depth)
) (
  input  logic        clk_i,

  // Port A (CPU)
  input  logic        a_req_i,
  input  logic        a_we_i,
  input  logic [Aw-1:0] a_addr_i,
  input  logic [31:0] a_wdata_i,
  input  logic [3:0]  a_be_i,
  output logic [31:0] a_rdata_o,

  // Port B (SPI core)
  input  logic        b_req_i,
  input  logic        b_we_i,
  input  logic [Aw-1:0] b_addr_i,
  input  logic [31:0] b_wdata_i,
  input  logic [3:0]  b_be_i,
  output logic [31:0] b_rdata_o
);

  (* ram_style = "block" *) logic [31:0] mem [0:Depth-1];

  // Port A
  always_ff @(posedge clk_i) begin
    if (a_req_i) begin
      if (a_we_i) begin
        if (a_be_i[0]) mem[a_addr_i][ 7: 0] <= a_wdata_i[ 7: 0];
        if (a_be_i[1]) mem[a_addr_i][15: 8] <= a_wdata_i[15: 8];
        if (a_be_i[2]) mem[a_addr_i][23:16] <= a_wdata_i[23:16];
        if (a_be_i[3]) mem[a_addr_i][31:24] <= a_wdata_i[31:24];
      end
      a_rdata_o <= mem[a_addr_i];
    end
  end

  // Port B
  always_ff @(posedge clk_i) begin
    if (b_req_i) begin
      if (b_we_i) begin
        if (b_be_i[0]) mem[b_addr_i][ 7: 0] <= b_wdata_i[ 7: 0];
        if (b_be_i[1]) mem[b_addr_i][15: 8] <= b_wdata_i[15: 8];
        if (b_be_i[2]) mem[b_addr_i][23:16] <= b_wdata_i[23:16];
        if (b_be_i[3]) mem[b_addr_i][31:24] <= b_wdata_i[31:24];
      end
      b_rdata_o <= mem[b_addr_i];
    end
  end

endmodule

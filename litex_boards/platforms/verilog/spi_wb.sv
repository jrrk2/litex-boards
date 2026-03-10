// Wishbone-native SPI controller for LiteX.
// Replaces TileLink bus layer from original OpenTitan spi.sv.
// Keeps spi_core and spi_buf_ram unchanged.
//
// 8KB address space:
//   0x0000-0x0FFF  Registers (bit 12 = 0)
//   0x1000-0x17FF  TX buffer (bit 12 = 1, bit 11 = 0) — 2KB
//   0x1800-0x1FFF  RX buffer (bit 12 = 1, bit 11 = 1) — 2KB

module spi_wb #(
  parameter int unsigned CSWidth  = 4,
  parameter int unsigned BufDepth = 2048
) (
  input              clk_i,
  input              rst_i,       // active-high (LiteX convention)

  // Wishbone slave
  input  [12:0]      wb_adr_i,    // byte address within 8KB
  input  [31:0]      wb_dat_i,
  output [31:0]      wb_dat_o,
  input  [3:0]       wb_sel_i,
  input              wb_we_i,
  input              wb_cyc_i,
  input              wb_stb_i,
  output             wb_ack_o,

  // Interrupt
  output             irq_o,

  // SPI pins
  output             spi_clk_o,
  output             spi_copi_o,
  input              spi_cipo_i,
  output [CSWidth-1:0] spi_cs_o
);

  localparam int unsigned BufAw    = $clog2(BufDepth);   // 11
  localparam int unsigned BufWords = BufDepth / 4;       // 512
  localparam int unsigned BufWaw   = $clog2(BufWords);   // 9

  wire rst_ni = ~rst_i;

  // ========================================================================
  // Address decode
  // ========================================================================
  wire is_reg = ~wb_adr_i[12];
  wire is_tx  =  wb_adr_i[12] & ~wb_adr_i[11];
  wire is_rx  =  wb_adr_i[12] &  wb_adr_i[11];
  wire [5:0] reg_addr = wb_adr_i[7:2];  // register word offset

  // ========================================================================
  // Wishbone handshake
  // ========================================================================
  // Registers: 1-cycle ack
  // Buffer write: 1-cycle ack
  // Buffer read: 2-cycle ack (BRAM read latency)

  wire wb_req = wb_cyc_i & wb_stb_i;

  reg ack_q;
  reg buf_rd_pending;

  always @(posedge clk_i) begin
    if (rst_i) begin
      ack_q          <= 1'b0;
      buf_rd_pending <= 1'b0;
    end else begin
      ack_q <= 1'b0;

      if (buf_rd_pending) begin
        ack_q          <= 1'b1;
        buf_rd_pending <= 1'b0;
      end else if (wb_req & ~ack_q) begin
        if (is_reg | wb_we_i) begin
          ack_q <= 1'b1;          // 1-cycle ack
        end else begin
          buf_rd_pending <= 1'b1; // buffer read, wait for BRAM
        end
      end
    end
  end

  assign wb_ack_o = ack_q;

  // Register write pulse (single cycle, one cycle before ack)
  wire reg_wr = wb_req & wb_we_i & is_reg & ~ack_q & ~buf_rd_pending;

  // ========================================================================
  // Register file
  // ========================================================================
  //
  // Offset  Name         Bits                          Access
  // 0x00    INTR_STATE   [4] complete                  R/W1C
  // 0x04    INTR_ENABLE  [4] complete                  RW
  // 0x0C    CFG          [15:0] half_clk, [29] msb_first, [30] cpha, [31] cpol  RW
  // 0x10    CONTROL      [31] sw_rst                   WO (self-clearing)
  // 0x14    STATUS       [26] idle                     RO
  // 0x18    START        [10:0] byte_count             WO (triggers transfer)
  // 0x28    CS           [CSWidth-1:0]                 RW

  reg [31:0] cfg_reg;
  reg [CSWidth-1:0] cs_reg;
  reg intr_complete;
  reg intr_enable_complete;

  // Extract CFG fields
  wire [15:0] spi_half_clk_period = cfg_reg[15:0];
  wire        spi_msb_first       = cfg_reg[29];
  wire        spi_cpha            = cfg_reg[30];
  wire        spi_cpol            = cfg_reg[31];

  // SPI core interface signals
  wire [7:0]  spi_data_in, spi_data_out;
  wire        spi_data_in_valid, spi_data_in_ready;
  wire        spi_data_out_valid, spi_data_out_ready;
  wire        spi_start, spi_idle;
  wire [10:0] spi_byte_count;

  // START: write triggers transfer, byte_count from write data
  assign spi_byte_count = wb_dat_i[10:0];
  assign spi_start      = reg_wr & (reg_addr == 6'd6) & spi_idle;

  // SW reset: write CONTROL[31]
  wire sw_reset = reg_wr & (reg_addr == 6'd4) & wb_dat_i[31];

  // ========================================================================
  // Interrupt: edge detect on spi_idle rising (transfer complete)
  // ========================================================================
  reg spi_idle_q;
  always @(posedge clk_i) begin
    if (rst_i) spi_idle_q <= 1'b1;
    else       spi_idle_q <= spi_idle;
  end
  wire event_complete = spi_idle & ~spi_idle_q;

  wire w1c_complete = reg_wr & (reg_addr == 6'd0) & wb_dat_i[4];

  always @(posedge clk_i) begin
    if (rst_i)
      intr_complete <= 1'b0;
    else if (event_complete)
      intr_complete <= 1'b1;   // event has priority over W1C
    else if (w1c_complete)
      intr_complete <= 1'b0;
  end

  assign irq_o = intr_complete & intr_enable_complete;

  // ========================================================================
  // Register writes
  // ========================================================================
  always @(posedge clk_i) begin
    if (rst_i) begin
      cfg_reg              <= 32'h0;
      cs_reg               <= {CSWidth{1'b1}};  // all CS de-asserted
      intr_enable_complete <= 1'b0;
    end else if (reg_wr) begin
      case (reg_addr)
        6'd1:  intr_enable_complete <= wb_dat_i[4];         // INTR_ENABLE
        6'd3:  cfg_reg              <= wb_dat_i;             // CFG
        6'd10: cs_reg               <= wb_dat_i[CSWidth-1:0]; // CS
        default: ;
      endcase
    end
  end

  // ========================================================================
  // Register reads
  // ========================================================================
  reg [31:0] reg_rdata;
  always @(*) begin
    reg_rdata = 32'h0;
    case (reg_addr)
      6'd0:  reg_rdata = {27'h0, intr_complete, 4'h0};       // INTR_STATE
      6'd1:  reg_rdata = {27'h0, intr_enable_complete, 4'h0}; // INTR_ENABLE
      6'd3:  reg_rdata = cfg_reg;                              // CFG
      6'd5:  reg_rdata = {5'h0, spi_idle, 26'h0};             // STATUS
      6'd10: reg_rdata = {{(32-CSWidth){1'b0}}, cs_reg};      // CS
      default: reg_rdata = 32'h0;
    endcase
  end

  // Registered for timing (available when ack fires)
  reg [31:0] reg_rdata_q;
  always @(posedge clk_i) reg_rdata_q <= reg_rdata;

  // CS output
  assign spi_cs_o = cs_reg;

  // ========================================================================
  // Buffer access — CPU side (port A of BRAMs)
  // ========================================================================
  wire tx_cpu_req   = wb_req & is_tx & ~ack_q & ~buf_rd_pending;
  wire tx_cpu_we    = wb_we_i;
  wire [BufWaw-1:0] tx_cpu_addr  = wb_adr_i[BufWaw+1:2];
  wire [31:0]       tx_cpu_wdata = wb_dat_i;
  wire [3:0]        tx_cpu_be    = wb_sel_i;
  wire [31:0]       tx_cpu_rdata;

  wire rx_cpu_req   = wb_req & is_rx & ~ack_q & ~buf_rd_pending;
  wire rx_cpu_we    = wb_we_i;
  wire [BufWaw-1:0] rx_cpu_addr  = wb_adr_i[BufWaw+1:2];
  wire [31:0]       rx_cpu_wdata = wb_dat_i;
  wire [3:0]        rx_cpu_be    = wb_sel_i;
  wire [31:0]       rx_cpu_rdata;

  // Remember which region was accessed for data output mux
  reg is_reg_q, is_tx_q;
  always @(posedge clk_i) begin
    if (wb_req & ~ack_q & ~buf_rd_pending) begin
      is_reg_q <= is_reg;
      is_tx_q  <= is_tx;
    end
  end

  // Data output mux
  reg [31:0] wb_dat_o_mux;
  always @(*) begin
    if (is_reg_q)
      wb_dat_o_mux = reg_rdata_q;
    else if (is_tx_q)
      wb_dat_o_mux = tx_cpu_rdata;
    else
      wb_dat_o_mux = rx_cpu_rdata;
  end
  assign wb_dat_o = wb_dat_o_mux;

  // ========================================================================
  // Transfer byte counter (from original spi.sv)
  // ========================================================================
  logic [BufAw-1:0] buf_byte_addr;

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni)
      buf_byte_addr <= '0;
    else if (spi_start)
      buf_byte_addr <= '0;
    else if (~spi_idle && spi_data_in_ready && spi_data_in_valid)
      buf_byte_addr <= buf_byte_addr + 1'b1;
  end

  // ========================================================================
  // TX path: SPI core reads from TX BRAM (port B)
  // ========================================================================
  logic [BufWaw-1:0] tx_word_addr;
  logic [1:0]        buf_byte_lane;
  assign tx_word_addr  = buf_byte_addr[BufAw-1:2];
  assign buf_byte_lane = buf_byte_addr[1:0];

  logic [1:0] tx_byte_lane_q;
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) tx_byte_lane_q <= '0;
    else         tx_byte_lane_q <= buf_byte_lane;
  end

  logic [31:0] tx_bram_rdata;
  logic [7:0]  tx_byte_from_buf;
  always_comb begin
    case (tx_byte_lane_q)
      2'd0: tx_byte_from_buf = tx_bram_rdata[ 7: 0];
      2'd1: tx_byte_from_buf = tx_bram_rdata[15: 8];
      2'd2: tx_byte_from_buf = tx_bram_rdata[23:16];
      2'd3: tx_byte_from_buf = tx_bram_rdata[31:24];
    endcase
  end

  // TX fetch FSM: issue SRAM read, wait 1 cycle, present data to core.
  logic tx_data_valid;
  logic tx_fetch_pending;

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni)
      tx_fetch_pending <= 1'b0;
    else if (spi_start)
      tx_fetch_pending <= 1'b1;  // pre-fetch first byte immediately
    else if (~spi_idle && !tx_data_valid && !tx_fetch_pending)
      tx_fetch_pending <= 1'b1;
    else
      tx_fetch_pending <= 1'b0;
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni)
      tx_data_valid <= 1'b0;
    else if (spi_start || spi_idle)
      tx_data_valid <= 1'b0;
    else if (tx_fetch_pending)
      tx_data_valid <= 1'b1;
    else if (spi_data_in_ready & tx_data_valid)
      tx_data_valid <= 1'b0;
  end

  assign spi_data_in       = tx_byte_from_buf;
  assign spi_data_in_valid = tx_data_valid;

  // TX BRAM port B: read-only from SPI core perspective
  logic tx_b_req;
  assign tx_b_req = tx_fetch_pending;

  // ========================================================================
  // RX path: SPI core writes to RX BRAM (port B)
  // ========================================================================
  assign spi_data_out_ready = ~spi_idle;

  // Separate RX byte counter
  logic [BufAw-1:0] rx_count;

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni)
      rx_count <= '0;
    else if (spi_start)
      rx_count <= '0;
    else if (spi_data_out_valid & spi_data_out_ready)
      rx_count <= rx_count + 1'b1;
  end

  logic        rx_wr_pending;
  logic [BufAw-1:0] rx_byte_addr;
  logic [7:0]  rx_byte_data;

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      rx_wr_pending <= 1'b0;
      rx_byte_addr  <= '0;
      rx_byte_data  <= '0;
    end else if (spi_data_out_valid & spi_data_out_ready) begin
      rx_wr_pending <= 1'b1;
      rx_byte_addr  <= rx_count;
      rx_byte_data  <= spi_data_out;
    end else begin
      rx_wr_pending <= 1'b0;
    end
  end

  // RX BRAM port B signals
  logic               rx_b_req;
  logic [BufWaw-1:0]  rx_b_addr;
  logic [31:0]        rx_b_wdata;
  logic [3:0]         rx_b_be;

  assign rx_b_req  = rx_wr_pending;
  assign rx_b_addr = rx_byte_addr[BufAw-1:2];

  always_comb begin
    rx_b_wdata = '0;
    rx_b_be    = '0;
    case (rx_byte_addr[1:0])
      2'd0: begin rx_b_wdata = {24'b0, rx_byte_data};       rx_b_be = 4'b0001; end
      2'd1: begin rx_b_wdata = {16'b0, rx_byte_data, 8'b0}; rx_b_be = 4'b0010; end
      2'd2: begin rx_b_wdata = {8'b0, rx_byte_data, 16'b0}; rx_b_be = 4'b0100; end
      2'd3: begin rx_b_wdata = {rx_byte_data, 24'b0};       rx_b_be = 4'b1000; end
    endcase
  end

  // ========================================================================
  // TX BRAM — true dual-port block RAM
  //   Port A: CPU (read/write via Wishbone)
  //   Port B: SPI core (read only)
  // ========================================================================
  spi_buf_ram #(
    .Depth (BufWords)
  ) u_tx_ram (
    .clk_i,
    .a_req_i   (tx_cpu_req),
    .a_we_i    (tx_cpu_we),
    .a_addr_i  (tx_cpu_addr),
    .a_wdata_i (tx_cpu_wdata),
    .a_be_i    (tx_cpu_be),
    .a_rdata_o (tx_cpu_rdata),
    .b_req_i   (tx_b_req),
    .b_we_i    (1'b0),
    .b_addr_i  (tx_word_addr),
    .b_wdata_i (32'b0),
    .b_be_i    (4'b0),
    .b_rdata_o (tx_bram_rdata)
  );

  // ========================================================================
  // RX BRAM — true dual-port block RAM
  //   Port A: CPU (read/write via Wishbone)
  //   Port B: SPI core (write only)
  // ========================================================================
  logic [31:0] rx_bram_unused_rdata;

  spi_buf_ram #(
    .Depth (BufWords)
  ) u_rx_ram (
    .clk_i,
    .a_req_i   (rx_cpu_req),
    .a_we_i    (rx_cpu_we),
    .a_addr_i  (rx_cpu_addr),
    .a_wdata_i (rx_cpu_wdata),
    .a_be_i    (rx_cpu_be),
    .a_rdata_o (rx_cpu_rdata),
    .b_req_i   (rx_b_req),
    .b_we_i    (1'b1),
    .b_addr_i  (rx_b_addr),
    .b_wdata_i (rx_b_wdata),
    .b_be_i    (rx_b_be),
    .b_rdata_o (rx_bram_unused_rdata)
  );

  // ========================================================================
  // SPI core
  // ========================================================================
  spi_core u_spi_core (
    .clk_i,
    .rst_ni,
    .sw_reset_i       (sw_reset),
    .data_in_i        (spi_data_in),
    .data_in_valid_i  (spi_data_in_valid),
    .data_in_ready_o  (spi_data_in_ready),
    .data_out_o       (spi_data_out),
    .data_out_valid_o (spi_data_out_valid),
    .data_out_ready_i (spi_data_out_ready),
    .start_i          (spi_start),
    .byte_count_i     (spi_byte_count),
    .idle_o           (spi_idle),
    .cpol_i           (spi_cpol),
    .cpha_i           (spi_cpha),
    .msb_first_i      (spi_msb_first),
    .half_clk_period_i(spi_half_clk_period),
    .copi_idle_i      (1'b0),
    .spi_copi_o,
    .spi_cipo_i,
    .spi_clk_o
  );

  // Tie off unused
  logic unused_rx_bram;
  assign unused_rx_bram = |rx_bram_unused_rdata;

endmodule

"""
LiteX wrapper for Wishbone-native SPI controller (spi_wb.sv).
Instantiates the OpenTitan-derived SPI engine with buffer BRAMs
as a Verilog black-box, exposing a Wishbone slave bus and IRQ.
"""

import os
from types import SimpleNamespace

from migen import *
from litex.gen import *
from litex.soc.interconnect import wishbone

_verilog_dir = os.path.join(os.path.dirname(__file__), "..", "platforms", "verilog")

class SPIHost(LiteXModule):
    def __init__(self, platform, pads, sys_clk_freq):
        # 8KB address space: 2K words (13-bit byte address / 4)
        self.bus = wishbone.Interface(data_width=32, adr_width=11)
        _irq = Signal()
        # LiteX IRQ system expects module.ev.irq — provide minimal shim
        # (interrupt enable/clear managed entirely in Verilog register file)
        self.ev = SimpleNamespace(irq=_irq)

        platform.add_source(os.path.join(_verilog_dir, "spi_wb.sv"))
        platform.add_source(os.path.join(_verilog_dir, "spi_core.sv"))
        platform.add_source(os.path.join(_verilog_dir, "spi_buf_ram.sv"))

        self.specials += Instance("spi_wb",
            # Parameters
            p_CSWidth  = 1,
            p_BufDepth = 2048,
            # Clock / reset
            i_clk_i    = ClockSignal(),
            i_rst_i    = ResetSignal(),
            # Wishbone: adr is word address, prepend 2 zero bits for byte address
            i_wb_adr_i = Cat(Signal(2, reset=0), self.bus.adr),
            i_wb_dat_i = self.bus.dat_w,
            o_wb_dat_o = self.bus.dat_r,
            i_wb_sel_i = self.bus.sel,
            i_wb_we_i  = self.bus.we,
            i_wb_cyc_i = self.bus.cyc,
            i_wb_stb_i = self.bus.stb,
            o_wb_ack_o = self.bus.ack,
            # Interrupt
            o_irq_o    = _irq,
            # SPI pins
            o_spi_clk_o  = pads.clk,
            o_spi_copi_o = pads.mosi,
            i_spi_cipo_i = pads.miso,
            o_spi_cs_o   = pads.cs_n,
        )

#!/usr/bin/env python3

#
# This file is part of LiteX-Boards.
#
# Support for the lowRISC Sonata board.
# FPGA: Xilinx Artix-7 XC7A50TCSG324-2
# https://github.com/lowRISC/sonata-system
#

from litex.build.generic_platform import *
from litex.build.xilinx import Xilinx7SeriesPlatform
from litex.build.openocd import OpenOCD

# IOs ----------------------------------------------------------------------------------------------

_io = [
    # Clk / Rst.
    ("clk25", 0, Pins("P15"), IOStandard("LVCMOS33")),

    # Serial (active UART header).
    ("serial", 0,
        Subsignal("tx", Pins("C17")),
        Subsignal("rx", Pins("D18")),
        IOStandard("LVCMOS33"),
    ),

    # SPIFlash.
    ("spiflash", 0,
        Subsignal("cs_n", Pins("D10")),
        Subsignal("clk",  Pins("A8")),
        Subsignal("mosi", Pins("C11")),
        Subsignal("miso", Pins("C10")),
        IOStandard("LVCMOS33"),
    ),
    ("spiflash4x", 0,
        Subsignal("cs_n", Pins("D10")),
        Subsignal("clk",  Pins("A8")),
        Subsignal("dq",   Pins("C11 C10 A10 A9")),
        IOStandard("LVCMOS33"),
    ),

    # HyperRAM.
    ("hyperram", 0,
        Subsignal("dq",    Pins("B1 E2 H1 A1 E1 B2 C1 D2"), IOStandard("LVCMOS18")),
        Subsignal("rwds",  Pins("F1"),                        IOStandard("LVCMOS18")),
        Subsignal("cs_n",  Pins("J2"),                        IOStandard("LVCMOS18")),
        Subsignal("rst_n", Pins("C2"),                        IOStandard("LVCMOS18")),
        Subsignal("clk",   Pins("H2"),                        IOStandard("LVCMOS18")),
        Misc("SLEW=FAST"),
    ),

    # MicroSD card slot.
    ("spisdcard", 0,
        Subsignal("clk",  Pins("U6")),
        Subsignal("mosi", Pins("R8"), Misc("PULLUP True")),
        Subsignal("cs_n", Pins("T8"), Misc("PULLUP True")),
        Subsignal("miso", Pins("V4"), Misc("PULLUP True")),
        Misc("SLEW=FAST"),
        IOStandard("LVCMOS33"),
    ),
    ("sdcard", 0,
        Subsignal("data", Pins("V4 R7 V5 T8"), Misc("PULLUP True")),
        Subsignal("cmd",  Pins("R8"),            Misc("PULLUP True")),
        Subsignal("clk",  Pins("U6")),
        Subsignal("cd",   Pins("B3"),            IOStandard("LVCMOS18")),
        Misc("SLEW=FAST"),
        IOStandard("LVCMOS33"),
    ),

    # KSZ8851SNL Ethernet (SPI interface, active-low IRQ).
    # Pin assignments from Sonata board (LVCMOS18 bank).
    ("spi_eth", 0,
        Subsignal("clk",  Pins("E3")),
        Subsignal("mosi", Pins("D5")),
        Subsignal("miso", Pins("D4")),
        Subsignal("cs_n", Pins("H5")),
        IOStandard("LVCMOS18"),
    ),
    ("eth_rst_n", 0, Pins("J5"), IOStandard("LVCMOS18")),
    ("eth_irq_n", 0, Pins("H6"), IOStandard("LVCMOS18"), Misc("PULLUP True")),

    # JTAG (external 20-pin ARM connector).
    ("jtag", 0,
        Subsignal("tck", Pins("E15")),
        Subsignal("tms", Pins("H15")),
        Subsignal("tdi", Pins("G17")),
        Subsignal("tdo", Pins("J14")),
        IOStandard("LVCMOS33"),
    ),

    # User LEDs.
    ("user_led", 0, Pins("B13"), IOStandard("LVCMOS33")),
    ("user_led", 1, Pins("B14"), IOStandard("LVCMOS33")),
    ("user_led", 2, Pins("C12"), IOStandard("LVCMOS33")),
    ("user_led", 3, Pins("B12"), IOStandard("LVCMOS33")),
    ("user_led", 4, Pins("B11"), IOStandard("LVCMOS33")),
    ("user_led", 5, Pins("A11"), IOStandard("LVCMOS33")),
    ("user_led", 6, Pins("F13"), IOStandard("LVCMOS33")),
    ("user_led", 7, Pins("F14"), IOStandard("LVCMOS33")),

    # Selector switches (active-low, LVCMOS18 with pull-ups).
    ("user_sw", 0, Pins("D3"), IOStandard("LVCMOS18"), Misc("PULLUP True")),
    ("user_sw", 1, Pins("F4"), IOStandard("LVCMOS18"), Misc("PULLUP True")),
    ("user_sw", 2, Pins("F3"), IOStandard("LVCMOS18"), Misc("PULLUP True")),
]

_connectors = []

# Platform -----------------------------------------------------------------------------------------

class Platform(Xilinx7SeriesPlatform):
    default_clk_name   = "clk25"
    default_clk_period = 1e9/25e6

    def __init__(self, toolchain="vivado"):
        Xilinx7SeriesPlatform.__init__(self, "xc7a50tcsg324-2", _io, _connectors, toolchain=toolchain)

    def create_programmer(self):
        return OpenOCD("openocd_xc7_ft232.cfg")

    def do_finalize(self, fragment):
        Xilinx7SeriesPlatform.do_finalize(self, fragment)
        self.add_period_constraint(self.lookup_request("clk25", loose=True), 1e9/25e6)
        # JTAG TCK comes from the external ARM connector, not a clock-capable pin.
        jtag = self.lookup_request("jtag", loose=True)
        if jtag is not None:
            self.add_platform_command("set_property CLOCK_DEDICATED_ROUTE FALSE [get_nets {tck}]", tck=jtag.tck)

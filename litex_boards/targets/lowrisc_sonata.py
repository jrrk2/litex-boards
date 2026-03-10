#!/usr/bin/env python3

#
# This file is part of LiteX-Boards.
#
# Target for the lowRISC Sonata board.
# FPGA: Xilinx Artix-7 XC7A50TCSG324-2
# RAM:  Winbond W956D8MBYA HyperRAM (8 MB)
#

from migen import *
from litex.gen import *

from litex_boards.platforms import lowrisc_sonata

from litex.soc.cores.clock import *
from litex.soc.integration.soc      import SoCRegion
from litex.soc.integration.soc_core import *
from litex.soc.integration.builder  import *
from litex.soc.cores.led            import LedChaser
from litex.soc.cores.hyperbus       import HyperRAM
from litex.soc.interconnect         import wishbone
from litex.soc.interconnect.csr     import *

# CRG ----------------------------------------------------------------------------------------------

class _CRG(LiteXModule):
    def __init__(self, platform, sys_clk_freq, with_sys2x=False):
        self.rst      = Signal()
        self.cd_sys   = ClockDomain()

        self.pll = pll = S7PLL(speedgrade=-2)
        self.comb += pll.reset.eq(self.rst)
        pll.register_clkin(platform.request("clk25"), 25e6)
        pll.create_clkout(self.cd_sys, sys_clk_freq)
        if with_sys2x:
            self.cd_sys2x = ClockDomain()
            pll.create_clkout(self.cd_sys2x, 2 * sys_clk_freq)

# BaseSoC ------------------------------------------------------------------------------------------

class BaseSoC(SoCCore):
    BUILD_VERSION = 8  # Increment on each meaningful build.

    def __init__(self, sys_clk_freq=50e6, with_hyperram=False, with_sdcard=False,
                 with_spi_flash=False, with_led_chaser=True, with_spi_eth=False,
                 l2_size=0, flash_cache_size=0, **kwargs):
        platform = lowrisc_sonata.Platform()

        # CRG --------------------------------------------------------------------------------------
        self.crg = _CRG(platform, sys_clk_freq, with_sys2x=with_hyperram)

        # SoCCore ----------------------------------------------------------------------------------
        import datetime
        build_ts = datetime.datetime.now().strftime("%Y%m%d_%H%M")
        ident = f"LiteX Sonata v{self.BUILD_VERSION} {build_ts}"
        SoCCore.__init__(self, platform, sys_clk_freq, ident=ident, **kwargs)

        # HyperRAM ---------------------------------------------------------------------------------
        if with_hyperram:
            hyperram_size = 8 * 1024 * 1024  # 8 MB
            hr_pads = platform.request("hyperram", 0)
            self.hyperram = HyperRAM(hr_pads, sys_clk_freq=sys_clk_freq, clk_ratio="2:1")

            # L2 Cache between bus and HyperRAM
            if l2_size:
                import math
                l2_size = 2**int(math.log2(l2_size))
                l2_cache_data_width = 128  # Wide cache lines (4 words per line)
                wb_hyperram = wishbone.Interface(
                    data_width=self.bus.data_width, address_width=32, addressing="word")
                self.bus.add_slave("main_ram", slave=wb_hyperram,
                    region=SoCRegion(origin=0x40000000, size=hyperram_size, mode="rwx"))
                from litex.soc.interconnect.wishbone import Cache, Converter
                from migen.fhdl.simplify import FullMemoryWE
                # L2 cache with wide slave interface for multi-word cache lines.
                wb_l2 = wishbone.Interface(
                    data_width=l2_cache_data_width, address_width=32, addressing="word")
                l2_cache = Cache(
                    cachesize = l2_size // 4,
                    master    = wb_hyperram,
                    slave     = wb_l2)
                l2_cache = FullMemoryWE()(l2_cache)
                self.l2_cache = l2_cache
                # Width converter: 128-bit L2 slave -> 32-bit HyperRAM bus.
                self.l2_conv = Converter(master=wb_l2, slave=self.hyperram.bus)
                self.add_config("L2_SIZE", l2_size)
            else:
                self.bus.add_slave("main_ram", slave=self.hyperram.bus,
                    region=SoCRegion(origin=0x40000000, size=hyperram_size, mode="rwx"))

            # Relocate OpenSBI to fit within 8MB HyperRAM.
            # Default is main_ram + 0xf00000 (15MB) which exceeds 8MB.
            # Place at 0x40780000 (7.5MB offset), leaving 512KB for OpenSBI.
            if "opensbi" in self.bus.regions:
                del self.bus.regions["opensbi"]
            self.bus.add_region("opensbi", SoCRegion(
                origin=0x40000000 + hyperram_size - 0x80000,
                size=0x80000, cached=True, linker=True))

        # SPI Flash (Quad, memory-mapped with XIP) ------------------------------------------------
        if with_spi_flash:
            from litespi.modules import W25Q256JVM
            from litespi.opcodes import SpiNorFlashOpCodes as Codes
            if flash_cache_size:
                # Manual LiteSPI instantiation with L2-style cache.
                import math
                from litespi import LiteSPI
                from litespi.phy.generic import LiteSPIPHY
                from migen.fhdl.simplify import FullMemoryWE

                flash_cache_size = 2**int(math.log2(flash_cache_size))
                default_divisor = math.ceil(sys_clk_freq / (2 * 20e6)) - 1
                spiflash_module = W25Q256JVM(Codes.READ_1_1_4)
                spiflash_pads = platform.request("spiflash4x")
                spiflash_phy = LiteSPIPHY(
                    spiflash_pads, spiflash_module,
                    device=platform.device, default_divisor=default_divisor, rate="1:1")
                spiflash = LiteSPI(spiflash_phy, mmap_endianness=self.cpu.endianness, with_master=True)
                spiflash.add_module(name="phy", module=spiflash_phy)
                self.add_module(name="spiflash", module=spiflash)

                # L2 cache with wide slave interface for multi-word cache lines.
                from litex.soc.interconnect.wishbone import Cache, Converter
                l2_cache_data_width = 128  # Wide cache lines (4 words per line)
                wb_cached = wishbone.Interface(
                    data_width=self.bus.data_width, address_width=32, addressing="word")
                wb_l2 = wishbone.Interface(
                    data_width=l2_cache_data_width, address_width=32, addressing="word")
                l2_cache = Cache(
                    cachesize = flash_cache_size // 4,
                    master    = wb_cached,
                    slave     = wb_l2,
                    reverse   = False)
                l2_cache = FullMemoryWE()(l2_cache)
                self.flash_cache = l2_cache
                # Width converter: 128-bit L2 slave -> 32-bit LiteSPI bus.
                self.flash_l2_conv = Converter(master=wb_l2, slave=spiflash.bus)

                spiflash_region = SoCRegion(
                    origin=self.mem_map.get("spiflash", None),
                    size=spiflash_module.total_size,
                    mode="rx")
                self.bus.add_slave("spiflash", slave=wb_cached,
                    region=spiflash_region, strip_origin=True)
                self.add_config("FLASH_CACHE_SIZE", flash_cache_size)

                # Add constants normally provided by add_spi_flash().
                clk_freq = int(sys_clk_freq / (2 * (default_divisor + 1)))
                self.add_constant("SPIFLASH_PHY_FREQUENCY", clk_freq)
                self.add_constant("SPIFLASH_MODULE_NAME", spiflash_module.name)
                self.add_constant("SPIFLASH_MODULE_TOTAL_SIZE", spiflash_module.total_size)
                self.add_constant("SPIFLASH_MODULE_PAGE_SIZE", spiflash_module.page_size)
                if spiflash_module.bus_width >= 4 and Codes.READ_1_1_4 in spiflash_module.supported_opcodes:
                    self.add_constant("SPIFLASH_MODULE_QUAD_CAPABLE")
            else:
                self.add_spi_flash(mode="4x", module=W25Q256JVM(Codes.READ_1_1_4), with_master=True)

        # SD Card ----------------------------------------------------------------------------------
        if with_sdcard:
            self.add_sdcard()

        # SPI Ethernet (KSZ8851SNL) ----------------------------------------------------------------
        if with_spi_eth:
            from litex.soc.cores.spi import SPIMaster
            spi_eth_pads = platform.request("spi_eth")
            self.spi_eth = SPIMaster(spi_eth_pads, data_width=8,
                sys_clk_freq=sys_clk_freq, spi_clk_freq=10e6)
            # Ethernet reset (active-low)
            eth_rst_n = platform.request("eth_rst_n")
            self.comb += eth_rst_n.eq(1)  # De-assert reset
            # Ethernet IRQ pin (directly exposed as GPIO for kernel driver)
            eth_irq_n = platform.request("eth_irq_n")
            self.spi_eth_irq = eth_irq_n

        # JTAG (external TAP) ----------------------------------------------------------------------
        # When VexRiscv SMP is built with --jtag-tap, connect to external JTAG pins.
        if hasattr(self, "cpu") and hasattr(self.cpu, "add_jtag"):
            from litex.soc.cores.cpu.vexriscv_smp.core import VexRiscvSMP
            if VexRiscvSMP.jtag_tap:
                jtag_pads = platform.request("jtag")
                self.cpu.add_jtag(jtag_pads)

        # LEDs -------------------------------------------------------------------------------------
        if with_led_chaser:
            self.leds = LedChaser(
                pads         = platform.request_all("user_led"),
                sys_clk_freq = sys_clk_freq)

# Build --------------------------------------------------------------------------------------------

def main():
    from litex.build.parser import LiteXArgumentParser
    parser = LiteXArgumentParser(platform=lowrisc_sonata.Platform, description="LiteX SoC on Sonata Board.")
    parser.add_target_argument("--sys-clk-freq",  default=50e6,  type=float, help="System clock frequency.")
    parser.add_target_argument("--with-hyperram",  action="store_true",       help="Add HyperRAM.")
    parser.add_target_argument("--with-sdcard",   action="store_true",       help="Add SD card support.")
    parser.add_target_argument("--flash-cache-size", default=0, type=int,  help="SPI flash read cache size in bytes (0=disabled, e.g. 65536 for 64KB 4-way).")

    args = parser.parse_args()

    soc = BaseSoC(
        sys_clk_freq     = args.sys_clk_freq,
        with_hyperram    = args.with_hyperram,
        with_sdcard      = args.with_sdcard,
        flash_cache_size = args.flash_cache_size,
        **parser.soc_argdict
    )

    builder = Builder(soc, **parser.builder_argdict)
    if args.build:
        builder.build(**parser.toolchain_argdict)

    if args.load:
        prog = soc.platform.create_programmer()
        prog.load_bitstream(builder.get_bitstream_filename(mode="sram"))

if __name__ == "__main__":
    main()

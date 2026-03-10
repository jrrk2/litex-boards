#
# 4-way set-associative read-only cache for SPI flash XIP.
#
# Sits between the wishbone bus fabric (master) and the LiteSPI flash
# controller (slave). Caches read transactions; writes pass through
# directly to the slave (for flash programming).
#
# Parameters:
#   size       -- total cache size in bytes (default 65536 = 64KB)
#   ways       -- associativity (default 4)
#   line_words -- words per cache line (default 4 = 16 bytes)
#
# BRAM usage on Xilinx 7-series (64KB, 4-way, 4-word lines):
#   Data: 4 ways x 4096 words x 32b = 64KB = 16 x 36Kb BRAMs
#   Tags: 4 x 1024 x 21b = distributed (LUT) RAM
#   PLRU: 1024 x 3b = distributed (LUT) RAM
#

from migen import *
from litex.gen import *
from litex.gen.fhdl.memory import READ_FIRST
from litex.soc.interconnect import wishbone


class FlashReadOnlyCache(LiteXModule):
    def __init__(self, master, slave, size=65536, ways=4, line_words=4):
        self.master = master
        self.slave  = slave

        # # #

        # Parameters.
        # -----------
        assert ways == 4, "Only 4-way associativity is supported"
        assert (size & (size - 1)) == 0, "Cache size must be a power of 2"
        assert (line_words & (line_words - 1)) == 0, "line_words must be a power of 2"

        way_size    = size // ways                         # bytes per way
        n_sets      = way_size // (line_words * 4)         # sets per way
        offset_bits = log2_int(line_words)                 # word offset in line
        set_bits    = log2_int(n_sets)                     # set index
        # master.adr is 32-bit word address; decompose into offset + set + tag.
        tag_bits    = 32 - set_bits - offset_bits

        # Master address decomposition.
        adr_offset = master.adr[0:offset_bits]
        adr_set    = master.adr[offset_bits:offset_bits + set_bits]
        adr_tag    = master.adr[offset_bits + set_bits:offset_bits + set_bits + tag_bits]

        # Latched refill parameters (stable through multi-cycle REFILL).
        refill_set    = Signal(set_bits)
        refill_tag    = Signal(tag_bits)
        refill_way    = Signal(max=ways)
        refill_word   = Signal(offset_bits)

        # Control signals (directly driven by FSM).
        do_refill    = Signal()   # In REFILL: route data port addresses for write
        write_data   = Signal()   # Write slave.dat_r into victim way's data RAM
        write_tag    = Signal()   # Write new tag (uses combinational victim_way)
        latch_refill = Signal()   # Latch miss parameters
        word_clr     = Signal()   # Clear refill word counter
        word_inc     = Signal()   # Increment refill word counter

        # Refill word counter.
        self.sync += [
            If(word_clr,
                refill_word.eq(0),
            ).Elif(word_inc,
                refill_word.eq(refill_word + 1),
            )
        ]

        # Latch refill parameters on miss.
        self.sync += If(latch_refill,
            refill_set.eq(adr_set),
            refill_tag.eq(adr_tag),
        )

        # -----------------------------------------------------------------
        # Tag Memory: 4 ways, each n_sets entries of (tag + valid) bits.
        # -----------------------------------------------------------------
        # Address is ALWAYS adr_set (from master.adr, which is stable per
        # wishbone protocol until ack). No address switching needed.
        tag_ports = []
        for i in range(ways):
            mem  = Memory(tag_bits + 1, n_sets, name="tag_way{}".format(i))
            port = mem.get_port(write_capable=True, mode=READ_FIRST)
            self.specials += mem, port
            tag_ports.append(port)
            self.comb += port.adr.eq(adr_set)

        # -----------------------------------------------------------------
        # Tag comparison.
        # -----------------------------------------------------------------
        way_hit  = Signal(ways)
        any_hit  = Signal()
        hit_way  = Signal(max=ways)
        hit_data = Signal(32)

        for i in range(ways):
            way_valid_i = tag_ports[i].dat_r[tag_bits]   # MSB = valid
            way_tag_i   = tag_ports[i].dat_r[0:tag_bits]
            self.comb += way_hit[i].eq(way_valid_i & (way_tag_i == adr_tag))

        self.comb += any_hit.eq(way_hit != 0)

        # Priority encoder: lowest-numbered matching way.
        self.comb += hit_way.eq(0)
        for i in reversed(range(ways)):
            self.comb += If(way_hit[i], hit_way.eq(i))

        # -----------------------------------------------------------------
        # PLRU: 3-bit binary tree for 4-way replacement.
        # -----------------------------------------------------------------
        #   Bit[2] = top node    (0→left{0,1}, 1→right{2,3})
        #   Bit[1] = left child  (0→way0, 1→way1)
        #   Bit[0] = right child (0→way2, 1→way3)
        # Bits point TOWARD the next victim.
        # On access, set bits to point AWAY from accessed way.
        plru_mem  = Memory(3, n_sets, name="plru")
        plru_port = plru_mem.get_port(write_capable=True, mode=READ_FIRST)
        self.specials += plru_mem, plru_port

        plru_bits = Signal(3)
        self.comb += [
            plru_port.adr.eq(adr_set),
            plru_bits.eq(plru_port.dat_r),
        ]

        # Victim way: follow the PLRU tree toward the LRU candidate.
        victim_way = Signal(max=ways)
        self.comb += [
            If(plru_bits[2] == 0,
                If(plru_bits[1] == 0,
                    victim_way.eq(0),
                ).Else(
                    victim_way.eq(1),
                )
            ).Else(
                If(plru_bits[0] == 0,
                    victim_way.eq(2),
                ).Else(
                    victim_way.eq(3),
                )
            )
        ]

        # PLRU update: given an accessed way, compute new PLRU bits.
        plru_update  = Signal(3)
        plru_upd_way = Signal(max=ways)
        self.comb += [
            plru_update.eq(plru_bits),
            If(plru_upd_way[1] == 0,
                # Accessed way 0 or 1: top→right (away), left→away from accessed.
                plru_update[2].eq(1),
                If(plru_upd_way[0] == 0,
                    plru_update[1].eq(1),
                ).Else(
                    plru_update[1].eq(0),
                )
            ).Else(
                # Accessed way 2 or 3: top→left (away), right→away from accessed.
                plru_update[2].eq(0),
                If(plru_upd_way[0] == 0,
                    plru_update[0].eq(1),
                ).Else(
                    plru_update[0].eq(0),
                )
            )
        ]

        # Tag write: driven in TEST_HIT miss path. Uses combinational victim_way
        # (valid because plru_port was read in the previous cycle).
        for i in range(ways):
            self.comb += If(write_tag & (victim_way == i),
                tag_ports[i].dat_w.eq(Cat(adr_tag, 1)),  # {valid=1, tag}
                tag_ports[i].we.eq(1),
            )

        # Latch victim way alongside other refill params.
        self.sync += If(latch_refill, refill_way.eq(victim_way))

        # -----------------------------------------------------------------
        # Data Memory: 4 ways, each n_sets*line_words entries x 32 bits.
        # -----------------------------------------------------------------
        data_ports = []
        for i in range(ways):
            mem  = Memory(32, n_sets * line_words, name="data_way{}".format(i))
            port = mem.get_port(write_capable=True, we_granularity=8, mode=READ_FIRST)
            self.specials += mem, port
            data_ports.append(port)
            self.comb += [
                If(do_refill,
                    port.adr.eq(Cat(refill_word, refill_set)),
                    If(write_data & (refill_way == i),
                        port.dat_w.eq(slave.dat_r),
                        port.we.eq(0xF),
                    ),
                ).Else(
                    port.adr.eq(Cat(adr_offset, adr_set)),
                ),
            ]

        # Mux data from hitting way.
        self.comb += hit_data.eq(0)
        for i in range(ways):
            self.comb += If(way_hit[i], hit_data.eq(data_ports[i].dat_r))

        # -----------------------------------------------------------------
        # FSM.
        # -----------------------------------------------------------------
        # Hit path:  IDLE → TEST_HIT(hit) → IDLE        (2 cycles)
        # Miss path: IDLE → TEST_HIT(miss) → REFILL(N)
        #                  → REFILL_DONE → TEST_HIT(hit) → IDLE

        self.fsm = fsm = FSM(reset_state="IDLE")

        fsm.act("IDLE",
            If(master.cyc & master.stb,
                If(master.we,
                    NextState("WRITE_THROUGH"),
                ).Else(
                    NextState("TEST_HIT"),
                )
            )
        )

        fsm.act("TEST_HIT",
            If(any_hit,
                # Hit: return data, ack, update PLRU.
                master.dat_r.eq(hit_data),
                master.ack.eq(1),
                plru_upd_way.eq(hit_way),
                plru_port.dat_w.eq(plru_update),
                plru_port.we.eq(1),
                NextState("IDLE"),
            ).Else(
                # Miss: write new tag, latch params, start refill.
                write_tag.eq(1),
                latch_refill.eq(1),
                word_clr.eq(1),
                NextState("REFILL"),
            )
        )

        fsm.act("REFILL",
            do_refill.eq(1),
            slave.cyc.eq(1),
            slave.stb.eq(1),
            slave.we.eq(0),
            slave.sel.eq(0xF),
            slave.adr.eq(Cat(refill_word, refill_set, refill_tag)),
            If(slave.ack,
                write_data.eq(1),
                word_inc.eq(1),
                If(refill_word == line_words - 1,
                    NextState("REFILL_DONE"),
                )
            )
        )

        fsm.act("REFILL_DONE",
            # 1-cycle wait: data port address switches from refill to read.
            # Tag/data reads will be valid in the next cycle (TEST_HIT).
            NextState("TEST_HIT"),
        )

        fsm.act("WRITE_THROUGH",
            slave.cyc.eq(1),
            slave.stb.eq(1),
            slave.we.eq(1),
            slave.adr.eq(master.adr),
            slave.dat_w.eq(master.dat_w),
            slave.sel.eq(master.sel),
            If(slave.ack,
                master.ack.eq(1),
                NextState("IDLE"),
            )
        )

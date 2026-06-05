#!/usr/bin/env python3
# CoralNPU gem5 Syscall-Emulation simulation script.
#
# Simulates the CoralNPU scalar pipeline (RV32IM, in-order, 4-wide dispatch)
# using gem5's MinorCPU model configured to match CoralNPU RTL timing.
#
# Usage
# -----
#   ./build/RISCV/gem5.opt  configs/coralnpu/coralnpu_se.py  \
#       --cmd <rv32im_elf>                                    \
#       [--options "arg1 arg2"]                               \
#       [--cpu  atomic|minor]                                 \
#       [--freq 1GHz]                                         \
#       [--itcm-start 0x0] [--itcm-size 8kB]                 \
#       [--dtcm-start 0x10000] [--dtcm-size 32kB]            \
#       [--ext-mem-start 0x80000000] [--ext-mem-size 256MB]   \
#       [--max-insts N]                                       \
#       [--outdir m5out]
#
# CPU models
# ----------
#   atomic  — AtomicSimpleCPU: fast functional (no pipeline timing)
#   minor   — CoralNPUMinorCPU: cycle-accurate in-order pipeline (default)
#
# Memory layout (defaults match coralnpu-mpact defaults)
# ------------------------------------------------------
#   ITCM : 0x00000000 – 0x00001FFF  (8 KB, 1-cycle, instruction memory)
#   DTCM : 0x00010000 – 0x00017FFF  (32 KB, 1-cycle, data memory)
#   ExtMem: 0x80000000 – …           (256 MB, 50 ns, AXI bus model)
#
# Statistics output
# -----------------
#   m5out/stats.txt  — cycle count, IPC, cache miss rates, FU occupancy
#   m5out/config.ini — full configuration snapshot

import argparse
import os
import sys

# Ensure gem5 can find the coralnpu config package.
_this_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(_this_dir))   # configs/

import m5
from m5.objects import (
    AddrRange,
    Cache,
    Process,
    Root,
    RiscvAtomicSimpleCPU,
    RiscvISA,
    SEWorkload,
    SimpleMemory,
    SrcClockDomain,
    System,
    SystemXBar,
    UartConsole,
    VoltageDomain,
)

from coralnpu.coralnpu_cpu import CoralNPUMinorCPU


# ── Argument parsing ─────────────────────────────────────────────────────────

def parse_args():
    p = argparse.ArgumentParser(
        description="gem5 cycle-accurate simulation of the CoralNPU scalar core.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument("--cmd", required=True,
                   help="Path to the rv32im ELF binary to simulate.")
    p.add_argument("--options", default="",
                   help="Space-separated arguments forwarded to the binary.")
    p.add_argument("--cpu", choices=["atomic", "minor"], default="minor",
                   help="CPU model: 'atomic' (fast functional) or "
                        "'minor' (cycle-accurate CoralNPU pipeline).")
    p.add_argument("--freq", default="1GHz",
                   help="Simulated clock frequency.")
    p.add_argument("--itcm-start", default="0x0",
                   help="ITCM start address (hex).")
    p.add_argument("--itcm-size", default="8kB",
                   help="ITCM size string (e.g. '8kB').")
    p.add_argument("--dtcm-start", default="0x10000",
                   help="DTCM start address (hex).")
    p.add_argument("--dtcm-size", default="32kB",
                   help="DTCM size string (e.g. '32kB').")
    p.add_argument("--ext-mem-start", default="0x80000000",
                   help="External memory start address (hex).")
    p.add_argument("--ext-mem-size", default="256MB",
                   help="External memory size string.")
    p.add_argument("--max-insts", type=int, default=0,
                   help="Stop after N committed instructions (0 = no limit).")
    p.add_argument("--uart-addr", default="0xFFFFFFF8",
                   help="MMIO address of the UART TX register used by firmware "
                        "printf (e.g. 'sb c, <addr>'). Set to 'none' to "
                        "disable the UART console device.")
    # gem5 passes --outdir itself; we leave it to gem5's option parser.
    return p.parse_args()


# ── Cache class ──────────────────────────────────────────────────────────────
# Defined locally so the script has no dependency on configs/common/Caches.py.

class CoralNPU_L1ICache(Cache):
    """L1 instruction cache matching CoralNPU: 8 KB, 4-way, 32-byte lines."""
    size            = "8kB"
    assoc           = 4
    tag_latency     = 1
    data_latency    = 1
    response_latency = 1
    mshrs           = 4
    tgts_per_mshr   = 8
    is_read_only    = True
    writeback_clean = True


class CoralNPU_L1DCache(Cache):
    """L1 data cache matching CoralNPU: 16 KB, 4-way, 32-byte lines."""
    size             = "16kB"
    assoc            = 4
    tag_latency      = 1
    data_latency     = 1
    response_latency = 1
    mshrs            = 4
    tgts_per_mshr    = 8
    write_buffers    = 8


# ── System builder ───────────────────────────────────────────────────────────

def build_system(args):
    itcm_start    = int(args.itcm_start,    16)
    dtcm_start    = int(args.dtcm_start,    16)
    ext_mem_start = int(args.ext_mem_start, 16)

    system = System()
    system.clk_domain = SrcClockDomain(
        clock=args.freq,
        voltage_domain=VoltageDomain(),
    )
    system.mem_ranges = [
        AddrRange(start=itcm_start,    size=args.itcm_size),
        AddrRange(start=dtcm_start,    size=args.dtcm_size),
        AddrRange(start=ext_mem_start, size=args.ext_mem_size),
    ]

    # ── CPU ──────────────────────────────────────────────────────────────────
    if args.cpu == "atomic":
        system.mem_mode = "atomic"
        cpu = RiscvAtomicSimpleCPU()
        cpu.isa = [RiscvISA(riscv_type="RV32", enable_rvv=False)]
    else:
        system.mem_mode = "timing"
        cpu = CoralNPUMinorCPU()

    if args.max_insts > 0:
        cpu.max_insts_any_thread = args.max_insts

    system.cpu = cpu
    cpu.createInterruptController()

    # ── Memory bus ───────────────────────────────────────────────────────────
    system.membus = SystemXBar()

    # ── Caches (MinorCPU only) ────────────────────────────────────────────────
    # AtomicSimpleCPU connects directly to the membus without caches.
    if args.cpu == "minor":
        cpu.icache = CoralNPU_L1ICache()
        cpu.dcache = CoralNPU_L1DCache()
        cpu.icache_port = cpu.icache.cpu_side
        cpu.dcache_port = cpu.dcache.cpu_side
        cpu.icache.mem_side = system.membus.cpu_side_ports
        cpu.dcache.mem_side = system.membus.cpu_side_ports
    else:
        cpu.icache_port = system.membus.cpu_side_ports
        cpu.dcache_port = system.membus.cpu_side_ports

    # Connect walk-cache port if present (required by gem5 MMU).
    if hasattr(cpu, "mmu"):
        cpu.mmu.connectWalkerPorts(
            system.membus.cpu_side_ports,
            system.membus.cpu_side_ports,
        )

    # ── ITCM — Instruction Tightly Coupled Memory ─────────────────────────────
    # 1-cycle SRAM latency at 1 GHz → 1 ns round-trip.
    # 256-bit bus (32-byte data path) matches CoralNPU ibus interface.
    itcm = SimpleMemory(
        range=AddrRange(start=itcm_start, size=args.itcm_size),
        latency="1ns",
        bandwidth="32GB/s",
    )
    system.itcm = itcm
    system.membus.mem_side_ports = itcm.port

    # ── DTCM — Data Tightly Coupled Memory ────────────────────────────────────
    # Same 1-cycle SRAM latency; 32-byte bus matches CoralNPU dbus.
    dtcm = SimpleMemory(
        range=AddrRange(start=dtcm_start, size=args.dtcm_size),
        latency="1ns",
        bandwidth="32GB/s",
    )
    system.dtcm = dtcm
    system.membus.mem_side_ports = dtcm.port

    # ── External memory (AXI bus model) ──────────────────────────────────────
    # 50 ns models typical AXI bus + off-chip SRAM/DDR latency.
    ext_mem = SimpleMemory(
        range=AddrRange(start=ext_mem_start, size=args.ext_mem_size),
        latency="50ns",
        bandwidth="8GB/s",
    )
    system.ext_mem = ext_mem
    system.membus.mem_side_ports = ext_mem.port

    # System port for functional accesses (ELF loader, etc.)
    system.system_port = system.membus.cpu_side_ports

    # ── UART console — MMIO character output ──────────────────────────────────
    # Intercepts firmware printf implemented as  sb char, <uart_addr>.
    # The device prints each written byte to host stdout; reads return 0.
    if args.uart_addr.lower() != "none":
        uart_addr = int(args.uart_addr, 16)
        system.uart_console = UartConsole(
            pio_addr=uart_addr,
            pio_size=8,
            pio_latency="1ns",
        )
        system.uart_console.pio = system.membus.mem_side_ports

    # ── Workload ──────────────────────────────────────────────────────────────
    process = Process()
    process.executable = args.cmd
    if args.options.strip():
        process.cmd = [args.cmd] + args.options.split()
    else:
        process.cmd = [args.cmd]
    cpu.workload = process
    cpu.createThreads()
    system.workload = SEWorkload.init_compatible(args.cmd)

    return system


# ── Main ─────────────────────────────────────────────────────────────────────

def main():
    args = parse_args()

    # Sanity-check the binary exists before starting the long build.
    if not os.path.isfile(args.cmd):
        print(f"[coralnpu_se] ERROR: binary not found: {args.cmd}", file=sys.stderr)
        sys.exit(1)

    system = build_system(args)
    root   = Root(full_system=False, system=system)

    m5.instantiate()

    cpu_label = "AtomicSimpleCPU" if args.cpu == "atomic" else "CoralNPUMinorCPU"
    print(f"\n[coralnpu_se] Simulation started")
    print(f"[coralnpu_se]   Binary : {args.cmd}")
    print(f"[coralnpu_se]   CPU    : {cpu_label}")
    print(f"[coralnpu_se]   Clock  : {args.freq}")
    print(f"[coralnpu_se]   ITCM   : {args.itcm_start}  size={args.itcm_size}")
    print(f"[coralnpu_se]   DTCM   : {args.dtcm_start}  size={args.dtcm_size}")
    if args.max_insts:
        print(f"[coralnpu_se]   Limit  : {args.max_insts} instructions")
    if args.uart_addr.lower() != "none":
        print(f"[coralnpu_se]   UART   : {args.uart_addr}  (printf MMIO console)")
    print()

    exit_event = m5.simulate()

    print(f"\n[coralnpu_se] Exit: {exit_event.getCause()}")
    print(f"[coralnpu_se] Ticks simulated: {m5.curTick()}")
    print(f"[coralnpu_se] Stats written to m5out/stats.txt\n")


if __name__ == "__m5_main__":
    main()

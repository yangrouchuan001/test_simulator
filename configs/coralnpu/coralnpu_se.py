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
#       [--profile]                                           \
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
#   m5out/stats.txt          — cycle count, IPC, cache miss rates, FU occupancy
#   m5out/config.ini         — full configuration snapshot
#   m5out/ftrace.system.cpu  — raw per-function trace (only when --profile is set)
#   m5out/stats.txt          — function profile table appended after gem5 stats (--profile only)

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
from m5.util.convert import toMemorySize

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
    p.add_argument("--profile", action="store_true",
                   help="Enable per-function cycle profiling. gem5 writes "
                        "m5out/ftrace.system.cpu on each function-boundary "
                        "crossing; a sorted summary is appended to stats.txt.")
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

    # Compute DTCM end and the gap to ExtMem.
    # gem5 SE mode places the RV32 user stack at 0x7FFFFFFF (base, 64 MiB max)
    # and heap/mmap from ELF-end up to 0x40000000.  Both fall in the gap
    # between DTCM end and ExtMem start.  Include the gap in mem_ranges so
    # gem5's address-space accounting covers the full user address space.
    # (system.mem_ranges must be set in a single assignment — re-assignment
    # after the fact does not reliably propagate through gem5's VectorParam.)
    _dtcm_end = dtcm_start + int(toMemorySize(args.dtcm_size))
    _has_gap  = _dtcm_end < ext_mem_start

    _mem_ranges = [
        AddrRange(start=itcm_start, size=args.itcm_size),
        AddrRange(start=dtcm_start, size=args.dtcm_size),
    ]
    if _has_gap:
        _mem_ranges.append(AddrRange(start=_dtcm_end,
                                     size=ext_mem_start - _dtcm_end))
    _mem_ranges.append(AddrRange(start=ext_mem_start, size=args.ext_mem_size))

    system = System()
    system.cache_line_size = 32  # CoralNPU 256-bit (32-byte) instruction bus
    system.clk_domain = SrcClockDomain(
        clock=args.freq,
        voltage_domain=VoltageDomain(),
    )
    system.mem_ranges = _mem_ranges

    # ── CPU ──────────────────────────────────────────────────────────────────
    if args.cpu == "atomic":
        system.mem_mode = "atomic"
        cpu = RiscvAtomicSimpleCPU()
        cpu.isa = [RiscvISA(riscv_type="RV32", enable_rvv=False,
                             privilege_mode_set="M")]
    else:
        system.mem_mode = "timing"
        cpu = CoralNPUMinorCPU()

    if args.max_insts > 0:
        cpu.max_insts_any_thread = args.max_insts

    if args.profile:
        cpu.function_trace = True
        cpu.function_trace_start = 0

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
    # latency="1ps": 1 picosecond — minimum gem5 value, sub-cycle at any
    # plausible clock (1ps << 1ns/cycle at 1 GHz) → effectively 0 latency.
    # bandwidth="0B/s": gem5 MemoryBandwidth(0) → 0 ticks/byte → infinite
    # bandwidth (no serialisation stall).
    #
    # conf_table_reported=False: excludes ITCM from PhysicalMemory::getConfAddrRanges().
    # gem5 SE MemPools are built from getConfAddrRanges(), which iterates an
    # address-sorted interval tree.  If ITCM (0x0) were conf-reported it would
    # become pool 0 — far too small for the 64 MiB SE stack.  With this flag
    # False, pool 0 becomes the gap fill (or ExtMem), which is large enough.
    # ITCM still responds to bus accesses normally via its port connection.
    itcm = SimpleMemory(
        range=AddrRange(start=itcm_start, size=args.itcm_size),
        latency="1ps",
        bandwidth="0B/s",
        conf_table_reported=False,
    )
    system.itcm = itcm
    system.membus.mem_side_ports = itcm.port

    # ── DTCM — Data Tightly Coupled Memory ────────────────────────────────────
    # Same near-zero latency and infinite bandwidth as ITCM above.
    # conf_table_reported=False for the same reason as ITCM above.
    dtcm = SimpleMemory(
        range=AddrRange(start=dtcm_start, size=args.dtcm_size),
        latency="1ps",
        bandwidth="0B/s",
        conf_table_reported=False,
    )
    system.dtcm = dtcm
    system.membus.mem_side_ports = dtcm.port

    # ── Process auxiliary memory — SE stack / heap gap fill ──────────────────
    # Range already declared in system.mem_ranges above; create the backing
    # SimpleMemory object here and wire it to the bus.
    if _has_gap:
        _gap_size = ext_mem_start - _dtcm_end
        proc_mem = SimpleMemory(
            range=AddrRange(start=_dtcm_end, size=_gap_size),
            latency="1ps",
            bandwidth="0B/s",
        )
        system.proc_mem = proc_mem
        system.membus.mem_side_ports = proc_mem.port

    # ── External memory (AXI bus model) ──────────────────────────────────────
    # Near-zero latency and infinite bandwidth (same rationale as ITCM/DTCM).
    ext_mem = SimpleMemory(
        range=AddrRange(start=ext_mem_start, size=args.ext_mem_size),
        latency="1ps",
        bandwidth="0B/s",
    )
    system.ext_mem = ext_mem
    system.membus.mem_side_ports = ext_mem.port

    # System port for functional accesses (ELF loader, etc.)
    system.system_port = system.membus.cpu_side_ports

    # ── UART console — MMIO character output ──────────────────────────────────
    # Intercepts firmware printf implemented as  sb char, <uart_addr>.
    # The device prints each written byte to host stdout; reads return 0.
    #
    # pio_addr is page-aligned (4 KB) so that the device covers the ENTIRE
    # page containing the UART register.  This is required because:
    #
    #   1. fixupFault (§11.13) identity-maps the 4 KB page containing the
    #      UART register so that the TLB can translate firmware MMIO writes.
    #
    #   2. After the page is mapped, speculative L1I instruction-fetch
    #      requests (32-byte cache-line fills) may land anywhere within the
    #      same 4 KB page.  gem5's SystemXBar requires a SINGLE device to
    #      cover the full 32-byte range of each request.  If the device only
    #      covers 8 bytes at the UART register offset, the xbar fatals:
    #      "Unable to find destination for [0xffffffe0:0x100000000]".
    #
    # With pio_size=4096 the UartConsole covers the full page; reads return
    # zeros (harmless for squashed speculative fetches), and writes to any
    # offset within the page still print the byte (firmware writes the TX
    # register, which happens to be at offset 0xFF8 within the page).
    if args.uart_addr.lower() != "none":
        uart_addr = int(args.uart_addr, 16)
        uart_page_base = uart_addr & ~4095   # 4 KB page-aligned start
        system.uart_console = UartConsole(
            pio_addr=uart_page_base,
            pio_size=4096,
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


# ── Function-trace profiler ──────────────────────────────────────────────────

def _parse_ftrace(ftrace_path, tick_freq_hz):
    """Parse gem5 function_trace output and return a summary dict.

    ftrace format (one entry per function-boundary crossing):
        " (<duration_ticks>)\n<current_tick>: <func_name>"

    Returns a list of (func_name, total_cycles, call_count) sorted by
    total_cycles descending.  tick_freq_hz converts ticks to cycles.
    """
    from collections import defaultdict

    ticks_per_cycle = 1000 if tick_freq_hz == 0 else (10**12 // tick_freq_hz)

    totals = defaultdict(int)   # func_name -> total ticks spent in it
    counts = defaultdict(int)   # func_name -> call count

    try:
        with open(ftrace_path) as f:
            content = f.read()
    except FileNotFoundError:
        return []

    # Each record looks like: " (DURATION)\nTICK: FUNCNAME"
    # We split on ")\n" to pair duration with the function that was *leaving*.
    # The leaving function's duration is captured in parentheses just before
    # the next function name.  We collect name→duration pairs.
    import re
    # Pattern: optional leading text, then "TICK: FUNCNAME (DURATION)"
    # gem5 writes: "TICK: FUNCNAME (DURATION_OF_FUNCNAME)\nNEXT_TICK: NEXT_FUNC"
    # So DURATION follows the function name it belongs to.
    records = re.findall(r'\d+:\s+(\S+)\s+\((\d+)\)', content)
    for func_name, duration_str in records:
        duration = int(duration_str)
        cycles = max(1, duration // ticks_per_cycle)
        totals[func_name] += cycles
        counts[func_name] += 1

    return sorted(
        [(fn, totals[fn], counts[fn]) for fn in totals],
        key=lambda x: x[1],
        reverse=True,
    )


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
    if args.profile:
        print(f"[coralnpu_se]   Profile: enabled  (m5out/ftrace.system.cpu)")
    print()

    exit_event = m5.simulate()

    print(f"\n[coralnpu_se] Exit: {exit_event.getCause()}")
    print(f"[coralnpu_se] Ticks simulated: {m5.curTick()}")
    print(f"[coralnpu_se] Stats written to m5out/stats.txt\n")

    if args.profile:
        # gem5 uses 1 ps ticks; 1 GHz clock = 1000 ticks/cycle.
        # We derive ticks_per_cycle from the simulated frequency string rather
        # than hardcoding so it works at any --freq value.
        import re as _re
        _m = _re.match(r'([\d.]+)\s*(GHz|MHz|kHz|Hz)', args.freq, _re.I)
        if _m:
            _val, _unit = float(_m.group(1)), _m.group(2).upper()
            _hz = _val * {"GHZ": 1e9, "MHZ": 1e6, "KHZ": 1e3, "HZ": 1.0}[_unit]
        else:
            _hz = 1e9  # fallback: assume 1 GHz

        _outdir = m5.options.outdir if hasattr(m5, "options") else "m5out"
        _ftrace = os.path.join(_outdir, "ftrace.system.cpu")
        _stats  = os.path.join(_outdir, "stats.txt")
        _rows = _parse_ftrace(_ftrace, int(_hz))

        if _rows:
            _total_cy = sum(r[1] for r in _rows)
            _lines = []
            _lines.append(f"\n---------- Function Profile "
                          f"({len(_rows)} functions) ----------\n")
            _lines.append(f"  {'Function':<40} {'Cycles':>12} {'%':>6} {'Calls':>8}\n")
            _lines.append(f"  {'-'*40} {'-'*12} {'-'*6} {'-'*8}\n")
            for _fn, _cy, _cnt in _rows:
                _pct = 100.0 * _cy / _total_cy if _total_cy else 0
                _lines.append(f"  {_fn:<40} {_cy:>12,} {_pct:>5.1f}% {_cnt:>8,}\n")
            _lines.append(f"  {'-'*40} {'-'*12} {'-'*6} {'-'*8}\n")
            _lines.append(f"  {'TOTAL':<40} {_total_cy:>12,} {'100.0%':>6}\n")
            _lines.append(f"\n---------- End Function Profile ----------\n")
            with open(_stats, "a") as _f:
                _f.writelines(_lines)
            print(f"[coralnpu_se] Function profile ({len(_rows)} functions) "
                  f"appended to {_stats}\n")
        else:
            print(f"[coralnpu_se] Profile: no data in {_ftrace} "
                  f"(check ELF has symbols and --profile was set)\n")


if __name__ == "__m5_main__":
    main()

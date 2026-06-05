# CoralNPU gem5 Simulation — Modifications and Usage Guide

> **Scope:** Documents all changes made to the gem5-consistent simulator at
> `/orange/ZSP/home/cn2095/test_simulator/` to support cycle-accurate simulation
> of the CoralNPU scalar pipeline, approximate-timing simulation of the
> CoralNPU RVV vector co-processor, and firmware `printf` output via MMIO UART.
>
> **Simulator root:** `/orange/ZSP/home/cn2095/test_simulator/`
> **Reference docs:** `gem5_for_coralnpu.md`, `mpact_improve.md`

---

## Table of Contents

1. [Overview of All Changes](#1-overview-of-all-changes)
2. [ISA Extension — `mpause` Instruction](#2-isa-extension--mpause-instruction)
   - 2.1 [formats/coralnpu.isa (new)](#21-formatscoralnpuisa-new)
   - 2.2 [formats/formats.isa (modified)](#22-formatsformatsisa-modified)
   - 2.3 [decoder.isa (modified)](#23-decoderisa-modified)
3. [CPU Configuration — Scalar Pipeline](#3-cpu-configuration--scalar-pipeline)
4. [CPU Configuration — RVV Co-processor (Level 2)](#4-cpu-configuration--rvv-co-processor-level-2)
   - 4.1 [ISA parameters](#41-isa-parameters)
   - 4.2 [Vector functional units](#42-vector-functional-units)
   - 4.3 [LSU vector memory op-classes](#43-lsu-vector-memory-op-classes)
   - 4.4 [Latency derivation from RTL](#44-latency-derivation-from-rtl)
5. [Complete FU Pool Reference](#5-complete-fu-pool-reference)
6. [UART Console — Firmware printf Support](#6-uart-console--firmware-printf-support)
   - 6.1 [Background](#61-background)
   - 6.2 [New files](#62-new-files)
   - 6.3 [Modified files](#63-modified-files)
   - 6.4 [How it works](#64-how-it-works)
7. [Build Instructions](#7-build-instructions)
8. [Running Simulations](#8-running-simulations)
   - 8.1 [Functional simulation](#81-functional-simulation)
   - 8.2 [Cycle-accurate scalar simulation](#82-cycle-accurate-scalar-simulation)
   - 8.3 [Cycle-accurate simulation with RVV](#83-cycle-accurate-simulation-with-rvv)
   - 8.4 [printf / UART address customisation](#84-printf--uart-address-customisation)
   - 8.5 [Debug flags and instruction trace](#85-debug-flags-and-instruction-trace)
   - 8.6 [Limiting simulation length](#86-limiting-simulation-length)
9. [Reading Simulation Output](#9-reading-simulation-output)
10. [Timing Accuracy and Limitations](#10-timing-accuracy-and-limitations)
    - 10.1 [Scalar timing accuracy](#101-scalar-timing-accuracy)
    - 10.2 [RVV timing accuracy (Level 2)](#102-rvv-timing-accuracy-level-2)
    - 10.3 [Known gaps table](#103-known-gaps-table)
11. [Build Fixes — Errors Encountered and Resolved](#11-build-fixes--errors-encountered-and-resolved)
    - 11.1 [ISA parser: "instruction format Unknown not defined"](#111-isa-parser-instruction-format-unknown-not-defined)
    - 11.2 [Linker: undefined reference to getLE / setLE](#112-linker-undefined-reference-to-getle--setle)
    - 11.3 [AttributeError: Invalid assignment for LocalBP parameter localHistoryTableSize](#113-attributeerror-invalid-assignment-for-localbp-parameter-localhistorytablesize)
    - 11.4 [Panic: Stats of the same group share the same name `opClasses`](#114-panic-stats-of-the-same-group-share-the-same-name-opclasses)
    - 11.5 [Fatal: Out of memory, please increase size of physical memory](#115-fatal-out-of-memory-please-increase-size-of-physical-memory)

---

## 1. Overview of All Changes

| File | Type | Purpose |
|------|------|---------|
| `src/arch/riscv/isa/decoder.isa` | **Modified** | Decode `mpause` via existing `SystemOp` format (custom-0, `0x0000000B`) |
| `src/dev/coralnpu/uart_console.hh` | **New** | `UartConsole` SimObject header |
| `src/dev/coralnpu/uart_console.cc` | **New** | `UartConsole` implementation — prints bytes on write |
| `src/dev/coralnpu/UartConsole.py` | **New** | gem5 Python param class for `UartConsole` |
| `src/dev/coralnpu/SConscript` | **New** | Build registration for the UART device |
| `configs/coralnpu/__init__.py` | **New** | Python package marker |
| `configs/coralnpu/coralnpu_cpu.py` | **New** | `CoralNPUMinorCPU` class — scalar + RVV FU pool |
| `configs/coralnpu/coralnpu_se.py` | **New / Modified** | Simulation entry-point; adds `--uart-addr` and wires `UartConsole` |

Timing customisation is done through gem5's Python configuration layer. The ISA extension (`mpause`) required one edit to `decoder.isa` (no new format file needed — it reuses the existing `SystemOp` format). The printf support required a minimal new C++ SimObject (`UartConsole`).

---

## 2. ISA Extension — `mpause` Instruction

CoralNPU firmware inserts `mpause` at RTL co-simulation synchronisation points.
In standalone gem5 simulation it is a no-op: no register is modified and `NoFault`
is returned. It is marked **serialising** so no instructions issue after it until
it commits (matching the hardware behaviour where `mpause` drains the pipeline
before syncing with RTL).

### `mpause` encoding

| Field | Bits | Value |
|-------|------|-------|
| QUADRANT | [1:0] | `11` (32-bit) |
| OPCODE5 (bits[6:2]) | [6:2] | `00010` = 0x02 (RISC-V custom-0 reserved space) |
| FUNCT3 | [14:12] | `000` |
| RD, RS1, RS2, FUNCT7 | remaining | all zero |
| **Full 32-bit word** | | **`0x0000000B`** |

### 2.1 `formats/coralnpu.isa`

No separate format file is needed.  `mpause` reuses the existing `SystemOp`
format (defined in `formats/standard.isa`) with an empty C++ code body — the
same pattern used by `ecall`, `ebreak`, `fence`, and `wfi`.

A standalone `CoralNPUOp` format was attempted first but caused the ISA parser
to fail before `unknown.isa` was included (gem5 ISA `{{ }}` blocks do not allow
`//` comments at the outer scope; they are stripped as ISA-level comments and
can leave malformed Python).  Using the pre-existing `SystemOp` format avoids
the problem entirely.

### 2.2 `formats/formats.isa` (unchanged)

No modification required — `SystemOp` is already included via
`##include "standard.isa"` earlier in the formats list.

### 2.3 `decoder.isa` (modified)

Inside `0x3: decode OPCODE5 {}`, immediately before the `0x03:` (FENCE) block:

```diff
+ 0x02: SystemOp::mpause({{ }}, IsNonSpeculative, IsSerializeAfter, No_OpClass);
+
  0x03: decode FUNCT3 {
      format FenceOp {
```

`{{ }}` is the empty C++ execute body — `SystemOp` returns `NoFault` by default.
`No_OpClass` maps `mpause` to the `System` functional unit (same as `ecall`).

---

## 3. CPU Configuration — Scalar Pipeline

File: `configs/coralnpu/coralnpu_cpu.py`

`CoralNPUMinorCPU` extends `RiscvMinorCPU`. All pipeline parameters are set to
match the CoralNPU scalar microarchitecture from
`coralnpu/doc/microarch/microarch.md` and `pipeline_and_instructions_organized.md`.

| gem5 parameter | Value | CoralNPU RTL source |
|----------------|-------|---------------------|
| `fetch1LineWidth` | 32 bytes | `fetchDataBits=256` → 32 bytes |
| `decodeInputWidth` | 4 | `instructionLanes=4` |
| `executeInputWidth` | 4 | dispatch width = 4 |
| `executeIssueLimit` | 4 | 4 FU lanes |
| `executeMemoryIssueLimit` | 1 | `dispatch.md`: 1 memory/cycle |
| `executeInputBufferSize` | 8 | `retirementBufferSize=8` |
| `executeBranchDelay` | 1 | 1-cycle branch redirect penalty |
| `executeLSQTransfersQueueSize` | 8 | LSU queue depth = 8 |
| `executeAllowEarlyMemoryIssue` | False | In-order LSU dispatch |

---

## 4. CPU Configuration — RVV Co-processor (Level 2)

### 4.1 ISA parameters

```python
isa = [RiscvISA(
    riscv_type = "RV32",
    enable_rvv = True,    # ← was False; enables RVV instruction decoding
    vlen       = 128,     # CoralNPU: rvvVlen=128 (16 bytes/register)
    elen       = 32,      # CoralNPU: max element width = 32 bits (int8/16/32)
)]
```

`vlen=128` and `elen=32` match `Parameters.scala` in the CoralNPU Chisel source.
Setting `elen=32` prevents gem5 from generating 64-bit element micro-ops that the
CoralNPU hardware does not support.

### 4.2 Vector functional units

Three new FUs are added to the pool, sized to match `rvv_backend_define.svh`
(`NUM_ALU=2, NUM_MUL=2, NUM_DIV=1`):

**`_CoralNPU_VEC_ALU` × 2 — integer arithmetic, logic, shift, compare, reduce, config**

```python
opLat=5, issueLat=1
srcRegsRelativeLats=[2, 2]   # effective v-to-v chain penalty = 5-2 = 3 cycles
```

Op-classes handled: `SimdAdd`, `SimdAlu`, `SimdCmp`, `SimdMisc`, `SimdShift`,
`SimdShiftAcc`, `SimdReduceAlu`, `SimdReduceCmp`, `SimdCvt`, `SimdExt`,
`SimdConfig` (includes `vsetvl*`)

**`_CoralNPU_VEC_MUL` × 2 — integer multiply/MAC, float arithmetic**

```python
opLat=5, issueLat=1
srcRegsRelativeLats=[2, 2]
```

Op-classes handled: `SimdMult`, `SimdMultAcc`, `SimdAddAcc`, `SimdReduceAdd`,
`SimdDotProd`, `SimdFloat{Add,Alu,Cmp,Cvt,Misc,Mult,MultAcc,ReduceAdd,ReduceCmp,Ext}`

**`_CoralNPU_VEC_DIV` × 1 — integer divide, float divide/sqrt**

```python
opLat=35, issueLat=35   # non-pipelined, worst-case
```

Op-classes handled: `SimdDiv`, `SimdSqrt`, `SimdFloatDiv`, `SimdFloatSqrt`

### 4.3 LSU vector memory op-classes

The existing scalar LSU FU is extended with all RVV memory access patterns.
No new FU is needed — all vector memory goes through the same LSU slot (matching
CoralNPU hardware where the LSU serves both scalar and vector):

```python
# Added to _CoralNPU_LSU opClasses:
"SimdUnitStrideLoad",               # vle<eew>.v
"SimdUnitStrideStore",              # vse<eew>.v
"SimdUnitStrideMaskLoad",           # vlm.v
"SimdUnitStrideMaskStore",          # vsm.v
"SimdStridedLoad",                  # vlse<eew>.v
"SimdStridedStore",                 # vsse<eew>.v
"SimdIndexedLoad",                  # vluxei / vloxei
"SimdIndexedStore",                 # vsuxei / vsoxei
"SimdUnitStrideFaultOnlyFirstLoad", # vle<eew>ff.v
"SimdWholeRegisterLoad",            # vl<n>r.v
"SimdWholeRegisterStore",           # vs<n>r.v
```

### 4.4 Latency derivation from RTL

All latency values come from `pipeline_and_instructions_organized.md` §2.2
and the RTL timing tables:

```
Total latency = 3 cy fixed overhead  (DE2-FF + ROB-FF + VRF-FF)
              + EX cycles

From RTL timing table (VLEN=128, single uop, N_uops=1):
  ALU total = 5 cy  →  EX = 2 cy   → opLat = 5
  MUL total = 5 cy  →  EX = 2 cy   → opLat = 5
  DIV total ≈ 35 cy →  EX ≈ 32 cy  → opLat = 35

v-to-v chain penalty:
  ROB bypass fires combinatorially 1 cycle before VRF write.
  → Effective v-to-v = 3 cy  (vs. 4 cy without bypass)
  → Modelled as: srcRegsRelativeLats=[2, 2] with opLat=5
    (consumer can issue at: producerIssueCycle + 5 - 2 = +3)

Multi-uop (LMUL>1 or vl > VLEN/EEW):
  gem5 generates one micro-op per LMUL group; pipelined with issueLat=1.
  Total ≈ 5 + (N_uops - 1)  — matches RTL formula.
```

---

## 5. Complete FU Pool Reference

| FU | Instances | `opLat` | `issueLat` | `extraAssumedLat` | Instructions |
|----|-----------|---------|-----------|------------------|-------------|
| ALU (scalar) | 4 | 1 | 1 | 0 | add, sub, and, or, xor, sll, slt, … |
| MLU (scalar) | 1 | 3 | 1 | 0 | mul, mulh, mulhu, mulhsu |
| DVU (scalar) | 1 | 34 | 34 | 0 | div, divu, rem, remu |
| FPU (scalar) | 1 | 3 | 1 | 0 | fadd, fsub, fmul, fdiv, … |
| LSU (shared) | 1 | 1 | 1 | 2 | lw/sw + all vle/vse patterns |
| CSR / System | 1 | 1 | 1 | 0 | csrrw, fence, wfi, mpause, … |
| VEC_ALU | 2 | 5 | 1 | 0 | vadd, vand, vcmp, vshift, vsetvl, … |
| VEC_MUL | 2 | 5 | 1 | 0 | vmul, vmacc, vfadd, vfmul, … |
| VEC_DIV | 1 | 35 | 35 | 0 | vdiv, vfsqrt, … |

---

## 6. UART Console — Firmware printf Support

### 6.1 Background

CoralNPU firmware implements `printf` (and all character output) by storing a
single byte to a fixed MMIO address, typically `0xFFFFFFF8`, which maps to the
UART TX register in hardware:

```c
// Compiler-generated equivalent of putchar(c):
sb  a0, -8(zero)      # store byte to 0xFFFFFFF8
```

The default gem5 memory map has no device at that address, so any such store
causes a bus error and aborts the simulation.  A new `UartConsole` PIO device
is added to intercept those writes and print the character to host stdout.

### 6.2 New files

#### `src/dev/coralnpu/uart_console.hh`

```cpp
class UartConsole : public BasicPioDevice {
public:
    PARAMS(UartConsole);
    UartConsole(const Params &p);
    Tick read(PacketPtr pkt) override;   // returns 0 (TX-ready)
    Tick write(PacketPtr pkt) override;  // putchar(byte); fflush on '\n'
};
```

#### `src/dev/coralnpu/uart_console.cc`

```cpp
#include "dev/coralnpu/uart_console.hh"

#include <cstdio>

#include "mem/packet.hh"
#include "mem/packet_access.hh"   // required: defines getLE/setLE inline bodies
```

`mem/packet_access.hh` **must** be included directly in the `.cc` file.
`io_device.hh` only includes `mem/tport.hh`; it does not pull in
`packet_access.hh`, so the compiler never sees the inline template definitions
for `getLE`/`setLE` without this explicit include (see §11.2).

Key logic in `write()`:

```cpp
Tick UartConsole::write(PacketPtr pkt) {
    if (pkt->getSize() >= 1) {
        char c = static_cast<char>(pkt->getLE<uint8_t>());
        putchar(c);
        if (c == '\n') fflush(stdout);
    }
    pkt->makeAtomicResponse();
    return pioDelay;
}
```

`fflush` is called on newline rather than every byte to avoid per-character
flush overhead on long output.

#### `src/dev/coralnpu/UartConsole.py`

```python
class UartConsole(BasicPioDevice):
    type      = 'UartConsole'
    cxx_header = 'dev/coralnpu/uart_console.hh'
    cxx_class  = 'gem5::UartConsole'
    pio_size   = Param.Addr(8, "Size of the MMIO window in bytes")
```

`pio_addr` and `pio_latency` are inherited from `BasicPioDevice`.

#### `src/dev/coralnpu/SConscript`

```python
Import('*')
if not env['CONF']['USE_RISCV_ISA']:
    Return()
SimObject('UartConsole.py', sim_objects=['UartConsole'], tags=['riscv isa'])
Source('uart_console.cc', tags=['riscv isa'])
```

gem5's `src/SConscript` uses `os.walk` to auto-discover all `SConscript` files
in subdirectories — no parent file modification is needed.

### 6.3 Modified files

#### `configs/coralnpu/coralnpu_se.py`

Two changes:

**1. Import `UartConsole`:**

```diff
 from m5.objects import (
     AddrRange,
     Cache,
     ...
+    UartConsole,
     VoltageDomain,
 )
```

**2. Add `--uart-addr` argument and device instantiation in `build_system()`:**

```diff
+p.add_argument("--uart-addr", default="0xFFFFFFF8",
+               help="MMIO address of the UART TX register. "
+                    "Set to 'none' to disable.")
```

```diff
+if args.uart_addr.lower() != "none":
+    uart_addr = int(args.uart_addr, 16)
+    system.uart_console = UartConsole(
+        pio_addr    = uart_addr,
+        pio_size    = 8,
+        pio_latency = "1ns",
+    )
+    system.uart_console.pio = system.membus.mem_side_ports
```

### 6.4 How it works

```
Firmware: sb char, 0xFFFFFFF8
    │
    ▼
CPU → SystemXBar (membus)
    │   address 0xFFFFFFF8 claimed by UartConsole.getAddrRanges()
    ▼
UartConsole::write(pkt)
    │   putchar(byte)
    ▼
host stdout  →  terminal / redirected file
```

Key points:

- `UartConsole` is a `PioDevice` (Programmed I/O Device), the standard gem5
  abstraction for MMIO peripherals.  It self-advertises its address range to the
  membus via `getAddrRanges()`; no entry in `system.mem_ranges` is needed.
- Works in both **atomic** (functional) and **timing** (cycle-accurate MinorCPU)
  modes because `PioDevice` handles the timing wrapper around `read()`/`write()`.
- RISC-V bare-metal firmware runs in M-mode with `satp=0` (bare address
  translation), so VA = PA.  The `sb` instruction reaches the membus as a
  physical write to `0xFFFFFFF8` and is routed to `UartConsole`.
- The `pio_latency="1ns"` models a 1-cycle response (matching a 1 GHz clock);
  this is negligible overhead for character I/O.

---

## 7. Build Instructions

gem5 uses the SCons build system. Build the RISC-V target only.

### Prerequisites (Ubuntu 22.04)

```bash
sudo apt-get install -y \
    build-essential git python3 python3-dev scons \
    libprotobuf-dev protobuf-compiler \
    libgoogle-perftools-dev libboost-all-dev \
    libhdf5-dev pkg-config
pip3 install six
```

### Build

```bash
cd /orange/ZSP/home/cn2095/test_simulator

# Optimised (recommended for most use)
scons build/RISCV/gem5.opt -j$(nproc)

# Debug — full assertions + GDB symbols (slower, for development)
scons build/RISCV/gem5.debug -j$(nproc)

# Fast — no assertions, maximum speed (for long production runs)
scons build/RISCV/gem5.fast -j$(nproc)
```

First build takes 15–30 minutes. Incremental rebuilds are fast.

### Verify

```bash
build/RISCV/gem5.opt \
    configs/coralnpu/coralnpu_se.py \
    --cmd tests/test-progs/hello/bin/riscv/linux/hello \
    --cpu atomic
```

---

## 8. Running Simulations

All commands are run from the simulator root:

```bash
cd /orange/ZSP/home/cn2095/test_simulator
```

### 8.1 Functional simulation

Fast correctness check — no pipeline timing, no cycle count.

```bash
build/RISCV/gem5.opt \
    configs/coralnpu/coralnpu_se.py \
    --cmd /path/to/firmware.elf \
    --cpu atomic \
    --freq 1GHz
```

Use this first to confirm the binary produces correct output before investing
time in cycle-accurate runs.

### 8.2 Cycle-accurate scalar simulation

Cycle-accurate timing for RV32IM code (no vector instructions).

```bash
build/RISCV/gem5.opt \
    configs/coralnpu/coralnpu_se.py \
    --cmd /path/to/firmware.elf \
    --cpu minor \
    --freq 1GHz \
    --itcm-start 0x0      --itcm-size 8kB \
    --dtcm-start 0x10000  --dtcm-size 32kB
```

### 8.3 Cycle-accurate simulation with RVV

ELF compiled with `-march=rv32imv_zve32x` or `-march=rv32imv` (VLEN=128, ELEN=32).

The setup is identical to scalar — no additional flags are needed. The
`CoralNPUMinorCPU` already has `enable_rvv=True, vlen=128, elen=32` configured.

```bash
build/RISCV/gem5.opt \
    configs/coralnpu/coralnpu_se.py \
    --cmd /path/to/rvv_firmware.elf \
    --cpu minor \
    --freq 1GHz \
    --itcm-start 0x0      --itcm-size 8kB \
    --dtcm-start 0x10000  --dtcm-size 32kB
```

#### Compiling RVV firmware for CoralNPU

Use a RISC-V GCC or Clang toolchain with the correct march string:

```bash
# GCC — integer vector (no float vector)
riscv32-unknown-elf-gcc -march=rv32imv_zve32x -mabi=ilp32 \
    -O2 -o firmware.elf firmware.c

# GCC — with float vector
riscv32-unknown-elf-gcc -march=rv32imfv -mabi=ilp32f \
    -O2 -o firmware.elf firmware.c

# Clang/LLVM
clang --target=riscv32 -march=rv32imv -mabi=ilp32 \
    -O2 -o firmware.elf firmware.c
```

In firmware, set VL/VTYPE before any vector operation:

```c
// vsetvli — set VL for int32 elements, LMUL=1
size_t vl;
asm volatile("vsetvli %0, %1, e32, m1, ta, ma" : "=r"(vl) : "r"(n));

// Example: vadd.vv
asm volatile("vadd.vv %0, %1, %2" : "=vr"(vd) : "vr"(vs1), "vr"(vs2));
```

#### Custom ITCM/DTCM size

Adjust to match your firmware's linker script:

```bash
build/RISCV/gem5.opt \
    configs/coralnpu/coralnpu_se.py \
    --cmd firmware.elf --cpu minor \
    --itcm-start 0x0       --itcm-size 16kB   \
    --dtcm-start 0x10000   --dtcm-size 64kB   \
    --ext-mem-start 0x80000000 --ext-mem-size 512MB
```

### 8.4 printf / UART address customisation

By default `coralnpu_se.py` instantiates `UartConsole` at `0xFFFFFFF8`.  Use
`--uart-addr` to match a different firmware UART address, or disable it entirely:

```bash
# Default — firmware writes chars to 0xFFFFFFF8
build/RISCV/gem5.opt configs/coralnpu/coralnpu_se.py \
    --cmd firmware.elf --cpu minor

# Different UART TX address
build/RISCV/gem5.opt configs/coralnpu/coralnpu_se.py \
    --cmd firmware.elf --cpu minor \
    --uart-addr 0xFFFF0000

# Disable UART console (firmware has no printf, or uses a different mechanism)
build/RISCV/gem5.opt configs/coralnpu/coralnpu_se.py \
    --cmd firmware.elf --cpu minor \
    --uart-addr none
```

Firmware output appears directly in the terminal alongside gem5's own log lines.
To separate them, redirect gem5's log to a file:

```bash
build/RISCV/gem5.opt --outdir m5out/ \
    configs/coralnpu/coralnpu_se.py \
    --cmd firmware.elf --cpu minor \
    2>gem5.log          # gem5 log → file; firmware printf → terminal
```

### 8.5 Debug flags and instruction trace

```bash
build/RISCV/gem5.opt \
    --debug-flags=MinorExecute,MinorScoreboard \
    --debug-file=coralnpu_trace.txt \
    configs/coralnpu/coralnpu_se.py \
    --cmd firmware.elf --cpu minor
```

Useful debug flags:

| Flag | Output |
|------|--------|
| `MinorExecute` | Per-cycle issue and commit events |
| `MinorScoreboard` | Scoreboard mark/clear per register |
| `MinorTrace` | Full per-instruction pipeline trace |
| `MinorMem` | LSU scalar and vector memory requests |
| `Cache` | L1I/L1D hit/miss per access |
| `Exec` | Instruction operand values at execute (very verbose) |

### 8.6 Limiting simulation length

```bash
# Stop after 1 million committed instructions
build/RISCV/gem5.opt \
    configs/coralnpu/coralnpu_se.py \
    --cmd firmware.elf --cpu minor \
    --max-insts 1000000

# Custom output directory
build/RISCV/gem5.opt --outdir my_results/ \
    configs/coralnpu/coralnpu_se.py \
    --cmd firmware.elf --cpu minor
```

---

## 9. Reading Simulation Output

gem5 writes to `m5out/` (or `--outdir`).

### Key statistics in `m5out/stats.txt`

```
# Throughput
system.cpu.numCycles                       # total simulated cycles
system.cpu.numInsts                        # committed instructions
system.cpu.ipc                             # instructions per cycle

# Pipeline detail
system.cpu.numFetchSuspends                # cycles fetch stalled (I-cache miss)
system.cpu.execute.numIssuedInsts          # total issued (≥ committed)
system.cpu.execute.numFUBusyCycles         # cycles each FU was occupied

# Cache performance
system.cpu.icache.overallMissRate::total   # L1I miss rate
system.cpu.dcache.overallMissRate::total   # L1D miss rate

# Branch prediction
system.cpu.branchPred.condPredicted        # branches predicted
system.cpu.branchPred.condIncorrect        # mispredictions
```

### Quick analysis script

```python
#!/usr/bin/env python3
# parse_stats.py
import re, sys

text = open(sys.argv[1] if len(sys.argv) > 1 else 'm5out/stats.txt').read()

def get(pat):
    m = re.search(pat, text)
    return float(m.group(1)) if m else None

ipc      = get(r'system\.cpu\.ipc\s+([\d.]+)')
cycles   = get(r'system\.cpu\.numCycles\s+([\d.]+)')
insts    = get(r'system\.cpu\.numInsts\s+([\d.]+)')
icache_m = get(r'system\.cpu\.icache\.overallMissRate::total\s+([\d.]+)')
dcache_m = get(r'system\.cpu\.dcache\.overallMissRate::total\s+([\d.]+)')

print(f"IPC           : {ipc:.4f}")
print(f"Cycles        : {int(cycles):,}")
print(f"Instructions  : {int(insts):,}")
print(f"L1I miss rate : {icache_m*100:.2f}%")
print(f"L1D miss rate : {dcache_m*100:.2f}%")
```

```bash
python3 parse_stats.py m5out/stats.txt
```

---

## 10. Timing Accuracy and Limitations

### 10.1 Scalar timing accuracy

For pure scalar RV32IM code the simulation is cycle-accurate for:

- ALU instruction latency (1 cycle)
- MLU latency (3 cycles)
- DVU worst-case latency (34 cycles; actual is data-dependent 32–34)
- RAW hazard stalls (scoreboard)
- Load-use hazards (extraAssumedLat=2 → 3-cycle total load latency from issue)
- Branch misprediction penalty (1 cycle)
- L1I/L1D cache miss stalls (modelled by gem5's cache hierarchy)

Expected accuracy vs. CoralNPU RTL: **< 5% cycle count error** for scalar workloads.

### 10.2 RVV timing accuracy (Level 2)

Level 2 provides **approximate** vector timing, not exact cycle-accurate matching.

**What is modelled correctly:**

| Behaviour | gem5 model |
|-----------|-----------|
| Single-uop ALU/MUL latency (opLat=5) | ✓ matches RTL 5-cy total |
| v-to-v chain penalty (srcRelLats=[2,2] → 3cy) | ✓ matches RTL bypass |
| Multi-uop pipelined throughput (≈ 5+(N-1) cy) | ✓ via issueLat=1 |
| DIV non-pipelined latency (opLat=35) | ✓ approximate |
| VLEN=128 (4 × int32 per register) | ✓ exact |
| Vector memory access patterns (all RVV variants) | ✓ functionally correct |

**What is NOT modelled (Level 2 limitations):**

| Gap | Effect on timing |
|-----|-----------------|
| **Scalar–vector overlap**: In hardware, the scalar core continues executing while the RVV co-processor runs. In gem5, the scalar pipeline stalls waiting for each vector instruction to commit. | IPC underestimated by **20–50%** for mixed scalar/vector workloads |
| **3-uop/cycle dispatch from UQ**: CoralNPU's RVV can dispatch 3 micro-ops per cycle from the Uop Queue. gem5's issue width is the full scalar `executeIssueLimit=4`, but vector uops still serialise through the scalar issue path. | Multi-uop throughput slightly underestimated |
| **Separate ROB for vector**: CoralNPU has an 8-entry vector ROB independent of the 8-entry scalar ROB. gem5 uses one shared commit path. | Vector backpressure on scalar is overestimated |

**Expected accuracy by workload type:**

| Workload | Expected cycle count error vs. RTL |
|----------|------------------------------------|
| Pure scalar (RV32IM) | < 5% |
| Pure vector loop (single-uop per vl) | ~10–15% |
| Pure vector loop (LMUL>1, multi-uop) | ~10–20% |
| Mixed scalar + vector | 20–50% |

### 10.3 Known gaps table

| # | Feature | gem5 behaviour | Impact | Fix effort |
|---|---------|---------------|--------|------------|
| 1 | Scalar–vector decoupling | Scalar stalls on every vector instruction | 20–50% IPC error (mixed) | High: new `RvvCoprocessor` SimObject |
| 2 | Lane-0-only FPU/DVU/CSR | Any lane can issue | Small IPC overestimate | Medium: patch execute.cc |
| 3 | Serialising CSR (drain ROB) | Stalls issue, not drain | Lower CSR stall count | Medium: patch execute.cc |
| 4 | Stripmining (1 front-end → 4 issues) | 1 front-end → 1 issue | IPC overestimate for SIMD | Medium: custom decoder |
| 5 | DVU variable latency (32–34 cy) | Fixed 34 cy worst-case | Slight pessimism on divide | Low: `TimingExpr` |
| 6 | L0 I-cache (1 KB inside Fetch) | L1I only | Minor fetch stall difference | Low: add 2nd cache level |
| 7 | Static branch predictor | `LocalBP` (256-entry, 2-bit counters) | Small misprediction difference | Low: patch pred/ |
| 8 | C-extension encoding reclaimed | RVC still decoded | Wrong for RVC bytes in firmware | Low: reject 16-bit in decoder |
| 9 | Vector ROB (8-entry, independent) | Shared scalar ROB path | Vector backpressure overestimated | High: separate SimObject |

---

## 11. Build Fixes — Errors Encountered and Resolved

This section records the two build errors that appeared during development and
how each was diagnosed and fixed.

---

### 11.1 ISA parser: "instruction format Unknown not defined"

**Full error**

```
In file included /src/arch/riscv/isa/decoder.isa:4:
instruction format "Unknown" not defined.
scons: *** [build/RISCV/arch/riscv/generated/decoder.cc] Explicit exit, status 1
```

**Root cause**

The original implementation defined a custom `CoralNPUOp` format in a new file
`src/arch/riscv/isa/formats/coralnpu.isa` and included it from `formats.isa`
between `m5ops.isa` and `unknown.isa`:

```
##include "m5ops.isa"
##include "coralnpu.isa"   ← new file
##include "unknown.isa"
```

Inside `coralnpu.isa`, the `def format CoralNPUOp() {{ ... }}` block contained
`//` comment lines at the outer Python scope:

```
def format CoralNPUOp() {{
    // Code executed when the instruction fires.   ← ISA-level comment
    code = '''
        // No architectural state is modified.     ← string content (OK)
    '''
    ...
}};
```

gem5's ISA preprocessor strips `//` text **before** passing the block to the
Python interpreter — this is intentional for ISA-level comments. The outer
`// ...` lines were stripped from the Python code, leaving the format body
syntactically incomplete. The ISA parser aborted while processing
`coralnpu.isa`, so `unknown.isa` was never reached.  Because `Unknown` (defined
in `unknown.isa`) was never registered, any use of it as a decode default
produced the error.

**Diagnosis**

The error points to `decoder.isa` because `Unknown::unknown()` is the default
in the top-level decode block (`decode QUADRANT default Unknown::unknown()`).
The `decoder.isa` itself was not the problem — the problem was that `Unknown`
was never defined due to the aborted `formats.isa` parse.

**Fix**

Dropped the custom format file entirely. `mpause` reuses the pre-existing
`SystemOp` format (defined in `formats/standard.isa`, already included) with
an empty C++ code body — the identical pattern used by `ecall`, `ebreak`, and
`wfi`:

```diff
- // in formats/formats.isa
- ##include "coralnpu.isa"          ← removed

  // in decoder.isa (inside 0x3: decode OPCODE5 {})
- 0x02: CoralNPUOp::mpause();
+ 0x02: SystemOp::mpause({{ }}, IsNonSpeculative, IsSerializeAfter, No_OpClass);
```

**Rule for future ISA format files**

Inside `def format Name() {{ ... }}`, use Python `#` for comments, not `//`.
`//` is valid only at ISA scope (outside `{{ }}` blocks) where it is
preprocessed as an ISA-level comment.

---

### 11.2 Linker: undefined reference to `getLE` / `setLE`

**Full error**

```
/usr/bin/ld: build/RISCV/dev/coralnpu/uart_console.o:
  uart_console.cc:26: undefined reference to
    `unsigned char gem5::Packet::getLE<unsigned char>() const'
/usr/bin/ld: build/RISCV/dev/coralnpu/uart_console.o:
  uart_console.cc:17: undefined reference to
    `void gem5::Packet::setLE<unsigned long>(unsigned long)'
collect2: error: ld returned 1 exit status
```

**Root cause**

`Packet::getLE<T>()` and `Packet::setLE<T>()` are declared in `mem/packet.hh`
but their **inline template bodies** are defined in a separate header,
`mem/packet_access.hh`:

```
mem/packet.hh      →  T getLE() const;          (declaration only)
mem/packet_access.hh → inline T Packet::getLE()  (body — must be included)
```

`uart_console.cc` originally included only `dev/coralnpu/uart_console.hh`,
which transitively pulls in `dev/io_device.hh`. Checking `io_device.hh`
reveals it includes only `mem/tport.hh`, `params/*.hh`, and
`sim/clocked_object.hh` — **not** `mem/packet_access.hh`. Because the inline
bodies were never visible during compilation, the compiler emitted external
references for the template specialisations instead of generating them inline,
and the linker could not resolve them.

**Diagnosis**

Every other device that uses `getLE`/`setLE` (e.g. `dev/isa_fake.cc`) includes
`mem/packet_access.hh` explicitly in the `.cc` file, independent of what the
header chain provides. Comparing `isa_fake.cc`'s include list against
`uart_console.cc`'s include list revealed the missing include.

**Fix**

Added the two missing includes directly in `uart_console.cc`:

```diff
  #include "dev/coralnpu/uart_console.hh"
  #include <cstdio>
+ #include "mem/packet.hh"
+ #include "mem/packet_access.hh"
```

**Rule for future device implementations**

Always include `mem/packet_access.hh` explicitly in any `.cc` file that calls
`pkt->getLE<T>()`, `pkt->setLE<T>()`, `pkt->getBE<T>()`, or `pkt->setBE<T>()`.
Do not rely on transitive inclusion through device or port headers.

---

### 11.3 AttributeError: Invalid assignment for LocalBP parameter `localHistoryTableSize`

**Full error**

```
AttributeError: Invalid assignment for Class LocalBP with parameter localHistoryTableSize
At:
  configs/coralnpu/coralnpu_cpu.py(374): CoralNPUMinorCPU
```

**Root cause**

The `CoralNPUMinorCPU` branch predictor was configured with four `LocalBP`
parameters copied from a different gem5 version's API:

```python
branchPred = BranchPredictor(
    conditionalBranchPred=LocalBP(
        localPredictorSize=256,
        localHistoryTableSize=256,   # ← does not exist in this build
        localCtrBits=2,
        numThreads=1,                # ← does not exist in this build
    )
)
```

In this gem5 version, `LocalBP` (defined in
`src/cpu/pred/BranchPredictor.py`) exposes only two parameters:

| Parameter | Type | Default | Meaning |
|-----------|------|---------|---------|
| `localPredictorSize` | `Unsigned` | 2048 | Number of PHT entries |
| `localCtrBits` | `Unsigned` | 2 | Saturating counter width |

`localHistoryTableSize` belongs to `TournamentBP`, not `LocalBP`. `numThreads`
is inherited from a parent class and is not settable at construction in this
version.

**Fix**

Removed the two invalid parameters from the `LocalBP` instantiation in
`configs/coralnpu/coralnpu_cpu.py`:

```diff
  branchPred = BranchPredictor(
      conditionalBranchPred=LocalBP(
          localPredictorSize=256,
-         localHistoryTableSize=256,
          localCtrBits=2,
-         numThreads=1,
      )
  )
```

The 256-entry PHT with 2-bit counters is retained — this matches the CoralNPU
scalar core's small local predictor. The history table size in `LocalBP` is
implicitly determined by `localPredictorSize`; there is no separate knob.

---

### 11.4 Panic: Stats of the same group share the same name `opClasses`

**Full error**

```
src/base/stats/group.cc:121: panic: panic condition
  statGroups.find(name) != statGroups.end() occurred:
  Stats of the same group share the same name `opClasses`.
Memory Usage: 687772 KBytes
Program aborted at tick 0
```

**Root cause**

The FU pool was defined by placing the **same Python SimObject** in multiple
pool slots:

```python
# WRONG — same object referenced N times
_CoralNPU_ALU = MinorFU(opClasses=..., opLat=1, ...)

CoralNPU_FUPool = MinorFUPool(funcUnits=[
    _CoralNPU_ALU,   # slot 0 \
    _CoralNPU_ALU,   # slot 1  }- all three point to the same C++ object
    _CoralNPU_ALU,   # slot 2 /
    ...
    _CoralNPU_VEC_ALU,  # RVV ALU slot 0 \- same object in two slots
    _CoralNPU_VEC_ALU,  # RVV ALU slot 1 /
    ...
])
```

In gem5, each Python `SimObject` instance maps to exactly one C++ object.
When the same Python object appears twice in the `funcUnits` list, gem5
instantiates the underlying C++ `MinorFU` object once and adds it to the
stats hierarchy twice — with the same parent-relative name `opClasses` both
times. The stats subsystem detects the collision and calls `panic`.

**Diagnosis**

The backtrace shows `statistics::Group::addStatGroup` → `panic`. The stat
group name `opClasses` is registered by the `MinorFU` C++ constructor.
The collision occurs at `m5.instantiate()` (tick 0) because that is when
Python SimObjects are realised into C++ objects.

**Fix**

Replaced the repeated singleton objects with **factory functions** that return
a fresh `MinorFU` instance on each call:

```diff
- _CoralNPU_ALU = MinorFU(opClasses=..., opLat=1, ...)
+ def _make_CoralNPU_ALU():
+     return MinorFU(opClasses=..., opLat=1, ...)

- _CoralNPU_VEC_ALU = MinorFU(opClasses=..., opLat=5, ...)
+ def _make_CoralNPU_VEC_ALU():
+     return MinorFU(opClasses=..., opLat=5, ...)

- _CoralNPU_VEC_MUL = MinorFU(opClasses=..., opLat=5, ...)
+ def _make_CoralNPU_VEC_MUL():
+     return MinorFU(opClasses=..., opLat=5, ...)

  CoralNPU_FUPool = MinorFUPool(funcUnits=[
-     _CoralNPU_ALU, _CoralNPU_ALU, _CoralNPU_ALU, _CoralNPU_ALU,
+     _make_CoralNPU_ALU(), _make_CoralNPU_ALU(),
+     _make_CoralNPU_ALU(), _make_CoralNPU_ALU(),
      _CoralNPU_MLU, _CoralNPU_DVU, _CoralNPU_FPU,
      _CoralNPU_LSU, _CoralNPU_CSR,
-     _CoralNPU_VEC_ALU, _CoralNPU_VEC_ALU,
+     _make_CoralNPU_VEC_ALU(), _make_CoralNPU_VEC_ALU(),
-     _CoralNPU_VEC_MUL, _CoralNPU_VEC_MUL,
+     _make_CoralNPU_VEC_MUL(), _make_CoralNPU_VEC_MUL(),
      _CoralNPU_VEC_DIV,
  ])
```

Single-instance FUs (`_CoralNPU_MLU`, `_CoralNPU_DVU`, `_CoralNPU_FPU`,
`_CoralNPU_LSU`, `_CoralNPU_CSR`, `_CoralNPU_VEC_DIV`) appear only once in
the pool so they are unaffected and remain as module-level objects.

**Rule for future FU pool definitions**

Never place the same Python `MinorFU` object in more than one pool slot.
Use a factory function (`def _make_X(): return MinorFU(...)`) for every FU
type that needs more than one instance.

---

### 11.5 Fatal: Out of memory, please increase size of physical memory

**Full error**

```
src/sim/mem_pool.cc:100: fatal: Out of memory, please increase size of physical memory.
Memory Usage: 689824 KBytes
Program aborted at tick 0
```

**Root cause**

gem5 SE mode performs its own OS-emulation memory management on top of the
declared `system.mem_ranges`.  For a 32-bit RISC-V process, gem5 places the
user stack and heap at fixed virtual addresses regardless of the firmware's
linker script:

| Region | Address range | Size |
|--------|--------------|------|
| Stack base | `0x7FFFFFFF` (growing downward) | 64 MiB max |
| Stack bottom | `0x40000000` | — |
| Heap / mmap | ELF image end → `0x40000000` | variable |

With the user's memory map:

| Region | Start | End |
|--------|-------|-----|
| ITCM | `0x00000000` | `0x000FFFFF` |
| DTCM | `0x00100000` | `0x001FFFFF` |
| *(gap)* | `0x00200000` | `0x7FFFFFFF` |
| ExtMem | `0x80000000` | `0x9FFFFFFF` |

The entire gem5 SE stack and heap (`0x00200000–0x7FFFFFFF`) falls in the
unmapped gap between DTCM and ExtMem.  When `m5.instantiate()` tries to
allocate physical pages for the user stack, the memory pool finds no pool
covering that range and calls `fatal("Out of memory")`.

This is a property of gem5 SE mode, not of the firmware itself.  The
firmware's own stack (set in the linker script) is within DTCM; the failing
allocation is gem5's OS-emulation stack.

**Fix**

Compute all memory ranges — including the gap fill — upfront and set
`system.mem_ranges` in a **single assignment** before the `System` object
is constructed.  Re-assigning `system.mem_ranges` after initial set does not
reliably propagate through gem5's `VectorParam`, so appending later is not
a valid approach.

```python
from m5.util.convert import toMemorySize   # added to imports

# At the top of build_system(), before System() is constructed:
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
system.mem_ranges = _mem_ranges   # single assignment — includes gap

# Later, after the bus is created, wire the backing SimpleMemory:
if _has_gap:
    proc_mem = SimpleMemory(
        range=AddrRange(start=_dtcm_end, size=ext_mem_start - _dtcm_end),
        latency="1ns", bandwidth="32GB/s",
    )
    system.proc_mem = proc_mem
    system.membus.mem_side_ports = proc_mem.port
```

**Why this does not waste host RAM**

`SimpleMemory` backs its address space with `mmap(MAP_ANONYMOUS | MAP_NORESERVE)`.
The host kernel does not commit physical pages until they are first touched.
For the user's default gap (`0x200000`–`0x80000000` ≈ 2 GB), only the pages
that the gem5 SE stack, heap, and loader actually access are committed — in
practice a few MiB for a simple firmware binary.

**Rule**

When declaring a non-contiguous CoralNPU memory map with a gap between DTCM
and ExtMem, gem5 SE mode always needs backing memory across the entire
32-bit user address space below ExtMem.  The gap-fill is now automatic in
`coralnpu_se.py` and requires no user action.

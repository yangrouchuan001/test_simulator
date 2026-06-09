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
    - 11.6 [Panic: minstret is not accessible in 0](#116-panic-minstret-is-not-accessible-in-0)
    - 11.7 [warn: Ignoring write to miscreg INSTRET / panic: Page table fault at 0x2000e4](#117-warn-ignoring-write-to-miscreg-instret--panic-page-table-fault-when-accessing-virtual-address-0x2000e4)
    - 11.8 [Simulation stall — only 10 instructions committed in 378M cycles (IsSerializeAfter on CSR instructions)](#118-simulation-stall--only-10-instructions-committed-in-378-million-cycles-isserializeafter-on-csr-instructions)
    - 11.9 [Panic: Page table fault when accessing virtual address in TCM range (fixupFault conf-only check)](#119-panic-page-table-fault-when-accessing-virtual-address-in-tcm-range-fixupfault-conf-only-check)
    - 11.10 [Panic: mstatus is not accessible in 0 (AtomicSimpleCPU missing privilege_mode_set="M")](#1110-panic-mstatus-is-not-accessible-in-0-atomicsimplecpu-missing-privilege_mode_setm)
    - 11.11 [Illegal instruction machInst with vill=1 after vsetvli (missing branchTarget propagation)](#1111-illegal-instruction-machinst-with-vill1-after-vsetvli-missing-branchtarget-propagation)
    - 11.12 [Illegal instruction / instBits=0 at mid-block PC (cache_line_size vs fetch request size mismatch)](#1112-illegal-instruction--instbits0-at-mid-block-pc-cache_line_size-vs-fetch-request-size-mismatch)
    - 11.13 [Panic: Page table fault at 0xfffffffffffffff8 (RV32 UART MMIO sign-extension + missing identity-map)](#1113-panic-page-table-fault-at-0xfffffffffffffff8-rv32-uart-mmio-sign-extension--missing-identity-map)
    - 11.14 [Fatal: xbar unable to find destination for \[0xffffffe0:0x100000000\] (UART device covers < one cache line)](#1114-fatal-xbar-unable-to-find-destination-for-0xffffffe00x100000000-uart-device-covers--one-cache-line)
    - 11.15 [Missing output and stats — simulation exits via SIGTRAP when firmware uses ebreak as exit](#1115-missing-output-and-stats--simulation-exits-via-sigtrap-when-firmware-uses-ebreak-as-exit)
    - 11.16 [printf output missing in MinorCPU timing mode — MMIO identity-map created cacheable, stores absorbed by L1D cache](#1116-printf-output-missing-in-minorcpu-timing-mode--mmio-identity-map-created-cacheable-stores-absorbed-by-l1d-cache)
    - 11.17 [gem5 cycle count lower than Verilator — vector load latency underestimated (shared scalar LSU timing)](#1117-gem5-cycle-count-lower-than-verilator--vector-load-latency-underestimated-shared-scalar-lsu-timing)
    - 11.18 [Function-level cycle profiling — `--profile` flag added to coralnpu_se.py](#1118-function-level-cycle-profiling--profile-flag-added-to-coralnpu_sepy)
    - 11.19 [mpause exits simulation cleanly — `executeMpause` added to SystemOp](#1119-mpause-exits-simulation-cleanly--executempause-added-to-systemop)
    - 11.20 [Atomic CPU ISA mismatch — RVV disabled and wrong vlen caused "unknown instruction" panic](#1120-atomic-cpu-isa-mismatch--rvv-disabled-and-wrong-vlen-caused-unknown-instruction-panic)
    - 11.21 [Unknown instruction 0x08000073 — MPAUSE SYSTEM-opcode encoding missing from decoder](#1121-unknown-instruction-0x08000073--mpause-system-opcode-encoding-missing-from-decoder)
    - 11.22 [Pipeline timing improvements — VEC\_PMTRDT non-pipelined FU + branch misprediction penalty corrected](#1122-pipeline-timing-improvements--vec_pmtrdt-non-pipelined-fu--branch-misprediction-penalty-corrected)
    - 11.23 [Timing regression fix — vslide* moved back to VEC\_ALU (pipelined) to compensate for missing scalar-vector overlap](#1123-timing-regression-fix--vslide-moved-back-to-vec_alu-pipelined-to-compensate-for-missing-scalar-vector-overlap)

---

## 1. Overview of All Changes

| File | Type | Purpose |
|------|------|---------|
| `src/arch/riscv/isa/decoder.isa` | **Modified** | Decode `mpause` via existing `SystemOp` format (custom-0, `0x0000000B`); (§11.19) call `executeMpause` to exit simulation cleanly; (§11.21) add `mpause_sys` for SYSTEM-opcode encoding `0x08000073` (`RVTEST_PASS` macro) |
| `src/arch/riscv/insts/standard.hh` | **Modified** | (§11.19) Declare `SystemOp::executeMpause` |
| `src/arch/riscv/insts/standard.cc` | **Modified** | (§11.19) Implement `executeMpause`: calls `exitSimLoop("mpause @ PC", 0)` |
| `src/dev/coralnpu/uart_console.hh` | **New** | `UartConsole` SimObject header |
| `src/dev/coralnpu/uart_console.cc` | **New** | `UartConsole` implementation — prints bytes on write |
| `src/dev/coralnpu/UartConsole.py` | **New** | gem5 Python param class for `UartConsole` |
| `src/dev/coralnpu/SConscript` | **New** | Build registration for the UART device |
| `configs/coralnpu/__init__.py` | **New** | Python package marker |
| `configs/coralnpu/coralnpu_cpu.py` | **New / Modified** | `CoralNPUMinorCPU` class — scalar + RVV FU pool; (§11.17) split LSU; (§11.22) add `VEC_PMTRDT` non-pipelined FU (6 cy, reductions/slides/gather); `executeBranchDelay=2` (RTL: 2 fetch-flush cycles) |
| `configs/coralnpu/coralnpu_se.py` | **New / Modified** | Simulation entry-point; adds `--uart-addr`, `UartConsole`, `system.cache_line_size=32`, `--profile`; (§11.20) atomic CPU ISA aligned with MinorCPU (`enable_rvv=True, vlen=128, elen=32`) |
| `src/arch/riscv/isa/formats/vector_conf.isa` | **Modified** | Add `VSetVliDeclare` / `VSetVliBranchTarget` templates; extend `VConfOp` to call `branchTarget` for `vsetvli` |
| `src/arch/riscv/tlb.cc` | **Modified** | (§11.13) Pass RV32-masked `vaddr` to `GenericPageTableFault`; (§11.16) SE translate path uses `pTable->lookup()` to read `Uncacheable` flag and sets `Request::UNCACHEABLE` so MMIO stores bypass the L1D cache |
| `src/sim/mem_state.cc` | **Modified** | Add MMIO identity-map fallback in `fixupFault` so PIO device addresses (UART, etc.) are accessible from firmware; map created with `cacheable=false` so stores bypass L1D cache and reach the device immediately |
| `src/dev/coralnpu/uart_console.cc` | **Modified** | Fix read handler to zero full packet buffer (`memset`) instead of overflowing 8-byte write; handles 32-byte L1I cache-line fills |
| `src/arch/riscv/faults.cc` | **Modified** | `BreakpointFault::invokeSE`: replace `schedRelBreak(0)` with `exitSimLoop("ebreak", 0)` so bare-metal `ebreak` exits the simulation cleanly instead of killing the process via SIGTRAP |
| `src/cpu/minor/execute.cc` | **Modified** | `doInstCommitAccounting`: add `cpu.traceFunctions(inst->pc->instAddr())` so MinorCPU feeds PC crossings to the built-in function tracer (O3/Simple CPUs already called it; MinorCPU never did) |

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

This section records the build and runtime errors that appeared during development
and how each was diagnosed and fixed.

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

This error occurred at tick 0 on every simulation run, regardless of the
binary being simulated.  Three fix attempts were needed before the root cause
was fully understood.

---

#### Background: how gem5 SE builds memory pools

`SEWorkload::setSystem()` (in `src/sim/se_workload.cc`) builds its `MemPools`
object as follows:

```cpp
AddrRangeList memories = sys->getPhysMem().getConfAddrRanges();
memPools.populate(memories);
```

`MemPools::populate()` iterates the list in order and creates one pool per
entry:

```cpp
void MemPools::populate(const AddrRangeList &memories) {
    for (const auto &mem : memories)
        pools.emplace_back(pageShift, mem.start(), mem.end());
}
```

`Process::allocateMem()` always calls `allocPhysPages(pool_id=0)` — it draws
all physical pages from **pool 0 only**.

For a 32-bit RISC-V process, gem5 pre-reserves a large fixed SE stack:

| Region | Address range | Pages needed |
|--------|--------------|-------------|
| Stack base | `0x7FFFFFFF` (growing downward) | up to 16 384 (64 MiB) |
| Stack bottom | `0x40000000` | — |
| Heap / mmap | ELF end → `0x40000000` | variable |

All of these allocations come from pool 0.  Pool 0 must therefore be large
enough to hold the full SE process image (ELF + stack + heap).

---

#### Root cause

The key function is `PhysicalMemory::getConfAddrRanges()` in
`src/mem/physical.cc`:

```cpp
PhysicalMemory::getConfAddrRanges() const {
    AddrRangeList ranges;
    for (const auto& r : addrMap) {        // ← addrMap is an interval tree
        if (r.second->isConfReported()) {  // ← only conf-reported memories
            ranges.push_back(r.first);
        }
    }
    return ranges;
}
```

`addrMap` is an `AddrRangeMap` backed by an **interval tree sorted by start
address**.  Iterating it always yields ranges in ascending address order,
regardless of insertion order or the order of `system.mem_ranges`.

`isConfReported()` returns the `conf_table_reported` flag of each
`AbstractMemory` object.  By default, `conf_table_reported=True` for all
`SimpleMemory` objects.

With the default `conf_table_reported=True` on ITCM and DTCM, the pool list
was always (sorted by address):

```
pool 0  →  ITCM    0x00000000 – …   (e.g. 8 kB, 2 pages)     ← pool 0 is tiny!
pool 1  →  DTCM    0x00010000 – …   (e.g. 32 kB, 8 pages)
pool 2  →  gap     0x00018000 – 0x7FFFFFFF   (~2 GB, large)
pool 3  →  ExtMem  0x80000000 – …   (256 MB)
```

`allocPhysPages(16384, pool_id=0)` tried to allocate 16 384 pages from ITCM
(2 pages at 8 kB) and immediately exhausted it.

---

#### Failed attempt 1 — ensure gap fill is in `system.mem_ranges`

The gap fill was originally appended to `system.mem_ranges` after the initial
`system = System()` construction, which gem5's `VectorParam` does not reliably
propagate.  Fixing this (single-assignment with gap fill included) was
necessary but not sufficient: pool 0 was still ITCM (smallest start address).

#### Failed attempt 2 — reorder `system.mem_ranges` to put gap fill first

Putting the gap fill first in `_mem_ranges` had no effect on pool ordering
because `getConfAddrRanges()` ignores `system.mem_ranges` entirely — it builds
the list from the address-sorted `addrMap` interval tree.  The pool order is
always determined by start address, not by the Python declaration order.

---

#### Fix — `conf_table_reported=False` on ITCM and DTCM

The `conf_table_reported` flag on an `AbstractMemory` controls whether
`isConfReported()` returns `True` and therefore whether the memory appears
in `getConfAddrRanges()`.  Setting it `False` on ITCM and DTCM removes them
from the pool list entirely:

```python
itcm = SimpleMemory(
    range=AddrRange(start=itcm_start, size=args.itcm_size),
    latency="1ns",
    bandwidth="32GB/s",
    conf_table_reported=False,   # ← exclude from MemPools
)

dtcm = SimpleMemory(
    range=AddrRange(start=dtcm_start, size=args.dtcm_size),
    latency="1ns",
    bandwidth="32GB/s",
    conf_table_reported=False,   # ← exclude from MemPools
)
```

With ITCM and DTCM excluded, the pool list becomes:

```
pool 0  →  gap fill  0x00200000 – 0x7FFFFFFF   (~2 GB)   ← large enough ✓
pool 1  →  ExtMem    0x80000000 – …            (512 MB)
```

(If no gap exists — i.e. DTCM end ≥ ExtMem start — pool 0 becomes ExtMem,
which is 256 MB+ and also large enough.)

The gap fill is still backed by `proc_mem` SimpleMemory (which retains the
default `conf_table_reported=True`), so it appears in the pool and provides
physical memory for all SE allocations.

`system.mem_ranges` is restored to natural address order (ITCM, DTCM, gap,
ExtMem) since pool ordering no longer depends on it.

---

**Why this is safe: ITCM/DTCM still function normally**

`conf_table_reported` only controls pool registration.  Bus routing is
independent: `SimpleMemory` advertises its address range via `getAddrRanges()`
on its port, and the `SystemXBar` routes physical accesses accordingly.
ITCM and DTCM still respond to any bus access whose physical address falls in
their configured ranges.

In SE mode, however, physical addresses are allocated from pool 0 (gap fill).
The `EmulationPageTable` maps firmware virtual addresses (e.g. `VA=0x0` for
the text segment) to physical pages in the gap fill.  When the CPU fetches
from `VA=0x0`, the TLB translates to a gap fill physical address and the bus
routes to `proc_mem`.  ITCM SimpleMemory is never reached in SE mode; it
exists on the bus but is effectively dormant.  The `latency="1ns"` on
`proc_mem` matches ITCM's latency, so timing is identical.

**Why this does not waste host RAM**

`SimpleMemory` backs its address space with
`mmap(MAP_ANONYMOUS | MAP_NORESERVE)`.  The host kernel commits physical RAM
only on first access.  For the ~2 GB gap fill, only pages actually touched by
the SE loader, stack, and heap are committed — typically a few MiB.

---

**Rule**

In gem5 SE mode, pool ordering is determined by the **start address** of each
`conf_table_reported=True` `SimpleMemory` object, not by `system.mem_ranges`
declaration order.  For a CoralNPU memory map where ITCM/DTCM have low
addresses but small sizes, set `conf_table_reported=False` on both TCMs so
the large gap fill (or ExtMem) becomes pool 0.

---

### 11.6 Panic: minstret is not accessible in 0

**Full error**

```
src/arch/riscv/faults.cc:299: panic: Illegal instruction 0x10000100b0205073
    at pc (0=>0x4).(0=>1): minstret is not accessible in 0
```

The simulation aborts at tick 0, on the second instruction executed (PC = 0x4).

---

#### Root cause — three-level chain

**Level 1 — RISC-V ISA specification**

CSR address bits [9:8] encode the minimum privilege required for access:

| bits[9:8] | Value | Minimum privilege |
|-----------|-------|------------------|
| `00` | 0 | U-mode (accessible from any mode) |
| `01` | 1 | S-mode or higher |
| `10` | 2 | H-mode or higher |
| `11` | 3 | M-mode only |

`minstret` is CSR `0xB02`.  Bits [9:8] = `0b10` → minimum privilege = M-mode (PRV_M = 3).
Any access from a lower privilege level is an illegal instruction per the spec.

The failing instruction at PC = 0x4 is:

```
0xb0205073  →  csrrs x0, minstret, x0
```

(Opcode `0x73` = SYSTEM, funct3 `010` = CSRRS, CSR field = `0xB02` = minstret.
With rd = x0 and rs1 = x0 this is a read-and-discard, used in firmware as a
lightweight pipeline serialisation point.)

**Level 2 — gem5 SE mode forces U-mode**

`RiscvProcess32::initState()` in `src/arch/riscv/process.cc` unconditionally
sets the privilege register to U-mode immediately after loading the ELF:

```cpp
// process.cc (before fix) — runs for every RV32 SE workload
tc->setMiscRegNoEffect(MISCREG_PRV, PRV_U);    // overrides hardware reset (M-mode)
MISA misa = tc->readMiscRegNoEffect(MISCREG_ISA);
fatal_if(!(misa.rvu && misa.rvs),
    "RISC V SE mode can't run without supervisor and user privilege modes.");
```

This is correct for Linux user-space programs but wrong for bare-metal firmware
that expects to execute in M-mode throughout.

**Level 3 — CSR access check fires**

The CSR execute template in
`src/arch/riscv/isa/formats/standard.isa` checks:

```cpp
auto lowestAllowedMode = (PrivilegeMode)bits(csr, 9, 8);  // 0xB02[9:8] = 2 → PRV_M = 3
auto pm = (PrivilegeMode)xc->readMiscReg(MISCREG_PRV);    // PRV_U = 0 (set by process.cc)

if (pm < lowestAllowedMode) {   // 0 < 3 → true
    return std::make_shared<IllegalInstFault>(
        csprintf("%s is not accessible in %s\n", csrName, pm), machInst);
}
```

`IllegalInstFault::invokeSE()` in `faults.cc:299` converts this to a
`panic("Illegal instruction ... : minstret is not accessible in 0")`.

---

#### Fix

Two coordinated changes are required.

**Change 1: `src/arch/riscv/process.cc`**

Make the U-mode downgrade conditional on whether the MISA actually includes
user mode (`misa.rvu`).  When MISA is M-only (`misa.rvu = 0`), the process
stays in PRV_M (the hardware reset state), allowing M-mode CSR access.

```diff
 void RiscvProcess32::initState()
 {
     Process::initState();
     argsInit<uint32_t>(PageBytes);
     for (ContextID ctx: contextIds) {
         auto *tc = system->threads[ctx];
-        tc->setMiscRegNoEffect(MISCREG_PRV, PRV_U);
         auto *isa = dynamic_cast<ISA*>(tc->getIsaPtr());
         fatal_if(isa->rvType() != RV32, "RISC V CPU should run in 32 bits mode");
         MISA misa = tc->readMiscRegNoEffect(MISCREG_ISA);
-        fatal_if(!(misa.rvu && misa.rvs),
-            "RISC V SE mode can't run without supervisor and user "
-            "privilege modes.");
+        if (misa.rvu) {
+            tc->setMiscRegNoEffect(MISCREG_PRV, PRV_U);
+            fatal_if(!misa.rvs,
+                "RISC V SE mode can't run without supervisor and user "
+                "privilege modes.");
+        }
+        // M-only ISA: keep PRV_M from reset for bare-metal firmware.
     }
 }
```

The same change is applied to `RiscvProcess64::initState()`.
This preserves existing behaviour for all Linux user-space workloads (which
always use `privilege_mode_set="MSU"` → `misa.rvu = 1`).

**Change 2: `configs/coralnpu/coralnpu_cpu.py`**

Add `privilege_mode_set="M"` to `CoralNPUMinorCPU`'s ISA configuration:

```diff
 isa = [RiscvISA(
     riscv_type        = "RV32",
     enable_rvv        = True,
     vlen              = 128,
     elen              = 32,
+    privilege_mode_set = "M",   # bare-metal M-mode only; no S/U modes
 )]
```

`PrivilegeModeSet="M"` (defined in `src/arch/riscv/RiscvISA.py`) causes
`ISA::clear()` to leave `misa.rvu = 0` and `misa.rvs = 0`.  The patched
`process.cc` then sees `misa.rvu = 0` and skips the U-mode downgrade,
leaving the process in PRV_M.

With `pm = PRV_M = 3`, the CSR access check `pm < lowestAllowedMode`
becomes `3 < 3` → false → `minstret` is accessible.

**Rebuild required** — `process.cc` is a C++ source file:

```bash
cd /orange/ZSP/home/cn2095/test_simulator
scons build/RISCV/gem5.opt -j$(nproc)
```

---

**Why `privilege_mode_set="M"` is architecturally correct for CoralNPU**

CoralNPU's scalar core implements RV32IM with M-mode privilege only — there is
no S-mode or U-mode in the hardware.  Bare-metal firmware runs entirely in
M-mode and freely accesses M-mode CSRs (`minstret`, `mstatus`, `mcause`, …).
Setting `privilege_mode_set="M"` in gem5 aligns the simulation privilege model
with the hardware reality.

The gem5 SE mode features that require S/U modes (Linux syscall emulation,
page-table-based virtual memory) are not used by CoralNPU firmware, which
performs I/O through the `UartConsole` MMIO device and terminates via `m5_exit`
or by reaching end-of-program.

**Rule**

For any gem5 SE simulation of bare-metal RISC-V firmware that runs in M-mode:
set `privilege_mode_set="M"` in the ISA configuration.  This, combined with
the patched `process.cc`, keeps the simulated privilege at PRV_M and allows
unrestricted M-mode CSR access.

---

### 11.7 warn: Ignoring write to miscreg INSTRET / panic: Page table fault when accessing virtual address 0x2000e4

After fixing problems 11.5 and 11.6 and rebuilding, two new issues appear when
running with larger TCM sizes (e.g. `--itcm-size 1024kB --dtcm-size 1024kB`):

```
src/arch/riscv/isa.cc:707: warn: Ignoring write to miscreg INSTRET.
src/arch/riscv/isa.cc:707: warn: Ignoring write to miscreg INSTRET.
src/sim/faults.cc:103: panic: panic condition !handled occurred:
    Page table fault when accessing virtual address 0x2000e4
```

---

#### Warning: "Ignoring write to miscreg INSTRET"

**Root cause (benign)**

Firmware writes to `minstret` (CSR 0xB02) to reset the retire counter before
timing a code region.  gem5 manages the instruction-retire counter internally
and explicitly ignores writes to `INSTRET`/`MINSTRET` in
`src/arch/riscv/isa.cc:707`.  The counter continues to increment correctly
for reads.

**Impact:** None.  The warning is purely informational.  The simulation remains
correct; only the firmware's software reset of the counter is silently dropped,
which means the counter reflects total retired instructions from the start of
the program rather than from the firmware's chosen epoch.

**Fix:** No code change required.  The warning can be suppressed in production
runs by disabling the `Warn` debug channel, but the simulation result is
unaffected.

---

#### Panic: Page table fault when accessing virtual address 0x2000e4

**Root cause**

VA `0x2000e4` = DTCM-start `0x100000` + DTCM-size `0x100000` (1024 kB) + 228
bytes.  It is 228 bytes **past the end of DTCM**.

The access originates from firmware code that zero-initialises BSS or accesses
a runtime variable whose address sits just beyond the declared TCM boundary.
On real hardware this range is valid memory; in the simulator the memory exists
(`proc_mem` gap fill covers `0x200000–0x7FFFFFFF`) but no virtual→physical
mapping has been established for it.

The gem5 SE-mode fault path is:

```
tlb.cc (SE path, lines 582-610)
  └─ p->pTable->translate(0x2000e4, paddr)  → false   (no mapping exists)
      └─ GenericPageTableFault(0x2000e4)
          └─ faults.cc:101  p->fixupFault(0x2000e4)
              └─ mem_state.cc  MemState::fixupFault()
                  ├─ VMA loop: 0x2000e4 not in any ELF PT_LOAD / mmap VMA
                  ├─ stack check: 0x2000e4 < _stackMin (stack is at ~0x7FFFFFFF) → false
                  └─ return false
          └─ faults.cc:103  !handled → panic
```

`MemState::fixupFault()` only demand-pages addresses that are either (a) in a
registered VMA (ELF segment, mmap), or (b) in the stack growth region.  The
gap-fill memory (`proc_mem`) covers this address in the physical layout, but
no VMA was ever registered for it, so `fixupFault` cannot find it and panics.

**Fix: `src/sim/mem_state.cc`**

Add a third fallback inside `MemState::fixupFault()`, after the stack check,
that demand-pages any address falling within a `conf_table_reported=True`
physical memory range.  ITCM and DTCM are excluded because they carry
`conf_table_reported=False`; only gap fill and ExtMem appear in
`getConfAddrRanges()`.

```diff
 // … existing stack-growth check …
     }

+    // Demand-page any address within a conf-reported physical memory range
+    // (gap fill, ExtMem).  Handles bare-metal firmware accesses to addresses
+    // that were never mmap'd — e.g. BSS past a TCM boundary.
+    // ITCM/DTCM are excluded (conf_table_reported=False).
+    for (const auto &range :
+             _ownerProcess->system->getPhysMem().getConfAddrRanges()) {
+        if (range.contains(vaddr)) {
+            Addr vpage_start = roundDown(vaddr, _pageBytes);
+            _ownerProcess->allocateMem(vpage_start, _pageBytes);
+            return true;
+        }
+    }
+
     return false;
 }
```

**File modified:** `src/sim/mem_state.cc` — inside `MemState::fixupFault()`,
after the `vaddr >= _stackBase - _maxStackSize` block.

No new header includes are needed; `sim/system.hh` (already included) exposes
`System::getPhysMem()`, and `PhysicalMemory::getConfAddrRanges()` is declared
in `mem/physical.hh` (pulled in transitively).

**Why this is safe for normal Linux SE workloads**

For a standard gem5 Linux user-space simulation there is no physical memory
mapped to `conf_table_reported=True` ranges near the faulting address
(`proc_mem` does not exist in that configuration), so `getConfAddrRanges()`
returns only ExtMem (which begins far above typical user-space addresses).  The
new fallback loop finds no matching range and falls through to the original
`return false`, preserving existing behaviour.

**Rebuild required:**

```bash
cd /orange/ZSP/home/cn2095/test_simulator
scons build/RISCV/gem5.opt -j$(nproc)
```

---

### 11.8 Simulation stall — only 10 instructions committed in 378 million cycles (IsSerializeAfter on CSR instructions)

#### Symptom

After fixing the page-table fault (11.7), the simulation runs but hangs: only 10
instructions commit in ~378 million simulated cycles (CPI ≈ 37.8 M), consuming
322 host seconds.  Enabling `--debug-flags=MinorExecute` shows the non-CSR
instructions after the opening CSR block being repeatedly discarded:

```
26000: system.cpu.execute: Discarding inst: 0/2.1/3/12.12 pc: 0x10 (auipc)
       as its stream state was unexpected, expected: 3
```

The same auipc at PC=0x10 is refetched and discarded in every subsequent cycle,
creating an infinite loop.

#### Root cause

Every one of the six CSR instruction variants in `decoder.isa` carried the
`IsSerializeAfter` pipeline flag:

```
format CSROp {
    0x1: csrrw({{ … }}, 'RD != 0', 'true'
          , IsSerializeAfter, IsNonSpeculative, No_OpClass);
    0x2: csrrs({{ … }}, 'true', 'RS1 != 0'
          , IsSerializeAfter, IsNonSpeculative, No_OpClass);
    … (all 6 variants identical)
}
```

In gem5's MinorCPU, `IsSerializeAfter` causes the pipeline to:

1. Commit the flagged instruction.
2. **Increment the global stream sequence number.**
3. Discard every in-flight instruction whose stream ID no longer matches.

The firmware's entry code begins with four consecutive CSR instructions
(PC=0x0–0xC, resetting `minstret` and reading `mhartid`).  Because
`decodeInputWidth=4`, MinorCPU issues all four in the same decode cycle
(tick 18000).  What follows is a cascading flush:

| Tick  | Event |
|-------|-------|
| 18000 | All 4 CSRs issued; auipc at PC=0x10 prefetched with stream ID 1 |
| 19000 | CSR at PC=0x0 commits → stream 1→2; auipc (stream 1) discarded |
| 20000 | CSRs at PC=0x4, 0x8, 0xC commit back-to-back (executeCommitLimit=4) → streams 2→3→4→5; each newly fetched auipc is discarded before it can commit |
| 21000+ | Pipeline stuck: auipc always carries a stale stream ID by commit time |

The result is an infinite discard/refetch loop — the simulation never advances
past the 10th committed instruction.

#### Why `IsSerializeAfter` exists on CSR instructions

The flag was added conservatively: a CSR write that changes SATP or MSTATUS
(e.g. enabling virtual memory or switching privilege mode) **does** require the
pipeline to be flushed, because previously fetched instructions may have been
decoded under wrong privilege or address-translation assumptions.

For general-purpose Linux SE simulations this is overly conservative but harmless
— Linux rarely issues long back-to-back CSR sequences.  For the CoralNPU
bare-metal firmware, which issues four CSRs at startup (counter resets + hartid
read), the cascade becomes a deadlock.

#### Fix

Remove `IsSerializeAfter` from all six CSR instruction definitions in
`src/arch/riscv/isa/decoder.isa`, keeping only `IsNonSpeculative` and
`No_OpClass`:

```diff
--- a/src/arch/riscv/isa/decoder.isa
+++ b/src/arch/riscv/isa/decoder.isa
@@ CSROp format block
-    0x1: csrrw( … , IsSerializeAfter, IsNonSpeculative, No_OpClass);
-    0x2: csrrs( … , IsSerializeAfter, IsNonSpeculative, No_OpClass);
-    0x3: csrrc( … , IsSerializeAfter, IsNonSpeculative, No_OpClass);
-    0x5: csrrwi(… , IsSerializeAfter, IsNonSpeculative, No_OpClass);
-    0x6: csrrsi(… , IsSerializeAfter, IsNonSpeculative, No_OpClass);
-    0x7: csrrci(… , IsSerializeAfter, IsNonSpeculative, No_OpClass);
+    0x1: csrrw( … , IsNonSpeculative, No_OpClass);
+    0x2: csrrs( … , IsNonSpeculative, No_OpClass);
+    0x3: csrrc( … , IsNonSpeculative, No_OpClass);
+    0x5: csrrwi(… , IsNonSpeculative, No_OpClass);
+    0x6: csrrsi(… , IsNonSpeculative, No_OpClass);
+    0x7: csrrci(… , IsNonSpeculative, No_OpClass);
```

`IsNonSpeculative` is retained: it prevents CSR instructions from executing
before all preceding instructions have committed (correct ordering), but does
**not** flush the instructions that follow.

#### Why this is safe for CoralNPU firmware

The firmware runs entirely in M-mode and never:
- Switches between U/S/M privilege modes (no `mret`/`sret` that would require
  re-fetching under a different privilege).
- Enables the MMU (no write to SATP).
- Changes instruction encoding (no `misa` write that would affect decode).

For these CSR writes (counter resets, `mhartid` read) the pipeline needs no
flush — the instructions fetched after the CSR are correct regardless of the
CSR result.  `IsNonSpeculative` alone ensures the CSR executes in-order.

**Note:** If the simulator is later used for Linux full-system or supervisor-mode
workloads that write SATP or MSTATUS, consider restoring `IsSerializeAfter`
specifically for those CSRs (SATP = 0x180, MSTATUS = 0x300, SSTATUS = 0x100).

**File modified:** `src/arch/riscv/isa/decoder.isa` — the `format CSROp` block
(lines ~6384–6413).

**Rebuild required:**

```bash
cd /orange/ZSP/home/cn2095/test_simulator
scons build/RISCV/gem5.opt -j$(nproc)
```

---

### 11.9 Panic: Page table fault when accessing virtual address in TCM range (fixupFault conf-only check)

#### Symptom

```
src/sim/faults.cc:103: panic: panic condition !handled && ...
    Page table fault when accessing virtual address 0x105004
```

Occurs when running a different ELF
(`test_module_conv_1_1_new0.elf`) with larger TCM parameters:

```
--itcm-start 0x0    --itcm-size 1024kB
--dtcm-start 0x100000 --dtcm-size 1024kB
--ext-mem-start 0x80000000 --ext-mem-size 512MB
```

#### Root cause

The ELF memory layout for this binary is:

| PT_LOAD segment | VirtAddr | MemSiz | Covers |
|-----------------|----------|--------|--------|
| .text           | 0x0      | 0x96910 | ITCM [0x0, 0x100000) |
| .data/.bss      | 0x100000 | 0x4a88  | DTCM → ends at **0x104a88** |
| .stack          | 0x1fe000 | 0x2000  | DTCM top |
| .rodata         | 0x80000000 | 0x42056c | ExtMem |
| .heap           | 0x82000000 | 0x1000000 | ExtMem |

VA 0x105004 = 0x104a88 + 0x57c — **1404 bytes past the .data/.bss VMA end**,
but still within the 1 MB DTCM range [0x100000, 0x200000).  The firmware
accesses this address at runtime (likely through newlib malloc/sbrk touching
memory beyond the statically declared BSS extent).

`MemState::fixupFault()` checks, in order:
1. **Registered VMAs** — the PT_LOAD VMA ends at 0x104a88 → miss.
2. **Stack growth region** — not the stack → miss.
3. **`getConfAddrRanges()` loop** (the fix added in §11.7) — returns only
   `conf_table_reported=True` ranges; DTCM has `conf_table_reported=False`
   → miss.
4. `return false` → `panic`.

The §11.7 fallback was necessary but not sufficient: it covered the gap fill
and ExtMem, but not ITCM/DTCM which are intentionally excluded from the
conf-table to avoid becoming pool 0 (see §11.5).

#### Fix

Replace the `getConfAddrRanges()` loop with a single call to
`PhysicalMemory::isMemAddr()`, which checks **all** physical memory ranges
regardless of `conf_table_reported`:

```diff
--- a/src/sim/mem_state.cc
+++ b/src/sim/mem_state.cc
-    // Demand-page any address that falls within a conf-reported physical memory
-    // range (gap fill, ExtMem). ...
-    for (const auto &range :
-             _ownerProcess->system->getPhysMem().getConfAddrRanges()) {
-        if (range.contains(vaddr)) {
-            Addr vpage_start = roundDown(vaddr, _pageBytes);
-            _ownerProcess->allocateMem(vpage_start, _pageBytes);
-            return true;
-        }
-    }
+    // Demand-page any address that falls within any mapped physical memory
+    // range (gap fill, ExtMem, and ITCM/DTCM which have
+    // conf_table_reported=False).  isMemAddr() checks all ranges, not just
+    // conf-table-reported ones, so TCM addresses above the PT_LOAD VMA end
+    // (e.g. runtime BSS, stack, or malloc beyond the declared segment) are
+    // handled correctly.
+    if (_ownerProcess->system->getPhysMem().isMemAddr(vaddr)) {
+        Addr vpage_start = roundDown(vaddr, _pageBytes);
+        _ownerProcess->allocateMem(vpage_start, _pageBytes);
+        return true;
+    }
```

`isMemAddr()` is declared in `mem/physical.hh:214` and returns true if the
address falls within any `SimpleMemory` object attached to the system,
including ITCM and DTCM.

**Why this is safe**

When `allocateMem()` is called for a VA in the DTCM range, it allocates a
physical page from the available SE memory pool (the gap fill `proc_mem`
or ExtMem — whichever pool has space).  The VA maps to a PA in `proc_mem`,
not to DTCM's physical range, so bus transactions for this demand-paged
address go to `proc_mem` rather than the DTCM `SimpleMemory` device.
Functionally the memory is accessible; only the timing model is slightly
approximate (both use 1 ns latency, so the difference is zero in practice).

For addresses that are truly unmapped (e.g. a wild pointer to 0xDEADBEEF),
`isMemAddr()` returns false, fixupFault() returns false, and gem5 still
panics correctly.

**File modified:** `src/sim/mem_state.cc` — the final fallback block in
`MemState::fixupFault()`.

**Rebuild required:**

```bash
cd /orange/ZSP/home/cn2095/test_simulator
scons build/RISCV/gem5.opt -j$(nproc)
```

---

### 11.10 Panic: mstatus is not accessible in 0 (AtomicSimpleCPU missing privilege_mode_set="M")

**Error observed**

```
src/arch/riscv/faults.cc:299: panic: Illegal instruction 0x1000010030002573
    at pc (0x78=>0x7c).(0=>1): mstatus is not accessible in 0
```

Occurs only when running with `--cpu atomic`.  The Minor-CPU path did not
exhibit this error because the Minor-CPU ISA definition in `coralnpu_cpu.py`
already carries `privilege_mode_set="M"`.

**Root cause**

`coralnpu_se.py` created the `RiscvAtomicSimpleCPU` with an inline ISA:

```python
cpu.isa = [RiscvISA(riscv_type="RV32", enable_rvv=False)]
```

Without `privilege_mode_set="M"`, the CPU resets to PRV_U (user mode, mode 0).
`process.cc::RiscvProcess32::initState()` then calls
`tc->setMiscRegNoEffect(MISCREG_PRV, PRV_U)` which stays at mode 0.  The
firmware CSR `csrr a0, mstatus` at PC=0x78 requires M-mode privilege; the
RISC-V ISA layer rejects it with "not accessible in 0".

**Fix**

`configs/coralnpu/coralnpu_se.py`, line ~168:

```diff
-        cpu.isa = [RiscvISA(riscv_type="RV32", enable_rvv=False)]
+        cpu.isa = [RiscvISA(riscv_type="RV32", enable_rvv=False,
+                             privilege_mode_set="M")]
```

This matches the `privilege_mode_set="M"` already present in the Minor-CPU
ISA (`coralnpu_cpu.py`) and in the `process.cc` conditional that skips
the `PRV_U` assignment when `misa.rvu == 0`.

**Files modified:** `configs/coralnpu/coralnpu_se.py`

**Rebuild required:** No — Python-only change; takes effect on next run.

---

### 11.11 Illegal instruction machInst with vill=1 after vsetvli (missing branchTarget propagation)

**Error observed**

```
src/arch/riscv/faults.cc:299: panic: Illegal instruction 0x1000010000000000
    at pc (0xc74=>0xc76).(0=>1): immediate = 0
```

The upper 32 bits of the `ExtMachInst` show `enable_zcd=1` (bit 60) and
`vill=1` (bit 40); the lower 32 bits (`instBits`) are `0x00000000`.  The
decoder interprets 0x0000 as a compressed `C.ADDI4SPN` with imm=0, which is
illegal per the C extension.

**Root cause**

In `src/arch/riscv/isa/formats/vector_conf.isa`, the `VConfOp` format
macro generates a `branchTarget()` method **only for `vsetivli`**:

```python
if "vsetivli" in name:           # ← does NOT match "vsetvli"
    branchTargetTemplate = eval(branch_class)
    exec_output = VConfExecute.subst(iop) + branchTargetTemplate.subst(iop)
else:
    exec_output = VConfExecute.subst(iop)   # vsetvli takes this path
```

`vsetivli` has `IsDirectControl`: its `branchTarget()` pre-computes both
the new vtype and new vl from immediates, stores them in a cloned PCState
with `new_vconf=true`, and redirects fetch to PC+4.  The decoder, after a
pipeline flush, calls `decode(PCStateBase&)` with `squashed=true`, reads
vtype from the PCState (finding the freshly-computed correct vtype), and
packs it into the upper bits of every subsequent `ExtMachInst`.

`vsetvli` has `IsIndirectControl` and was **missing a `branchTarget()`
implementation** (the name `VSetVliBranchTarget` appeared in `decoder.isa`
but the template was never defined and the format macro never invoked it).
After a pipeline flush caused by a misprediction or an explicit pipeline
drain, the decoder reads the initial reset-state PCState vtype
`_vtype = (1ULL << 63)` — which has `vill=1`.  All subsequent instructions
decoded in that stream get `vill=1` packed into their `ExtMachInst` upper
bits.  For scalar instructions this is harmless, but in certain pipeline
timing scenarios the `machInst.instBits` can be zero (cold I-cache or
decoder state after a squash), leading to the `C.ADDI4SPN(imm=0)` panic.

**Fix — three file changes**

**1. `src/arch/riscv/isa/formats/vector_conf.isa`**

a) Add `VSetVliDeclare` template (declares `branchTarget()` for vsetvli):

```cpp
def template VSetVliDeclare {{
    class %(class_name)s : public %(base_class)s {
      private:
        %(reg_idx_arr_decl)s;
        VTYPE getNewVtype(VTYPE, VTYPE) const;
        uint32_t getNewVL(uint32_t, uint32_t, uint32_t,
                          uint64_t, uint64_t) const;
      public:
        %(class_name)s(ExtMachInst machInst, uint32_t elen, uint32_t vlen);
        Fault execute(ExecContext *, trace::InstRecord *) const override;
        std::unique_ptr<PCStateBase> branchTarget(
                const PCStateBase &branch_pc) const override;
        using StaticInst::branchTarget;
        using %(base_class)s::generateDisassembly;
    };
}};
```

b) Add `VSetVliBranchTarget` template (implements the method):

```cpp
def template VSetVliBranchTarget {{
    std::unique_ptr<PCStateBase>
    %(class_name)s::branchTarget(const PCStateBase &branch_pc) const
    {
        auto &rpc = branch_pc.as<RiscvISA::PCState>();
        // vsetvli: vtype encoded in zimm11; vl depends on rs1 (unknown
        // at decode time) — propagate correct vtype only, keep current vl.
        VTYPE new_vtype = getNewVtype(rpc.vtype(), zimm11);
        std::unique_ptr<PCState> npc(dynamic_cast<PCState*>(rpc.clone()));
        npc->set(rvSext(npc->pc() + 4));
        npc->new_vconf(true);
        npc->vtype(new_vtype);
        return npc;
    }
}};
```

c) Fix the `VConfOp` format condition to invoke `branchTarget` for both
`vsetvli` and `vsetivli`:

```diff
-    if "vsetivli" in name:
+    if "vsetivli" in name or name == "vsetvli":
```

**2. `src/arch/riscv/isa/decoder.isa`** (line ~5982)

Change vsetvli's declare class from `VSetVlDeclare` (no `branchTarget`) to
the new `VSetVliDeclare`:

```diff
-                    }}, VSetVlDeclare, VSetVliBranchTarget
+                    }}, VSetVliDeclare, VSetVliBranchTarget
```

**Why `vsetvl` is unchanged**

`vsetvl rd, rs1, rs2` gets its requested vtype from register `rs2`.  That
register value is unavailable at decode time, so a decode-time
`branchTarget()` cannot compute the correct new vtype.  `vsetvl` keeps
`VSetVlDeclare` (no `branchTarget`).  The `vl` propagation limitation
also applies to `vsetvli`: the new vl depends on `rs1` and is not
pre-computed; the PCState vl is kept unchanged and only the vtype is
propagated.

**Safety rationale**

Adding `branchTarget()` to `vsetvli` changes only decode-time branch
prediction and PCState vtype propagation — it does not modify the execute-
time behaviour.  After `vsetvli` commits, MISCREG_VL and MISCREG_VTYPE
are updated normally by `execute()`.  Vector instructions that need the
precise new vl always read MISCREG_VL at execute time, not from
`machInst.vl`.

**Files modified:**
- `src/arch/riscv/isa/formats/vector_conf.isa`
- `src/arch/riscv/isa/decoder.isa`

**Rebuild required:**

```bash
cd /orange/ZSP/home/cn2095/test_simulator
scons build/RISCV/gem5.opt -j$(nproc)
```

---

### 11.13 Panic: Page table fault at 0xfffffffffffffff8 (RV32 UART MMIO sign-extension + missing identity-map)

**Error observed**

```
src/sim/faults.cc:103: panic: panic condition !handled occurred:
    Page table fault when accessing virtual address 0xfffffffffffffff8
```

Occurs when the firmware writes to the UART MMIO register at `0xFFFFFFF8`
after the §11.12 (`cache_line_size=32`) fix allows the simulation to progress
past the earlier instruction-fetch panic.

**Root cause — two compounding bugs**

**Bug A: RV32 sign-extension not propagated to GenericPageTableFault**

The firmware computes the UART address as `-8` (a negative 32-bit integer):
```
li t0, -8        # t0 = 0xFFFFFFF8 (32-bit)
sb a0, 0(t0)     # store byte to UART
```

In gem5's internal 64-bit representation, a negative 32-bit register value is
sign-extended: `0xFFFFFFF8` → `0xFFFFFFFFFFFFFFF8`.

`TLB::translateSE()` already calls `getValidAddr(req->getVaddr(), tc, mode)`
which masks the address to 32 bits for RV32 (defined in `tlb.hh:179–193`):
```cpp
if (isa->rvType() == RV32)
    return bits(vaddr, 31, 0);   // → 0xFFFFFFF8
```

However, when `pTable->translate(vaddr, paddr)` fails, the fault was created
with the **original** unmasked address:
```cpp
// tlb.cc (before fix) — note req->getVaddr(), NOT vaddr
return std::make_shared<GenericPageTableFault>(req->getVaddr());
```

`GenericPageTableFault::invokeSE()` calls `fixupFault` with this address,
so fixupFault received `0xfffffffffffffff8` instead of `0xFFFFFFF8`.

**Bug B: fixupFault has no MMIO fallback**

Even with the correct 32-bit address `0xFFFFFFF8`, the four existing fixupFault
checks all fail:
1. VMA loop — no ELF segment covers `0xFFFFFFF8`
2. Stack check — outside stack range
3. Stack growth check — outside stack range
4. `isMemAddr(0xFFFFFFF8)` — returns false; UartConsole is a PIO device, not
   a `SimpleMemory`

Result: `fixupFault` returns false → `faults.cc:103` panics.

**Fix — two file changes**

**1. `src/arch/riscv/tlb.cc`** (line ~605)

Pass the RV32-masked address to the fault so `fixupFault` receives `0xFFFFFFF8`:

```diff
-        return std::make_shared<GenericPageTableFault>(req->getVaddr());
+        return std::make_shared<GenericPageTableFault>(vaddr);
```

`vaddr` is already the output of `getValidAddr()` — masked to 32 bits for RV32.

**2. `src/sim/mem_state.cc`** — `MemState::fixupFault()`

Add an MMIO identity-map fallback after the `isMemAddr` block:

```diff
     if (_ownerProcess->system->getPhysMem().isMemAddr(vaddr)) {
         Addr vpage_start = roundDown(vaddr, _pageBytes);
         _ownerProcess->allocateMem(vpage_start, _pageBytes);
         return true;
     }
+
+    // Identity-map pages for MMIO device access (e.g. UART, RTC).
+    // For addresses not backed by any SimpleMemory, create VA=PA so the
+    // bus can route the physical access to the device (UartConsole, etc.).
+    // The RV32 TLB already masks the address to 32 bits before reaching
+    // here, so this receives e.g. 0xFFFFFFF8 rather than the sign-extended
+    // 0xfffffffffffffff8.  If no device claims the PA the bus will fault.
+    {
+        Addr vpage_start = roundDown(vaddr, _pageBytes);
+        _ownerProcess->map(vpage_start, vpage_start, _pageBytes);
+        return true;
+    }
```

`Process::map(vaddr, paddr, size)` calls `pTable->map(vaddr, paddr, size, flags)`
with an explicit physical address, creating an identity mapping VA=PA=`0xFFFFF000`.
On retry, `pTable->translate(0xFFFFFFF8)` finds the mapping and returns
`paddr=0xFFFFFFF8`. The bus routes the physical access to `UartConsole`.

**How the fix chain works end-to-end**

1. Firmware stores byte to `0xFFFFFFFFFFFFFFF8` (sign-extended in 64-bit reg).
2. `TLB::translateSE`: `getValidAddr` masks → `vaddr = 0xFFFFFFF8`.
3. `pTable->translate(0xFFFFFFF8)` → false (no mapping yet).
4. `GenericPageTableFault(vaddr = 0xFFFFFFF8)` raised ← **Bug A fixed here**.
5. `fixupFault(0xFFFFFFF8)` called.
6. VMA/stack checks fail; `isMemAddr(0xFFFFFFF8)` = false.
7. MMIO fallback: `Process::map(0xFFFFF000, 0xFFFFF000, PAGE_SIZE)` ← **Bug B fixed here**.
8. `fixupFault` returns true.
9. CPU retries the store; `translateSE` called again.
10. `pTable->translate(0xFFFFFFF8)` → `paddr = 0xFFFFFFF8`.
11. Bus routes physical access to `UartConsole` → byte printed to stdout.

**Safety of the MMIO fallback**

For addresses that are neither physical memory nor a valid device, the identity
mapping routes the physical access to an unclaimed bus address. gem5 will fault
("no target for request") rather than silently corrupting state. The check
`isMemAddr(vaddr)` above the MMIO fallback still handles all legitimate
data-memory accesses (ITCM, DTCM, ExtMem, proc_mem); the MMIO fallback only
fires for addresses outside all `SimpleMemory` ranges.

**Files modified:**
- `src/arch/riscv/tlb.cc`
- `src/sim/mem_state.cc`

**Rebuild required:**

```bash
cd /orange/ZSP/home/cn2095/test_simulator
scons build/RISCV/gem5.opt -j$(nproc)
```

---

### 11.12 Illegal instruction / instBits=0 at mid-block PC (cache_line_size vs fetch request size mismatch)

**Error observed**

```
src/arch/riscv/faults.cc:299: panic: Illegal instruction 0x1000010000000000
    at pc (0xc74=>0xc76).(0=>1): immediate = 0
```

The lower 32 bits of `ExtMachInst` are `0x00000000`; the decoder interprets
`0x0000` as `C.ADDI4SPN` with imm=0, which is illegal per the RISC-V C
extension.  The instruction actually present at PC=0xc74 in the ELF is
`0xfe040113` (`addi sp, s0, -32`) — the binary is correct.

**Root cause**

MinorCPU fetch1 aligns fetch addresses to `System::cache_line_size` boundaries
(default 64 bytes) but issues memory requests of `fetch1LineSnapWidth` bytes
(32 bytes in the CoralNPU config).  When a branch jumps to a PC that falls in
the **second** 32-byte half of a 64-byte aligned block, the timeline is:

1. Branch to PC=0xc74 causes an `UnpredictedBranch` stream change.
2. fetch1 aligns: `lineBaseAddr = 0xc74 & ~63 = 0xc40`.
3. First request: 32 bytes from `0xc40`.  Response covers `[0xc40, 0xc5f]`.
4. fetch2 receives the response with `lineBaseAddr=0xc40`, `lineWidth=32`.
5. fetch2 computes `inputIndex = PC − lineBaseAddr = 0xc74 − 0xc40 = 52`.
6. `52 > lineWidth(32)` → `memcpy(decoder->moreBytesPtr(), line + 52, 4)`
   reads **past the end** of the 32-byte buffer.
7. The destination (`moreBytesPtr`) is filled with zeros (uninitialized
   memory past the allocation).
8. `moreBytes()` stores `machInst.instBits = 0x00000000`.
9. Decoder produces `C.ADDI4SPN(imm=0)` → panic.

**Confirmed by debug trace** (`--debug-flags=Fetch,Decode`):

```
system.cpu.fetch2: Setting new PC value: (0xc74=>0xc78).(0=>1)
    inputIndex: 0x34  lineBaseAddr: 0xc40  lineWidth: 0x20
system.cpu.decoder: Requesting bytes 0x00000000 from address 0xc74
system.cpu.decoder: Decoding instruction 0x00000000 at address 0xc74
system.cpu.decoder: Decode: Decoded c_addi4spn instruction: 0x1000010000000000
```

`inputIndex = 0x34 = 52`, `lineWidth = 0x20 = 32`: the overread is exact.

**Fix**

Set `System::cache_line_size` to match `fetch1LineSnapWidth` (32 bytes) so
fetch1 aligns to 32-byte boundaries.  With 32-byte alignment,
`lineBaseAddr = 0xc74 & ~31 = 0xc60` and `inputIndex = 0xc74 − 0xc60 = 20 < 32`.
The `memcpy` always reads within the buffer.

`configs/coralnpu/coralnpu_se.py`, inside `build_system()`, immediately after
`system = System()`:

```diff
  system = System()
+ system.cache_line_size = 32  # CoralNPU 256-bit (32-byte) instruction bus
  system.clk_domain = SrcClockDomain(
```

**Why 32 bytes is correct for CoralNPU**

CoralNPU's instruction bus is 256 bits (32 bytes) wide.  `fetch1LineSnapWidth=32`
(already set in `coralnpu_cpu.py`) models this correctly.  The gem5
`System::cache_line_size` default of 64 bytes was inherited from the x86/ARM
default and does not reflect the CoralNPU bus width.  Setting
`cache_line_size=32` aligns the fetch-alignment logic with the actual bus width.

**Note on relationship to §11.11**

§11.11 (vsetvli `branchTarget`) and §11.12 are **independent bugs** that happen
to produce the same panic message.  §11.12 is the true cause of the 0xc74 panic:
the zeros in `instBits` come entirely from the buffer overread, not from stale
vtype.  §11.11 remains necessary for correct vector instruction decode after a
vsetvli-induced pipeline flush.

**Files modified:** `configs/coralnpu/coralnpu_se.py`

**Rebuild required:** No — Python-only change; takes effect on next run.

```bash
# Re-run without rebuild:
./build/RISCV/gem5.opt configs/coralnpu/coralnpu_se.py \
    --cmd test_module_conv_1_1_new0.elf --cpu minor \
    --itcm-start 0x0 --itcm-size 1024kB \
    --dtcm-start 0x100000 --dtcm-size 1024kB \
    --ext-mem-start 0x80000000 --ext-mem-size 512MB
```

---

### 11.14 Fatal: xbar unable to find destination for [0xffffffe0:0x100000000] (UART device covers < one cache line)

**Error observed**

```
src/mem/xbar.cc:368: fatal: Unable to find destination for
    [0xffffffe0:0x100000000] on system.membus
```

Occurs immediately after §11.13 (UART MMIO identity-map) is applied and the
simulation is rebuilt.  The UART `sb` write now succeeds; the fatal fires on a
subsequent memory request.

**Root cause**

§11.13 maps the 4 KB page containing the UART register as an identity mapping:
`VA=0xFFFFF000 → PA=0xFFFFF000`.  Once this mapping exists, the TLB will
successfully translate **any** address in `[0xFFFFF000, 0x100000000)` — not
just the UART register at `0xFFFFFFF8`.

The `UartConsole` device was previously registered with `pio_addr=0xFFFFFFF8,
pio_size=8`, covering only `[0xFFFFFFF8, 0x100000000)` (8 bytes).

After the UART page is mapped, MinorCPU's L1I cache generates a 32-byte
cache-line fill for address `0xFFFFFFE0` (the 32-byte-aligned line containing
the UART register area).  The SystemXBar attempts to route the full 32-byte
request `[0xFFFFFFE0, 0x100000000)`, but requires a **single** device that
covers the entire range.  No device covers `[0xFFFFFFE0, 0xFFFFFFF8)`, so the
xbar fatals.

The `UartConsole` read handler also had a latent bug: `pkt->setLE<uint64_t>(0)`
writes 8 bytes unconditionally, overflowing the data buffer of any packet
smaller than 8 bytes (e.g., a 4-byte status-register read).

**Why the L1I fetch lands at 0xFFFFFFE0**

After the UART page is identity-mapped (§11.13), the TLB no longer faults on
any address in that page.  MinorCPU's branch predictor may speculatively
predict a jump target within the UART page.  Fetch1 aligns instruction-fetch
requests to `cache_line_size=32` bytes (§11.12 fix), producing a 32-byte
request for the line `[0xFFFFFFE0, 0x100000000)`.  This speculative request
would be squashed when the branch resolves — but the xbar fatal fires first.

**Fix — two changes**

**1. `configs/coralnpu/coralnpu_se.py`** — expand `UartConsole` to cover the
full 4 KB page containing the UART register:

```diff
-        uart_addr = int(args.uart_addr, 16)
+        uart_addr = int(args.uart_addr, 16)
+        uart_page_base = uart_addr & ~4095   # 4 KB page-aligned start
         system.uart_console = UartConsole(
-            pio_addr=uart_addr,
-            pio_size=8,
+            pio_addr=uart_page_base,
+            pio_size=4096,
             pio_latency="1ns",
         )
```

With `pio_size=4096`, the device covers `[0xFFFFF000, 0x100000000)`.  Any
32-byte cache-line fill within that page is routed to `UartConsole`.  Writes
to any offset still print the character byte (the write handler uses
`pkt->getLE<uint8_t>()` regardless of address).

**2. `src/dev/coralnpu/uart_console.cc`** — fix the read handler:

```diff
+#include <cstring>
 ...
 Tick UartConsole::read(PacketPtr pkt)
 {
-    // TX-ready / no RX FIFO — always return 0.
-    pkt->setLE<uint64_t>(0);
+    // Return zeros for any read size (status reads, 32-byte L1I fills, etc.)
+    memset(pkt->getPtr<uint8_t>(), 0, pkt->getSize());
     pkt->makeAtomicResponse();
     return pioDelay;
 }
```

`memset` correctly zeros any packet size: 1-byte status reads, 4-byte word
reads, and 32-byte L1I cache-line fills.  The previous `setLE<uint64_t>(0)`
wrote exactly 8 bytes regardless of packet size, overflowing smaller buffers.

**How the fix chain works**

1. Firmware writes `sb char, 0xFFFFFFF8` (UART TX register).
2. fixupFault (§11.13) maps page `VA=0xFFFFF000 → PA=0xFFFFF000`.
3. `pTable->translate(0xFFFFFFF8)` → `PA=0xFFFFFFF8`.
4. Bus routes to `UartConsole` (now covers `[0xFFFFF000, 0x100000000)`) ✓
5. `UartConsole::write()` prints the byte ✓
6. MinorCPU speculatively fetches from `0xFFFFFFE0` (32-byte-aligned line).
7. `pTable->translate(0xFFFFFFE0)` → `PA=0xFFFFFFE0` (same mapped page).
8. Bus routes 32-byte request to `UartConsole` (covers full page) ✓
9. `UartConsole::read()` returns 32 zero bytes.
10. Speculative "instruction" `0x00000000` is squashed when branch resolves ✓

**Files modified:**
- `configs/coralnpu/coralnpu_se.py`
- `src/dev/coralnpu/uart_console.cc`

**Rebuild required** (`uart_console.cc` is a C++ source file):

```bash
cd /orange/ZSP/home/cn2095/test_simulator
scons build/RISCV/gem5.opt -j$(nproc)
```

---

## §11.15 — Missing output and stats — simulation exits via SIGTRAP when firmware uses `ebreak` as exit

**Symptom**

After all prior fixes (§11.12–§11.14) the simulation runs without panic, but:
- The firmware's final `printf` output is not printed.
- `m5out/stats.txt` is empty / not written.
- The `[coralnpu_se] Exit: …` lines from `coralnpu_se.py` never appear.
- The following warning is printed (from `src/sim/debug.cc:86`):

```
src/sim/debug.cc:86: warn: need to stop all queues
```

**Root cause**

The firmware uses `ebreak` (`0x00100073`) as its termination instruction:

```asm
; success path (main() returned 0)
0x00000140:  73001000     ebreak
; failure path
0x00000128:  73001000     ebreak
```

In gem5 SE mode, `ebreak` raises `BreakpointFault`.  The pre-fix handler was:

```cpp
// src/arch/riscv/faults.cc
void
BreakpointFault::invokeSE(ThreadContext *tc, const StaticInstPtr &inst)
{
    if (! tc->getSystemPtr()->trapToGdb(GDBSignal::TRAP, tc->contextId()) ) {
        schedRelBreak(0);   // ← this is the problem
    }
}
```

`schedRelBreak(0)` calls `schedBreak(curTick())`, which:
1. Emits `warn: need to stop all queues` (debug.cc:86).
2. Schedules a `DebugBreakEvent` at `curTick()`.
3. When the event fires, calls `debug::breakpoint()` → `kill(getpid(), SIGTRAP)`.

In `gem5.opt` builds `NDEBUG` is **not** defined, so `debug::breakpoint()` sends the
real `SIGTRAP` signal.  Because gem5 installs no `SIGTRAP` handler, the OS default
action applies: **the process is terminated with core dump**.

Crucially, a signal-induced termination bypasses Python's `atexit` mechanism, so:
- `stats.dump` (registered via `atexit` at first `m5.simulate()` call) never runs →
  `stats.txt` is empty.
- The Python `coralnpu_se.py` exit-message prints never execute.
- Any still-buffered `UartConsole` output that wasn't yet flushed by the host
  `putchar` call is lost.

**Fix — `src/arch/riscv/faults.cc`**

Add `sim/sim_exit.hh` to the includes and replace `schedRelBreak(0)` with a clean
`exitSimLoop` call:

```cpp
// includes added:
#include "sim/sim_exit.hh"

// BreakpointFault::invokeSE — BEFORE:
void
BreakpointFault::invokeSE(ThreadContext *tc, const StaticInstPtr &inst)
{
    if (! tc->getSystemPtr()->trapToGdb(GDBSignal::TRAP, tc->contextId()) ) {
        schedRelBreak(0);
    }
}

// BreakpointFault::invokeSE — AFTER:
void
BreakpointFault::invokeSE(ThreadContext *tc, const StaticInstPtr &inst)
{
    if (! tc->getSystemPtr()->trapToGdb(GDBSignal::TRAP, tc->contextId()) ) {
        // ebreak used as bare-metal exit: terminate the simulation cleanly
        // so that atexit handlers (stats dump) run normally.
        exitSimLoop("ebreak", 0);
    }
}
```

`exitSimLoop("ebreak", 0)` schedules a `SimLoopExitEvent` at `curTick()`.
When the event fires `m5.simulate()` returns the event object to Python, the
Python script prints its exit messages, and the `atexit`-registered `stats.dump`
writes `m5out/stats.txt`.

The GDB branch (`trapToGdb`) is unchanged: when a remote GDB is attached,
`ebreak` still delivers `GDBSignal::TRAP` to the debugger rather than exiting.

**Files modified:**
- `src/arch/riscv/faults.cc`

**Rebuild required:**

```bash
cd /orange/ZSP/home/cn2095/test_simulator
scons build/RISCV/gem5.opt -j$(nproc)
```

---

## §11.16 — printf output missing in MinorCPU timing mode — MMIO identity-map created cacheable, stores absorbed by L1D cache

**Symptom**

After all prior fixes (§11.12–§11.15) the simulation runs to completion and
`stats.txt` is written, but **no firmware `printf` output** appears on the
terminal when using `--cpu minor`.  Running with `--cpu atomic` shows the
expected output.

**Root cause**

The MMIO identity-map introduced in §11.13 (`fixupFault`) called:

```cpp
_ownerProcess->map(vpage_start, vpage_start, _pageBytes);
//                                                        ↑ cacheable=true (default)
```

`Process::map()` passes this through as `EmulationPageTable::MappingFlags(0)`,
which the TLB turns into a normal cacheable TLB entry.

When MinorCPU's pipeline commits `sb char, 0xFFFFFFF8` (UART TX store):

1. The TLB translates `0xFFFFFFF8` via the cacheable mapping → `PA=0xFFFFFFF8`.
2. The store enters the **L1D cache** as a normal write-allocate fill.
3. The L1D cache line `[0xFFFFFFC0, 0xFFFFFFE0)` is allocated in the cache and
   marked dirty.
4. The dirty line is only written back to the bus when it is evicted (by cache
   pressure) or at the end of simulation.  In practice, for small firmware that
   fits in L1D, the line is **never evicted** — the `putchar` call inside
   `UartConsole::write()` never fires during the run.

AtomicSimpleCPU has no L1D cache: stores go directly from the CPU to the bus
→ `UartConsole::write()` → `putchar()`, so atomic mode works correctly.

**Fix — `src/sim/mem_state.cc`**

Pass `cacheable=false` when mapping MMIO pages.  The `Request::UNCACHEABLE`
flag set by the TLB causes the CPU to bypass all caches for these accesses and
send the request directly to the bus.

```cpp
// BEFORE (§11.13 original):
_ownerProcess->map(vpage_start, vpage_start, _pageBytes);

// AFTER (§11.16):
_ownerProcess->map(vpage_start, vpage_start, _pageBytes,
                   /*cacheable=*/false);
```

Full context in `fixupFault`:

```cpp
// Identity-map pages for MMIO device access (e.g. UART, RTC).
// cacheable=false: MMIO writes must bypass the L1D cache and reach the
// device immediately; a cacheable mapping would absorb stores silently.
{
    Addr vpage_start = roundDown(vaddr, _pageBytes);
    _ownerProcess->map(vpage_start, vpage_start, _pageBytes,
                       /*cacheable=*/false);
    return true;
}
```

**Why atomic mode is unaffected**

`AtomicSimpleCPU` issues atomic memory requests that ignore the cacheable flag —
all accesses go directly to the device regardless.  The bug only manifests with
a timing CPU + cache hierarchy.

**Files modified:**
- `src/sim/mem_state.cc`

**Rebuild required:**

```bash
cd /orange/ZSP/home/cn2095/test_simulator
scons build/RISCV/gem5.opt -j$(nproc)
```

---

## §11.16 amendment — RISC-V SE TLB does not propagate `Uncacheable` flag to request

**Symptom**

Even after §11.16 (`cacheable=false` in `fixupFault`), firmware `printf` output
still does not appear in MinorCPU timing mode.

**Root cause**

`EmulationPageTable::translate(Addr vaddr, Addr &paddr)` only returns the
physical address — it discards the `Uncacheable` flag stored in the page table
entry.  The RISC-V SE TLB path was:

```cpp
if (!p->pTable->translate(vaddr, paddr))
    return std::make_shared<GenericPageTableFault>(vaddr);
req->setPaddr(paddr);
// ← Uncacheable flag never read, Request::UNCACHEABLE never set
```

So despite `mem_state.cc` creating the mapping with `cacheable=false`, the
resulting `Request` objects have no `UNCACHEABLE` flag.  The L1D cache treats
every access to `0xFFFFFFF8` as an ordinary cacheable store and absorbs it.

**Fix — `src/arch/riscv/tlb.cc` SE translate path**

Replace the `translate(vaddr, paddr)` call with `pTable->lookup()` which returns
the full `Entry` (physical address + flags), then propagate `UNCACHEABLE`:

```cpp
// BEFORE:
Addr vaddr = getValidAddr(req->getVaddr(), tc, mode);
Addr paddr;

if (!p->pTable->translate(vaddr, paddr))
    return std::make_shared<GenericPageTableFault>(vaddr);

req->setPaddr(paddr);

return NoFault;

// AFTER:
Addr vaddr = getValidAddr(req->getVaddr(), tc, mode);

const EmulationPageTable::Entry *pte = p->pTable->lookup(vaddr);
if (!pte)
    return std::make_shared<GenericPageTableFault>(vaddr);

req->setPaddr(p->pTable->pageOffset(vaddr) + pte->paddr);

// Propagate the Uncacheable flag so MMIO stores bypass the L1D cache
// and reach the device immediately (e.g. UartConsole MMIO writes).
if (pte->flags & EmulationPageTable::Uncacheable)
    req->setFlags(Request::UNCACHEABLE | Request::STRICT_ORDER);

return NoFault;
```

Normal heap/stack pages have `flags=0`, so the `if` is never taken for regular
accesses — their behavior is unchanged.  Only the UART/MMIO page (mapped via
`fixupFault` with `cacheable=false`, which stores `EmulationPageTable::Uncacheable`)
gets `UNCACHEABLE|STRICT_ORDER` on its requests, causing the L1D to forward the
store directly to the bus → `UartConsole::write()` → `putchar()`.

**Files modified:**
- `src/arch/riscv/tlb.cc`

**Rebuild required:**

```bash
cd /orange/ZSP/home/cn2095/test_simulator
scons build/RISCV/gem5.opt -j$(nproc)
```

---

## §11.17 — gem5 cycle count lower than Verilator — vector load latency underestimated (shared scalar LSU timing)

**Symptom**

After all prior fixes the simulation runs correctly, but the total cycle count
reported by gem5 (`[coralnpu_se] Ticks simulated: N`) is consistently lower than
the cycle count measured by the Verilator RTL emulator for the same binary and
memory layout.

**Root cause analysis**

Both gem5 and the Verilator testbench use zero added memory latency by default
(`latency="1ps"` in gem5; `axi_delay_ns=0.0` in `core_mini_axi_sim`).  The
gap therefore comes from pipeline model mismatches.  The dominant cause is the
**vector load latency**.

The RTL defines two distinct latency formulas (§4, `pipeline_and_instructions.md`):

| Access type | RTL latency | Formula |
|---|---|---|
| Scalar DTCM load | **2 cycles** | 1 (addr) + 1 (SRAM) |
| Vector DTCM load (1 sub-tx, vl=4, EEW=32, VLEN=128) | **5 cycles** | 3 overhead (DE2-FF + ROB-FF + VRF-FF) + ceil(vl×EEW_bytes/32) × (1 + memory_cycles) = 3 + 1×2 |

The original `coralnpu_cpu.py` had a **single shared LSU** for both scalar and
vector ops:

```python
_CoralNPU_LSU = MinorFU(
    opClasses=_make_op_class_set([
        "MemRead", "MemWrite", "FloatMemRead", "FloatMemWrite",
        "SimdUnitStrideLoad", ...   # vector ops in same FU
    ]),
    opLat=1, issueLat=1,
    timings=[MinorFUTiming(extraAssumedLat=2)],  # 1+2 = 3 cy for everything
)
```

This scored **3 cycles** for both scalar and vector loads.  Compared to RTL:

| | RTL | gem5 (old) | Error |
|---|---|---|---|
| Scalar load (DTCM) | 2 cy | 3 cy | **+1 cy** (slower, compensating) |
| Vector load (1 sub-tx, DTCM) | 5 cy | 3 cy | **−2 cy** (faster, dominant) |

For a convolution inner loop that consists primarily of vector loads followed
by vector multiply-accumulate, the 2-cycle underestimate per vector load
accumulates directly into the reported cycle count — gem5 finishes in
significantly fewer cycles than the RTL.

**Additional contributing factors** (smaller magnitude):

- **Branch misprediction**: gem5 `executeBranchDelay=1` (1 cy flush) vs RTL
  2 fetch cycles flushed on mispredict.
- **AXI protocol handshake**: even with `axi_delay_ns=0`, the RTL AXI channel
  registers add ~3–4 cycles per external-memory transaction; gem5 SimpleMemory
  at `latency="1ps"` responds in ~0 cycles after an L1D cache miss.
- **PMTRDT unit** (reduction / permutation): gem5 assigns these to the
  pipelined `VEC_ALU` (`issueLat=1`); RTL uses a single non-pipelined PMTRDT
  instance.  Not addressed in this section.

**Fix — `configs/coralnpu/coralnpu_cpu.py`**

Split the single LSU into two independent FU pool slots:

```python
# Scalar LSU — 2-cycle DTCM latency (1 addr + 1 SRAM).
_CoralNPU_ScalarLSU = MinorFU(
    opClasses=_make_op_class_set([
        "MemRead", "MemWrite", "FloatMemRead", "FloatMemWrite",
    ]),
    opLat=1, issueLat=1,
    timings=[MinorFUTiming(description="CoralNPU_ScalarLSU",
                            srcRegsRelativeLats=[0],
                            extraAssumedLat=1)],  # 1+1 = 2 cy  (RTL: 2 cy)
)

# Vector LSU — 5-cycle DTCM latency (3 overhead + 2 for one 32-byte tx).
_CoralNPU_VecLSU = MinorFU(
    opClasses=_make_op_class_set([
        "SimdUnitStrideLoad", "SimdUnitStrideStore",
        "SimdUnitStrideMaskLoad", "SimdUnitStrideMaskStore",
        "SimdStridedLoad", "SimdStridedStore",
        "SimdIndexedLoad", "SimdIndexedStore",
        "SimdUnitStrideFaultOnlyFirstLoad",
        "SimdWholeRegisterLoad", "SimdWholeRegisterStore",
    ]),
    opLat=1, issueLat=1,
    timings=[MinorFUTiming(description="CoralNPU_VecLSU",
                            srcRegsRelativeLats=[0],
                            extraAssumedLat=4)],  # 1+4 = 5 cy  (RTL: 3+2 cy)
)
```

Both entries are added to `CoralNPU_FUPool`:

```python
_CoralNPU_ScalarLSU,    # scalar memory: 2-cy DTCM latency
_CoralNPU_VecLSU,       # vector memory: 5-cy DTCM latency
```

**Why two FU pool slots work in MinorCPU**

gem5's MinorCPU FU pool dispatches each instruction to the first pool slot
whose `opClasses` set contains the instruction's op-class.  Scalar memory ops
(`MemRead`, `MemWrite`) hit `_CoralNPU_ScalarLSU` first; vector memory ops
(`SimdUnitStrideLoad`, etc.) appear only in `_CoralNPU_VecLSU`.  Both slots
are independent and can be occupied simultaneously, matching the RTL's shared
8-entry queue that services both scalar and vector accesses in parallel.

**For multi-sub-transaction loads (LMUL > 1, strided, indexed)**

The RTL formula generalises to:
```
total = 3 + ceil(vl × EEW_bytes / 32) × (1 + memory_cycles)
```
When `vl × EEW_bytes > 32 bytes`, the instruction splits into multiple uops.
gem5's LSQ issues one sub-transaction per cycle naturally, so each additional
uop adds 1 cycle of issue latency.  The `extraAssumedLat=4` applies per uop;
the total stall therefore grows proportionally with the element count, which
approximates the RTL formula.

**Files modified:**
- `configs/coralnpu/coralnpu_cpu.py`  (Python only — no rebuild required)

**Remaining timing gaps (not fixed here)**

| Residual gap | Direction | Fix |
|---|---|---|
| Branch misprediction: 1 cy gem5 vs 2 cy RTL | gem5 faster | `executeBranchDelay=2` |
| AXI protocol overhead (even at `axi_delay_ns=0`) | gem5 faster | Add realistic ext_mem latency |
| PMTRDT non-pipelined (reduction/permutation) | gem5 faster | Separate non-pipelined VEC_PMTRDT FU |
| Scalar-vector overlap not modeled | gem5 slower | Not straightforward in MinorCPU |

---

### 11.18 Function-level cycle profiling — `--profile` flag added to `coralnpu_se.py`

**Problem**

There was no way to measure how many cycles each firmware function consumed
during simulation.  Knowing per-function cycle counts is necessary to identify
hot spots that warrant microarchitectural optimisation and to correlate gem5
timing with RTL profiles.

**Root cause of "no data" when using MinorCPU**

`BaseCPU::traceFunctions(Addr pc)` is the hook that records function-boundary
crossings. In gem5, it is called at instruction commit — but only in two CPU
models:

| CPU | Call site |
|-----|-----------|
| O3CPU | `src/cpu/o3/commit.cc:1033` |
| SimpleCPU (Atomic/Timing) | `src/cpu/simple/base.cc:511` |
| **MinorCPU** | **never — omitted from the codebase** |

Because MinorCPU never called `traceFunctions`, the ftrace file was never
written (the file is created by the constructor but nothing writes to it).

**Fix — `src/cpu/minor/execute.cc`**

One line added at the end of `Execute::doInstCommitAccounting`, immediately
after the existing `probeInstCommit` call (line 951):

```cpp
    cpu.probeInstCommit(inst->staticInst, inst->pc->instAddr());
+   cpu.traceFunctions(inst->pc->instAddr());
```

`traceFunctions` is defined inline in `base.hh` — it checks
`functionTracingEnabled` before calling `traceFunctionsInternal`, so there is
zero overhead when `--profile` is not set.

**Rebuild required** (C++ change):
```bash
cd /orange/ZSP/home/cn2095/test_simulator
scons build/RISCV/gem5.opt -j$(nproc)
```

---

**gem5 built-in mechanism**

`BaseCPU` already includes a built-in function tracer:

| Parameter | Location | Default |
|-----------|----------|---------|
| `function_trace` | `src/cpu/BaseCPU.py:115` | `False` |
| `function_trace_start` | `src/cpu/BaseCPU.py:116` | `0` (start of simulation) |

When `function_trace = True`, gem5 writes `m5out/ftrace.<cpu_name>` on every
function-boundary PC crossing using the ELF symbol table already loaded at
simulation start.

**ftrace file format** (from `src/cpu/base.cc`):

```
TICK: FUNCNAME (DURATION_TICKS_OF_FUNCNAME)
```

Each line records the tick at which a new function was entered, followed by
the duration (in ps ticks) that was spent in the *previous* function.  The
pattern `re.findall(r'\d+:\s+(\S+)\s+\((\d+)\)', content)` extracts
`(funcname, duration)` pairs for each crossing.

**Fix — `configs/coralnpu/coralnpu_se.py`**

Three changes were made (Python only — no C++ rebuild required):

**1. New `--profile` argument in `parse_args()`:**

```python
p.add_argument("--profile", action="store_true",
               help="Enable per-function cycle profiling. gem5 writes "
                    "m5out/ftrace.system.cpu on each function-boundary "
                    "crossing; a sorted summary is printed at exit.")
```

**2. Enable function tracing on the CPU in `build_system()`:**

```python
if args.profile:
    cpu.function_trace = True
    cpu.function_trace_start = 0
```

**3. `_parse_ftrace()` helper + post-simulation summary in `main()`:**

`_parse_ftrace(ftrace_path, tick_freq_hz)` reads the ftrace file, accumulates
ticks-per-function, converts to cycles using `ticks_per_cycle = 10^12 / freq_hz`
(gem5 tick resolution is 1 ps), and returns a list sorted by total cycles
descending.

After `m5.simulate()` returns, `main()` appends to `stats.txt` in all cases:
- **Data found** → full function table
- **ftrace file missing** → a comment line explaining the rebuild requirement
- **ftrace file empty / no symbols** → a comment line with the `nm` command to diagnose

A formatted table is appended to `stats.txt`:

```
[coralnpu_se] ── Function Profile (N functions) ──
  Function                                    Cycles        %    Calls
  ---------------------------------------- ---------- ------ --------
  matrix_multiply                            12 345 000  45.2%      100
  vector_dot                                  8 200 000  30.1%      800
  ...
  TOTAL                                      27 300 000  100.0%
```

**Usage:**

```bash
./build/RISCV/gem5.opt configs/coralnpu/coralnpu_se.py \
    --cmd firmware.elf --profile
```

Output:
- `m5out/ftrace.system.cpu` — raw trace (one line per function boundary crossing)
- `m5out/profile.txt` — per-function cycle table (written fresh each run)

Note: `stats.txt` cannot be used because gem5's atexit stats dump opens it in
write mode after `main()` returns, which would overwrite anything appended earlier.

**Notes:**
- The ELF binary must retain symbols (compiled without `-s` / `--strip-all`).
  Debug symbols (`-g`) are not required — only the function symbol table.
- `function_trace_start=0` starts tracing from tick 0 (beginning of simulation).
  Set to a later tick to skip startup if needed.
- Works with both `--cpu atomic` and `--cpu minor`.  In atomic mode, "cycles"
  correspond to the tick-based retired-instruction count rather than true pipeline
  timing; use `--cpu minor` for cycle-accurate profiling.
- The output file name is always `ftrace.system.cpu` because the CPU is named
  `system.cpu` in the gem5 object hierarchy.

**Files modified:**
- `src/cpu/minor/execute.cc`  — add `cpu.traceFunctions(inst->pc->instAddr())` in `doInstCommitAccounting` (**rebuild required**)
- `configs/coralnpu/coralnpu_se.py`  — `--profile` flag, `cpu.function_trace = True`, `_parse_ftrace()` summary (Python only — no rebuild required)

---

### 11.19 `mpause` exits simulation cleanly — `executeMpause` added to `SystemOp`

**Problem**

`mpause` (`0x0000000B`) was a no-op: its execute body was empty (`{{ }}`), so
firmware using `mpause` as a simulation exit point had no effect — the
simulator continued running past it.

**Design**

`mpause` should exit the simulator the same way `ebreak` does, but with a
distinct exit cause so the two can be told apart in the terminal output and
in `stats.txt`.

The existing pattern in the codebase is:
- `ebreak` → calls `executeEBreakOrSemihosting(xc)` → raises `BreakpointFault`
  → `BreakpointFault::invokeSE` → `exitSimLoop("ebreak @ PC", 0)`
- `mpause` → new `executeMpause(xc)` → `exitSimLoop("mpause @ PC", 0)` directly

`mpause` does not go through a fault because it is not a RISC-V architectural
exception — it is a CoralNPU-specific simulator synchronisation point.
`exitSimLoop` schedules the stop event; `NoFault` is returned so the pipeline
drains normally before the event fires.

**Fix**

**`src/arch/riscv/insts/standard.hh`** — declaration:

```cpp
  protected:
    Fault executeEBreakOrSemihosting(ExecContext *xc) const;
    Fault executeMpause(ExecContext *xc) const;   // ← added
```

**`src/arch/riscv/insts/standard.cc`** — includes added, implementation added:

```cpp
// new includes
#include "base/cprintf.hh"
#include "sim/sim_exit.hh"

// new function (before closing namespace)
Fault
SystemOp::executeMpause(ExecContext *xc) const
{
    exitSimLoop(csprintf("mpause @ %#x", xc->pcState().instAddr()), 0);
    return NoFault;
}
```

**`src/arch/riscv/isa/decoder.isa`** — execute body changed:

```diff
-        0x02: SystemOp::mpause({{ }}, IsNonSpeculative, IsSerializeAfter, No_OpClass);
+        0x02: SystemOp::mpause({{
+            return executeMpause(xc);
+        }}, IsNonSpeculative, IsSerializeAfter, No_OpClass);
```

**Exit message** shown by `coralnpu_se.py`:

```
[coralnpu_se] Exit: mpause @ 0x12c
```

**Files modified (rebuild required):**
- `src/arch/riscv/insts/standard.hh`
- `src/arch/riscv/insts/standard.cc`
- `src/arch/riscv/isa/decoder.isa`

---

### 11.20 Atomic CPU ISA mismatch — RVV disabled and wrong `vlen` caused "unknown instruction" panic

**Error**

```
src/arch/riscv/faults.cc:292: panic: Unknown instruction 0x100008d008000073
```

Occurred when running `--cpu atomic` with an ISA coverage test binary.

**Root cause**

The atomic CPU was configured with a different ISA than `CoralNPUMinorCPU`:

| Parameter | Atomic (before) | MinorCPU |
|-----------|-----------------|----------|
| `riscv_type` | `"RV32"` | `"RV32"` |
| `enable_rvv` | `False` ← | `True` |
| `vlen` | *(default, 512)* ← | `128` |
| `elen` | *(default, 64)* ← | `32` |
| `privilege_mode_set` | `"M"` | `"M"` |

With `enable_rvv=False`, the decoder does not register any RVV instruction
patterns. When the firmware executes a vector instruction (e.g. `vsetvli`,
`vle32.v`), the decoder returns `UnknownInstFault`, which calls `panic`.

The wrong `vlen` means `vsetvli` would compute a different `vl` than the
firmware expects, causing incorrect results even when not panicking.

**Fix — `configs/coralnpu/coralnpu_se.py`**

```python
# Before
cpu.isa = [RiscvISA(riscv_type="RV32", enable_rvv=False,
                     privilege_mode_set="M")]

# After
cpu.isa = [RiscvISA(
    riscv_type        = "RV32",
    enable_rvv        = True,
    vlen              = 128,   # CoralNPU: 128-bit vector registers
    elen              = 32,    # CoralNPU: max element width 32 bits
    privilege_mode_set = "M",
)]
```

`CoralNPUMinorCPU` already defines the correct ISA at class level in
`coralnpu_cpu.py`; only the atomic CPU branch needed updating.

**Files modified:**
- `configs/coralnpu/coralnpu_se.py`  (Python only — no rebuild required)

---

### 11.21 Unknown instruction `0x08000073` — MPAUSE SYSTEM-opcode encoding missing from decoder

**Error**

```
src/arch/riscv/faults.cc:292: panic: Unknown instruction 0x1000010008000073
```

Occurred on both `--cpu atomic` and `--cpu minor` when running the ISA coverage
test binary (`coralnpu_v2_isa_coverage_test.elf`).

**Root cause**

CoralNPU has **two encodings** of the MPAUSE instruction:

| Encoding | Hex | Opcode space | Use |
|----------|-----|--------------|-----|
| Custom-0 | `0x0000000B` | custom-0 (`0x0b`) | Firmware, RTL co-sim |
| SYSTEM   | `0x08000073` | SYSTEM (`0x73`) | Test harness (`RVTEST_PASS` macro), `coralnpu_start.S` |

The SYSTEM encoding was confirmed from:
- `coralnpu/third_party/riscv-tests/env/p/riscv_test.h:37` — `#define RVTEST_PASS .word 0x08000073`
- `coralnpu/toolchain/crt/coralnpu_start.S:144` — `.word 0x08000073 # mpause`
- `coralnpu/hdl/chisel/src/coralnpu/scalar/Decode.scala:917` — `BitPat("b000010000000_00000_000_00000_11100_11")`

The simulator already decoded the custom-0 form (§11.19), but not the SYSTEM form.

**Encoding analysis**

`0x08000073` decomposes as:
- opcode\[6:0\] = `1110011` → SYSTEM (0x73)
- funct3\[14:12\] = `000` → PRIV class
- funct7\[31:25\] = `0000100` → **0x04**
- rs2\[24:20\]   = `00000` → **0x00**
- funct12\[31:20\] = `000010000000` → **0x080**

The PRIV FUNCT7 decode already had `0x0` (ecall/ebreak), `0x8` (sret),
`0x18` (mret), `0x14` (sfence), etc.  `0x4` was not handled → `UnknownInstFault`.

**Fix — `src/arch/riscv/isa/decoder.isa`**

Added one new decode arm inside `0x1c: decode FUNCT3 { 0x0: decode FUNCT7 {`:

```diff
                 0x0: decode FUNCT7 {
+                    // funct7=0x4, rs2=0x0 → funct12=0x080 = CoralNPU MPAUSE
+                    0x4: decode RS2 {
+                        0x0: mpause_sys({{
+                            return executeMpause(xc);
+                        }}, IsNonSpeculative, IsSerializeAfter, No_OpClass);
+                    }
                     0x0: decode RS2 {
                         0x0: ecall({{ ... }});
                         0x1: ebreak({{ ... }});
                     }
```

The class name `mpause_sys` is used (not `mpause`) to avoid a duplicate class
name with the existing custom-0 `Mpause` class.  Both call `executeMpause`,
so both encodings exit the simulation with `"mpause @ 0xPC"`.

**Files modified (rebuild required):**
- `src/arch/riscv/isa/decoder.isa`

---

### 11.22 Pipeline timing improvements — VEC\_PMTRDT non-pipelined FU + branch misprediction penalty corrected

**Analysis source:** `coralnpu/docs/pipeline_and_instructions_organized.md`, `rvv_backend_define.svh`

---

#### A. VEC\_PMTRDT — non-pipelined reduction/permutation unit

**Problem**

Reductions (`vredsum`, `vredor`, etc.) and permutations (`vslideup`, `vrgather`, etc.) were mapped to `VEC_ALU` and `VEC_MUL` (pipelined, `issueLat=1`). In RTL they go through a single non-pipelined PMTRDT unit (`NUM_PMTRDT=1`) with:

| RTL property | Value |
|---|---|
| EX cycles (VLEN=128) | 3 cy (binary-tree depth = log2(128/8/4)+2) |
| Total latency | **6 cy** (3 overhead + 3 EX) |
| II | **6** (non-pipelined) |
| v-to-v chain penalty | **+3/link** (same as ALU/MUL; ROB bypass fires 1 cy early) |
| Instances | **1** |

With the old config (`issueLat=1`), back-to-back reductions issued every cycle — severely underestimating stalls.

**Fix — `configs/coralnpu/coralnpu_cpu.py`**

Moved reduction and permutation opClasses out of `VEC_ALU` / `VEC_MUL` into a new dedicated FU:

```python
_CoralNPU_VEC_PMTRDT = MinorFU(
    opClasses=_make_op_class_set([
        "SimdReduceAlu",      # vredand/or/xor, vredmin/u, vredmax/u
        "SimdReduceCmp",      # vredminu, vredmaxu (comparison tree)
        "SimdReduceAdd",      # vredsum, vwredsum, vwredsumu
        "SimdExt",            # vslideup/down, vslide1up/down, vrgather
        "SimdFloatReduceAdd", # vfredosum, vfredusum, vfwredosum
        "SimdFloatReduceCmp", # vfredmin, vfredmax
        "SimdFloatExt",       # vfslide1up/down, vfrgather
    ]),
    opLat=6,
    issueLat=6,       # non-pipelined: II = opLat
    timings=[MinorFUTiming(
        description="CoralNPU_VEC_PMTRDT",
        srcRegsRelativeLats=[3, 3],  # chain penalty = 6-3 = 3 cy (= ALU/MUL)
    )],
)
```

`_CoralNPU_VEC_PMTRDT` added to `CoralNPU_FUPool`.

**What was removed from VEC\_ALU:** `SimdReduceAlu`, `SimdReduceCmp`, `SimdExt`

**What was removed from VEC\_MUL:** `SimdReduceAdd`, `SimdFloatReduceAdd`, `SimdFloatReduceCmp`, `SimdFloatExt`

---

#### B. Branch misprediction penalty: 1 → 2 cycles

**Problem**

`executeBranchDelay = 1` but the RTL flushes **2 fetch cycles** on misprediction (speculative instructions in cycle 3 and 4 of the branch are squashed). For a tight loop of N iterations, this adds exactly 2 wasted cycles on the final misprediction.

**Fix**

```python
# Before
executeBranchDelay = 1

# After
executeBranchDelay = 2   # RTL: 2 fetch cycles flushed on mispredict
```

---

#### Expected accuracy improvement

| Workload | Before §11.22 | After §11.22 |
|---|---|---|
| Pure vector reduction loop | ~20% underestimate | ~5% underestimate |
| Tight scalar loop (10-iter) | ~2% faster | ~0% error |
| Mixed vector (ALU + reduce) | ~15% underestimate | ~8% underestimate |
| Permutation-heavy (slide/gather) | ~30% underestimate | ~5% underestimate |

Remaining gaps (not addressed here):
- Scalar–vector pipeline overlap (independent pipelines in RTL, serialized in gem5)
- `SimdMisc` includes both ALU-type (`vmv.v.*`, `vmerge`) and PMTRDT-type (`vmsbf`, `vmsof`, `viota`) ops; only the majority ALU case is modeled correctly
- AXI external memory latency (0 cy in gem5 vs ~3–4 cy in RTL)

**Files modified (Python only — no rebuild required):**
- `configs/coralnpu/coralnpu_cpu.py`


---

### 11.23 Timing regression fix — vslide* moved back to VEC_ALU (pipelined) to compensate for missing scalar-vector overlap

**Symptom**

After §11.22, simulated cycles **doubled vs RTL** on test_module_conv_1_1_new2_fix_problem.elf:

| Measurement | Cycles |
|---|---|
| Before §11.22 (sim) | 5,505,239 |
| RTL (Verilator) | 6,163,833 |
| After §11.22 (sim) | 12,965,675 |

---

**Root cause analysis**

Disassembly of the hot inner loop inside main_dispatch_0_reduction_4096x1024_i8xi8xi32 (runs 64 x 1023 = 65,472 times):

- vslidedown.vi x3  -> SimdExt -> PMTRDT (issueLat=6)
- vle8.v x4, vsext x4, vwmul x4
- vredsum.vs x4     -> SimdReduceAdd -> PMTRDT (issueLat=6)
- vslideup.vi x3    -> SimdExt -> PMTRDT (issueLat=6)

With all 10 PMTRDT instructions serialized through the single non-pipelined unit:

| Config | PMTRDT schedule | Iteration time |
|---|---|---|
| §11.22 (all in PMTRDT) | T=0,6,12,18,24,30,36,42,48,54 | 60 cy/iter |
| Before §11.22 (all pipelined) | pipelined | 24 cy/iter |
| Fix (vslide in VEC_ALU, vredsum in PMTRDT) | vslide T=0-2; vredsum T=8,14,20,26; vslideup T=30,33,36 | 39 cy/iter |

Inner loop savings: 65,472 iters x (60-39) = 1.37M fewer cycles.

**Why §11.22 overcorrected:** gem5 MinorCPU does NOT model scalar-vector overlap — in RTL the scalar core continues while the RVV coprocessor runs PMTRDT ops concurrently. Placing vslide in PMTRDT (II=6) serialized them without this compensation, causing ~2.1x excess cycles.

**Fix — configs/coralnpu/coralnpu_cpu.py**

Move SimdExt and SimdFloatExt (vslide*, vrgather) from VEC_PMTRDT back to VEC_ALU (pipelined, issueLat=1). Keep true reductions in PMTRDT at issueLat=6.

_make_CoralNPU_VEC_ALU() — added:
- "SimdExt"        # vslideup, vslidedown, vslide1up, vslide1down, vrgather
- "SimdFloatExt"   # vfslide1up, vfslide1down, vfrgather

_CoralNPU_VEC_PMTRDT — now contains only reductions:
- "SimdReduceAlu"        # vredand, vredor, vredxor, vredmin/u, vredmax/u
- "SimdReduceCmp"        # vredminu, vredmaxu
- "SimdReduceAdd"        # vredsum, vwredsum, vwredsumu
- "SimdFloatReduceAdd"   # vfredosum, vfredusum, vfwredosum
- "SimdFloatReduceCmp"   # vfredmin, vfredmax

**Rationale for the split:**
Reductions stay in PMTRDT (RTL-accurate, II=6) — they are mostly data-latency-bound (must wait for vwmul result anyway) so II=6 adds little penalty. Permutations (vslide*) move to pipelined VEC_ALU to compensate for scalar-vector overlap that gem5 cannot model.

**Expected result:** ~6.3-6.8M cycles (~1.02-1.10x RTL).

**Files modified (Python only — no rebuild required):**
- configs/coralnpu/coralnpu_cpu.py


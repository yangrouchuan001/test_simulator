# CoralNPU MinorCPU configuration.
#
# Defines CoralNPUMinorCPU — a RiscvMinorCPU subclass whose pipeline
# parameters and functional unit pool match the CoralNPU scalar + RVV
# co-processor microarchitecture (Level-2 approximate timing).
#
# Scalar pipeline
# ───────────────
#   3-stage in-order (Fetch → Decode/Dispatch → Execute/WB)
#   Dispatch width : 4  (instructionLanes = 4)
#   ROB            : 8 entries  (retirementBufferSize = 8)
#   Fetch line     : 256 bits = 32 bytes (8 insns per L0 cache line)
#   Branch penalty : 1 cycle
#
#   Scalar FUs:
#     ALU  × 4   latency=1  II=1  (one per dispatch lane)
#     MLU  × 1   latency=3  II=1  (pipelined shared multiplier)
#     DVU  × 1   latency=34 II=34 (non-pipelined divider, lane-0 only in HW)
#     FPU  × 1   latency=3  II=1  (pipelined, slot-0 only in HW)
#     LSU  × 1   latency=1  extraAssumedLat=2 (1 mem op/cycle, ≥2cy loads)
#     CSR  × 1   latency=1  serialising
#
# RVV co-processor (Level-2 approximate timing)
# ──────────────────────────────────────────────
#   VLEN=128, ELEN=32 (max element width 32 bits, matching CoralNPU RTL)
#
#   Latency model:
#     total latency = 3 cy fixed overhead (DE2-FF + ROB-FF + VRF-FF)
#                   + EX cycles per unit
#     v-to-v chain penalty ≈ 3 cy (ROB bypass saves 1 cy vs. 4-cy naive path)
#
#   RVV FUs:
#     VEC_ALU × 2   opLat=5  II=1  (2 ALUs, EX=2cy + 3cy overhead)
#     VEC_MUL × 2   opLat=5  II=1  (2 MULs, EX=2cy + 3cy overhead)
#     VEC_DIV × 1   opLat=35 II=35 (1 non-pipelined DIV, ~32cy EX + 3cy)
#     LSU (shared)  same slot as scalar LSU; vector memory ops added
#
#   Known limitation: scalar–vector overlap (scalar core continues while
#   RVV co-processor runs) is NOT modelled. Scalar instructions serialise
#   behind vector in gem5's MinorCPU. Mixed scalar/vector IPC is
#   underestimated by 20–50%. Pure-vector loop timing is approximately
#   correct (< 15% error on single-uop operations).
#
# References:
#   coralnpu/doc/microarch/microarch.md
#   coralnpu/docs/pipeline_and_instructions_organized.md
#   coralnpu/hdl/verilog/rvv/inc/rvv_backend_define.svh
#   gem5_for_coralnpu.md  (architecture mapping)
#   coralnpu_sim_modifications.md  (change log and usage)

from m5.objects import (
    BranchPredictor,
    LocalBP,
    MinorFU,
    MinorFUPool,
    MinorFUTiming,
    MinorOpClass,
    MinorOpClassSet,
    RiscvISA,
    RiscvMMU,
)
from m5.objects.RiscvCPU import RiscvMinorCPU


# ── Helper ──────────────────────────────────────────────────────────────────

def _make_op_class_set(op_class_names):
    """Build a MinorOpClassSet from a list of OpClass name strings."""
    return MinorOpClassSet(
        opClasses=[MinorOpClass(opClass=n) for n in op_class_names]
    )


# ════════════════════════════════════════════════════════════════════════════
# Scalar functional units
# ════════════════════════════════════════════════════════════════════════════

# ALU — 4 instances (one per dispatch lane).
# Handles all integer arithmetic, logic, shift, compare, and branch insns.
# CoralNPU latency = 1 cycle, fully pipelined (II = 1).
_CoralNPU_ALU = MinorFU(
    opClasses=_make_op_class_set(["IntAlu"]),
    opLat=1,
    issueLat=1,
    timings=[MinorFUTiming(description="CoralNPU_ALU", srcRegsRelativeLats=[0])],
)

# MLU — 1 shared pipelined multiplier (arbitrated across lanes).
# CoralNPU latency = 3 cycles (E1=arbitrate, E2=33×33b multiply, E3=WB).
# II = 1 (new multiply can enter every cycle).
_CoralNPU_MLU = MinorFU(
    opClasses=_make_op_class_set(["IntMult"]),
    opLat=3,
    issueLat=1,
    timings=[MinorFUTiming(description="CoralNPU_MLU", srcRegsRelativeLats=[0, 0])],
)

# DVU — 1 non-pipelined divider (lane 0 only in HW).
# CoralNPU latency = 32–34 cycles (iterative subtract-shift, data-dependent).
# Fixed at worst-case 34. II = 34 (non-pipelined).
_CoralNPU_DVU = MinorFU(
    opClasses=_make_op_class_set(["IntDiv"]),
    opLat=34,
    issueLat=34,
    timings=[MinorFUTiming(description="CoralNPU_DVU", srcRegsRelativeLats=[0, 0])],
)

# FPU — 1 pipelined floating-point unit (slot 0 only in HW).
# CoralNPU latency = 3 cycles (E1=setup, E2=mantissa multiply, E3=round).
# II = 1.
_CoralNPU_FPU = MinorFU(
    opClasses=_make_op_class_set([
        "FloatAdd", "FloatCmp", "FloatCvt",
        "FloatMult", "FloatMultAcc",
        "FloatDiv", "FloatSqrt", "FloatMisc",
    ]),
    opLat=3,
    issueLat=1,
    timings=[MinorFUTiming(description="CoralNPU_FPU", srcRegsRelativeLats=[0, 0])],
)

# CSR / System — serialising instructions (csrrw/csrrs/csrrc/fence/wfi/…).
_CoralNPU_CSR = MinorFU(
    opClasses=_make_op_class_set(["System"]),
    opLat=1,
    issueLat=1,
    timings=[MinorFUTiming(description="CoralNPU_CSR", srcRegsRelativeLats=[0])],
)


# ════════════════════════════════════════════════════════════════════════════
# LSU — shared by scalar and vector memory operations
# ════════════════════════════════════════════════════════════════════════════

# LSU — 1 slot (queue depth 8 in HW).
#
# Scalar ops:
#   opLat=1: address calculation.
#   extraAssumedLat=2: scoreboard holds load dest for 2 extra cycles
#   (total 3 cy from issue = 1-cy SRAM + 1 pipeline reg + sign-extend).
#
# Vector memory ops (RVV):
#   All RVV load/store patterns are routed through the same LSU slot.
#   gem5 models vector memory as a sequence of scalar-width transactions
#   through the D-cache; the latency visible to the scoreboard is the
#   same opLat+extraAssumedLat as scalar loads.  Strided and indexed
#   patterns incur additional latency from address-generation iterations,
#   modelled implicitly by gem5's LSQ state machine.
_CoralNPU_LSU = MinorFU(
    opClasses=_make_op_class_set([
        # ── Scalar memory ────────────────────────────────────────────────
        "MemRead", "MemWrite", "FloatMemRead", "FloatMemWrite",
        # ── Vector memory (all RVV access patterns) ──────────────────────
        "SimdUnitStrideLoad",              # vle<eew>.v
        "SimdUnitStrideStore",             # vse<eew>.v
        "SimdUnitStrideMaskLoad",          # vlm.v
        "SimdUnitStrideMaskStore",         # vsm.v
        "SimdStridedLoad",                 # vlse<eew>.v
        "SimdStridedStore",                # vsse<eew>.v
        "SimdIndexedLoad",                 # vluxei/vloxei
        "SimdIndexedStore",                # vsuxei/vsoxei
        "SimdUnitStrideFaultOnlyFirstLoad",# vle<eew>ff.v
        "SimdWholeRegisterLoad",           # vl<n>r.v
        "SimdWholeRegisterStore",          # vs<n>r.v
    ]),
    opLat=1,
    issueLat=1,
    timings=[MinorFUTiming(
        description="CoralNPU_LSU",
        srcRegsRelativeLats=[0],
        extraAssumedLat=2,
    )],
)


# ════════════════════════════════════════════════════════════════════════════
# RVV co-processor functional units (Level-2 approximate timing)
#
# Latency derivation
# ──────────────────
# From coralnpu/docs/pipeline_and_instructions_organized.md §2.2:
#
#   Total latency = 3 cy fixed overhead (DE2-FF + ROB-FF + VRF-FF)
#                 + EX cycles (unit-dependent)
#
#   Single-uop (N_uops=1) totals from RTL timing table:
#     ALU total = 5 cy  → EX = 2 cy
#     MUL total = 5 cy  → EX = 2 cy
#     DIV total ≈ 35 cy → EX ≈ 32 cy (same as scalar DVU)
#
#   v-to-v chain penalty:
#     ROB result bypass (combinatorial, fires 1 cy before VRF write) reduces
#     the effective v-to-v latency from 4 to 3 cycles.
#     Modelled via srcRegsRelativeLats=[2, 2] with opLat=5:
#       consumer can issue when: curCycle >= producerIssue + (5 - 2) = +3
#
#   Multi-uop (LMUL>1 or vl > VLEN/EEW):
#     gem5 generates one micro-op per LMUL group, issued sequentially.
#     With issueLat=1 (pipelined), consecutive uops overlap in the FU,
#     giving: total ≈ 5 + (N_uops - 1), which matches the RTL formula.
#
# Hardware inventory (from rvv_backend_define.svh):
#   NUM_ALU = 2,  NUM_MUL = 2,  NUM_DIV = 1
# ════════════════════════════════════════════════════════════════════════════

# VEC_ALU × 2 — integer arithmetic, logic, shift, compare, reduce, convert,
#               mask ops, and vsetvl* configuration.
# opLat=5 (3 overhead + 2 EX). II=1 (pipelined). srcRelLats=[2,2] → v-to-v=3cy.
_CoralNPU_VEC_ALU = MinorFU(
    opClasses=_make_op_class_set([
        "SimdAdd",          # vadd, vsub, vrsub, vneg
        "SimdAlu",          # vand, vor, vxor, vnot
        "SimdCmp",          # vmseq, vmsne, vmsltu, vmslt, vmsleu, vmsle, vmsgtu, vmsgt
        "SimdMisc",         # vmv.v.*, vmerge, vfirst, vmsbf, vmsif, vmsof, vid, viota
        "SimdShift",        # vsll, vsrl, vsra, vnsrl, vnsra
        "SimdShiftAcc",     # vssrl, vssra (shift with rounding)
        "SimdReduceAlu",    # vredand, vredor, vredxor, vredmin/u, vredmax/u
        "SimdReduceCmp",    # vminu, vmin, vmaxu, vmax reductions
        "SimdCvt",          # vzext, vsext (zero/sign extend)
        "SimdExt",          # vslideup, vslidedown, vslide1up, vslide1down, vrgather
        "SimdConfig",       # vsetvl, vsetvli, vsetivli
    ]),
    opLat=5,
    issueLat=1,
    timings=[MinorFUTiming(
        description="CoralNPU_VEC_ALU",
        srcRegsRelativeLats=[2, 2],
    )],
)

# VEC_MUL × 2 — integer multiply/MAC, dot-product, float arithmetic.
# opLat=5 (3 overhead + 2 EX). II=1 (pipelined). srcRelLats=[2,2] → v-to-v=3cy.
_CoralNPU_VEC_MUL = MinorFU(
    opClasses=_make_op_class_set([
        "SimdMult",             # vmul, vmulh, vmulhu, vmulhsu
        "SimdMultAcc",          # vmacc, vnmsac, vmadd, vnmsub
        "SimdAddAcc",           # vsadd, vsaddu, vssub, vssubu (saturating)
        "SimdReduceAdd",        # vredsum, vwredsum, vwredsumu
        "SimdDotProd",          # vdot (if available)
        # Floating-point (routes through RVV co-processor's MUL pipeline)
        "SimdFloatAdd",         # vfadd, vfsub, vfrsub
        "SimdFloatAlu",         # vfsgnj, vfsgnjn, vfsgnjx, vfmin, vfmax
        "SimdFloatCmp",         # vmfeq, vmfne, vmflt, vmfle, vmfgt, vmfge
        "SimdFloatCvt",         # vfcvt.*, vfwcvt.*, vfncvt.*
        "SimdFloatMisc",        # vfmerge, vfmv.*
        "SimdFloatMult",        # vfmul, vfwmul
        "SimdFloatMultAcc",     # vfmacc, vfnmacc, vfmsac, vfnmsac, vfmadd, vfnmadd
        "SimdFloatReduceAdd",   # vfredosum, vfredusum, vfwredosum, vfwredusum
        "SimdFloatReduceCmp",   # vfredmin, vfredmax
        "SimdFloatExt",         # vfslide*, vfrgather
    ]),
    opLat=5,
    issueLat=1,
    timings=[MinorFUTiming(
        description="CoralNPU_VEC_MUL",
        srcRegsRelativeLats=[2, 2],
    )],
)

# VEC_DIV × 1 — integer divide/remainder and float divide/sqrt.
# Non-pipelined: EX ≈ 32 cy + 3 cy overhead = 35 cy total.
# II = 35 (next divide cannot enter until this one finishes).
_CoralNPU_VEC_DIV = MinorFU(
    opClasses=_make_op_class_set([
        "SimdDiv",          # vdivu, vdiv, vremu, vrem
        "SimdSqrt",         # (not in base RVV integer but included for completeness)
        "SimdFloatDiv",     # vfdiv, vfrdiv
        "SimdFloatSqrt",    # vfsqrt
    ]),
    opLat=35,
    issueLat=35,
    timings=[MinorFUTiming(description="CoralNPU_VEC_DIV", srcRegsRelativeLats=[0, 0])],
)


# ════════════════════════════════════════════════════════════════════════════
# Full FU pool
# ════════════════════════════════════════════════════════════════════════════

CoralNPU_FUPool = MinorFUPool(funcUnits=[
    # ── Scalar units ────────────────────────────────────────────────────────
    _CoralNPU_ALU,      # lane 0
    _CoralNPU_ALU,      # lane 1
    _CoralNPU_ALU,      # lane 2
    _CoralNPU_ALU,      # lane 3
    _CoralNPU_MLU,      # 1 shared scalar multiplier
    _CoralNPU_DVU,      # 1 scalar divider (lane-0-only not enforced)
    _CoralNPU_FPU,      # 1 scalar FPU (slot-0-only not enforced)
    _CoralNPU_LSU,      # 1 LSU slot — handles scalar AND vector memory
    _CoralNPU_CSR,      # System / CSR / serialising
    # ── RVV co-processor units ──────────────────────────────────────────────
    _CoralNPU_VEC_ALU,  # RVV ALU instance 0  (NUM_ALU=2 in RTL)
    _CoralNPU_VEC_ALU,  # RVV ALU instance 1
    _CoralNPU_VEC_MUL,  # RVV MUL instance 0  (NUM_MUL=2 in RTL)
    _CoralNPU_VEC_MUL,  # RVV MUL instance 1
    _CoralNPU_VEC_DIV,  # RVV DIV instance 0  (NUM_DIV=1 in RTL)
])


# ════════════════════════════════════════════════════════════════════════════
# CoralNPUMinorCPU
# ════════════════════════════════════════════════════════════════════════════

class CoralNPUMinorCPU(RiscvMinorCPU):
    """
    RiscvMinorCPU parameterised to match the CoralNPU scalar + RVV microarch.

    Key pipeline dimensions
    -----------------------
    fetch1LineWidth              = 32   bytes  (256-bit L0 cache line)
    decodeInputWidth             = 4           (4-lane dispatch)
    executeInputWidth            = 4
    executeIssueLimit            = 4           (max 4 issues/cycle)
    executeMemoryIssueLimit      = 1           (1 LSU op/cycle)
    executeCommitLimit           = 4
    executeInputBufferSize       = 8           (matches ROB = 8 entries)
    executeBranchDelay           = 1           (1-cycle branch redirect)
    executeLSQTransfersQueueSize = 8           (LSU queue depth)

    ISA: RV32IMF + V  (VLEN=128, ELEN=32)
    """

    # ── ISA: RV32IM + RVV with CoralNPU vector parameters ────────────────────
    # enable_rvv=True  : enable standard RVV instruction decoding
    # vlen=128         : CoralNPU rvvVlen=128 (16-byte vector registers)
    # elen=32          : CoralNPU max element width = 32 bits (int8/16/32 only)
    isa = [RiscvISA(
        riscv_type = "RV32",
        enable_rvv = True,
        vlen       = 128,
        elen       = 32,
    )]
    mmu = RiscvMMU()

    # ── Fetch stage ───────────────────────────────────────────────────────────
    fetch1LineWidth = 32                # 256-bit L0 cache line
    fetch1FetchLimit = 1
    fetch1ToFetch2ForwardDelay = 1
    fetch1ToFetch2BackwardDelay = 1

    fetch2InputBufferSize = 4
    fetch2ToDecodeForwardDelay = 1
    fetch2CycleInput = True

    # ── Decode stage ──────────────────────────────────────────────────────────
    decodeInputWidth = 4               # instructionLanes = 4
    decodeInputBufferSize = 4
    decodeToExecuteForwardDelay = 1
    decodeCycleInput = True

    # ── Execute / Issue / Commit ──────────────────────────────────────────────
    executeInputWidth = 4
    executeCycleInput = True
    executeIssueLimit = 4
    executeMemoryIssueLimit = 1        # 1 LSU dispatch/cycle
    executeCommitLimit = 4
    executeMemoryCommitLimit = 1
    executeInputBufferSize = 8         # ROB depth = 8

    executeBranchDelay = 1
    executeAllowEarlyMemoryIssue = False

    # LSU queue sizes.
    executeLSQRequestsQueueSize = 1
    executeLSQTransfersQueueSize = 8   # HW queue depth = 8
    executeLSQStoreBufferSize = 8
    executeLSQMaxStoreBufferStoresPerCycle = 1

    # ── Functional unit pool ──────────────────────────────────────────────────
    executeFuncUnits = CoralNPU_FUPool

    # ── Branch predictor ──────────────────────────────────────────────────────
    branchPred = BranchPredictor(
        conditionalBranchPred=LocalBP(
            localPredictorSize=256,
            localHistoryTableSize=256,
            localCtrBits=2,
            numThreads=1,
        )
    )

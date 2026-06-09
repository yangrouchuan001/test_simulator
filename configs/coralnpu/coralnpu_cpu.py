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
#   Branch penalty : 2 cycles (2 fetch cycles flushed on misprediction)
#
#   Scalar FUs:
#     ALU  × 4   latency=1  II=1  (one per dispatch lane)
#     MLU  × 1   latency=3  II=1  (pipelined shared multiplier)
#     DVU  × 1   latency=34 II=34 (non-pipelined divider, lane-0 only in HW)
#     FPU  × 1   latency=3  II=1  (pipelined, slot-0 only in HW)
#     ScalarLSU × 1  latency=1  extraAssumedLat=1  (2cy total; RTL: 1addr+1SRAM)
#     VecLSU    × 1  latency=1  extraAssumedLat=4  (5cy total; RTL: 3ovhd+2EX)
#     CSR  × 1   latency=1  serialising
#
# RVV co-processor (Level-2 approximate timing)
# ──────────────────────────────────────────────
#   VLEN=128, ELEN=32 (max element width 32 bits, matching CoralNPU RTL)
#
#   Latency model (from pipeline_and_instructions_organized.md):
#     total latency = 3 cy fixed overhead (DE2-FF + ROB-FF + VRF-FF)
#                   + EX cycles per unit
#     v-to-v chain penalty = opLat - srcRegsRelativeLat
#       ALU/MUL/PMTRDT: opLat - 2 = 3 cy  (ROB bypass fires 1 cy early)
#       Note: PMTRDT srcRelLat=3 to give same 3cy chain as ALU/MUL
#
#   RVV FUs (from rvv_backend_define.svh: NUM_ALU=2, NUM_MUL=2, NUM_DIV=1, NUM_PMTRDT=1):
#     VEC_ALU    × 2  opLat=5  II=1   (2 ALUs, EX=2cy; vadd/vsub/vshift/vcmp/vmask)
#     VEC_MUL    × 2  opLat=5  II=1   (2 MULs, EX=2cy; vmul/vmacc/vfadd/vfmul)
#     VEC_DIV    × 1  opLat=35 II=35  (1 non-pipelined DIV, EX≈32cy)
#     VEC_PMTRDT × 1  opLat=6  II=6   (1 non-pipelined, EX=3cy; reductions/slides/gather)
#     VecLSU     × 1  opLat=5cy total (shared port, 5cy DTCM)
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
# Factory function: each call returns a NEW MinorFU object so that gem5 can
# assign each pool slot a unique stats-group name (reusing the same object
# across slots causes "Stats of the same group share the same name" panic).
def _make_CoralNPU_ALU():
    return MinorFU(
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
# LSU — separate scalar and vector slots to match RTL latencies
# ════════════════════════════════════════════════════════════════════════════
#
# RTL latency (pipeline_and_instructions.md):
#
#   Scalar DTCM load:  2 cy  (E1: address calc; E2: SRAM response)
#     → opLat=1 + extraAssumedLat=1 = 2 cy total
#
#   Vector DTCM load:  3 cy overhead (DE2-FF + ROB-FF + VRF-FF)
#                    + ceil(vl × EEW_bytes / 32) × (1 + memory_cycles)
#     For vl=4, EEW=32, VLEN=128, DTCM (memory_cycles=1):
#       = 3 + ceil(16B/32B) × 2 = 3 + 1×2 = 5 cy
#     → opLat=1 + extraAssumedLat=4 = 5 cy total
#
# gem5's MinorCPU dispatches scalar and vector memory ops through
# independent FU pool slots; having two separate entries lets the
# scoreboard apply the correct latency to each class.

# Scalar LSU — 2-cycle DTCM latency (1 addr + 1 SRAM).
_CoralNPU_ScalarLSU = MinorFU(
    opClasses=_make_op_class_set([
        "MemRead", "MemWrite", "FloatMemRead", "FloatMemWrite",
    ]),
    opLat=1,
    issueLat=1,
    timings=[MinorFUTiming(
        description="CoralNPU_ScalarLSU",
        srcRegsRelativeLats=[0],
        extraAssumedLat=1,    # 1+1 = 2 cy total  (RTL: 2 cy)
    )],
)

# Vector LSU — 5-cycle DTCM latency (3 overhead + 2 for one 32-byte tx).
# extraAssumedLat=4 applies to all patterns; multi-sub-transaction accesses
# (LMUL>1, vl > VLEN/EEW, strided, indexed) naturally stall longer because
# gem5's LSQ issues one sub-transaction per cycle, adding the correct extra
# cycles implicitly.
_CoralNPU_VecLSU = MinorFU(
    opClasses=_make_op_class_set([
        "SimdUnitStrideLoad",               # vle<eew>.v
        "SimdUnitStrideStore",              # vse<eew>.v
        "SimdUnitStrideMaskLoad",           # vlm.v
        "SimdUnitStrideMaskStore",          # vsm.v
        "SimdStridedLoad",                  # vlse<eew>.v
        "SimdStridedStore",                 # vsse<eew>.v
        "SimdIndexedLoad",                  # vluxei/vloxei
        "SimdIndexedStore",                 # vsuxei/vsoxei
        "SimdUnitStrideFaultOnlyFirstLoad", # vle<eew>ff.v
        "SimdWholeRegisterLoad",            # vl<n>r.v
        "SimdWholeRegisterStore",           # vs<n>r.v
    ]),
    opLat=1,
    issueLat=1,
    timings=[MinorFUTiming(
        description="CoralNPU_VecLSU",
        srcRegsRelativeLats=[0],
        extraAssumedLat=4,    # 1+4 = 5 cy total  (RTL: 3 overhead + 2 EX)
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

# VEC_ALU × 2 — integer arithmetic, logic, shift, compare, convert, mask ops,
#               and vsetvl* configuration.
# opLat=5 (3 overhead + 2 EX). II=1 (pipelined). srcRelLats=[2,2] → v-to-v=3cy.
# NOTE: reductions (SimdReduce*) and permutations (SimdExt/SimdFloat*Ext) are
#       handled by the single non-pipelined VEC_PMTRDT unit below.
def _make_CoralNPU_VEC_ALU():
    return MinorFU(
        opClasses=_make_op_class_set([
            "SimdAdd",          # vadd, vsub, vrsub, vneg
            "SimdAlu",          # vand, vor, vxor, vnot
            "SimdCmp",          # vmseq, vmsne, vmsltu, vmslt, vmsleu, vmsle, vmsgtu, vmsgt
            "SimdMisc",         # vmv.v.*, vmerge, vfirst, vmsbf, vmsif, vmsof, vid, viota
            "SimdShift",        # vsll, vsrl, vsra, vnsrl, vnsra
            "SimdShiftAcc",     # vssrl, vssra (shift with rounding)
            "SimdCvt",          # vzext, vsext (zero/sign extend)
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
# NOTE: float reductions (SimdFloatReduce*) and float permutations (SimdFloatExt)
#       go to VEC_PMTRDT (non-pipelined), NOT here.
def _make_CoralNPU_VEC_MUL():
    return MinorFU(
        opClasses=_make_op_class_set([
            "SimdMult",             # vmul, vmulh, vmulhu, vmulhsu
            "SimdMultAcc",          # vmacc, vnmsac, vmadd, vnmsub
            "SimdAddAcc",           # vsadd, vsaddu, vssub, vssubu (saturating)
            "SimdDotProd",          # vdot (if available)
            # Floating-point (routes through RVV co-processor's MUL pipeline)
            "SimdFloatAdd",         # vfadd, vfsub, vfrsub
            "SimdFloatAlu",         # vfsgnj, vfsgnjn, vfsgnjx, vfmin, vfmax
            "SimdFloatCmp",         # vmfeq, vmfne, vmflt, vmfle, vmfgt, vmfge
            "SimdFloatCvt",         # vfcvt.*, vfwcvt.*, vfncvt.*
            "SimdFloatMisc",        # vfmerge, vfmv.*
            "SimdFloatMult",        # vfmul, vfwmul
            "SimdFloatMultAcc",     # vfmacc, vfnmacc, vfmsac, vfnmsac, vfmadd, vfnmadd
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

# VEC_PMTRDT × 1 — Permutation / Reduce / Tree unit (single non-pipelined instance).
#
# RTL source: rvv_backend_pmtrdt_unit.sv, _reduction.sv  (NUM_PMTRDT=1)
# EX latency (VLEN=128): 3 cy  (binary-tree depth = log2(128/8/4) + 2 = 3)
# Total: 3 cy overhead + 3 cy EX = 6 cy.
# II = 6 (non-pipelined: next reduction waits for current to finish).
# v-to-v chain penalty = opLat - srcRelLat = 6 - 3 = 3 cy
#   (same 3-cycle penalty as ALU/MUL — ROB bypass fires 1 cy before VRF write).
#
# Covers:
#   Reductions  : vredsum, vredand, vredor, vredxor, vredmin/u, vredmax/u,
#                 vwredsum/u, vfredosum, vfredusum, vfredmin, vfredmax
#   Permutations: vslideup, vslidedown, vslide1up, vslide1down,
#                 vrgather.vv/vx/vi, vrgatherei16
_CoralNPU_VEC_PMTRDT = MinorFU(
    opClasses=_make_op_class_set([
        # Integer reductions
        "SimdReduceAlu",        # vredand, vredor, vredxor, vredmin/u, vredmax/u
        "SimdReduceCmp",        # vredminu, vredmaxu (comparison-tree variants)
        "SimdReduceAdd",        # vredsum, vwredsum, vwredsumu
        # Permutations and gather
        "SimdExt",              # vslideup, vslidedown, vslide1up, vslide1down, vrgather
        # Float reductions
        "SimdFloatReduceAdd",   # vfredosum, vfredusum, vfwredosum, vfwredusum
        "SimdFloatReduceCmp",   # vfredmin, vfredmax
        # Float permutations
        "SimdFloatExt",         # vfslide1up, vfslide1down, vfrgather
    ]),
    opLat=6,
    issueLat=6,     # non-pipelined: II = opLat
    timings=[MinorFUTiming(
        description="CoralNPU_VEC_PMTRDT",
        srcRegsRelativeLats=[3, 3],   # chain penalty = 6 - 3 = 3 cy (= ALU/MUL penalty)
    )],
)


# ════════════════════════════════════════════════════════════════════════════
# Full FU pool
# ════════════════════════════════════════════════════════════════════════════

CoralNPU_FUPool = MinorFUPool(funcUnits=[
    # ── Scalar units ────────────────────────────────────────────────────────
    _make_CoralNPU_ALU(),   # lane 0  — each call returns a distinct object
    _make_CoralNPU_ALU(),   # lane 1
    _make_CoralNPU_ALU(),   # lane 2
    _make_CoralNPU_ALU(),   # lane 3
    _CoralNPU_MLU,          # 1 shared scalar multiplier
    _CoralNPU_DVU,          # 1 scalar divider (lane-0-only not enforced)
    _CoralNPU_FPU,          # 1 scalar FPU (slot-0-only not enforced)
    _CoralNPU_ScalarLSU,    # scalar memory: 2-cy DTCM latency
    _CoralNPU_VecLSU,       # vector memory: 5-cy DTCM latency
    _CoralNPU_CSR,          # System / CSR / serialising
    # ── RVV co-processor units ──────────────────────────────────────────────
    _make_CoralNPU_VEC_ALU(),   # RVV ALU instance 0  (NUM_ALU=2 in RTL)
    _make_CoralNPU_VEC_ALU(),   # RVV ALU instance 1
    _make_CoralNPU_VEC_MUL(),   # RVV MUL instance 0  (NUM_MUL=2 in RTL)
    _make_CoralNPU_VEC_MUL(),   # RVV MUL instance 1
    _CoralNPU_VEC_DIV,          # RVV DIV instance 0  (NUM_DIV=1 in RTL)
    _CoralNPU_VEC_PMTRDT,       # RVV PMTRDT instance 0  (NUM_PMTRDT=1 in RTL)
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
    executeBranchDelay           = 2           (2-cycle branch flush; RTL: 2 fetch cycles)
    executeLSQTransfersQueueSize = 8           (LSU queue depth)

    ISA: RV32IMF + V  (VLEN=128, ELEN=32)
    """

    # ── ISA: RV32IM + RVV with CoralNPU vector parameters ────────────────────
    # enable_rvv=True  : enable standard RVV instruction decoding
    # vlen=128         : CoralNPU rvvVlen=128 (16-byte vector registers)
    # elen=32          : CoralNPU max element width = 32 bits (int8/16/32 only)
    isa = [RiscvISA(
        riscv_type        = "RV32",
        enable_rvv        = True,
        vlen              = 128,
        elen              = 32,
        privilege_mode_set = "M",   # bare-metal M-mode only; no S/U modes
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

    executeBranchDelay = 2           # RTL: 2 fetch cycles flushed on mispredict
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
            localCtrBits=2,
        )
    )

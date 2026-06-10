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
#       ALU/MUL:   opLat(5) - srcRelLat(2) = 3 cy  (ROB bypass fires 1 cy early)
#       PMTRDT:    opLat(6) - srcRelLat(3) = 3 cy  (same chain as ALU/MUL)
#
#   RVV FUs (from rvv_backend_define.svh: NUM_ALU=2, NUM_MUL=2,
#            NUM_PMTRDT=1, NUM_DIV=1):
#     VEC_ALU    × 2  opLat=5  II=1   CMP+ALU exe_unit (element-wise arith/logic/cmp)
#     VEC_MUL    × 2  opLat=5  II=1   MUL+MAC exe_unit (multiply/MAC/float-arith)
#     VEC_PMTRDT × 1  opLat=6  II=6   MISC+PMT+RDT exe_unit (permute/reduce/misc)
#     VEC_DIV    × 1  opLat=35 II=35  DIV exe_unit (non-pipelined)
#     VecLSU     × 1  opLat=5cy total LSU exe_unit (5cy DTCM)
#
#   Scalar-vector decoupling (§11.25-§11.27):
#     Scalar instructions bypass stalled/pending vector ops at BOTH issue and
#     retirement, matching RTL CQ/UQ/ROB decoupled scalar-vector retirement.
#     The vectorPendingQueue acts as the PMTRDT RS (depth 4 → modelled by
#     vectorPendingQueueSize=16 which also covers ALU/MUL RS).
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
        "SimdUnitStrideFaultOnlyFirstLoad",          # vle<eew>ff.v
        "SimdWholeRegisterLoad",                     # vl<n>r.v
        "SimdWholeRegisterStore",                    # vs<n>r.v
        "SimdUnitStrideSegmentedLoad",               # vlseg2e8.v … vlseg8e64.v
        "SimdUnitStrideSegmentedStore",              # vsseg2e8.v … vsseg8e64.v
        "SimdUnitStrideSegmentedFaultOnlyFirstLoad", # vlseg<n>e<eew>ff.v
        "SimdStrideSegmentedLoad",                   # vlsseg<n>e<eew>.v
        "SimdStrideSegmentedStore",                  # vssseg<n>e<eew>.v
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

# VEC_ALU × 2 — RTL exe_unit: CMP + ALU
#   Handles element-wise integer arithmetic, logic, shift, compare, convert,
#   and vsetvl config.  Matches RTL ALU RS (rs_ready_alu2dp).
# opLat=5 (3 overhead + 2 EX). II=1 (pipelined). srcRelLats=[2,2] → v-to-v=3cy.
def _make_CoralNPU_VEC_ALU():
    return MinorFU(
        opClasses=_make_op_class_set([
            "SimdAdd",          # vadd, vsub, vrsub, vneg         — ALU
            "SimdAlu",          # vand, vor, vxor, vnot           — ALU
            "SimdCmp",          # vmseq/vmsne/vmsltu/… (→ mask)   — CMP
            "SimdShift",        # vsll, vsrl, vsra, vnsrl, vnsra  — ALU
            "SimdShiftAcc",     # vssrl, vssra (rounding shift)   — ALU
            "SimdCvt",          # vzext, vsext                    — ALU
            "SimdConfig",       # vsetvl, vsetvli, vsetivli       — special
        ]),
        opLat=5,
        issueLat=1,
        timings=[MinorFUTiming(
            description="CoralNPU_VEC_ALU",
            srcRegsRelativeLats=[2, 2],
        )],
    )

# VEC_MUL × 2 — RTL exe_unit: MUL + MAC
#   Handles integer multiply/MAC, dot-product, and float arithmetic.
#   Matches RTL MUL RS (rs_ready_mul2dp).
# opLat=5 (3 overhead + 2 EX). II=1 (pipelined). srcRelLats=[2,2] → v-to-v=3cy.
def _make_CoralNPU_VEC_MUL():
    return MinorFU(
        opClasses=_make_op_class_set([
            "SimdMult",             # vmul, vmulh, vmulhu, vmulhsu      — MUL
            "SimdMultAcc",          # vmacc, vnmsac, vmadd, vnmsub       — MAC
            "SimdAddAcc",           # vsadd, vsaddu, vssub, vssubu       — ALU (saturation)
            "SimdDotProd",          # vdot                               — MAC
            # Floating-point (RTL: FMA/FNCMP/FCMP/FTBL/FCVT exe_unit, ZVE32F_ON)
            "SimdFloatAdd",         # vfadd, vfsub, vfrsub
            "SimdFloatAlu",         # vfsgnj*, vfmin, vfmax
            "SimdFloatCmp",         # vmfeq, vmfne, vmflt, vmfle, vmfgt, vmfge
            "SimdFloatCvt",         # vfcvt.*, vfwcvt.*, vfncvt.*
            "SimdFloatMisc",        # vfmerge, vfmv.*
            "SimdFloatMult",        # vfmul, vfwmul
            "SimdFloatMultAcc",     # vfmacc, vfnmacc, vfmsac, vfnmsac, …
        ]),
        opLat=5,
        issueLat=1,
        timings=[MinorFUTiming(
            description="CoralNPU_VEC_MUL",
            srcRegsRelativeLats=[2, 2],
        )],
    )

# VEC_PMTRDT × 1 — RTL exe_unit: MISC + PMT + RDT
#   Handles permutation (vslide*, vrgather), reduction (vred*), and
#   miscellaneous ops (vmv.v.*, vmerge, viota, vid, vfirst, vmsbf/vmsif/vmsof).
#   Matches RTL PMTRDT RS (rs_ready_pmtrdt2dp, NUM_PMTRDT=1, PMTRDT_RS_DEPTH=4).
#
#   Latency (from rvv_backend_pmtrdt_unit.sv, VLENB=16B = VLEN=128):
#     Permutation/MISC: 2 EX → 3+2 = 5 cy  ("2-cycle for each uop" comment)
#     Reduction:        3 EX → 3+3 = 6 cy  (log2 tree: 2+log2(VLENB/8) stages)
#   Using worst-case opLat=6 (reduction).  Permutation/MISC are over-conservative
#   by 1 cy, but with bypass commit (§11.26/§11.27) scalar retirement is unaffected.
#
#   v-to-v chain: opLat(6) - srcRelLat(3) = 3 cy  (same as ALU/MUL).
#
#   issueLat=1: PIPELINED (§11.35). The RTL uses a handshake_ff pipeline
#     (rvv_backend_pmtrdt_unit.sv): inready = ~outvalid | outready, which
#     allows a new instruction into stage-0 every cycle regardless of whether
#     the previous instruction has completed.  II=1, not II=6.
#     The vectorPendingQueue (§11.25) still acts as the RS dispatch buffer.
def _make_CoralNPU_VEC_PMTRDT():
    return MinorFU(
        opClasses=_make_op_class_set([
            # MISC (RTL: MISC → PMTRDT RS)
            "SimdMisc",             # vmv.v.i/x/v, vmerge, vfirst, vmsbf/vmsif/vmsof, vid, viota, vcpop
            # PMT — permutation (RTL: PMT → PMTRDT RS)
            "SimdExt",              # vslideup, vslidedown, vslide1up, vslide1down, vrgather, vcompress
            "SimdFloatExt",         # vfslide1up, vfslide1down, vfrgather
            # RDT — integer reduction (RTL: RDT → PMTRDT RS)
            "SimdReduceAlu",        # vredand, vredor, vredxor, vredmin/u, vredmax/u
            "SimdReduceCmp",        # vredminu, vredmaxu (alias in gem5)
            "SimdReduceAdd",        # vredsum, vwredsum, vwredsumu
            # RDT — float reduction (RTL: RDT → PMTRDT RS, ZVE32F_ON)
            "SimdFloatReduceAdd",   # vfredosum, vfredusum, vfwredosum
            "SimdFloatReduceCmp",   # vfredmin, vfredmax
        ]),
        opLat=6,
        issueLat=1,
        timings=[MinorFUTiming(
            description="CoralNPU_VEC_PMTRDT",
            srcRegsRelativeLats=[3, 3],  # v-to-v chain = 6-3 = 3cy (same as ALU/MUL)
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
    _make_CoralNPU_VEC_ALU(),    # RVV ALU instance 0  (NUM_ALU=2 in RTL)
    _make_CoralNPU_VEC_ALU(),    # RVV ALU instance 1
    _make_CoralNPU_VEC_MUL(),    # RVV MUL instance 0  (NUM_MUL=2 in RTL)
    _make_CoralNPU_VEC_MUL(),    # RVV MUL instance 1
    _make_CoralNPU_VEC_PMTRDT(), # RVV PMTRDT instance 0  (NUM_PMTRDT=1 in RTL)
    _CoralNPU_VEC_DIV,           # RVV DIV instance 0  (NUM_DIV=1 in RTL)
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
    executeInputBufferSize       = 8           (RTL retirementBufferSize=8; §11.36)
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
    executeInputBufferSize = 8         # RTL retirementBufferSize=8 (scalar ROB); §11.36

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

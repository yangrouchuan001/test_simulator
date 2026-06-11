---
name: project-fregfile-regd-only
description: FRegfile.scala has only a registered (regd) scoreboard — no comb forwarding for float registers
metadata:
  type: project
---

FRegfile.scala (`/home/cn2095/git space/coralnpu/coralnpu/hdl/chisel/src/coralnpu/scalar/FRegfile.scala`) exposes only a single registered scoreboard:

```scala
scoreboard := (scoreboard & ~scoreboard_clr) | io.scoreboard_set  // registered
io.scoreboard := scoreboard                                         // no comb output
```

Unlike Regfile.scala (integer), which has both `regd` (registered, clears T+1) and `comb` (combinatorial, clears T+0) outputs, the float register file has NO same-cycle forwarding.

**Why:** All FPU consumers reading float source registers must wait for the registered float scoreboard to clear (T+opLat+1 relative to the producer).

**How to apply:**
- In gem5 `coralnpu_cpu.py`: FPU_FF, FPU_FI, FDV use `srcRegsRelativeLats=[0,0]` (regd consumer).
- Do NOT use `srcRegsRelativeLats=[1,1]` for FPU FUs — float regs have no comb path.
- FPU FU `extraAssumedLat=1` correctly models the float regd +1 cycle delay.
- ALU/MLU/DVU/CSR: `srcRegsRelativeLats=[1,...]` (integer comb consumers) ← different.
- ScalarLSU/VecLSU: `srcRegsRelativeLats=[0]` (integer regd consumers) ← same as FPU.

Also confirmed in Decode.scala:
```scala
val fcomb = floatScoreboardScan.map(_ | io.fscoreboard.getOrElse(0.U))
val floatReadAfterWrite = (floatReadScoreboard(i) & fcomb(i)) =/= 0.U
```
`io.fscoreboard` is the registered output of FRegfile — no comb path.

Related: [[project-regd-comb-scoreboard]] (§11.38 dual-scoreboard for integer regs)

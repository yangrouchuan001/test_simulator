/*
 * Backward-Taken Forward-Not-Taken (BTFN) static branch predictor.
 *
 * Models CoralNPU RTL Fetch.scala PredecodeDe: all backward conditional
 * branches predicted taken, all forward conditional branches predicted
 * not-taken.  Used to match RTL branch-prediction behaviour in gem5.
 *
 * Because gem5's ConditionalPredictor::lookup() only receives the branch PC
 * (not the instruction encoding), direction is inferred from the resolved
 * target of committed taken branches and cached in a per-PC map.  Entries
 * are only written on taken resolutions so that loop-exit not-taken updates
 * (where target = PC+4 > PC) do not corrupt the backward-branch entries.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef __CPU_PRED_BTFN_HH__
#define __CPU_PRED_BTFN_HH__

#include <unordered_map>

#include "base/types.hh"
#include "cpu/pred/conditional.hh"
#include "params/BTFNBP.hh"

namespace gem5
{

namespace branch_prediction
{

class BTFNBP : public ConditionalPredictor
{
  public:
    BTFNBP(const BTFNBPParams &params);

    bool lookup(ThreadID tid, Addr pc, void * &bp_history) override;

    void updateHistories(ThreadID tid, Addr pc, bool uncond, bool taken,
                         Addr target, const StaticInstPtr &inst,
                         void * &bp_history) override;

    void update(ThreadID tid, Addr pc, bool taken, void * &bp_history,
                bool squashed, const StaticInstPtr &inst,
                Addr target) override;

    void squash(ThreadID tid, void * &bp_history) override {}

  private:
    /** Per-PC direction cache: true = backward branch = predict taken. */
    std::unordered_map<Addr, bool> dirCache;
};

} // namespace branch_prediction
} // namespace gem5

#endif // __CPU_PRED_BTFN_HH__

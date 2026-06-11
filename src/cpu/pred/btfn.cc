/*
 * Backward-Taken Forward-Not-Taken (BTFN) static branch predictor.
 * See btfn.hh for design notes.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "cpu/pred/btfn.hh"

namespace gem5
{

namespace branch_prediction
{

BTFNBP::BTFNBP(const BTFNBPParams &params)
    : ConditionalPredictor(params)
{}

bool
BTFNBP::lookup(ThreadID tid, Addr pc, void * &bp_history)
{
    bp_history = nullptr;
    auto it = dirCache.find(pc);
    if (it != dirCache.end())
        return it->second;  // true = backward = taken
    return false;           // default not-taken for unseen branches
}

void
BTFNBP::updateHistories(ThreadID tid, Addr pc, bool uncond, bool taken,
                         Addr target, const StaticInstPtr &inst,
                         void * &bp_history)
{
    // Static predictor has no speculative history to manage.
}

void
BTFNBP::update(ThreadID tid, Addr pc, bool taken, void * &bp_history,
               bool squashed, const StaticInstPtr &inst, Addr target)
{
    if (squashed)
        return;
    // Only update on taken resolutions: target is then the real branch
    // destination.  Skipping not-taken updates avoids loop-exit (target=PC+4)
    // corrupting backward-branch entries.
    if (taken)
        dirCache[pc] = (target < pc);
}

} // namespace branch_prediction
} // namespace gem5

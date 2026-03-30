#include "server/simulation/timestep/timestep_controller.hpp"

#include <algorithm>
#include <cmath>

namespace spaceship::server
{

double TimestepController::ladderDt(int k, double dt_max) noexcept
{
    // dt_k = dt_max / 2^k
    return dt_max / static_cast<double>(1u << k);
}

int TimestepController::quantizeToLadder(
    double dt_target,
    double dt_max,
    int    k_max) noexcept
{
    // Find largest k such that ladderDt(k) <= dt_target.
    // ladderDt(k) = dt_max / 2^k, so 2^k >= dt_max / dt_target
    // k >= log2(dt_max / dt_target)
    if (dt_target >= dt_max)
        return 0;   // coarsest level

    const double ratio = dt_max / dt_target;
    const int kRaw = static_cast<int>(std::ceil(std::log2(ratio)));
    return std::clamp(kRaw, 0, k_max);
}

int TimestepController::applyHysteresis(
    int                         kDesired,
    double                      frameDt,
    TimestepState&              state,
    const TimestepLadderConfig& cfg) noexcept
{
    const int kCurrent = state.currentK;

    if (kDesired > kCurrent)
    {
        // Step down (finer dt) — apply immediately (or after 1-tick delay).
        // We step down by at most one level per call to enforce ratio constraint.
        const int kNew = std::min(kDesired, kCurrent + 1);
        state.currentK       = kNew;
        state.tStableSeconds = 0.0;
    }
    else if (kDesired < kCurrent)
    {
        // Step up (coarser dt) — require dwell time before applying.
        state.tStableSeconds += frameDt;
        if (state.tStableSeconds >= cfg.tau_raise_seconds)
        {
            // Step up by at most one level.
            const int kNew       = std::max(kDesired, kCurrent - 1);
            state.currentK       = kNew;
            state.tStableSeconds = 0.0;
        }
        // else: stay at current level
    }
    else
    {
        // No change requested — accumulate stable time, capped to avoid unbounded growth.
        state.tStableSeconds = std::min(state.tStableSeconds + frameDt, 2.0 * cfg.tau_raise_seconds);
    }

    return state.currentK;
}

SubstepPlan TimestepController::planSubsteps(
    double frameDt,
    int    k,
    double dt_max) noexcept
{
    const double dtLadder = ladderDt(k, dt_max);

    // N = ceil(frameDt / dtLadder), minimum 1
    const int n = std::max(1, static_cast<int>(std::ceil(frameDt / dtLadder)));

    // Realized substep dt covers frameDt exactly (no remainder drift).
    const double dtSub = frameDt / static_cast<double>(n);

    return SubstepPlan{n, dtSub, k};
}

} // namespace spaceship::server

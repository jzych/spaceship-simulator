#pragma once

// TimestepController manages the quantized adaptive timestep ladder.
//
// Responsibilities:
//   - Map a desired dt to the nearest ladder level.
//   - Apply hysteresis: step down immediately, step up only after dwell time.
//   - Compute a SubstepPlan: how many Verlet substeps to take, and at what dt.
//
// The controller holds no simulation state. Callers own TimestepState per entity.

#include "server/simulation/timestep/timestep_types.hpp"

namespace spaceship::server
{

// Result of planning how to cover an outer frame interval.
struct SubstepPlan
{
    int    count {};   // number of Verlet substeps
    double dt    {};   // substep dt (seconds); count * dt == frameDt
    int    k     {};   // ladder level used
};

class TimestepController
{
  public:
    // dt value for ladder level k: dt_max / 2^k
    [[nodiscard]] static double ladderDt(int k, double dt_max) noexcept;

    // Find the largest k such that ladderDt(k) <= dt_target.
    // Result is clamped to [0, k_max].
    [[nodiscard]] static int quantizeToLadder(
        double dt_target,
        double dt_max,
        int    k_max) noexcept;

    // Apply hysteresis rules to dt_desired against current state.
    // Updates state.tStableSeconds and state.currentK.
    // Returns the applied ladder level.
    [[nodiscard]] static int applyHysteresis(
        int                      kDesired,
        double                   frameDt,
        TimestepState&           state,
        const TimestepLadderConfig& cfg) noexcept;

    // Compute how many substeps and at what dt to cover frameDt
    // given the current applied ladder level k.
    // The realized dt_sub = frameDt / N so there is no time-drift remainder.
    [[nodiscard]] static SubstepPlan planSubsteps(
        double frameDt,
        int    k,
        double dt_max) noexcept;
};

} // namespace spaceship::server

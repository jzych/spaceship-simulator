#pragma once

// Holds tunable fixed-step parameters for the authoritative simulation server.

#include "shared/sim_types.hpp"

namespace spaceship::server
{

struct SimulationConfig
{
    double fixedDeltaSeconds {shared::constants::kFixedDeltaSeconds};
    shared::Tick snapshotIntervalTicks {3};
    double projectileDefaultTtlSeconds {10.0};
};

} // namespace spaceship::server

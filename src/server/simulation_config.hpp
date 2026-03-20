#pragma once

// Holds tunable fixed-step parameters for the authoritative simulation server.

#include "shared/sim_types.hpp"

namespace spaceship::server
{

struct SimulationConfig
{
    double fixedDeltaSeconds {shared::constants::kFixedDeltaSeconds};
    shared::Tick snapshotIntervalTicks {3};
    double shipDefaultMassKg {1'000.0};
    double shipDefaultRadiusMeters {5.0};
    double shipMaxThrustNewtons {20'000.0};
    double projectileMassKg {1.0};
    double projectileRadiusMeters {0.1};
    double projectileDefaultTtlSeconds {10.0};
    double projectileMuzzleSpeedMetersPerSecond {1'000.0};
};

} // namespace spaceship::server

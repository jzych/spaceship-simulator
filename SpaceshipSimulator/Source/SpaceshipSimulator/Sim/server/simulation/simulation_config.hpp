#pragma once

// Holds tunable fixed-step parameters for the authoritative simulation server.

#include "server/simulation/timestep/timestep_types.hpp"
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

    // Reference body selection hysteresis
    double referenceBodyHysteresisDelta {0.15};
    double referenceBodyDwellTimeSeconds {5.0};

    // Muzzle offset: projectile spawns this far beyond (ship_radius + proj_radius) along forward
    double projectileSpawnOffsetSkinMeters {0.1};

    // Collision energy thresholds (SI units — Joules)
    double projectileProjectileDestroyEnergyJoules {1'000.0};
    double shipProjectileDestroyShipEnergyJoules   {50'000.0};
    double shipShipDestroyBothEnergyJoules         {500'000.0};

    // Adaptive timestep control (off by default — preserves fixed-step behaviour)
    bool useAdaptiveTimestep {false};
    TimestepLadderConfig timestepLadder {};
};

} // namespace spaceship::server

#pragma once

// Defines types for the quantized adaptive timestep control layer.
// TimestepLadderConfig holds all tunable parameters.
// TimestepState is per-entity and carries hysteresis state and jerk history.
// TimestepDiagnostics records per-entity per-frame telemetry (see SimulationServer::timestepDiagnostics).

#include "shared/sim_types.hpp"

namespace spaceship::server
{

// Tunable parameters for the timestep ladder and heuristics.
struct TimestepLadderConfig
{
    // Ladder: dt_k = dt_max / 2^k, k in [0, k_max]
    double dt_max {1.0 / 60.0};    // coarsest step (one 60 Hz tick)
    int    k_max  {6};             // finest step = dt_max / 64 ≈ 0.26 ms

    // Orbital/curvature heuristic: dt ≈ alpha_orbit / angular_rate
    double alpha_orbit {0.10};

    // Acceleration heuristic: dt ≈ alpha_acc * sqrt(|r_rel| / |a|)
    double alpha_acc   {0.50};

    // Jerk heuristic: dt ≈ alpha_jerk * |a| / |j|
    double alpha_jerk  {0.30};

    // Close-approach heuristic: dt ≈ alpha_close * distance / relative_speed
    double alpha_close {0.50};

    // Epsilon floors to prevent division by zero
    double r_eps {1.0};        // metres
    double a_eps {1e-6};       // m/s²
    double j_eps {1e-9};       // m/s³
    double v_eps {0.01};       // m/s

    // Hysteresis: minimum time spent at current level before stepping up
    double tau_raise_seconds {0.5};

    // Derived: dt_min = dt_max / 2^k_max
    [[nodiscard]] double dt_min() const noexcept
    {
        return dt_max / static_cast<double>(1u << k_max);
    }
};

// Per-entity state carried across ticks by the timestep controller.
struct TimestepState
{
    int    currentK       {};     // current ladder level (0 = coarsest)
    double tStableSeconds {};     // time spent at currentK without step-down pressure
    shared::Vec3 aPrev    {};     // acceleration from previous substep (for jerk)
    double dtPrev         {};     // dt used in previous substep (for jerk)
};

// Per-entity per-frame diagnostics recorded by the controller.
struct TimestepDiagnostics
{
    shared::NetId netId          {};
    double        dtRequested    {};  // target dt from heuristics (pre-quantization)
    double        dtApplied      {};  // actual substep dt used
    int           ladderLevel    {};  // k applied this frame
    int           substepCount   {};  // number of Verlet substeps executed
};


} // namespace spaceship::server

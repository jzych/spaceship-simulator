#pragma once

// Pure functions that map per-entity physical state to a recommended timestep.
// Each heuristic returns a dt in seconds. computeTargetTimestep combines them.

#include "server/simulation/simulation_world.hpp"
#include "server/simulation/timestep/timestep_types.hpp"
#include "shared/sim_types.hpp"

#include <span>

namespace spaceship::server
{

// 1. Orbital / curvature timescale: dt ≈ alpha_orbit / (|v| / |r_rel|)
[[nodiscard]] double computeOrbitalTimescale(
    const shared::Vec3& position,
    const shared::Vec3& velocity,
    const shared::Vec3& nearestBodyPosition,
    const TimestepLadderConfig& cfg) noexcept;

// 2. Acceleration timescale: dt ≈ alpha_acc * sqrt(|r_rel| / |a|)
[[nodiscard]] double computeAccelerationTimescale(
    const shared::Vec3& relativePosition,
    const shared::Vec3& acceleration,
    const TimestepLadderConfig& cfg) noexcept;

// 3. Jerk timescale: dt ≈ alpha_jerk * |a| / |j|
//    Returns cfg.dt_max when no previous acceleration is available (dtPrev == 0).
[[nodiscard]] double computeJerkTimescale(
    const shared::Vec3& acceleration,
    const shared::Vec3& previousAcceleration,
    double              dtPrev,
    const TimestepLadderConfig& cfg) noexcept;

// 4. Close-approach timescale: dt ≈ alpha_close * min_distance / relative_speed
[[nodiscard]] double computeCloseApproachTimescale(
    const shared::Vec3&               position,
    const shared::Vec3&               velocity,
    std::span<const MassiveBodyState> bodies,
    const TimestepLadderConfig&       cfg) noexcept;

// Combine all four heuristics and clamp to [dt_min, dt_max].
[[nodiscard]] double computeTargetTimestep(
    const shared::Vec3&               position,
    const shared::Vec3&               velocity,
    const shared::Vec3&               acceleration,
    const shared::Vec3&               previousAcceleration,
    double                            dtPrev,
    std::span<const MassiveBodyState> bodies,
    const TimestepLadderConfig&       cfg) noexcept;

} // namespace spaceship::server

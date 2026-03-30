#pragma once

// Continuous sphere-sweep TOI solver.
// Solves |p + u*t|^2 = R^2 for t in [0, dt].
// Returns the earliest valid root, or nullopt if no hit occurs.
// Already-overlapping pairs (c <= 0 at t=0) return toi = 0.

#include "shared/sim_types.hpp"

#include <optional>

namespace spaceship::server
{

// Compute time-of-impact for two moving spheres over interval [0, dt].
//   posA0, posB0 : positions at start of interval
//   velA,  velB  : constant velocities over the interval
//   rA, rB       : sphere radii
//   dt           : interval length in seconds
//
// Returns toi in [0, dt] on hit, nullopt on miss.
[[nodiscard]] std::optional<double> sphereSweepTOI(
    const shared::Vec3& posA0,
    const shared::Vec3& velA,
    const shared::Vec3& posB0,
    const shared::Vec3& velB,
    double rA,
    double rB,
    double dt) noexcept;

} // namespace spaceship::server

#pragma once

// Computes Keplerian orbital elements from a state vector (position, velocity)
// relative to a reference body. Pure functions — no side effects.

#include "server/simulation/gravity/orbit_cache.hpp"

namespace spaceship::server
{

// Converts relative state vector to Keplerian orbital elements.
// Sets isElliptic = false for hyperbolic/parabolic/degenerate states.
[[nodiscard]] OrbitCache fitOrbit(
    const shared::Vec3& relativePosition,
    const shared::Vec3& relativeVelocity,
    double mu,
    double bodyRadius,
    shared::NetId referenceBodyId,
    shared::Tick currentTick);

// Quality score: 1.0 for pure two-body, approaches 0 with increasing perturbation.
[[nodiscard]] double computeQualityScore(
    const shared::Vec3& totalAcceleration,
    const shared::Vec3& relativePosition,
    double mu);

} // namespace spaceship::server

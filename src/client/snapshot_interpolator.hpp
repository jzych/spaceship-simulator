#pragma once

// Pure stateless interpolation math.
// No ownership, no buffers — accepts two bracketing snapshots and a render
// time, returns a fully interpolated world state.
//
// Positions and velocities are linearly interpolated.
// Orientations use spherical linear interpolation (slerp) with short-arc
// correction and a fallback to normalised lerp when the quaternions are
// nearly identical (dot > 0.9995) to avoid division by near-zero sin(θ).
//
// Entity matching is by netId:
//   - Present in both s0 and s1 → interpolated.
//   - Only in s1 (spawned mid-interval) → copied verbatim from s1.
//   - Only in s0 (despawned mid-interval) → dropped from output.

#include "client/client_types.hpp"
#include "server/snapshot/snapshot_types.hpp"

namespace spaceship::client::interpolator
{

// Scalar and vector lerp.
[[nodiscard]] double       lerp(double a, double b, double t);
[[nodiscard]] shared::Vec3 lerp(const shared::Vec3& a, const shared::Vec3& b, double t);

// Quaternion slerp — always takes the short arc, result is unit-normalised.
[[nodiscard]] shared::Quaternion slerp(const shared::Quaternion& q0,
                                       const shared::Quaternion& q1,
                                       double                    t);

// Build an InterpolatedWorldState from two bracketing snapshots.
// renderTime must lie within [s0.elapsedSeconds, s1.elapsedSeconds];
// alpha = (renderTime - s0.elapsedSeconds) / (s1.elapsedSeconds - s0.elapsedSeconds).
[[nodiscard]] InterpolatedWorldState interpolateWorldState(const server::WorldSnapshot& s0,
                                                           const server::WorldSnapshot& s1,
                                                           double                       renderTime);

// Lift a single snapshot directly into an InterpolatedWorldState (alpha = 0).
// Used when only one snapshot is available (renderTime == snapshot time).
[[nodiscard]] InterpolatedWorldState fromSnapshot(const server::WorldSnapshot& snap,
                                                  double                       renderTime);

} // namespace spaceship::client::interpolator

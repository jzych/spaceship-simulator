#include "client/snapshot_interpolator.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>

namespace spaceship::client::interpolator
{

// ---------------------------------------------------------------------------
// Scalar and vector lerp
// ---------------------------------------------------------------------------

double lerp(double a, double b, double t)
{
    return a + t * (b - a);
}

shared::Vec3 lerp(const shared::Vec3& a, const shared::Vec3& b, double t)
{
    return {lerp(a.x, b.x, t), lerp(a.y, b.y, t), lerp(a.z, b.z, t)};
}

// ---------------------------------------------------------------------------
// Quaternion slerp
//
// Algorithm:
//   1. Compute dot = q0 · q1.
//   2. If dot < 0, negate q1 so we always travel the short arc.
//   3. If dot > 0.9995 the angle is nearly zero — fall back to normalised
//      lerp to avoid dividing by sin(θ) ≈ 0.
//   4. Otherwise use the standard slerp formula:
//        scaleFrom = sin((1-t)·θ) / sin(θ)
//        scaleTo   = sin(t·θ)     / sin(θ)
//        result = scaleFrom·q0 + scaleTo·q1adj
// ---------------------------------------------------------------------------

shared::Quaternion slerp(const shared::Quaternion& q0,
                         const shared::Quaternion& q1,
                         double                    t)
{
    assert(t >= 0.0 && t <= 1.0);

    // Short-arc correction: ensure we rotate the "short way round"
    const double rawDot = q0.w*q1.w + q0.x*q1.x + q0.y*q1.y + q0.z*q1.z;
    const bool flip = rawDot < 0.0;
    const shared::Quaternion q1adj = flip ? shared::Quaternion{-q1.w, -q1.x, -q1.y, -q1.z} : q1;
    // Clamp numerical noise
    const double dot = std::clamp(flip ? -rawDot : rawDot, 0.0, 1.0);

    // Threshold below which sin(θ) ≈ 0 and the slerp formula would divide by zero
    constexpr double kSlerpThreshold = 0.9995;
    // Smallest nlerp result length considered non-degenerate (handles near-zero quaternions)
    constexpr double kMinQuatLength   = 1e-15;

    if (dot > kSlerpThreshold)
    {
        // Quaternions nearly parallel — fall back to lerp + normalize
        const double rw  = q0.w + t * (q1adj.w - q0.w);
        const double rx  = q0.x + t * (q1adj.x - q0.x);
        const double ry  = q0.y + t * (q1adj.y - q0.y);
        const double rz  = q0.z + t * (q1adj.z - q0.z);
        const double len = std::sqrt(rw*rw + rx*rx + ry*ry + rz*rz);
        if (len < kMinQuatLength) return q0;
        return {rw/len, rx/len, ry/len, rz/len};
    }

    // Standard slerp
    const double theta0    = std::acos(dot);                    // angle between q0 and q1adj
    const double sinTheta0 = std::sin(theta0);
    const double scaleFrom = std::sin((1.0 - t) * theta0) / sinTheta0;
    const double scaleTo   = std::sin(t           * theta0) / sinTheta0;

    return {
        scaleFrom * q0.w + scaleTo * q1adj.w,
        scaleFrom * q0.x + scaleTo * q1adj.x,
        scaleFrom * q0.y + scaleTo * q1adj.y,
        scaleFrom * q0.z + scaleTo * q1adj.z,
    };
}

// ---------------------------------------------------------------------------
// World-state interpolation
// ---------------------------------------------------------------------------

InterpolatedWorldState interpolateWorldState(const server::WorldSnapshot& s0,
                                             const server::WorldSnapshot& s1,
                                             double                       renderTime)
{
    const double dt    = s1.elapsedSeconds - s0.elapsedSeconds;
    const double alpha = std::clamp(
        (dt > 0.0) ? (renderTime - s0.elapsedSeconds) / dt : 0.0,
        0.0, 1.0);

    InterpolatedWorldState result;
    result.renderTime = renderTime;

    // ---- Massive bodies ----
    for (const auto& body1 : s1.massiveBodies)
    {
        const auto it = std::find_if(s0.massiveBodies.begin(), s0.massiveBodies.end(),
            [id = body1.netId](const auto& b) { return b.netId == id; });

        if (it != s0.massiveBodies.end())
        {
            result.massiveBodies.push_back({
                .netId        = body1.netId,
                .position     = lerp(it->position,     body1.position,     alpha),
                .velocity     = lerp(it->velocity,     body1.velocity,     alpha),
                .radiusMeters = lerp(it->radiusMeters, body1.radiusMeters, alpha),
            });
        }
        else
        {
            // Spawned mid-interval — appear at s1 state
            result.massiveBodies.push_back({body1.netId, body1.position, body1.velocity, body1.radiusMeters});
        }
    }
    // Bodies only in s0 are dropped (despawned)

    // ---- Ships ----
    for (const auto& ship1 : s1.ships)
    {
        const auto it = std::find_if(s0.ships.begin(), s0.ships.end(),
            [id = ship1.netId](const auto& s) { return s.netId == id; });

        if (it != s0.ships.end())
        {
            result.ships.push_back({
                .netId        = ship1.netId,
                .position     = lerp(it->position,     ship1.position,     alpha),
                .velocity     = lerp(it->velocity,     ship1.velocity,     alpha),
                .orientation  = slerp(it->orientation, ship1.orientation,  alpha),
                .radiusMeters = lerp(it->radiusMeters, ship1.radiusMeters, alpha),
                .throttle     = lerp(it->throttle,     ship1.throttle,     alpha),
            });
        }
        else
        {
            result.ships.push_back({
                ship1.netId, ship1.position, ship1.velocity,
                ship1.orientation, ship1.radiusMeters, ship1.throttle
            });
        }
    }

    // ---- Projectiles ----
    for (const auto& proj1 : s1.projectiles)
    {
        const auto it = std::find_if(s0.projectiles.begin(), s0.projectiles.end(),
            [id = proj1.netId](const auto& p) { return p.netId == id; });

        if (it != s0.projectiles.end())
        {
            result.projectiles.push_back({
                .netId        = proj1.netId,
                .position     = lerp(it->position,     proj1.position,     alpha),
                .velocity     = lerp(it->velocity,     proj1.velocity,     alpha),
                .radiusMeters = lerp(it->radiusMeters, proj1.radiusMeters, alpha),
                .ownerNetId   = proj1.ownerNetId,
            });
        }
        else
        {
            result.projectiles.push_back({
                proj1.netId, proj1.position, proj1.velocity, proj1.radiusMeters, proj1.ownerNetId
            });
        }
    }

    return result;
}

InterpolatedWorldState fromSnapshot(const server::WorldSnapshot& snap,
                                    double                       renderTime)
{
    InterpolatedWorldState result;
    result.renderTime = renderTime;

    for (const auto& body : snap.massiveBodies)
        result.massiveBodies.push_back({body.netId, body.position, body.velocity, body.radiusMeters});

    for (const auto& ship : snap.ships)
        result.ships.push_back({ship.netId, ship.position, ship.velocity, ship.orientation, ship.radiusMeters, ship.throttle});

    for (const auto& proj : snap.projectiles)
        result.projectiles.push_back({proj.netId, proj.position, proj.velocity, proj.radiusMeters, proj.ownerNetId});

    return result;
}

} // namespace spaceship::client::interpolator

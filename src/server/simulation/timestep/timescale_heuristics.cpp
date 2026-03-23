#include "server/simulation/timestep/timescale_heuristics.hpp"
#include "server/simulation/simulation_math.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>

namespace spaceship::server
{

double computeOrbitalTimescale(
    const shared::Vec3& position,
    const shared::Vec3& velocity,
    const shared::Vec3& nearestBodyPosition,
    const TimestepLadderConfig& cfg) noexcept
{
    const shared::Vec3 rRel = subtract(position, nearestBodyPosition);
    const double rLen = std::max(length(rRel), cfg.r_eps);
    const double vLen = std::max(length(velocity), cfg.v_eps);
    const double nEst = vLen / rLen;   // approximate angular rate (rad/s); vLen >= v_eps > 0
    return cfg.alpha_orbit / nEst;
}

double computeAccelerationTimescale(
    const shared::Vec3& relativePosition,
    const shared::Vec3& acceleration,
    const TimestepLadderConfig& cfg) noexcept
{
    const double rLen = std::max(length(relativePosition), cfg.r_eps);
    const double aLen = std::max(length(acceleration), cfg.a_eps);
    return cfg.alpha_acc * std::sqrt(rLen / aLen);
}

double computeJerkTimescale(
    const shared::Vec3& acceleration,
    const shared::Vec3& previousAcceleration,
    double              dtPrev,
    const TimestepLadderConfig& cfg) noexcept
{
    if (dtPrev <= 0.0)
        return cfg.dt_max;  // no history — do not refine
    const shared::Vec3 jVec = scale(subtract(acceleration, previousAcceleration), 1.0 / dtPrev);
    const double jLen = std::max(length(jVec), cfg.j_eps);
    const double aLen = std::max(length(acceleration), cfg.a_eps);
    return cfg.alpha_jerk * aLen / jLen;
}

double computeCloseApproachTimescale(
    const shared::Vec3&               position,
    const shared::Vec3&               velocity,
    std::span<const MassiveBodyState> bodies,
    const TimestepLadderConfig&       cfg) noexcept
{
    // Start unclamped — clamping to [dt_min, dt_max] is done by computeTargetTimestep.
    double minDt = std::numeric_limits<double>::max();
    for (const auto& body : bodies)
    {
        const shared::Vec3 rRel = subtract(position, body.transform.position);
        const shared::Vec3 vRel = subtract(velocity, body.velocity.linear);
        const double dist      = std::max(length(rRel), cfg.r_eps);
        const double relSpeed  = std::max(length(vRel), cfg.v_eps);
        const double dt        = cfg.alpha_close * dist / relSpeed;
        minDt = std::min(minDt, dt);
    }
    return minDt;
}

double computeTargetTimestep(
    const shared::Vec3&               position,
    const shared::Vec3&               velocity,
    const shared::Vec3&               acceleration,
    const shared::Vec3&               previousAcceleration,
    double                            dtPrev,
    std::span<const MassiveBodyState> bodies,
    const TimestepLadderConfig&       cfg) noexcept
{
    // Find the nearest body for orbital and acceleration heuristics.
    const MassiveBodyState* nearest = nullptr;
    double minDist2 = std::numeric_limits<double>::max();
    for (const auto& body : bodies)
    {
        const double d2 = lengthSquared(subtract(position, body.transform.position));
        if (d2 < minDist2)
        {
            minDist2 = d2;
            nearest  = &body;
        }
    }

    const shared::Vec3 rRelNearest = nearest
        ? subtract(position, nearest->transform.position)
        : shared::Vec3{};

    const double dtOrbit = computeOrbitalTimescale(
        position, velocity,
        nearest ? nearest->transform.position : shared::Vec3{},
        cfg);

    const double dtAcc = computeAccelerationTimescale(rRelNearest, acceleration, cfg);

    const double dtJerk = computeJerkTimescale(acceleration, previousAcceleration, dtPrev, cfg);

    const double dtClose = bodies.empty()
        ? std::numeric_limits<double>::max()
        : computeCloseApproachTimescale(position, velocity, bodies, cfg);

    const double raw = std::min({dtOrbit, dtAcc, dtJerk, dtClose, cfg.dt_max});
    return std::clamp(raw, cfg.dt_min(), cfg.dt_max);
}

} // namespace spaceship::server

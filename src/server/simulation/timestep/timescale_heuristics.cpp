#include "server/simulation/timestep/timescale_heuristics.hpp"
#include "server/simulation/simulation_math.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>

namespace spaceship::server
{

// Orbital timescale: inversely proportional to estimated angular rate.
//   dt_orbit = alpha_orbit / (|v| / |r|)
// Captures how quickly the entity sweeps through its orbit — tighter orbits
// (higher angular rate) need smaller timesteps.
double computeOrbitalTimescale(
    const shared::Vec3& position,
    const shared::Vec3& velocity,
    const shared::Vec3& nearestBodyPosition,
    const TimestepLadderConfig& cfg) noexcept
{
    const shared::Vec3 relPosition = subtract(position, nearestBodyPosition);
    const double distanceFromBody  = std::max(length(relPosition), cfg.r_eps);
    const double speed             = std::max(length(velocity), cfg.v_eps);
    const double angularRateEstimate = speed / distanceFromBody;  // approximate rad/s
    return cfg.alpha_orbit / angularRateEstimate;
}

// Acceleration timescale: how long before acceleration changes position significantly.
//   dt_acc = alpha_acc * sqrt(|r| / |a|)
// Derived from dimensional analysis: position error ~ 0.5 * a * dt^2, so
// dt ~ sqrt(r / a) gives a step where the acceleration-induced displacement
// is a controlled fraction of the orbital radius.
double computeAccelerationTimescale(
    const shared::Vec3& relativePosition,
    const shared::Vec3& acceleration,
    const TimestepLadderConfig& cfg) noexcept
{
    const double distanceFromBody  = std::max(length(relativePosition), cfg.r_eps);
    const double accelerationMag   = std::max(length(acceleration), cfg.a_eps);
    return cfg.alpha_acc * std::sqrt(distanceFromBody / accelerationMag);
}

// Jerk timescale: how quickly acceleration is changing (rate of change = jerk).
//   jerk = (a_current - a_previous) / dt_prev
//   dt_jerk = alpha_jerk * |a| / |jerk|
// When jerk is high (acceleration changing rapidly), smaller steps are needed
// to track the evolving force field accurately.
double computeJerkTimescale(
    const shared::Vec3& acceleration,
    const shared::Vec3& previousAcceleration,
    double              dtPrev,
    const TimestepLadderConfig& cfg) noexcept
{
    if (dtPrev <= 0.0)
        return cfg.dt_max;  // No history — do not refine

    const shared::Vec3 jerkVector = scale(subtract(acceleration, previousAcceleration), 1.0 / dtPrev);
    const double jerkMag         = std::max(length(jerkVector), cfg.j_eps);
    const double accelerationMag = std::max(length(acceleration), cfg.a_eps);
    return cfg.alpha_jerk * accelerationMag / jerkMag;
}

// Close-approach timescale: time for the entity to traverse its current distance
// to the nearest body at relative speed.
//   dt_close = alpha_close * distance / relativeSpeed
// Prevents tunneling through or past a massive body between timesteps.
double computeCloseApproachTimescale(
    const shared::Vec3&               position,
    const shared::Vec3&               velocity,
    std::span<const MassiveBodyState> bodies,
    const TimestepLadderConfig&       cfg) noexcept
{
    double minimumTimescale = std::numeric_limits<double>::max();
    for (const auto& body : bodies)
    {
        const shared::Vec3 relPosition    = subtract(position, body.transform.position);
        const shared::Vec3 relVelocity    = subtract(velocity, body.velocity.linear);
        const double distance             = std::max(length(relPosition), cfg.r_eps);
        const double relativeSpeed        = std::max(length(relVelocity), cfg.v_eps);
        const double closeApproachTimestep = cfg.alpha_close * distance / relativeSpeed;
        minimumTimescale = std::min(minimumTimescale, closeApproachTimestep);
    }
    return minimumTimescale;
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
    // Find the nearest massive body for orbital and acceleration heuristics.
    const MassiveBodyState* nearest = nullptr;
    double minDistanceSquared = std::numeric_limits<double>::max();
    for (const auto& body : bodies)
    {
        const double distSq = lengthSquared(subtract(position, body.transform.position));
        if (distSq < minDistanceSquared)
        {
            minDistanceSquared = distSq;
            nearest = &body;
        }
    }

    const shared::Vec3 relPositionNearest = nearest
        ? subtract(position, nearest->transform.position)
        : shared::Vec3{};

    // Compute all four heuristic timescales and take the minimum.
    const double dtOrbit = computeOrbitalTimescale(
        position, velocity,
        nearest ? nearest->transform.position : shared::Vec3{},
        cfg);

    const double dtAccel = computeAccelerationTimescale(relPositionNearest, acceleration, cfg);

    const double dtJerk = computeJerkTimescale(acceleration, previousAcceleration, dtPrev, cfg);

    const double dtClose = bodies.empty()
        ? std::numeric_limits<double>::max()
        : computeCloseApproachTimescale(position, velocity, bodies, cfg);

    const double rawTimestep = std::min({dtOrbit, dtAccel, dtJerk, dtClose, cfg.dt_max});
    return std::clamp(rawTimestep, cfg.dt_min(), cfg.dt_max);
}

} // namespace spaceship::server

#include "server/collision/sweep_math.hpp"
#include "server/simulation/simulation_math.hpp"

#include <cmath>
#include <limits>

namespace spaceship::server
{

std::optional<double> sphereSweepTOI(
    const shared::Vec3& posA0,
    const shared::Vec3& velA,
    const shared::Vec3& posB0,
    const shared::Vec3& velB,
    double rA,
    double rB,
    double dt) noexcept
{
    // Solve for the time of impact (TOI) when two moving spheres first touch.
    //
    // Problem: find smallest t in [0, dt] such that |posA(t) - posB(t)| = rA + rB.
    //
    // With linear motion posA(t) = posA0 + velA*t, posB(t) = posB0 + velB*t,
    // define relative quantities:
    //   relativePosition p = posA0 - posB0    (separation at t=0)
    //   relativeVelocity u = velA  - velB     (closing speed)
    //   combinedRadius   R = rA + rB
    //
    // Then |p + u*t|^2 = R^2 expands to the quadratic:
    //   a*t^2 + b*t + c = 0
    // where:
    //   a = dot(u, u)           = |u|^2       (relative speed squared)
    //   b = 2 * dot(p, u)                     (approach rate term)
    //   c = dot(p, p) - R^2     = |p|^2 - R^2 (initial separation minus combined radius)

    const shared::Vec3 relativePosition = subtract(posA0, posB0);
    const shared::Vec3 relativeVelocity = subtract(velA,  velB);

    const double combinedRadius        = rA + rB;
    const double combinedRadiusSquared = combinedRadius * combinedRadius;

    // c < 0 means spheres already overlap at interval start.
    const double separationTermC = dot(relativePosition, relativePosition) - combinedRadiusSquared;
    if (separationTermC <= 0.0)
    {
        return 0.0;  // Already overlapping → immediate contact
    }

    // a = |u|^2: relative speed squared. If near zero, objects are stationary
    // relative to each other and not overlapping → no hit possible.
    const double relativeSpeedSqA = dot(relativeVelocity, relativeVelocity);
    if (relativeSpeedSqA < std::numeric_limits<double>::epsilon())
    {
        return std::nullopt;
    }

    // b = 2 * dot(p, u): proportional to the rate of change of separation distance.
    // If b >= 0, the objects are moving apart (or tangent) → no future contact.
    const double approachRateB = 2.0 * dot(relativePosition, relativeVelocity);
    if (approachRateB >= 0.0)
    {
        return std::nullopt;
    }

    // discriminant = b^2 - 4ac
    // Negative discriminant means the trajectories never bring the spheres
    // within combined radius — they pass each other in 3D space.
    const double discriminant = approachRateB * approachRateB - 4.0 * relativeSpeedSqA * separationTermC;
    if (discriminant < 0.0)
    {
        return std::nullopt;
    }

    // Earliest root: t = (-b - sqrt(discriminant)) / (2a)
    // We want the first contact (smaller root). The larger root is when the
    // spheres would separate again after passing through each other.
    const double timeOfImpact = (-approachRateB - std::sqrt(discriminant)) / (2.0 * relativeSpeedSqA);

    if (timeOfImpact < 0.0 || timeOfImpact > dt)
    {
        return std::nullopt;  // Contact occurs outside the simulation interval
    }

    return timeOfImpact;
}

} // namespace spaceship::server

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
    // Relative position and velocity
    const shared::Vec3 p = subtract(posA0, posB0);  // A - B at t=0
    const shared::Vec3 u = subtract(velA,  velB);   // relative velocity

    const double R = rA + rB;   // combined radius
    const double RR = R * R;

    // c = |p|^2 - R^2  (negative means already overlapping)
    const double c = dot(p, p) - RR;
    if (c <= 0.0)
    {
        // Already overlapping at interval start
        return 0.0;
    }

    // a = |u|^2
    const double a = dot(u, u);
    if (a < std::numeric_limits<double>::epsilon())
    {
        // No relative motion and not overlapping → no hit
        return std::nullopt;
    }

    // b = 2 * dot(p, u)
    const double b = 2.0 * dot(p, u);

    // If b >= 0 spheres are separating (or stationary) → no hit
    if (b >= 0.0)
    {
        return std::nullopt;
    }

    // discriminant = b^2 - 4ac
    const double disc = b * b - 4.0 * a * c;
    if (disc < 0.0)
    {
        // Complex roots → no hit (spheres pass each other)
        return std::nullopt;
    }

    // Earliest root: t = (-b - sqrt(disc)) / (2a)
    const double t = (-b - std::sqrt(disc)) / (2.0 * a);

    if (t < 0.0 || t > dt)
    {
        return std::nullopt;
    }

    return t;
}

} // namespace spaceship::server

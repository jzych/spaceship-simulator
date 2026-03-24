#include "server/simulation/gravity/gravity_system.hpp"
#include "server/simulation/simulation_math.hpp"

namespace spaceship::server
{

shared::Vec3 computeGravitationalAcceleration(
    const shared::Vec3& position,
    std::span<const MassiveBodyState> massiveBodies)
{
    // Sum Newtonian gravitational acceleration from all massive bodies.
    // Each body j contributes:
    //   a_j = mu_j * (x_j - x) / |x_j - x|^3
    // where mu_j = G * M_j is the gravitational parameter.
    shared::Vec3 totalAcceleration {};

    for (const auto& body : massiveBodies)
    {
        const shared::Vec3 toBody = subtract(body.transform.position, position);
        const double distance = length(toBody);

        // Skip contributions closer than 1 m to avoid division-by-zero or
        // numerically explosive accelerations from degenerate positions.
        // Physically realistic minimum distances are many orders of magnitude larger.
        constexpr double kMinDistanceMeters = 1.0;
        if (distance < kMinDistanceMeters)
            continue;

        // Inverse-cube law: a = mu * r_hat / |r|^2 = mu * r / |r|^3
        const double inverseCubeDistance = body.definition.muMetersCubedPerSecondSquared
                                           / (distance * distance * distance);
        totalAcceleration = add(totalAcceleration, scale(toBody, inverseCubeDistance));
    }

    return totalAcceleration;
}

void GravitySystem::update(
    std::span<const MassiveBodyState> massiveBodies,
    std::span<ShipState> ships,
    std::span<ProjectileState> projectiles) const
{
    for (auto& ship : ships)
        ship.acceleration = computeGravitationalAcceleration(ship.transform.position, massiveBodies);

    for (auto& projectile : projectiles)
        projectile.acceleration =
            computeGravitationalAcceleration(projectile.transform.position, massiveBodies);
}

} // namespace spaceship::server

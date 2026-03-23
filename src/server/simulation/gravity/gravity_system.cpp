#include "server/simulation/gravity/gravity_system.hpp"
#include "server/simulation/simulation_math.hpp"

namespace spaceship::server
{

shared::Vec3 computeGravitationalAcceleration(
    const shared::Vec3& position,
    std::span<const MassiveBodyState> massiveBodies)
{
    shared::Vec3 total {};

    for (const auto& body : massiveBodies)
    {
        const shared::Vec3 r = subtract(body.transform.position, position);
        const double d = length(r);

        // Skip contributions closer than 1 m to avoid division-by-zero or
        // numerically explosive accelerations from degenerate positions.
        // Physically realistic minimum distances are many orders of magnitude larger.
        constexpr double kMinDistanceMeters = 1.0;
        if (d < kMinDistanceMeters)
            continue;

        // a_j = μ_j * (x_j - x) / |x_j - x|³
        total = add(total, scale(r, body.definition.muMetersCubedPerSecondSquared / (d * d * d)));
    }

    return total;
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

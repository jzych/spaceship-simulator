#include "server/gravity_system.hpp"

#include "server/gravity_field.hpp"
#include "server/simulation_math.hpp"

namespace spaceship::server
{

void GravitySystem::update(
    std::span<MassiveBodyState> massiveBodies,
    std::span<ShipState> ships,
    std::span<ProjectileState> projectiles,
    double elapsedSeconds) const
{
    massiveBodyMotionSystem_.update(massiveBodies, elapsedSeconds);

    for (ShipState& ship : ships)
    {
        ship.acceleration = add(
            ship.thrustAcceleration,
            computeGravityAcceleration(ship.transform.position, massiveBodies));
    }

    for (ProjectileState& projectile : projectiles)
    {
        projectile.acceleration = computeGravityAcceleration(projectile.transform.position, massiveBodies);
    }
}

} // namespace spaceship::server

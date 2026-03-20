#include "server/gravity_system.hpp"

#include "server/gravity_field.hpp"
#include <vector>

namespace spaceship::server
{

void GravitySystem::update(std::span<MassiveBodyState> massiveBodies, double elapsedSeconds) const
{
    massiveBodyMotionSystem_.update(massiveBodies, elapsedSeconds);
}

void GravitySystem::seedAccelerations(
    std::span<const MassiveBodyState> massiveBodies,
    std::span<ShipState> ships,
    std::span<ProjectileState> projectiles) const
{
    for (ShipState& ship : ships)
    {
        ship.gravityAcceleration = computeGravityAcceleration(ship.transform.position, massiveBodies);
        ship.acceleration = add(ship.gravityAcceleration, ship.thrustAcceleration);
    }

    for (ProjectileState& projectile : projectiles)
    {
        projectile.acceleration = computeGravityAcceleration(projectile.transform.position, massiveBodies);
    }
}

shared::Vec3 GravitySystem::computeGravityAcceleration(
    const shared::Vec3& position,
    std::span<const MassiveBodyState> massiveBodies) const
{
    return spaceship::server::computeGravityAcceleration(position, massiveBodies);
}

shared::Vec3 GravitySystem::computeGravityAccelerationAtTime(
    const shared::Vec3& position,
    std::span<const MassiveBodyState> massiveBodies,
    double elapsedSeconds) const
{
    std::vector<MassiveBodyState> projectedBodies {massiveBodies.begin(), massiveBodies.end()};
    massiveBodyMotionSystem_.update(projectedBodies, elapsedSeconds);
    return computeGravityAcceleration(position, projectedBodies);
}

} // namespace spaceship::server

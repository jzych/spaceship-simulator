#include "server/integration_system.hpp"
#include "server/simulation_math.hpp"

namespace spaceship::server
{

namespace
{

template <typename Entity>
void integratePosition(Entity& entity, double dt)
{
    entity.previousAcceleration = entity.acceleration;
    entity.transform.position = add(
        add(entity.transform.position, scale(entity.velocity.linear, dt)),
        scale(entity.acceleration, 0.5 * dt * dt));
}

template <typename Entity>
void integrateVelocity(Entity& entity, double dt)
{
    const shared::Vec3 avgAccel = scale(
        add(entity.previousAcceleration, entity.acceleration), 0.5);
    entity.velocity.linear = add(entity.velocity.linear, scale(avgAccel, dt));
}

} // namespace

void IntegrationSystem::integratePositions(
    std::span<ShipState> ships,
    std::span<ProjectileState> projectiles,
    const SimulationConfig& config) const
{
    for (auto& ship : ships)
        integratePosition(ship, config.fixedDeltaSeconds);
    for (auto& projectile : projectiles)
        integratePosition(projectile, config.fixedDeltaSeconds);
}

void IntegrationSystem::integrateVelocities(
    std::span<ShipState> ships,
    std::span<ProjectileState> projectiles,
    const SimulationConfig& config) const
{
    for (auto& ship : ships)
        integrateVelocity(ship, config.fixedDeltaSeconds);
    for (auto& projectile : projectiles)
        integrateVelocity(projectile, config.fixedDeltaSeconds);
}

void IntegrationSystem::decrementTtl(
    std::span<ProjectileState> projectiles,
    const SimulationConfig& config) const
{
    for (auto& projectile : projectiles)
        projectile.params.ttlSeconds -= config.fixedDeltaSeconds;
}

} // namespace spaceship::server

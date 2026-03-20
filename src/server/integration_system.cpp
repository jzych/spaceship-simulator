#include "server/integration_system.hpp"

#include "server/gravity_field.hpp"
#include "server/simulation_math.hpp"

namespace spaceship::server
{

namespace
{

constexpr double kHalf = 0.5;

void integrateShip(
    ShipState& ship,
    std::span<const MassiveBodyState> massiveBodies,
    const SimulationConfig& config)
{
    const double fixedDeltaSeconds = config.fixedDeltaSeconds;
    const double halfDeltaSquared = kHalf * fixedDeltaSeconds * fixedDeltaSeconds;
    const shared::Vec3 currentAcceleration = ship.acceleration;

    ship.transform.position = add(
        ship.transform.position,
        add(
            scale(ship.velocity.linear, fixedDeltaSeconds),
            scale(currentAcceleration, halfDeltaSquared)));

    const shared::Vec3 nextAcceleration = add(
        ship.thrustAcceleration,
        computeGravityAcceleration(ship.transform.position, massiveBodies));
    ship.velocity.linear = add(
        ship.velocity.linear,
        scale(add(currentAcceleration, nextAcceleration), kHalf * fixedDeltaSeconds));
    ship.acceleration = nextAcceleration;
}

void integrateProjectile(
    ProjectileState& projectile,
    std::span<const MassiveBodyState> massiveBodies,
    const SimulationConfig& config)
{
    const double fixedDeltaSeconds = config.fixedDeltaSeconds;
    const double halfDeltaSquared = kHalf * fixedDeltaSeconds * fixedDeltaSeconds;
    const shared::Vec3 currentAcceleration = projectile.acceleration;

    projectile.transform.position = add(
        projectile.transform.position,
        add(
            scale(projectile.velocity.linear, fixedDeltaSeconds),
            scale(currentAcceleration, halfDeltaSquared)));

    const shared::Vec3 nextAcceleration = computeGravityAcceleration(projectile.transform.position, massiveBodies);
    projectile.velocity.linear = add(
        projectile.velocity.linear,
        scale(add(currentAcceleration, nextAcceleration), kHalf * fixedDeltaSeconds));
    projectile.acceleration = nextAcceleration;
    projectile.params.ttlSeconds -= fixedDeltaSeconds;
}

} // namespace

void IntegrationSystem::update(
    std::span<const MassiveBodyState> massiveBodies,
    std::span<ShipState> ships,
    std::span<ProjectileState> projectiles,
    const SimulationConfig& config) const
{
    for (ShipState& ship : ships)
    {
        integrateShip(ship, massiveBodies, config);
    }

    for (ProjectileState& projectile : projectiles)
    {
        integrateProjectile(projectile, massiveBodies, config);
    }
}

} // namespace spaceship::server

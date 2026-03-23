#include "server/simulation/integration_system.hpp"
#include "server/simulation/simulation_math.hpp"

#include <concepts>

namespace spaceship::server
{

namespace
{

// Structural requirements for entities that can be Velocity Verlet integrated.
template <typename T>
concept IntegrableEntity = requires(T& e) {
    e.transform.position;
    e.velocity.linear;
    e.acceleration;
    e.thrustAcceleration;
    e.previousAcceleration;
};

// Full acceleration = gravity component + thrust component.
// Ships carry both fields; projectiles have thrustAcceleration == {0,0,0}.
template <IntegrableEntity Entity>
shared::Vec3 fullAcceleration(const Entity& entity)
{
    return add(entity.acceleration, entity.thrustAcceleration);
}

template <IntegrableEntity Entity>
void integratePosition(Entity& entity, double dt)
{
    const shared::Vec3 a = fullAcceleration(entity);
    entity.previousAcceleration = a;
    entity.transform.position = add(
        add(entity.transform.position, scale(entity.velocity.linear, dt)),
        scale(a, 0.5 * dt * dt));
}

template <IntegrableEntity Entity>
void integrateVelocity(Entity& entity, double dt)
{
    // previousAcceleration holds a_n (saved during integratePositions).
    // acceleration now holds gravity(x_{n+1}); thrustAcceleration is unchanged.
    const shared::Vec3 avgAccel = scale(
        add(entity.previousAcceleration, fullAcceleration(entity)), 0.5);
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

} // namespace spaceship::server

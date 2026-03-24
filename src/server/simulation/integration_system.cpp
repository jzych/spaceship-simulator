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

// Velocity Verlet position update (first half of the leapfrog step):
//   x_{n+1} = x_n + v_n * dt + 0.5 * a_n * dt^2
// Saves a_n into previousAcceleration for the velocity half-step.
template <IntegrableEntity Entity>
void integratePosition(Entity& entity, double dt)
{
    const shared::Vec3 currentAccel = fullAcceleration(entity);
    entity.previousAcceleration = currentAccel;

    const shared::Vec3 velocityDisplacement = scale(entity.velocity.linear, dt);
    const shared::Vec3 accelDisplacement    = scale(currentAccel, 0.5 * dt * dt);
    entity.transform.position = add(add(entity.transform.position, velocityDisplacement), accelDisplacement);
}

// Velocity Verlet velocity update (second half of the leapfrog step):
//   v_{n+1} = v_n + 0.5 * (a_n + a_{n+1}) * dt
// previousAcceleration holds a_n (saved during integratePosition).
// acceleration now holds gravity(x_{n+1}); thrustAcceleration is unchanged.
template <IntegrableEntity Entity>
void integrateVelocity(Entity& entity, double dt)
{
    const shared::Vec3 averageAcceleration = scale(
        add(entity.previousAcceleration, fullAcceleration(entity)), 0.5);
    entity.velocity.linear = add(entity.velocity.linear, scale(averageAcceleration, dt));
}

} // namespace

void IntegrationSystem::integratePositions(
    std::span<ShipState> ships,
    std::span<ProjectileState> projectiles,
    const SimulationConfig& config) const
{
    integratePositions(ships, projectiles, config.fixedDeltaSeconds);
}

void IntegrationSystem::integrateVelocities(
    std::span<ShipState> ships,
    std::span<ProjectileState> projectiles,
    const SimulationConfig& config) const
{
    integrateVelocities(ships, projectiles, config.fixedDeltaSeconds);
}

void IntegrationSystem::integratePositions(
    std::span<ShipState> ships,
    std::span<ProjectileState> projectiles,
    double dt) const
{
    for (auto& ship : ships)
        integratePosition(ship, dt);
    for (auto& projectile : projectiles)
        integratePosition(projectile, dt);
}

void IntegrationSystem::integrateVelocities(
    std::span<ShipState> ships,
    std::span<ProjectileState> projectiles,
    double dt) const
{
    for (auto& ship : ships)
        integrateVelocity(ship, dt);
    for (auto& projectile : projectiles)
        integrateVelocity(projectile, dt);
}

} // namespace spaceship::server

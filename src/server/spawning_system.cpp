#include "server/spawning_system.hpp"
#include "server/gravity_system.hpp"
#include "server/simulation_math.hpp"

namespace spaceship::server
{

namespace
{

shared::MassProperties makeMassProperties(double massKg)
{
    return {massKg, massKg > 0.0 ? 1.0 / massKg : 0.0};
}

} // namespace

shared::NetId SpawningSystem::spawnShip(
    std::vector<ShipState>& ships,
    const ShipSpawnRequest& request,
    const SimulationConfig& config,
    std::span<const MassiveBodyState> massiveBodies)
{
    const shared::NetId shipNetId = nextShipNetId_++;
    const shared::Vec3 accel = request.initialAcceleration.has_value()
        ? *request.initialAcceleration
        : computeGravitationalAcceleration(request.transform.position, massiveBodies);

    ships.push_back(ShipState {
        shipNetId,
        request.transform,
        request.velocity,
        makeMassProperties(config.shipDefaultMassKg),
        {config.shipDefaultRadiusMeters},
        {0.0, request.transform.orientation, false},
        accel,  // acceleration (gravity, carry-forward)
        {},     // thrustAcceleration (zero until first ShipControlSystem update)
        {},     // previousAcceleration
    });

    return shipNetId;
}

shared::NetId SpawningSystem::spawnProjectile(
    std::vector<ProjectileState>& projectiles,
    const ShipState& ship,
    const SimulationConfig& config,
    std::span<const MassiveBodyState> massiveBodies)
{
    const shared::NetId projectileNetId = nextProjectileNetId_++;
    const shared::Vec3 muzzleVelocity =
        scale(forwardDirection(ship.transform.orientation), config.projectileMuzzleSpeedMetersPerSecond);
    const shared::Vec3 accel =
        computeGravitationalAcceleration(ship.transform.position, massiveBodies);

    projectiles.push_back(ProjectileState {
        projectileNetId,
        {ship.transform.position, ship.transform.orientation},
        {add(ship.velocity.linear, muzzleVelocity)},
        makeMassProperties(config.projectileMassKg),
        {config.projectileRadiusMeters},
        {config.projectileDefaultTtlSeconds, ship.netId},
        accel,  // acceleration (gravity, carry-forward)
        {},     // thrustAcceleration (always zero for projectiles)
        {},     // previousAcceleration
    });

    return projectileNetId;
}

void SpawningSystem::update(
    std::span<ShipState> ships,
    std::vector<ProjectileState>& projectiles,
    std::span<const MassiveBodyState> massiveBodies,
    const SimulationConfig& config)
{
    for (auto& ship : ships)
    {
        if (ship.control.fire)
        {
            spawnProjectile(projectiles, ship, config, massiveBodies);
            ship.control.fire = false;
        }
    }
}

} // namespace spaceship::server

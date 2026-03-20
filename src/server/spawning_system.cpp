#include "server/spawning_system.hpp"

#include <cmath>

namespace spaceship::server
{

namespace
{

shared::Vec3 add(const shared::Vec3& lhs, const shared::Vec3& rhs)
{
    return {lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z};
}

shared::Vec3 scale(const shared::Vec3& value, double factor)
{
    return {value.x * factor, value.y * factor, value.z * factor};
}

shared::MassProperties makeMassProperties(double massKg)
{
    return {massKg, massKg > 0.0 ? 1.0 / massKg : 0.0};
}

shared::Quaternion conjugate(const shared::Quaternion& quaternion)
{
    return {quaternion.w, -quaternion.x, -quaternion.y, -quaternion.z};
}

shared::Quaternion multiply(const shared::Quaternion& lhs, const shared::Quaternion& rhs)
{
    return {
        lhs.w * rhs.w - lhs.x * rhs.x - lhs.y * rhs.y - lhs.z * rhs.z,
        lhs.w * rhs.x + lhs.x * rhs.w + lhs.y * rhs.z - lhs.z * rhs.y,
        lhs.w * rhs.y - lhs.x * rhs.z + lhs.y * rhs.w + lhs.z * rhs.x,
        lhs.w * rhs.z + lhs.x * rhs.y - lhs.y * rhs.x + lhs.z * rhs.w,
    };
}

shared::Vec3 rotateVector(const shared::Quaternion& rotation, const shared::Vec3& vector)
{
    const shared::Quaternion pureVector {0.0, vector.x, vector.y, vector.z};
    const shared::Quaternion rotated = multiply(multiply(rotation, pureVector), conjugate(rotation));
    return {rotated.x, rotated.y, rotated.z};
}

shared::Vec3 forwardDirection(const shared::Quaternion& orientation)
{
    const shared::Vec3 localForward {1.0, 0.0, 0.0};
    const shared::Vec3 worldForward = rotateVector(orientation, localForward);
    const double length =
        std::sqrt(worldForward.x * worldForward.x + worldForward.y * worldForward.y + worldForward.z * worldForward.z);

    if (length == 0.0)
    {
        return localForward;
    }

    return {worldForward.x / length, worldForward.y / length, worldForward.z / length};
}

} // namespace

ShipState& SpawningSystem::spawnShip(
    std::vector<ShipState>& ships,
    const ShipSpawnRequest& request,
    const SimulationConfig& config) const
{
    ships.push_back(ShipState {
        request.netId,
        request.transform,
        request.velocity,
        makeMassProperties(config.shipDefaultMassKg),
        {config.shipDefaultRadiusMeters},
        {0.0, request.transform.orientation, false},
    });

    return ships.back();
}

ProjectileState& SpawningSystem::spawnProjectile(
    std::vector<ProjectileState>& projectiles,
    const ShipState& ship,
    const SimulationConfig& config)
{
    const shared::Vec3 muzzleVelocity =
        scale(forwardDirection(ship.transform.orientation), config.projectileMuzzleSpeedMetersPerSecond);

    projectiles.push_back(ProjectileState {
        nextProjectileNetId_++,
        {ship.transform.position, ship.transform.orientation},
        {add(ship.velocity.linear, muzzleVelocity)},
        makeMassProperties(config.projectileMassKg),
        {config.projectileRadiusMeters},
        {config.projectileDefaultTtlSeconds, ship.netId},
    });

    return projectiles.back();
}

void SpawningSystem::update(
    std::span<ShipState> ships,
    std::vector<ProjectileState>& projectiles,
    const SimulationConfig& config)
{
    for (auto& ship : ships)
    {
        if (ship.control.fire)
        {
            spawnProjectile(projectiles, ship, config);
            ship.control.fire = false;
        }
    }
}

} // namespace spaceship::server

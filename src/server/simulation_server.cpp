#include "server/simulation_server.hpp"

#include "server/bootstrap.hpp"

#include <algorithm>
#include <utility>

namespace spaceship::server
{

SimulationServer::SimulationServer(const SimulationConfig& config)
    : config_(config), world_(createInitialWorld(BootstrapWorld::Default))
{
}

SimulationServer::SimulationServer(const SimulationConfig& config, BootstrapWorld worldPreset)
    : config_(config), world_(createInitialWorld(worldPreset))
{
}

SimulationServer::SimulationServer(SimulationWorld initialWorld, const SimulationConfig& config)
    : config_(config), world_(std::move(initialWorld))
{
}

shared::NetId SimulationServer::spawnShip(const ShipSpawnRequest& request)
{
    const shared::NetId shipNetId = spawningSystem_.spawnShip(world_.ships, request, config_);
    ShipState& ship = world_.ships.back();
    ship.gravityAcceleration = gravitySystem_.computeGravityAcceleration(ship.transform.position, world_.massiveBodies);
    ship.acceleration = ship.gravityAcceleration;
    return shipNetId;
}

bool SimulationServer::updateShipControl(shared::NetId shipNetId, const shared::ShipControl& control)
{
    const auto ship = findShip(shipNetId);
    if (!ship.has_value())
    {
        return false;
    }

    ship->get().control = control;
    return true;
}

void SimulationServer::tick()
{
    const double nextElapsedSeconds = static_cast<double>(tickCount_ + 1U) * config_.fixedDeltaSeconds;
    const std::size_t projectileCountBeforeSpawn = world_.projectiles.size();

    shipControlSystem_.update(world_.ships, config_);
    if (!accelerationsSeeded_)
    {
        gravitySystem_.seedAccelerations(world_.massiveBodies, world_.ships, world_.projectiles);
        accelerationsSeeded_ = true;
    }
    integrationSystem_.update(
        world_.massiveBodies,
        world_.ships,
        world_.projectiles,
        gravitySystem_,
        nextElapsedSeconds,
        config_);
    gravitySystem_.update(world_.massiveBodies, nextElapsedSeconds);
    spawningSystem_.update(world_.ships, world_.projectiles, config_);

    for (std::size_t projectileIndex = projectileCountBeforeSpawn; projectileIndex < world_.projectiles.size(); ++projectileIndex)
    {
        ProjectileState& projectile = world_.projectiles[projectileIndex];
        projectile.acceleration =
            gravitySystem_.computeGravityAcceleration(projectile.transform.position, world_.massiveBodies);
    }

    collisionSystem_.update(world_.projectiles);

    ++tickCount_;

    if (tickCount_ % config_.snapshotIntervalTicks == 0)
    {
        lastSnapshotSummary_ = snapshotSystem_.buildSnapshotSummary(world_);
    }
}

shared::Tick SimulationServer::tickCount() const
{
    return tickCount_;
}

const SimulationWorld& SimulationServer::world() const
{
    return world_;
}

const std::string& SimulationServer::lastSnapshotSummary() const
{
    return lastSnapshotSummary_;
}

std::optional<std::reference_wrapper<ShipState>> SimulationServer::findShip(shared::NetId shipNetId)
{
    const auto it = std::find_if(
        world_.ships.begin(),
        world_.ships.end(),
        [shipNetId](const ShipState& ship) { return ship.netId == shipNetId; });

    if (it == world_.ships.end())
    {
        return std::nullopt;
    }

    return std::ref(*it);
}

} // namespace spaceship::server

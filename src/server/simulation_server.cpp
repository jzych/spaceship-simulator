#include "server/simulation_server.hpp"

#include "server/bootstrap.hpp"

#include <algorithm>

namespace spaceship::server
{

SimulationServer::SimulationServer(const SimulationConfig& config)
    : config_(config), world_(createInitialWorld())
{
}

SimulationServer::SimulationServer(SimulationWorld world, const SimulationConfig& config)
    : config_(config), world_(std::move(world))
{
}

shared::NetId SimulationServer::spawnShip(const ShipSpawnRequest& request)
{
    return spawningSystem_.spawnShip(world_.ships, request, config_, world_.massiveBodies);
}

void SimulationServer::updateShipControl(shared::NetId shipNetId, const shared::ShipControl& control)
{
    const auto ship = findShip(shipNetId);
    if (ship.has_value())
        ship->get().control = control;
}

void SimulationServer::tick()
{
    massiveBodyMotionSystem_.update(world_.massiveBodies, elapsedSeconds_);
    elapsedSeconds_ += config_.fixedDeltaSeconds;

    spawningSystem_.update(world_.ships, world_.projectiles, world_.massiveBodies, config_);

    // ship.acceleration carries gravity(x_n) from the end of the previous tick.
    // ShipControlSystem writes fresh thrust into ship.thrustAcceleration.
    // integratePositions computes a_n = gravity(x_n) + thrust, saves it, advances x.
    shipControlSystem_.update(world_.ships, config_);
    integrationSystem_.integratePositions(world_.ships, world_.projectiles, config_);

    // One gravity call per tick: gravity(x_{n+1}) stored in ship.acceleration.
    // integrateVelocities computes a_{n+1} = gravity(x_{n+1}) + thrust (same tick).
    gravitySystem_.update(world_.massiveBodies, world_.ships, world_.projectiles);

    // Verlet phase 2: v_{n+1} = v_n + 0.5*(a_n + a_{n+1})*dt
    integrationSystem_.integrateVelocities(world_.ships, world_.projectiles, config_);

    integrationSystem_.decrementTtl(world_.projectiles, config_);
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

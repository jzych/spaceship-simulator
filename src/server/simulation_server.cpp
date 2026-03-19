#include "server/simulation_server.hpp"

#include "server/bootstrap.hpp"

namespace spaceship::server
{

SimulationServer::SimulationServer(SimulationConfig config)
    : config_(config), world_(createInitialWorld())
{
}

void SimulationServer::tick()
{
    spawningSystem_.update(world_.ships);
    gravitySystem_.update(world_.events);
    integrationSystem_.update(world_.projectiles, config_);
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

} // namespace spaceship::server

#pragma once

// Owns the authoritative world and runs the ordered server simulation tick.

#include "server/collision_system.hpp"
#include "server/gravity_system.hpp"
#include "server/integration_system.hpp"
#include "server/simulation_config.hpp"
#include "server/simulation_world.hpp"
#include "server/snapshot_system.hpp"
#include "server/spawning_system.hpp"

#include <string>

namespace spaceship::server
{

class SimulationServer
{
  public:
    explicit SimulationServer(SimulationConfig config = {});

    void tick();

    [[nodiscard]] shared::Tick tickCount() const;
    [[nodiscard]] const SimulationWorld& world() const;
    [[nodiscard]] const std::string& lastSnapshotSummary() const;

  private:
    SimulationConfig config_ {};
    SimulationWorld world_ {};
    shared::Tick tickCount_ {};
    std::string lastSnapshotSummary_ {};

    SpawningSystem spawningSystem_ {};
    GravitySystem gravitySystem_ {};
    IntegrationSystem integrationSystem_ {};
    CollisionSystem collisionSystem_ {};
    SnapshotSystem snapshotSystem_ {};
};

} // namespace spaceship::server

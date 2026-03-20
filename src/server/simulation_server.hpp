#pragma once

// Owns the authoritative world and runs the ordered server simulation tick.

#include "server/collision_system.hpp"
#include "server/gravity_system.hpp"
#include "server/integration_system.hpp"
#include "server/simulation_config.hpp"
#include "server/simulation_world.hpp"
#include "server/snapshot_system.hpp"
#include "server/spawning_system.hpp"

#include <functional>
#include <optional>
#include <string>

namespace spaceship::server
{

class SimulationServer
{
  public:
    explicit SimulationServer(const SimulationConfig& config = {});

    shared::NetId spawnShip(const ShipSpawnRequest& request);
    std::optional<shared::NetId> fireProjectile(shared::NetId shipNetId);
    void tick();

    [[nodiscard]] shared::Tick tickCount() const;
    [[nodiscard]] const SimulationWorld& world() const;
    [[nodiscard]] const std::string& lastSnapshotSummary() const;

  private:
    std::optional<std::reference_wrapper<ShipState>> findShip(shared::NetId shipNetId);

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

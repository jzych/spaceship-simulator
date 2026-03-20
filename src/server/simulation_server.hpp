#pragma once

// Owns the authoritative world and runs the ordered server simulation tick.

#include "server/collision_system.hpp"
#include "server/gravity_system.hpp"
#include "server/integration_system.hpp"
#include "server/ship_control_system.hpp"
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
    bool updateShipControl(shared::NetId shipNetId, const shared::ShipControl& control);
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
    ShipControlSystem shipControlSystem_ {};
    GravitySystem gravitySystem_ {};
    IntegrationSystem integrationSystem_ {};
    CollisionSystem collisionSystem_ {};
    SnapshotSystem snapshotSystem_ {};
};

} // namespace spaceship::server

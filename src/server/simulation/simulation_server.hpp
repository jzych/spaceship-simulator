#pragma once

// Owns the authoritative world and runs the ordered server simulation tick.

#include "server/collision/collision_system.hpp"
#include "server/simulation/gravity/gravity_system.hpp"
#include "server/simulation/gravity/orbit_cache_system.hpp"
#include "server/simulation/integration_system.hpp"
#include "server/simulation/gravity/massive_body_motion_system.hpp"
#include "server/control/ship_control_system.hpp"
#include "server/simulation/simulation_config.hpp"
#include "server/simulation/simulation_world.hpp"
#include "server/snapshot/snapshot_system.hpp"
#include "server/spawning/spawning_system.hpp"

#include <functional>
#include <optional>
#include <string>

namespace spaceship::server
{

class SimulationServer
{
  public:
    explicit SimulationServer(const SimulationConfig& config = {});
    SimulationServer(SimulationWorld world, const SimulationConfig& config = {});

    shared::NetId spawnShip(const ShipSpawnRequest& request);
    void updateShipControl(shared::NetId shipNetId, const shared::ShipControl& control);
    void tick();

    [[nodiscard]] shared::Tick tickCount() const;
    [[nodiscard]] const SimulationWorld& world() const;
    [[nodiscard]] const std::string& lastSnapshotSummary() const;

  private:
    std::optional<std::reference_wrapper<ShipState>> findShip(shared::NetId shipNetId);

    SimulationConfig config_ {};
    SimulationWorld world_ {};
    shared::Tick tickCount_ {};
    double elapsedSeconds_ {};
    std::string lastSnapshotSummary_ {};

    MassiveBodyMotionSystem massiveBodyMotionSystem_ {};
    SpawningSystem spawningSystem_ {};
    ShipControlSystem shipControlSystem_ {};
    GravitySystem gravitySystem_ {};
    IntegrationSystem integrationSystem_ {};
    OrbitCacheSystem orbitCacheSystem_ {};
    CollisionSystem collisionSystem_ {};
    SnapshotSystem snapshotSystem_ {};
};

} // namespace spaceship::server

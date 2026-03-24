#pragma once

// Owns the authoritative world and runs the ordered server simulation tick.
// System headers are hidden behind a pImpl — changes to any system header
// do not force recompilation of translation units that include this header.

#include "server/simulation/simulation_config.hpp"
#include "server/simulation/simulation_world.hpp"
#include "server/simulation/timestep/timestep_types.hpp"

#include <memory>
#include <optional>
#include <functional>
#include <string>
#include <vector>

namespace spaceship::server
{

class SimulationServer
{
  public:
    explicit SimulationServer(const SimulationConfig& config = {});
    explicit SimulationServer(SimulationWorld world, const SimulationConfig& config = {});

    // Required for pImpl with unique_ptr — defined in .cpp where Impl is complete.
    ~SimulationServer();
    SimulationServer(SimulationServer&&) noexcept;
    SimulationServer& operator=(SimulationServer&&) noexcept;

    shared::NetId spawnShip(const ShipSpawnRequest& request);
    void updateShipControl(shared::NetId shipNetId, const shared::ShipControl& control);
    void tick();

    [[nodiscard]] shared::Tick tickCount() const;
    [[nodiscard]] const SimulationWorld& world() const;
    [[nodiscard]] const std::string& lastSnapshotSummary() const;
    [[nodiscard]] const std::vector<TimestepDiagnostics>& timestepDiagnostics() const;

#ifdef BUILD_TESTING
    // Inject a pre-built projectile directly into the world for test scenarios
    // that need to set up collisions without going through the full spawn pipeline.
    void injectProjectile(ProjectileState projectile);
#endif

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace spaceship::server

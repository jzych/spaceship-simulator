#pragma once

// Owns the authoritative world and runs the ordered server simulation tick.
// System headers are hidden behind a pImpl — changes to any system header
// do not force recompilation of translation units that include this header.

#include "server/simulation/simulation_config.hpp"
#include "server/simulation/simulation_world.hpp"

#include <memory>
#include <optional>
#include <functional>
#include <string>

namespace spaceship::server
{

class SimulationServer
{
  public:
    explicit SimulationServer(const SimulationConfig& config = {});
    SimulationServer(SimulationWorld world, const SimulationConfig& config = {});

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

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace spaceship::server

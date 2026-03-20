#pragma once

// Defines the integration stage that advances dynamic entities over a fixed tick.

#include "server/gravity_system.hpp"
#include "server/simulation_config.hpp"
#include "server/simulation_world.hpp"

#include <span>

namespace spaceship::server
{

class IntegrationSystem
{
  public:
    void update(
        std::span<const MassiveBodyState> massiveBodies,
        std::span<ShipState> ships,
        std::span<ProjectileState> projectiles,
        const GravitySystem& gravitySystem,
        double nextElapsedSeconds,
        const SimulationConfig& config) const;
};

} // namespace spaceship::server

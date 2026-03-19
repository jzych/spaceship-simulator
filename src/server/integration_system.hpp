#pragma once

// Defines the integration stage that advances dynamic entities over a fixed tick.

#include "server/simulation_config.hpp"
#include "server/simulation_world.hpp"

#include <span>

namespace spaceship::server
{

class IntegrationSystem
{
  public:
    void update(std::span<ProjectileState> projectiles, const SimulationConfig& config) const;
};

} // namespace spaceship::server

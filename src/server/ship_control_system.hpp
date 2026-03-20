#pragma once

// Applies authoritative ship control state during the fixed server tick.

#include "server/simulation_config.hpp"
#include "server/simulation_world.hpp"

#include <span>

namespace spaceship::server
{

class ShipControlSystem
{
  public:
    void update(std::span<ShipState> ships, const SimulationConfig& config) const;
};

} // namespace spaceship::server

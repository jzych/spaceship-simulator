#pragma once

// Refreshes OrbitCache on each ship at a variable rate:
// active ships (throttle != 0) every tick, inactive every ~60 ticks.

#include "server/simulation_config.hpp"
#include "server/simulation_world.hpp"

#include <span>

namespace spaceship::server
{

class OrbitCacheSystem
{
  public:
    void update(
        std::span<ShipState> ships,
        std::span<const MassiveBodyState> massiveBodies,
        shared::Tick currentTick,
        const SimulationConfig& config,
        double elapsedSeconds) const;
};

} // namespace spaceship::server

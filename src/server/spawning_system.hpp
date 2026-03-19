#pragma once

// Declares the server-side spawn stage for ship-fired projectiles and control resets.

#include "server/simulation_world.hpp"

#include <span>

namespace spaceship::server
{

class SpawningSystem
{
  public:
    void update(std::span<ShipState> ships) const;
};

} // namespace spaceship::server

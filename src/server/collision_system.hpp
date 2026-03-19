#pragma once

// Defines the server-side collision cleanup stage for the simulation tick.

#include "server/simulation_world.hpp"

namespace spaceship::server
{

class CollisionSystem
{
  public:
    void update(std::vector<ProjectileState>& projectiles) const;
};

} // namespace spaceship::server

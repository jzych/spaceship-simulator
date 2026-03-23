#pragma once

// Defines the server-side collision cleanup stage for the simulation tick.

#include "server/simulation/simulation_config.hpp"
#include "server/simulation/simulation_world.hpp"

#include <span>

namespace spaceship::server
{

class CollisionSystem
{
  public:
    // Decrements TTL on all projectiles by one fixed time step.
    void decrementTtl(std::span<ProjectileState> projectiles, const SimulationConfig& config) const;

    // Removes projectiles whose TTL has expired.
    void update(std::vector<ProjectileState>& projectiles) const;
};

} // namespace spaceship::server

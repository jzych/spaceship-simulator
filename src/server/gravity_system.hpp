#pragma once

// Declares the gravity stage responsible for authoritative acceleration updates.

#include "server/simulation_world.hpp"

namespace spaceship::server
{

class GravitySystem
{
  public:
    void update(std::vector<PendingEvent>& events) const;
};

} // namespace spaceship::server

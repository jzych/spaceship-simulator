#pragma once

// Advances massive body positions and velocities analytically from elapsed
// simulation time using pre-computed circular-orbit parameters.

#include "server/simulation_world.hpp"

#include <vector>

namespace spaceship::server
{

class MassiveBodyMotionSystem
{
  public:
    void update(std::vector<MassiveBodyState>& massiveBodies, double elapsedSeconds) const;
};

} // namespace spaceship::server

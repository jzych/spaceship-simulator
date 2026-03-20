#pragma once

// Updates Sun, Earth, and Moon using predefined stable orbital motion.

#include "server/simulation_world.hpp"

#include <span>

namespace spaceship::server
{

class MassiveBodyMotionSystem
{
  public:
    void update(std::span<MassiveBodyState> massiveBodies, double elapsedSeconds) const;
};

} // namespace spaceship::server

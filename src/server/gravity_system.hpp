#pragma once

// Declares the gravity stage responsible for authoritative acceleration updates.

#include "server/massive_body_motion_system.hpp"
#include "server/simulation_world.hpp"

#include <span>

namespace spaceship::server
{

class GravitySystem
{
  public:
    void update(
        std::span<MassiveBodyState> massiveBodies,
        std::span<ShipState> ships,
        std::span<ProjectileState> projectiles,
        double elapsedSeconds) const;

  private:
    MassiveBodyMotionSystem massiveBodyMotionSystem_ {};
};

} // namespace spaceship::server

#pragma once

// Declares the gravity stage responsible for authoritative acceleration updates.

#include "server/massive_body_motion_system.hpp"
#include "server/simulation_world.hpp"

#include <span>
#include <vector>

namespace spaceship::server
{

class GravitySystem
{
  public:
    void update(std::span<MassiveBodyState> massiveBodies, double elapsedSeconds) const;
    void seedAccelerations(
        std::span<const MassiveBodyState> massiveBodies,
        std::span<ShipState> ships,
        std::span<ProjectileState> projectiles) const;
    [[nodiscard]] shared::Vec3 computeGravityAcceleration(
        const shared::Vec3& position,
        std::span<const MassiveBodyState> massiveBodies) const;
    [[nodiscard]] shared::Vec3 computeGravityAccelerationAtTime(
        const shared::Vec3& position,
        std::span<const MassiveBodyState> massiveBodies,
        double elapsedSeconds) const;

  private:
    MassiveBodyMotionSystem massiveBodyMotionSystem_ {};
};

} // namespace spaceship::server

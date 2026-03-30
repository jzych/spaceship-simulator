#pragma once

// Declares the gravity stage responsible for authoritative acceleration updates.

#include "server/simulation/simulation_world.hpp"

#include <span>

namespace spaceship::server
{

// Computes the total gravitational acceleration at a position from all
// massive bodies. Used by GravitySystem::update() and by the spawning
// system to initialise newly created entities with a valid a_n.
[[nodiscard]] shared::Vec3 computeGravitationalAcceleration(
    const shared::Vec3& position,
    std::span<const MassiveBodyState> massiveBodies);

class GravitySystem
{
  public:
    // Overwrites .acceleration on every ship and projectile with the
    // gravitational acceleration from all massive bodies.
    // Massive bodies themselves are not modified.
    void update(
        std::span<const MassiveBodyState> massiveBodies,
        std::span<ShipState> ships,
        std::span<ProjectileState> projectiles) const;
};

} // namespace spaceship::server

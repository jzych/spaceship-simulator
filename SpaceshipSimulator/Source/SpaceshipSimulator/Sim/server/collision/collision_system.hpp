#pragma once

// Server-side collision pipeline for one simulation interval.
// Provides TTL cleanup (decrementTtl / update) for projectile lifetime,
// and detectAndResolve for CCD sphere-sweep detection and energy-based despawn.

#include "server/simulation/simulation_config.hpp"
#include "server/simulation/simulation_world.hpp"

#include <span>

namespace spaceship::server
{

class CollisionSystem
{
  public:
    // Decrements TTL on all projectiles by dt seconds.
    void decrementTtl(std::span<ProjectileState> projectiles, double dt) const noexcept;

    // Removes projectiles whose TTL has expired.
    void update(std::vector<ProjectileState>& projectiles) const noexcept;

    // Run one full CCD collision pass over [0, dt].
    //   - Builds swept-AABB broad phase for all small objects.
    //   - Runs narrow-phase sphere-sweep TOI for candidate pairs and all small-vs-massive-body pairs.
    //   - Resolves hits in ascending (toi, pairIdLow, pairIdHigh) order.
    //   - Applies energy-based despawn per gameplay policy.
    //   - Appends CollisionEvents to outEvents.
    //   - Directly erases despawned entities from ships and projectiles.
    //   - Writes v_cm to the velocity of any survivor after a partially-inelastic hit.
    // Caller must rebuild shipIndex after this call if any ships were removed.
    //
    // Known limitation: the swept-AABB end position and narrow-phase TOI both assume
    // constant velocity over dt (linear motion). Gravitational deflection within the
    // interval is not modelled in the sweep; this is a deliberate first-pass approximation
    // acceptable at 60 Hz with the bodies used in this simulation.
    void detectAndResolve(
        std::vector<ShipState>& ships,
        std::vector<ProjectileState>& projectiles,
        std::span<const MassiveBodyState> massiveBodies,
        std::vector<CollisionEvent>& outEvents,
        const SimulationConfig& config,
        double dt) const;
};

} // namespace spaceship::server

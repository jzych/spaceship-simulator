#include "server/collision/collision_system.hpp"

namespace spaceship::server
{

void CollisionSystem::decrementTtl(
    std::span<ProjectileState> projectiles,
    const SimulationConfig& config) const
{
    for (auto& projectile : projectiles)
        projectile.params.ttlSeconds -= config.fixedDeltaSeconds;
}

void CollisionSystem::update(std::vector<ProjectileState>& projectiles) const
{
    std::erase_if(
        projectiles,
        [](const ProjectileState& projectile) { return projectile.params.ttlSeconds <= 0.0; });
}

} // namespace spaceship::server

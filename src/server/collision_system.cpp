#include "server/collision_system.hpp"

#include <algorithm>

namespace spaceship::server
{

void CollisionSystem::update(std::vector<ProjectileState>& projectiles) const
{
    projectiles.erase(
        std::remove_if(
            projectiles.begin(),
            projectiles.end(),
            [](const ProjectileState& projectile) { return projectile.params.ttlSeconds <= 0.0; }),
        projectiles.end());
}

} // namespace spaceship::server

#include "server/integration_system.hpp"

namespace spaceship::server
{

void IntegrationSystem::update(std::span<ProjectileState> projectiles, const SimulationConfig& config) const
{
    for (auto& projectile : projectiles)
    {
        projectile.params.ttlSeconds -= config.fixedDeltaSeconds;
    }
}

} // namespace spaceship::server

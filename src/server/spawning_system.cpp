#include "server/spawning_system.hpp"

namespace spaceship::server
{

void SpawningSystem::update(std::span<ShipState> ships) const
{
    for (auto& ship : ships)
    {
        ship.control.fire = false;
    }
}

} // namespace spaceship::server

#include "server/snapshot_system.hpp"

namespace spaceship::server
{

std::string SnapshotSystem::buildSnapshotSummary(const SimulationWorld& world) const
{
    return "snapshot bodies=" + std::to_string(world.massiveBodies.size()) +
           " ships=" + std::to_string(world.ships.size()) +
           " projectiles=" + std::to_string(world.projectiles.size());
}

} // namespace spaceship::server

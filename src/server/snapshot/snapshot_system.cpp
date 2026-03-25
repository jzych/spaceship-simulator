#include "server/snapshot/snapshot_system.hpp"

namespace spaceship::server
{

WorldSnapshot SnapshotSystem::buildSnapshot(
    const SimulationWorld& world,
    shared::Tick           serverTick,
    double                 elapsedSeconds) const
{
    WorldSnapshot snapshot;
    snapshot.serverTick     = serverTick;
    snapshot.elapsedSeconds = elapsedSeconds;

    // Reserve once per vector to avoid per-entity allocations.
    snapshot.massiveBodies.reserve(world.massiveBodies.size());
    for (const auto& body : world.massiveBodies)
    {
        snapshot.massiveBodies.push_back({
            .netId        = body.definition.netId,
            .position     = body.transform.position,
            .velocity     = body.velocity.linear,
            .radiusMeters = body.definition.radiusMeters,
        });
    }

    snapshot.ships.reserve(world.ships.size());
    for (const auto& ship : world.ships)
    {
        snapshot.ships.push_back({
            .netId        = ship.netId,
            .position     = ship.transform.position,
            .velocity     = ship.velocity.linear,
            .orientation  = ship.transform.orientation,
            .radiusMeters = ship.collider.radiusMeters,
            .throttle     = ship.control.throttle,
        });
    }

    snapshot.projectiles.reserve(world.projectiles.size());
    for (const auto& projectile : world.projectiles)
    {
        snapshot.projectiles.push_back({
            .netId        = projectile.netId,
            .position     = projectile.transform.position,
            .velocity     = projectile.velocity.linear,
            .radiusMeters = projectile.collider.radiusMeters,
            .ownerNetId   = projectile.params.ownerNetId,
        });
    }

    // Collision events are cleared at the start of each tick's collision pass
    // and accumulate only for the current tick. Events from ticks that do not
    // produce a snapshot are intentionally not accumulated — each snapshot
    // reflects only the collisions that occurred in its own tick.
    snapshot.collisionEvents = world.collisionEvents;

    return snapshot;
}

std::string SnapshotSystem::debugSummary(const WorldSnapshot& snapshot)
{
    return "snapshot bodies=" + std::to_string(snapshot.massiveBodies.size()) +
           " ships="          + std::to_string(snapshot.ships.size()) +
           " projectiles="    + std::to_string(snapshot.projectiles.size()) +
           " collisions="     + std::to_string(snapshot.collisionEvents.size());
}

} // namespace spaceship::server

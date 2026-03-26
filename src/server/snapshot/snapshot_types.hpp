#pragma once

// Structured world snapshot produced every snapshotIntervalTicks.
// Designed for serialization-readiness (FlatBuffers, Task 13) and
// client-side interpolation without string parsing.
//
// The three entity snapshot types are POD structs (trivially copyable, safe to
// memcpy) containing only the fields the client needs to render and interpolate.
// WorldSnapshot itself is not trivially copyable due to std::vector members.

#include "server/collision/collision_types.hpp"
#include "shared/sim_types.hpp"

#include <vector>

namespace spaceship::server
{

// Snapshot of a single massive body (Sun / Earth / Moon).
struct MassiveBodySnapshot
{
    shared::NetId netId         {};
    shared::Vec3  position      {};   // metres, SI
    shared::Vec3  velocity      {};   // m/s, SI
    double        radiusMeters  {};
};

// Snapshot of a single player-controlled ship.
struct ShipSnapshot
{
    shared::NetId       netId         {};
    shared::Vec3        position      {};
    shared::Vec3        velocity      {};
    shared::Quaternion  orientation   {};
    double              radiusMeters  {};
    double              throttle      {};
};

// Snapshot of a single projectile.
struct ProjectileSnapshot
{
    shared::NetId netId         {};
    shared::Vec3  position      {};
    shared::Vec3  velocity      {};
    double        radiusMeters  {};
    shared::NetId ownerNetId    {};
};

// Full world snapshot: one entry per snapshotIntervalTicks.
struct WorldSnapshot
{
    shared::Tick serverTick     {};
    double       elapsedSeconds {};   // simulation time elapsed — used for client interpolation

    // One entry per entity present at snapshot time.
    std::vector<MassiveBodySnapshot> massiveBodies   {};
    std::vector<ShipSnapshot>        ships           {};
    std::vector<ProjectileSnapshot>  projectiles     {};

    // Collision events that occurred during the tick that produced this snapshot.
    std::vector<CollisionEvent>      collisionEvents {};
};

} // namespace spaceship::server

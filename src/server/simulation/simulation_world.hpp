#pragma once

// Defines the in-memory authoritative world state used by server subsystems.

#include "server/simulation/gravity/orbit_cache.hpp"
#include "server/simulation/gravity/reference_body_selector.hpp"
#include "server/simulation/timestep/timestep_types.hpp"
#include "shared/sim_types.hpp"

#include <optional>
#include <string_view>
#include <vector>

namespace spaceship::server
{

struct OrbitalParams
{
    double orbitRadiusMeters {};           // 0 = stationary (Sun)
    double angularVelocityRadPerSec {};
    double initialPhaseRadians {};
    shared::NetId centerNetId {};          // NetId of the body being orbited
};

struct MassiveBodyState
{
    shared::MassiveBodyDefinition definition {};
    shared::Transform transform {};
    shared::Velocity velocity {};
    OrbitalParams orbital {};
    double spinAngleRadians {};               // current body rotation, updated by MassiveBodyMotionSystem
};

struct ShipState
{
    shared::NetId netId {};
    shared::Transform transform {};
    shared::Velocity velocity {};
    shared::MassProperties massProperties {};
    shared::ColliderSphere collider {};
    shared::ShipControl control {};
    shared::Vec3 acceleration {};          // gravity only — carried forward each tick
    shared::Vec3 thrustAcceleration {};    // thrust only — recomputed each tick by ShipControlSystem
    shared::Vec3 previousAcceleration {};
    OrbitCache orbitCache {};
    ReferenceBodySelector referenceBodySelector {};
    TimestepState timestepState {};
};

struct ProjectileState
{
    shared::NetId netId {};
    shared::Transform transform {};
    shared::Velocity velocity {};
    shared::MassProperties massProperties {};
    shared::ColliderSphere collider {};
    shared::ProjectileParams params {};
    shared::Vec3 acceleration {};          // gravity only — carried forward each tick
    shared::Vec3 thrustAcceleration {};    // always zero for projectiles
    shared::Vec3 previousAcceleration {};
    TimestepState timestepState {};
};

// Request type passed by callers to SimulationServer::spawnShip.
// Lives here so callers only need simulation_world.hpp, not spawning_system.hpp.
struct ShipSpawnRequest
{
    shared::Transform transform {};
    shared::Velocity velocity {};
    std::optional<shared::Vec3> initialAcceleration {};
};

struct PendingEvent
{
    std::string_view description {};
};

struct SimulationWorld
{
    std::vector<MassiveBodyState> massiveBodies {};
    std::vector<ShipState> ships {};
    std::vector<ProjectileState> projectiles {};
    std::vector<PendingEvent> events {};
};

} // namespace spaceship::server

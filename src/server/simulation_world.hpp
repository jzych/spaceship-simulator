#pragma once

// Defines the in-memory authoritative world state used by server subsystems.

#include "shared/sim_types.hpp"

#include <string_view>
#include <vector>

namespace spaceship::server
{

struct MassiveBodyState
{
    shared::MassiveBodyDefinition definition {};
    shared::Transform transform {};
    shared::Velocity velocity {};
};

struct ShipState
{
    shared::NetId netId {};
    shared::Transform transform {};
    shared::Velocity velocity {};
    shared::Vec3 gravityAcceleration {};
    shared::Vec3 acceleration {};
    shared::Vec3 thrustAcceleration {};
    shared::MassProperties massProperties {};
    shared::ColliderSphere collider {};
    shared::ShipControl control {};
};

struct ProjectileState
{
    shared::NetId netId {};
    shared::Transform transform {};
    shared::Velocity velocity {};
    shared::Vec3 acceleration {};
    shared::MassProperties massProperties {};
    shared::ColliderSphere collider {};
    shared::ProjectileParams params {};
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

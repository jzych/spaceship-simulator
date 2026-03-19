#pragma once

// Provides shared simulation primitives, IDs, and constants used across modules.

#include <cstdint>
#include <string_view>

namespace spaceship::shared
{

using NetId = std::uint32_t;
using Tick = std::uint64_t;

enum class EntityKind : std::uint8_t
{
    MassiveBody,
    Ship,
    Projectile,
};

struct Vec3
{
    double x {};
    double y {};
    double z {};
};

struct Quaternion
{
    double w {1.0};
    double x {};
    double y {};
    double z {};
};

struct Transform
{
    Vec3 position {};
    Quaternion orientation {};
};

struct Velocity
{
    Vec3 linear {};
};

struct MassProperties
{
    double massKg {};
    double inverseMass {};
};

struct ColliderSphere
{
    double radiusMeters {};
};

struct ShipControl
{
    double throttle {};
    Quaternion desiredOrientation {};
    bool fire {};
};

struct ProjectileParams
{
    double ttlSeconds {};
    NetId ownerNetId {};
};

struct MassiveBodyDefinition
{
    NetId netId {};
    std::string_view name {};
    double muMetersCubedPerSecondSquared {};
    double radiusMeters {};
};

namespace constants
{
constexpr double kServerTickHz = 60.0;
constexpr double kFixedDeltaSeconds = 1.0 / kServerTickHz;
constexpr double kAstronomicalUnitMeters = 149'597'870'700.0;
} // namespace constants

} // namespace spaceship::shared

#pragma once

// Declares creation of the initial authoritative simulation world.

#include "server/simulation_world.hpp"

namespace spaceship::server
{

inline constexpr shared::NetId kSunNetId = 0U;
inline constexpr shared::NetId kEarthNetId = 1U;
inline constexpr shared::NetId kMoonNetId = 2U;

inline constexpr double kEarthOrbitRadiusMeters = shared::constants::kAstronomicalUnitMeters;
inline constexpr double kEarthOrbitalSpeedMetersPerSecond = 29'780.0;
inline constexpr double kMoonOrbitRadiusMeters = 384'400'000.0;
inline constexpr double kMoonOrbitalSpeedRelativeToEarthMetersPerSecond = 1'022.0;

enum class BootstrapWorld
{
    Default,
    EarthOnly,
    SunEarth,
};

SimulationWorld createInitialWorld(BootstrapWorld world = BootstrapWorld::Default);

} // namespace spaceship::server

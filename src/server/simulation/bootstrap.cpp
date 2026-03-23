#include "server/simulation/bootstrap.hpp"

namespace spaceship::server
{

namespace
{

constexpr shared::NetId kSunNetId = 0U;
constexpr shared::NetId kEarthNetId = 1U;
constexpr shared::NetId kMoonNetId = 2U;

constexpr double kEarthOrbitalSpeedMetersPerSecond = 29'780.0;
constexpr double kMoonDistanceMeters = 384'400'000.0;
constexpr double kMoonOrbitalSpeedRelativeToEarthMetersPerSecond = 1'022.0;

constexpr double kEarthAngularVelocityRadPerSec =
    kEarthOrbitalSpeedMetersPerSecond / shared::constants::kAstronomicalUnitMeters;
constexpr double kMoonAngularVelocityRadPerSec =
    kMoonOrbitalSpeedRelativeToEarthMetersPerSecond / kMoonDistanceMeters;

} // namespace

SimulationWorld createInitialWorld()
{
    using shared::MassiveBodyDefinition;

    SimulationWorld world;
    world.massiveBodies = {
        MassiveBodyState {
            MassiveBodyDefinition {kSunNetId, "Sun", 1.32712440018e20, 6.9634e8, 0.0, 0.0},
            {{0.0, 0.0, 0.0}, {}},
            {{0.0, 0.0, 0.0}},
            // OrbitalParams: orbitRadius == 0 → stationary sentinel; centerNetId is unused
            {0.0, 0.0, 0.0, kSunNetId}},
        MassiveBodyState {
            MassiveBodyDefinition {kEarthNetId, "Earth", 3.986004418e14, 6.371e6, 86'164.1, 0.0},
            {{shared::constants::kAstronomicalUnitMeters, 0.0, 0.0}, {}},
            {{0.0, kEarthOrbitalSpeedMetersPerSecond, 0.0}},
            // OrbitalParams: orbitRadius > 0 → analytic circular orbit around Sun
            {shared::constants::kAstronomicalUnitMeters, kEarthAngularVelocityRadPerSec, 0.0, kSunNetId}},
        MassiveBodyState {
            MassiveBodyDefinition {kMoonNetId, "Moon", 4.9048695e12, 1.7374e6, 2'360'591.5, 0.0},
            {{shared::constants::kAstronomicalUnitMeters + kMoonDistanceMeters, 0.0, 0.0}, {}},
            {{0.0, kEarthOrbitalSpeedMetersPerSecond + kMoonOrbitalSpeedRelativeToEarthMetersPerSecond, 0.0}},
            // OrbitalParams: orbitRadius > 0 → analytic circular orbit around Earth
            {kMoonDistanceMeters, kMoonAngularVelocityRadPerSec, 0.0, kEarthNetId}},
    };

    return world;
}

SimulationWorld createEarthOnlyAtOriginWorld()
{
    using shared::MassiveBodyDefinition;

    constexpr double kEarthMu = 3.986004418e14;
    constexpr double kEarthRadius = 6.371e6;

    SimulationWorld world;
    world.massiveBodies = {
        MassiveBodyState {
            MassiveBodyDefinition {kEarthNetId, "Earth", kEarthMu, kEarthRadius, 86'164.1, 0.0},
            {{0.0, 0.0, 0.0}, {}},
            {{0.0, 0.0, 0.0}},
            // OrbitalParams: orbitRadius == 0 → stationary at origin; centerNetId is unused
            {0.0, 0.0, 0.0, kEarthNetId}},
    };

    return world;
}

} // namespace spaceship::server

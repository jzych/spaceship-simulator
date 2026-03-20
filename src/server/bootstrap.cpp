#include "server/bootstrap.hpp"

namespace spaceship::server
{

namespace
{

constexpr double kEarthOrbitalSpeedMetersPerSecond = 29'780.0;
constexpr double kMoonDistanceMeters = 384'400'000.0;
constexpr double kMoonOrbitalSpeedRelativeToEarthMetersPerSecond = 1'022.0;

} // namespace

SimulationWorld createInitialWorld()
{
    using shared::MassiveBodyDefinition;

    SimulationWorld world;
    world.massiveBodies = {
        MassiveBodyState {
            MassiveBodyDefinition {0U, "Sun", 1.32712440018e20, 6.9634e8},
            {{0.0, 0.0, 0.0}, {}},
            {{0.0, 0.0, 0.0}}},
        MassiveBodyState {
            MassiveBodyDefinition {1U, "Earth", 3.986004418e14, 6.371e6},
            {{shared::constants::kAstronomicalUnitMeters, 0.0, 0.0}, {}},
            {{0.0, kEarthOrbitalSpeedMetersPerSecond, 0.0}}},
        MassiveBodyState {
            MassiveBodyDefinition {2U, "Moon", 4.9048695e12, 1.7374e6},
            {{shared::constants::kAstronomicalUnitMeters + kMoonDistanceMeters, 0.0, 0.0}, {}},
            {{0.0, kEarthOrbitalSpeedMetersPerSecond + kMoonOrbitalSpeedRelativeToEarthMetersPerSecond, 0.0}}},
    };

    return world;
}

} // namespace spaceship::server

#include "server/bootstrap.hpp"

namespace spaceship::server
{

namespace
{

MassiveBodyState makeSun()
{
    return {
        {kSunNetId, "Sun", 1.32712440018e20, 6.9634e8},
        {{0.0, 0.0, 0.0}, {}},
        {{0.0, 0.0, 0.0}},
    };
}

MassiveBodyState makeEarthAtOrigin()
{
    return {
        {kEarthNetId, "Earth", 3.986004418e14, 6.371e6},
        {{0.0, 0.0, 0.0}, {}},
        {{0.0, 0.0, 0.0}},
    };
}

MassiveBodyState makeEarthInSolarOrbit()
{
    return {
        {kEarthNetId, "Earth", 3.986004418e14, 6.371e6},
        {{kEarthOrbitRadiusMeters, 0.0, 0.0}, {}},
        {{0.0, kEarthOrbitalSpeedMetersPerSecond, 0.0}},
    };
}

MassiveBodyState makeMoon()
{
    return {
        {kMoonNetId, "Moon", 4.9048695e12, 1.7374e6},
        {{kEarthOrbitRadiusMeters + kMoonOrbitRadiusMeters, 0.0, 0.0}, {}},
        {{0.0, kEarthOrbitalSpeedMetersPerSecond + kMoonOrbitalSpeedRelativeToEarthMetersPerSecond, 0.0}},
    };
}

} // namespace

SimulationWorld createInitialWorld(BootstrapWorld world)
{
    SimulationWorld initialWorld;

    switch (world)
    {
        case BootstrapWorld::EarthOnly:
            initialWorld.massiveBodies = {makeEarthAtOrigin()};
            break;
        case BootstrapWorld::SunEarth:
            initialWorld.massiveBodies = {makeSun(), makeEarthInSolarOrbit()};
            break;
        case BootstrapWorld::Default:
        default:
            initialWorld.massiveBodies = {makeSun(), makeEarthInSolarOrbit(), makeMoon()};
            break;
    }

    return initialWorld;
}

} // namespace spaceship::server

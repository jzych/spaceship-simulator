#include "server/massive_body_motion_system.hpp"

#include "server/bootstrap.hpp"
#include "server/simulation_math.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

namespace spaceship::server
{

namespace
{

constexpr double kFullTurnRadians = 2.0 * std::numbers::pi_v<double>;

shared::Vec3 makeCircularOrbitPosition(double radiusMeters, double angleRadians)
{
    return {radiusMeters * std::cos(angleRadians), radiusMeters * std::sin(angleRadians), 0.0};
}

shared::Vec3 makeCircularOrbitVelocity(double radiusMeters, double angularSpeedRadiansPerSecond, double angleRadians)
{
    return {
        -radiusMeters * angularSpeedRadiansPerSecond * std::sin(angleRadians),
        radiusMeters * angularSpeedRadiansPerSecond * std::cos(angleRadians),
        0.0,
    };
}

} // namespace

void MassiveBodyMotionSystem::update(std::span<MassiveBodyState> massiveBodies, double elapsedSeconds) const
{
    MassiveBodyState* sun = nullptr;
    MassiveBodyState* earth = nullptr;
    MassiveBodyState* moon = nullptr;

    for (MassiveBodyState& body : massiveBodies)
    {
        if (body.definition.netId == kSunNetId)
        {
            sun = &body;
        }
        else if (body.definition.netId == kEarthNetId)
        {
            earth = &body;
        }
        else if (body.definition.netId == kMoonNetId)
        {
            moon = &body;
        }
    }

    if (sun == nullptr || earth == nullptr || moon == nullptr)
    {
        return;
    }

    const double earthAngularSpeedRadiansPerSecond = kEarthOrbitalSpeedMetersPerSecond / kEarthOrbitRadiusMeters;
    const double moonAngularSpeedRadiansPerSecond =
        kMoonOrbitalSpeedRelativeToEarthMetersPerSecond / kMoonOrbitRadiusMeters;
    const double earthAngleRadians =
        std::fmod(earthAngularSpeedRadiansPerSecond * elapsedSeconds, kFullTurnRadians);
    const double moonAngleRadians =
        std::fmod(moonAngularSpeedRadiansPerSecond * elapsedSeconds, kFullTurnRadians);

    sun->transform.position = {0.0, 0.0, 0.0};
    sun->velocity.linear = {0.0, 0.0, 0.0};

    earth->transform.position = makeCircularOrbitPosition(kEarthOrbitRadiusMeters, earthAngleRadians);
    earth->velocity.linear =
        makeCircularOrbitVelocity(kEarthOrbitRadiusMeters, earthAngularSpeedRadiansPerSecond, earthAngleRadians);

    const shared::Vec3 moonRelativePosition = makeCircularOrbitPosition(kMoonOrbitRadiusMeters, moonAngleRadians);
    const shared::Vec3 moonRelativeVelocity =
        makeCircularOrbitVelocity(kMoonOrbitRadiusMeters, moonAngularSpeedRadiansPerSecond, moonAngleRadians);

    moon->transform.position = add(earth->transform.position, moonRelativePosition);
    moon->velocity.linear = add(earth->velocity.linear, moonRelativeVelocity);
}

} // namespace spaceship::server

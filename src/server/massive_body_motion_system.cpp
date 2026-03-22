#include "server/massive_body_motion_system.hpp"

#include <algorithm>
#include <cmath>

namespace spaceship::server
{

namespace
{

shared::Vec3 findCenterPosition(
    const std::vector<MassiveBodyState>& massiveBodies,
    shared::NetId centerNetId)
{
    const auto it = std::find_if(
        massiveBodies.begin(),
        massiveBodies.end(),
        [centerNetId](const MassiveBodyState& body)
        { return body.definition.netId == centerNetId; });

    return (it != massiveBodies.end()) ? it->transform.position : shared::Vec3 {};
}

} // namespace

void MassiveBodyMotionSystem::update(
    std::vector<MassiveBodyState>& massiveBodies,
    double elapsedSeconds) const
{
    for (auto& body : massiveBodies)
    {
        if (body.orbital.orbitRadiusMeters == 0.0)
            continue;

        // Center position is already updated earlier in the same pass
        // (guaranteed by Sun→Earth→Moon index order in bootstrap)
        const shared::Vec3 center =
            findCenterPosition(massiveBodies, body.orbital.centerNetId);

        const double phase =
            body.orbital.angularVelocityRadPerSec * elapsedSeconds +
            body.orbital.initialPhaseRadians;

        const double r = body.orbital.orbitRadiusMeters;
        const double omega = body.orbital.angularVelocityRadPerSec;

        body.transform.position = {
            center.x + r * std::cos(phase),
            center.y + r * std::sin(phase),
            center.z,
        };

        body.velocity.linear = {
            -r * omega * std::sin(phase),
             r * omega * std::cos(phase),
            0.0,
        };
    }
}

} // namespace spaceship::server

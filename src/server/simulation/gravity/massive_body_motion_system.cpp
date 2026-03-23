#include "server/simulation/gravity/massive_body_motion_system.hpp"
#include "server/simulation/simulation_math.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <span>

namespace spaceship::server
{


void MassiveBodyMotionSystem::update(
    std::span<MassiveBodyState> massiveBodies,
    double elapsedSeconds) const
{
    for (auto& body : massiveBodies)
    {
        body.spinAngleRadians = bodySpinAngle(
            body.definition.siderealRotationPeriodSeconds,
            body.definition.initialRotationPhaseRadians,
            elapsedSeconds);

        if (body.orbital.orbitRadiusMeters == 0.0)
            continue;

        // Invariant: the center body must appear at a lower index so its position
        // has already been updated in this same pass. Bootstrapped order: Sun < Earth < Moon.
        // Violating this order produces one-tick-stale center coordinates.
        const auto centerIt = std::find_if(
            massiveBodies.begin(),
            massiveBodies.end(),
            [&body](const MassiveBodyState& b)
            { return b.definition.netId == body.orbital.centerNetId; });
        // Invariant: center body index must be less than this body's index.
        // Computed via pointer arithmetic — avoids a second O(n) scan.
        assert(centerIt - massiveBodies.begin() < &body - massiveBodies.data() &&
               "center body must precede orbiting body in massiveBodies vector");

        const shared::Vec3 center =
            (centerIt != massiveBodies.end()) ? centerIt->transform.position : shared::Vec3 {};

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

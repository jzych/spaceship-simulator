#pragma once

// Computes gravitational acceleration on small bodies from the massive-body field.

#include "server/simulation_math.hpp"
#include "server/simulation_world.hpp"

#include <cmath>
#include <span>

namespace spaceship::server
{

inline shared::Vec3 computeGravityAcceleration(
    const shared::Vec3& position,
    std::span<const MassiveBodyState> massiveBodies)
{
    shared::Vec3 totalAcceleration {};

    for (const MassiveBodyState& body : massiveBodies)
    {
        const shared::Vec3 displacement = subtract(body.transform.position, position);
        const double distanceSquared = magnitudeSquared(displacement);

        if (distanceSquared == 0.0)
        {
            continue;
        }

        const double inverseDistance = 1.0 / std::sqrt(distanceSquared);
        const double inverseDistanceCubed = inverseDistance * inverseDistance * inverseDistance;
        const double accelerationScale = body.definition.muMetersCubedPerSecondSquared * inverseDistanceCubed;

        totalAcceleration = add(totalAcceleration, scale(displacement, accelerationScale));
    }

    return totalAcceleration;
}

} // namespace spaceship::server

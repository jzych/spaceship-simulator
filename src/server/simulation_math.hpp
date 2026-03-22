#pragma once

// Provides small shared math helpers for authoritative simulation systems.

#include "shared/sim_types.hpp"

#include <cmath>

namespace spaceship::server
{

inline shared::Vec3 add(const shared::Vec3& lhs, const shared::Vec3& rhs)
{
    return {lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z};
}

inline shared::Vec3 scale(const shared::Vec3& value, double factor)
{
    return {value.x * factor, value.y * factor, value.z * factor};
}

inline shared::Vec3 subtract(const shared::Vec3& lhs, const shared::Vec3& rhs)
{
    return {lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z};
}

inline double dot(const shared::Vec3& lhs, const shared::Vec3& rhs)
{
    return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
}

inline double lengthSquared(const shared::Vec3& value)
{
    return dot(value, value);
}

inline double length(const shared::Vec3& value)
{
    return std::sqrt(lengthSquared(value));
}

inline shared::Vec3 cross(const shared::Vec3& lhs, const shared::Vec3& rhs)
{
    return {
        lhs.y * rhs.z - lhs.z * rhs.y,
        lhs.z * rhs.x - lhs.x * rhs.z,
        lhs.x * rhs.y - lhs.y * rhs.x,
    };
}

inline shared::Vec3 normalize(const shared::Vec3& value)
{
    const double len = length(value);
    constexpr double kEpsilon = 1e-15;
    if (len < kEpsilon)
        return {};
    return scale(value, 1.0 / len);
}

inline shared::Vec3 negate(const shared::Vec3& value)
{
    return {-value.x, -value.y, -value.z};
}

// Requires planeNormal to be a unit vector.
inline shared::Vec3 projectOntoPlane(const shared::Vec3& v, const shared::Vec3& planeNormal)
{
    return subtract(v, scale(planeNormal, dot(v, planeNormal)));
}

struct GeographicCoordinates
{
    double altitudeMeters {};
    double longitudeRadians {};
    double latitudeRadians {};
};

// Compute body spin angle at elapsed time from sidereal period and initial phase.
inline double bodySpinAngle(double siderealPeriodSeconds, double initialPhaseRadians, double elapsedSeconds)
{
    if (siderealPeriodSeconds <= 0.0)
        return initialPhaseRadians;
    constexpr double kTwoPi = 2.0 * 3.14159265358979323846;
    const double angularVelocity = kTwoPi / siderealPeriodSeconds;
    return initialPhaseRadians + angularVelocity * elapsedSeconds;
}

// Transform inertial-relative position to body-fixed geographic coordinates.
// spinAngle: current rotation angle of the body around its spin axis (Y-axis).
inline GeographicCoordinates toBodyFixedGeographic(
    const shared::Vec3& relativePosition,
    double bodyRadius,
    double spinAngle)
{
    const double r = length(relativePosition);
    const double cosTheta = std::cos(spinAngle);
    const double sinTheta = std::sin(spinAngle);

    // Rotate by -spinAngle around Y-axis to get body-fixed position
    const double xBf = relativePosition.x * cosTheta + relativePosition.z * sinTheta;
    const double yBf = relativePosition.y;
    const double zBf = -relativePosition.x * sinTheta + relativePosition.z * cosTheta;

    GeographicCoordinates geo {};
    geo.altitudeMeters = r - bodyRadius;

    constexpr double kMinRadius = 1.0;
    if (r < kMinRadius)
        return geo;

    geo.longitudeRadians = std::atan2(zBf, xBf);
    geo.latitudeRadians = std::asin(yBf / r);

    return geo;
}

inline shared::Quaternion conjugate(const shared::Quaternion& quaternion)
{
    return {quaternion.w, -quaternion.x, -quaternion.y, -quaternion.z};
}

inline shared::Quaternion multiply(const shared::Quaternion& lhs, const shared::Quaternion& rhs)
{
    return {
        lhs.w * rhs.w - lhs.x * rhs.x - lhs.y * rhs.y - lhs.z * rhs.z,
        lhs.w * rhs.x + lhs.x * rhs.w + lhs.y * rhs.z - lhs.z * rhs.y,
        lhs.w * rhs.y - lhs.x * rhs.z + lhs.y * rhs.w + lhs.z * rhs.x,
        lhs.w * rhs.z + lhs.x * rhs.y - lhs.y * rhs.x + lhs.z * rhs.w,
    };
}

inline shared::Vec3 rotateVector(const shared::Quaternion& rotation, const shared::Vec3& vector)
{
    const shared::Quaternion pureVector {0.0, vector.x, vector.y, vector.z};
    const shared::Quaternion rotated = multiply(multiply(rotation, pureVector), conjugate(rotation));
    return {rotated.x, rotated.y, rotated.z};
}

inline shared::Vec3 forwardDirection(const shared::Quaternion& orientation)
{
    constexpr shared::Vec3 kLocalForward {1.0, 0.0, 0.0};

    const shared::Vec3 worldForward = rotateVector(orientation, kLocalForward);
    const double worldForwardLength = length(worldForward);

    if (worldForwardLength == 0.0)
    {
        return kLocalForward;
    }

    return scale(worldForward, 1.0 / worldForwardLength);
}

} // namespace spaceship::server

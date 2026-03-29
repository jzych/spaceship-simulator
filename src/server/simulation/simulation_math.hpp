#pragma once

// Provides small shared math helpers for authoritative simulation systems.

#include "shared/sim_types.hpp"

#include <cmath>
#include <numbers>

namespace spaceship::server
{

constexpr shared::Vec3 add(const shared::Vec3& lhs, const shared::Vec3& rhs) noexcept
{
    return {lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z};
}

constexpr shared::Vec3 scale(const shared::Vec3& value, double factor) noexcept
{
    return {value.x * factor, value.y * factor, value.z * factor};
}

constexpr shared::Vec3 subtract(const shared::Vec3& lhs, const shared::Vec3& rhs) noexcept
{
    return {lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z};
}

constexpr double dot(const shared::Vec3& lhs, const shared::Vec3& rhs) noexcept
{
    return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
}

constexpr double lengthSquared(const shared::Vec3& value) noexcept
{
    return dot(value, value);
}

inline double length(const shared::Vec3& value) noexcept
{
    return std::sqrt(lengthSquared(value));
}

constexpr shared::Vec3 cross(const shared::Vec3& lhs, const shared::Vec3& rhs) noexcept
{
    return {
        lhs.y * rhs.z - lhs.z * rhs.y,
        lhs.z * rhs.x - lhs.x * rhs.z,
        lhs.x * rhs.y - lhs.y * rhs.x,
    };
}

inline shared::Vec3 normalize(const shared::Vec3& value) noexcept
{
    const double len = length(value);
    constexpr double kEpsilon = 1e-15;
    if (len < kEpsilon)
        return {};
    return scale(value, 1.0 / len);
}

constexpr shared::Vec3 negate(const shared::Vec3& value) noexcept
{
    return {-value.x, -value.y, -value.z};
}

// Requires planeNormal to be a unit vector.
constexpr shared::Vec3 projectOntoPlane(const shared::Vec3& v, const shared::Vec3& planeNormal) noexcept
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
inline double bodySpinAngle(double siderealPeriodSeconds, double initialPhaseRadians, double elapsedSeconds) noexcept
{
    if (siderealPeriodSeconds <= 0.0)
        return initialPhaseRadians;
    constexpr double kTwoPi = 2.0 * std::numbers::pi;
    const double angularVelocity = kTwoPi / siderealPeriodSeconds;
    return initialPhaseRadians + angularVelocity * elapsedSeconds;
}

// Transform inertial-relative position to body-fixed geographic coordinates.
// spinAngle: current rotation angle of the body around its spin axis (Y-axis).
inline GeographicCoordinates toBodyFixedGeographic(
    const shared::Vec3& relativePosition,
    double bodyRadius,
    double spinAngle) noexcept
{
    const double r = length(relativePosition);
    const double cosSpinAngle = std::cos(spinAngle);
    const double sinSpinAngle = std::sin(spinAngle);

    // Rotate by -spinAngle around Y-axis to get body-fixed position
    const double bodyFixedX = relativePosition.x * cosSpinAngle + relativePosition.z * sinSpinAngle;
    const double bodyFixedY = relativePosition.y;
    const double bodyFixedZ = -relativePosition.x * sinSpinAngle + relativePosition.z * cosSpinAngle;

    GeographicCoordinates geo {};
    geo.altitudeMeters = r - bodyRadius;

    constexpr double kMinRadius = 1.0;
    if (r < kMinRadius)
        return geo;

    geo.longitudeRadians = std::atan2(bodyFixedZ, bodyFixedX);
    geo.latitudeRadians = std::asin(bodyFixedY / r);

    return geo;
}

constexpr shared::Quaternion conjugate(const shared::Quaternion& quaternion) noexcept
{
    return {quaternion.w, -quaternion.x, -quaternion.y, -quaternion.z};
}

constexpr shared::Quaternion multiply(const shared::Quaternion& lhs, const shared::Quaternion& rhs) noexcept
{
    return {
        lhs.w * rhs.w - lhs.x * rhs.x - lhs.y * rhs.y - lhs.z * rhs.z,
        lhs.w * rhs.x + lhs.x * rhs.w + lhs.y * rhs.z - lhs.z * rhs.y,
        lhs.w * rhs.y - lhs.x * rhs.z + lhs.y * rhs.w + lhs.z * rhs.x,
        lhs.w * rhs.z + lhs.x * rhs.y - lhs.y * rhs.x + lhs.z * rhs.w,
    };
}

constexpr shared::Vec3 rotateVector(const shared::Quaternion& rotation, const shared::Vec3& vector) noexcept
{
    const shared::Quaternion pureVector {0.0, vector.x, vector.y, vector.z};
    const shared::Quaternion rotated = multiply(multiply(rotation, pureVector), conjugate(rotation));
    return {rotated.x, rotated.y, rotated.z};
}

inline shared::Vec3 forwardDirection(const shared::Quaternion& orientation) noexcept
{
    constexpr shared::Vec3 kLocalForward {1.0, 0.0, 0.0};

    const shared::Vec3 worldForward = rotateVector(orientation, kLocalForward);
    const double worldForwardLength = length(worldForward);

    constexpr double kEpsilon = 1e-15;
    if (worldForwardLength < kEpsilon)
    {
        return kLocalForward;
    }

    return scale(worldForward, 1.0 / worldForwardLength);
}

} // namespace spaceship::server

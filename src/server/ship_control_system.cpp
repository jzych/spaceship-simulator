#include "server/ship_control_system.hpp"

#include <algorithm>
#include <cmath>

namespace spaceship::server
{

namespace
{

shared::Vec3 add(const shared::Vec3& lhs, const shared::Vec3& rhs)
{
    return {lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z};
}

shared::Vec3 scale(const shared::Vec3& value, double factor)
{
    return {value.x * factor, value.y * factor, value.z * factor};
}

shared::Quaternion conjugate(const shared::Quaternion& quaternion)
{
    return {quaternion.w, -quaternion.x, -quaternion.y, -quaternion.z};
}

shared::Quaternion multiply(const shared::Quaternion& lhs, const shared::Quaternion& rhs)
{
    return {
        lhs.w * rhs.w - lhs.x * rhs.x - lhs.y * rhs.y - lhs.z * rhs.z,
        lhs.w * rhs.x + lhs.x * rhs.w + lhs.y * rhs.z - lhs.z * rhs.y,
        lhs.w * rhs.y - lhs.x * rhs.z + lhs.y * rhs.w + lhs.z * rhs.x,
        lhs.w * rhs.z + lhs.x * rhs.y - lhs.y * rhs.x + lhs.z * rhs.w,
    };
}

shared::Vec3 rotateVector(const shared::Quaternion& rotation, const shared::Vec3& vector)
{
    const shared::Quaternion pureVector {0.0, vector.x, vector.y, vector.z};
    const shared::Quaternion rotated = multiply(multiply(rotation, pureVector), conjugate(rotation));
    return {rotated.x, rotated.y, rotated.z};
}

shared::Vec3 forwardDirection(const shared::Quaternion& orientation)
{
    constexpr shared::Vec3 kLocalForward {1.0, 0.0, 0.0};

    const shared::Vec3 worldForward = rotateVector(orientation, kLocalForward);
    const double length =
        std::sqrt(worldForward.x * worldForward.x + worldForward.y * worldForward.y + worldForward.z * worldForward.z);

    if (length == 0.0)
    {
        return kLocalForward;
    }

    return {worldForward.x / length, worldForward.y / length, worldForward.z / length};
}

} // namespace

void ShipControlSystem::update(std::span<ShipState> ships, const SimulationConfig& config) const
{
    for (auto& ship : ships)
    {
        ship.transform.orientation = ship.control.desiredOrientation;

        const double throttle = std::clamp(ship.control.throttle, 0.0, 1.0);
        const double thrustNewtons = throttle * config.shipMaxThrustNewtons;
        const double accelerationMetersPerSecondSquared = thrustNewtons * ship.massProperties.inverseMass;
        const shared::Vec3 thrustVelocityDelta =
            scale(forwardDirection(ship.transform.orientation), accelerationMetersPerSecondSquared * config.fixedDeltaSeconds);

        ship.velocity.linear = add(ship.velocity.linear, thrustVelocityDelta);
    }
}

} // namespace spaceship::server

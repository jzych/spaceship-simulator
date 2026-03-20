#include "server/ship_control_system.hpp"
#include "server/simulation_math.hpp"

#include <algorithm>

namespace spaceship::server
{

namespace
{

constexpr double kMinimumThrottle = 0.0;
constexpr double kMaximumThrottle = 1.0;

} // namespace

void ShipControlSystem::update(std::span<ShipState> ships, const SimulationConfig& config) const
{
    for (auto& ship : ships)
    {
        ship.transform.orientation = ship.control.desiredOrientation;

        const double throttle = std::clamp(ship.control.throttle, kMinimumThrottle, kMaximumThrottle);
        const double thrustNewtons = throttle * config.shipMaxThrustNewtons;
        const double accelerationMetersPerSecondSquared = thrustNewtons * ship.massProperties.inverseMass;
        ship.thrustAcceleration =
            scale(forwardDirection(ship.transform.orientation), accelerationMetersPerSecondSquared);
    }
}

} // namespace spaceship::server

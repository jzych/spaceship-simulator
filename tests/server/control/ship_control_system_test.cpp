#include "server/control/ship_control_system.hpp"
#include "server/simulation/simulation_math.hpp"

#include <gtest/gtest.h>
#include <numbers>

using namespace spaceship::server;

// ---------------------------------------------------------------------------
// ShipControlSystemTest — direct unit tests for thrust and orientation
// ---------------------------------------------------------------------------

class ShipControlSystemTest : public ::testing::Test
{
  protected:
    ShipControlSystem system;
    SimulationConfig config {};

    ShipState makeShip(double throttle, spaceship::shared::Quaternion orientation, double massKg = 1'000.0)
    {
        ShipState ship {};
        ship.netId = 100;
        ship.massProperties = {massKg, massKg > 0.0 ? 1.0 / massKg : 0.0};
        ship.control.throttle = throttle;
        ship.control.desiredOrientation = orientation;
        return ship;
    }
};

TEST_F(ShipControlSystemTest, GivenZeroThrottle_WhenUpdated_ThenThrustAccelerationIsZero)
{
    std::vector<ShipState> ships {makeShip(0.0, {1.0, 0.0, 0.0, 0.0})};
    system.update(ships, config);

    EXPECT_DOUBLE_EQ(ships[0].thrustAcceleration.x, 0.0);
    EXPECT_DOUBLE_EQ(ships[0].thrustAcceleration.y, 0.0);
    EXPECT_DOUBLE_EQ(ships[0].thrustAcceleration.z, 0.0);
}

TEST_F(ShipControlSystemTest, GivenFullThrottle_WhenUpdated_ThenThrustAccelerationIsMaximum)
{
    std::vector<ShipState> ships {makeShip(1.0, {1.0, 0.0, 0.0, 0.0})};
    system.update(ships, config);

    // F = 20,000 N, m = 1,000 kg → a = 20 m/s²
    const double expectedAccel = config.shipMaxThrustNewtons / 1'000.0;
    const double aMag = length(ships[0].thrustAcceleration);
    EXPECT_NEAR(aMag, expectedAccel, 1e-9);
}

TEST_F(ShipControlSystemTest, GivenOrientationRotated90DegreesAroundZ_WhenUpdated_ThenThrustIsInYDirection)
{
    // 90° rotation around Z-axis: forward goes from +X to +Y
    const double halfAngle = std::numbers::pi / 4.0;
    const spaceship::shared::Quaternion rotation {
        std::cos(halfAngle), 0.0, 0.0, std::sin(halfAngle)};
    std::vector<ShipState> ships {makeShip(1.0, rotation)};

    system.update(ships, config);

    EXPECT_NEAR(ships[0].thrustAcceleration.x, 0.0, 1e-9);
    EXPECT_GT(ships[0].thrustAcceleration.y, 0.0);
}

TEST_F(ShipControlSystemTest, GivenDesiredOrientation_WhenUpdated_ThenOrientationSnappedToDesired)
{
    const spaceship::shared::Quaternion desired {0.5, 0.5, 0.5, 0.5};
    std::vector<ShipState> ships {makeShip(0.0, desired)};
    ships[0].transform.orientation = {1.0, 0.0, 0.0, 0.0}; // start at identity

    system.update(ships, config);

    EXPECT_DOUBLE_EQ(ships[0].transform.orientation.w, desired.w);
    EXPECT_DOUBLE_EQ(ships[0].transform.orientation.x, desired.x);
    EXPECT_DOUBLE_EQ(ships[0].transform.orientation.y, desired.y);
    EXPECT_DOUBLE_EQ(ships[0].transform.orientation.z, desired.z);
}

TEST_F(ShipControlSystemTest, GivenThrottleAboveOne_WhenUpdated_ThenClampedToFullThrottle)
{
    std::vector<ShipState> ships {makeShip(2.0, {1.0, 0.0, 0.0, 0.0})};
    system.update(ships, config);

    // Throttle clamped to 1.0 → same as full throttle
    const double expectedAccel = config.shipMaxThrustNewtons / 1'000.0;
    const double aMag = length(ships[0].thrustAcceleration);
    EXPECT_NEAR(aMag, expectedAccel, 1e-9);
}

TEST_F(ShipControlSystemTest, GivenNegativeThrottle_WhenUpdated_ThenClampedToZeroThrust)
{
    std::vector<ShipState> ships {makeShip(-0.5, {1.0, 0.0, 0.0, 0.0})};
    system.update(ships, config);

    EXPECT_DOUBLE_EQ(ships[0].thrustAcceleration.x, 0.0);
    EXPECT_DOUBLE_EQ(ships[0].thrustAcceleration.y, 0.0);
    EXPECT_DOUBLE_EQ(ships[0].thrustAcceleration.z, 0.0);
}

TEST_F(ShipControlSystemTest, GivenTwoShipsWithDifferentThrottle_WhenUpdated_ThenEachProcessedIndependently)
{
    std::vector<ShipState> ships {
        makeShip(1.0, {1.0, 0.0, 0.0, 0.0}),
        makeShip(0.0, {1.0, 0.0, 0.0, 0.0}),
    };
    system.update(ships, config);

    EXPECT_GT(length(ships[0].thrustAcceleration), 0.0);
    EXPECT_DOUBLE_EQ(length(ships[1].thrustAcceleration), 0.0);
}

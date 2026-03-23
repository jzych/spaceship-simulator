#include "server/gravity_system.hpp"
#include "server/simulation_math.hpp"

#include <gtest/gtest.h>
#include <cmath>

using namespace spaceship::server;

// ---------------------------------------------------------------------------
// GravitySystemTest — direct unit tests for gravitational acceleration
// ---------------------------------------------------------------------------

class GravitySystemTest : public ::testing::Test
{
  protected:
    GravitySystem system;

    static constexpr double kEarthMu = 3.986004418e14;
    static constexpr double kEarthRadius = 6.371e6;
    static constexpr double kLeoRadius = kEarthRadius + 400'000.0;

    MassiveBodyState makeEarth(spaceship::shared::Vec3 pos = {}) const
    {
        return MassiveBodyState {{0, "Earth", kEarthMu, kEarthRadius}, {pos}, {}, {}};
    }
};

TEST_F(GravitySystemTest, GivenShipAtLeo_WhenGravityComputed_ThenMagnitudeMatchesTwoBodyFormula)
{
    std::vector<MassiveBodyState> bodies {makeEarth()};
    const spaceship::shared::Vec3 pos {kLeoRadius, 0.0, 0.0};

    const auto accel = computeGravitationalAcceleration(pos, bodies);

    const double expectedG = kEarthMu / (kLeoRadius * kLeoRadius);
    EXPECT_NEAR(length(accel), expectedG, 0.01);
}

TEST_F(GravitySystemTest, GivenShipOnPositiveXAxis_WhenGravityComputed_ThenAccelerationPointsTowardBody)
{
    std::vector<MassiveBodyState> bodies {makeEarth()};
    const spaceship::shared::Vec3 pos {kLeoRadius, 0.0, 0.0};

    const auto accel = computeGravitationalAcceleration(pos, bodies);

    EXPECT_LT(accel.x, 0.0); // toward origin
    EXPECT_NEAR(accel.y, 0.0, 1e-15);
    EXPECT_NEAR(accel.z, 0.0, 1e-15);
}

TEST_F(GravitySystemTest, GivenTwoBodies_WhenGravityComputed_ThenBothContribute)
{
    std::vector<MassiveBodyState> bodies {
        makeEarth({0.0, 0.0, 0.0}),
        MassiveBodyState {{1, "Sun", 1.32712440018e20, 6.9634e8}, {{1e11, 0.0, 0.0}}, {}, {}},
    };
    const spaceship::shared::Vec3 pos {kLeoRadius, 0.0, 0.0};

    const auto accelTwo = computeGravitationalAcceleration(pos, bodies);
    const auto accelOne = computeGravitationalAcceleration(pos, std::span(bodies.data(), 1));

    // With two bodies, the acceleration magnitude should differ
    EXPECT_NE(length(accelTwo), length(accelOne));
}

TEST_F(GravitySystemTest, GivenShipAtBodyCenter_WhenGravityComputed_ThenZeroAcceleration)
{
    std::vector<MassiveBodyState> bodies {makeEarth()};
    const spaceship::shared::Vec3 pos {0.0, 0.0, 0.0}; // at body center

    const auto accel = computeGravitationalAcceleration(pos, bodies);

    EXPECT_DOUBLE_EQ(accel.x, 0.0);
    EXPECT_DOUBLE_EQ(accel.y, 0.0);
    EXPECT_DOUBLE_EQ(accel.z, 0.0);
}

TEST_F(GravitySystemTest, GivenShipAtLeo_WhenSystemUpdated_ThenAccelerationSeeded)
{
    std::vector<MassiveBodyState> bodies {makeEarth()};
    std::vector<ShipState> ships;
    std::vector<ProjectileState> projectiles;

    ShipState ship {};
    ship.transform.position = {kLeoRadius, 0.0, 0.0};
    ships.push_back(ship);

    system.update(bodies, ships, projectiles);

    EXPECT_LT(ships[0].acceleration.x, 0.0);
}

TEST_F(GravitySystemTest, GivenProjectileAtLeo_WhenSystemUpdated_ThenAccelerationSeeded)
{
    std::vector<MassiveBodyState> bodies {makeEarth()};
    std::vector<ShipState> ships;
    std::vector<ProjectileState> projectiles;

    ProjectileState proj {};
    proj.transform.position = {kLeoRadius, 0.0, 0.0};
    projectiles.push_back(proj);

    system.update(bodies, ships, projectiles);

    EXPECT_LT(projectiles[0].acceleration.x, 0.0);
}

TEST_F(GravitySystemTest, GivenNoBodies_WhenGravityComputed_ThenZeroAcceleration)
{
    std::vector<MassiveBodyState> bodies;
    const spaceship::shared::Vec3 pos {kLeoRadius, 0.0, 0.0};

    const auto accel = computeGravitationalAcceleration(pos, bodies);

    EXPECT_DOUBLE_EQ(accel.x, 0.0);
    EXPECT_DOUBLE_EQ(accel.y, 0.0);
    EXPECT_DOUBLE_EQ(accel.z, 0.0);
}

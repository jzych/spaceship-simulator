#include "server/simulation/simulation_math.hpp"
#include "server/simulation/simulation_server.hpp"

#include <gtest/gtest.h>
#include <cmath>

// ---------------------------------------------------------------------------
// MassiveBodyMotionTest — full Sun/Earth/Moon world
// ---------------------------------------------------------------------------

class MassiveBodyMotionTest : public ::testing::Test
{
  protected:
    spaceship::server::SimulationServer server {};
};

TEST_F(MassiveBodyMotionTest, GivenSunAtOrigin_WhenManyTicksElapsed_ThenPositionUnchanged)
{
    for (int i = 0; i < 3600; ++i)
        server.tick();

    const auto& sun = server.world().massiveBodies[0];
    EXPECT_NEAR(sun.transform.position.x, 0.0, 1.0);
    EXPECT_NEAR(sun.transform.position.y, 0.0, 1.0);
    EXPECT_NEAR(sun.transform.position.z, 0.0, 1.0);
}

TEST_F(MassiveBodyMotionTest, GivenEarthInOrbit_WhenOneMinuteElapsed_ThenAngularPositionAdvances)
{
    // t=0: Earth at angle 0 (on +x axis)
    const double initialAngle =
        std::atan2(server.world().massiveBodies[1].transform.position.y,
                   server.world().massiveBodies[1].transform.position.x);

    for (int i = 0; i < 3600; ++i)
        server.tick();

    const auto& earth = server.world().massiveBodies[1];
    const double finalAngle = std::atan2(earth.transform.position.y, earth.transform.position.x);

    EXPECT_GT(finalAngle, initialAngle); // angle increases (counter-clockwise orbit)
}

TEST_F(MassiveBodyMotionTest, GivenEarthInOrbit_WhenOneMinuteElapsed_ThenOrbitalRadiusStable)
{
    const double initialRadius = spaceship::server::length(
        server.world().massiveBodies[1].transform.position);

    for (int i = 0; i < 3600; ++i)
        server.tick();

    const auto& earth = server.world().massiveBodies[1];
    const double finalRadius = spaceship::server::length(earth.transform.position);

    EXPECT_NEAR(finalRadius, initialRadius, 1.0); // radius stable to 1 m
}

TEST_F(MassiveBodyMotionTest, GivenMoonInOrbit_When3600TicksElapsed_ThenDistanceFromEarthStable)
{
    constexpr double kMoonDistanceMeters = 384'400'000.0;
    constexpr double kTolerance = kMoonDistanceMeters * 0.01; // 1%

    for (int i = 0; i < 3600; ++i)
        server.tick();

    const auto& earth = server.world().massiveBodies[1];
    const auto& moon = server.world().massiveBodies[2];
    const auto diff = spaceship::server::subtract(moon.transform.position, earth.transform.position);

    EXPECT_NEAR(spaceship::server::length(diff), kMoonDistanceMeters, kTolerance);
}

TEST_F(MassiveBodyMotionTest, GivenMoonInOrbit_When3600TicksElapsed_ThenAngularPositionRelativeToEarthAdvances)
{
    const auto& earth0 = server.world().massiveBodies[1];
    const auto& moon0 = server.world().massiveBodies[2];
    const auto diff0 = spaceship::server::subtract(moon0.transform.position, earth0.transform.position);
    const double initialAngle = std::atan2(diff0.y, diff0.x);

    for (int i = 0; i < 3600; ++i)
        server.tick();

    const auto& earth = server.world().massiveBodies[1];
    const auto& moon = server.world().massiveBodies[2];
    const auto diff = spaceship::server::subtract(moon.transform.position, earth.transform.position);
    const double finalAngle = std::atan2(diff.y, diff.x);

    EXPECT_GT(finalAngle, initialAngle);
}

TEST_F(MassiveBodyMotionTest, GivenEarthInOrbit_WhenOneTick_ThenVelocityIsTangentialToOrbit)
{
    server.tick();

    const auto& earth = server.world().massiveBodies[1];
    // Sun at origin → position IS the radius vector
    const double dotResult = spaceship::server::dot(earth.transform.position, earth.velocity.linear);
    const double posMag = spaceship::server::length(earth.transform.position);
    const double velMag = spaceship::server::length(earth.velocity.linear);

    EXPECT_NEAR(dotResult / (posMag * velMag), 0.0, 1e-6);
}

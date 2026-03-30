#include "server/simulation/integration_system.hpp"
#include "server/simulation/simulation_math.hpp"

#include <gtest/gtest.h>
#include <cmath>

using namespace spaceship::server;

// ---------------------------------------------------------------------------
// IntegrationSystemTest — direct Verlet integration unit tests
// ---------------------------------------------------------------------------

class IntegrationSystemTest : public ::testing::Test
{
  protected:
    IntegrationSystem system;
    SimulationConfig config {};
    std::vector<ShipState> ships;
    std::vector<ProjectileState> projectiles;

    ShipState makeShip(
        spaceship::shared::Vec3 pos,
        spaceship::shared::Vec3 vel,
        spaceship::shared::Vec3 accel = {},
        spaceship::shared::Vec3 thrustAccel = {})
    {
        ShipState ship {};
        ship.netId = 100;
        ship.transform.position = pos;
        ship.velocity.linear = vel;
        ship.acceleration = accel;
        ship.thrustAcceleration = thrustAccel;
        return ship;
    }
};

TEST_F(IntegrationSystemTest, GivenShipWithVelocity_WhenPositionsIntegrated_ThenPositionAdvances)
{
    ships.push_back(makeShip({0.0, 0.0, 0.0}, {10.0, 0.0, 0.0}));

    system.integratePositions(ships, projectiles, config);

    const double dt = config.fixedDeltaSeconds;
    EXPECT_NEAR(ships[0].transform.position.x, 10.0 * dt, 1e-12);
}

TEST_F(IntegrationSystemTest, GivenShipWithAcceleration_WhenPositionsIntegrated_ThenPositionIncludesHalfAtSquared)
{
    // x_{n+1} = x_n + v_n*dt + 0.5*a_n*dt²
    ships.push_back(makeShip({0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {10.0, 0.0, 0.0}));

    system.integratePositions(ships, projectiles, config);

    const double dt = config.fixedDeltaSeconds;
    EXPECT_NEAR(ships[0].transform.position.x, 0.5 * 10.0 * dt * dt, 1e-12);
}

TEST_F(IntegrationSystemTest, GivenGravityAndThrust_WhenPositionsIntegrated_ThenTotalAccelerationApplied)
{
    // Total accel = gravity + thrust
    ships.push_back(makeShip({0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {5.0, 0.0, 0.0}, {3.0, 0.0, 0.0}));

    system.integratePositions(ships, projectiles, config);

    const double dt = config.fixedDeltaSeconds;
    EXPECT_NEAR(ships[0].transform.position.x, 0.5 * 8.0 * dt * dt, 1e-12);
}

TEST_F(IntegrationSystemTest, GivenShipWithAcceleration_WhenPositionsIntegrated_ThenPreviousAccelerationSaved)
{
    ships.push_back(makeShip({0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {5.0, 3.0, 1.0}, {2.0, 0.0, 0.0}));

    system.integratePositions(ships, projectiles, config);

    // previousAcceleration = gravity + thrust at time of integration
    EXPECT_DOUBLE_EQ(ships[0].previousAcceleration.x, 7.0);
    EXPECT_DOUBLE_EQ(ships[0].previousAcceleration.y, 3.0);
    EXPECT_DOUBLE_EQ(ships[0].previousAcceleration.z, 1.0);
}

TEST_F(IntegrationSystemTest, GivenDifferentOldAndNewAcceleration_WhenVelocitiesIntegrated_ThenAverageAccelerationUsed)
{
    // v_{n+1} = v_n + 0.5*(a_n + a_{n+1})*dt
    ships.push_back(makeShip({0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}));
    ships[0].previousAcceleration = {10.0, 0.0, 0.0};  // a_n
    ships[0].acceleration = {20.0, 0.0, 0.0};           // a_{n+1} (gravity)
    ships[0].thrustAcceleration = {0.0, 0.0, 0.0};

    system.integrateVelocities(ships, projectiles, config);

    const double dt = config.fixedDeltaSeconds;
    // avg = 0.5 * (10 + 20) = 15
    EXPECT_NEAR(ships[0].velocity.linear.x, 15.0 * dt, 1e-12);
}


TEST_F(IntegrationSystemTest, GivenProjectileWithVelocity_WhenPositionsIntegrated_ThenPositionAdvances)
{
    projectiles.push_back(ProjectileState {10'000});
    projectiles[0].velocity.linear = {100.0, 0.0, 0.0};

    system.integratePositions(ships, projectiles, config);

    const double dt = config.fixedDeltaSeconds;
    EXPECT_NEAR(projectiles[0].transform.position.x, 100.0 * dt, 1e-9);
}

TEST_F(IntegrationSystemTest, GivenNoEntities_WhenIntegrated_ThenNoChange)
{
    EXPECT_NO_FATAL_FAILURE(system.integratePositions(ships, projectiles, config));
    EXPECT_NO_FATAL_FAILURE(system.integrateVelocities(ships, projectiles, config));
}

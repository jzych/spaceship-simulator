#include "server/tests/test_helpers.hpp"

#include "server/spawning/spawning_system.hpp"

using namespace spaceship::test;

// ---------------------------------------------------------------------------
// Gravity acceleration tests — using single Earth at origin
// ---------------------------------------------------------------------------

TEST_F(EarthOnlyCenteredTest, GivenShipSpawnedNearEarth_WhenNoTickNeeded_ThenAccelerationSeededFromGravity)
{
    // Gravity at spawn position is immediately available (no tick needed)
    spaceship::server::ShipSpawnRequest request {};
    request.transform.position = {kLeoRadius, 0.0, 0.0};
    server.spawnShip(request);

    const auto& ship = server.world().ships.front();
    EXPECT_LT(ship.acceleration.x, 0.0); // points toward Earth at origin
    EXPECT_NEAR(ship.acceleration.y, 0.0, 1e-6);
}

TEST_F(EarthOnlyCenteredTest, GivenCustomInitialAcceleration_WhenShipSpawned_ThenAccelerationOverridesGravity)
{
    const spaceship::shared::Vec3 kCustomAccel {1.0, 2.0, 3.0};
    spaceship::server::ShipSpawnRequest request {};
    request.transform.position = {kLeoRadius, 0.0, 0.0};
    request.initialAcceleration = kCustomAccel;
    server.spawnShip(request);

    const auto& ship = server.world().ships.front();
    EXPECT_DOUBLE_EQ(ship.acceleration.x, 1.0);
    EXPECT_DOUBLE_EQ(ship.acceleration.y, 2.0);
    EXPECT_DOUBLE_EQ(ship.acceleration.z, 3.0);
}

TEST_F(EarthOnlyCenteredTest, GivenShipAtLeo_WhenTicked_ThenGravityMagnitudeMatchesTwoBody)
{
    // g at LEO = μ / r² ≈ 8.67 m/s²
    const double expectedG = kEarthMu / (kLeoRadius * kLeoRadius);

    spaceship::server::ShipSpawnRequest request {};
    request.transform.position = {kLeoRadius, 0.0, 0.0};
    server.spawnShip(request);
    server.tick();

    const auto& ship = server.world().ships.front();
    const double aMag = spaceship::server::length(ship.acceleration);
    EXPECT_NEAR(aMag, expectedG, 0.01);
}

TEST_F(EarthOnlyCenteredTest, GivenShipOnPositiveXAxis_WhenTicked_ThenGravityPointsNegativeX)
{
    spaceship::server::ShipSpawnRequest request {};
    request.transform.position = {kLeoRadius, 0.0, 0.0};
    server.spawnShip(request);
    server.tick();

    const auto& ship = server.world().ships.front();
    EXPECT_LT(ship.acceleration.x, 0.0);           // points toward Earth at origin
    EXPECT_NEAR(ship.acceleration.y, 0.0, 1e-6);
    EXPECT_NEAR(ship.acceleration.z, 0.0, 1e-6);
}

TEST_F(EarthOnlyCenteredTest, GivenShipOnNegativeYAxis_WhenTicked_ThenGravityPointsPositiveY)
{
    spaceship::server::ShipSpawnRequest request {};
    request.transform.position = {0.0, -kLeoRadius, 0.0};
    server.spawnShip(request);
    server.tick();

    const auto& ship = server.world().ships.front();
    EXPECT_NEAR(ship.acceleration.x, 0.0, 1e-6);
    EXPECT_GT(ship.acceleration.y, 0.0);            // points toward Earth at origin
    EXPECT_NEAR(ship.acceleration.z, 0.0, 1e-6);
}

TEST_F(EarthOnlyCenteredTest, GivenProjectileNearEarth_WhenTicked_ThenGravityApplied)
{
    spaceship::server::ShipSpawnRequest request {};
    request.transform.position = {kLeoRadius, 0.0, 0.0};
    const auto shipId = server.spawnShip(request);
    server.updateShipControl(shipId, {0.0, {1.0, 0.0, 0.0, 0.0}, true});
    server.tick();

    ASSERT_EQ(server.world().projectiles.size(), 1U);
    const auto& proj = server.world().projectiles.front();
    const double aMag = spaceship::server::length(proj.acceleration);
    EXPECT_GT(aMag, 0.0);
}

TEST_F(EarthOnlyCenteredTest, GivenEarthAtOrigin_WhenTicked_ThenEarthVelocityUnchanged)
{
    spaceship::server::ShipSpawnRequest request {};
    request.transform.position = {kLeoRadius, 0.0, 0.0};
    server.spawnShip(request);
    server.tick();

    // Earth should have no acceleration field (OrbitalParams drives its motion)
    const auto& earth = server.world().massiveBodies[0];
    EXPECT_DOUBLE_EQ(earth.velocity.linear.x, 0.0); // Earth at origin has zero velocity
}

TEST_F(EarthOnlyCenteredTest, GivenShipNearEarth_WhenTicked_ThenNoGravityEvents)
{
    spaceship::server::ShipSpawnRequest request {};
    request.transform.position = {kLeoRadius, 0.0, 0.0};
    server.spawnShip(request);
    server.tick();

    EXPECT_TRUE(server.world().collisionEvents.empty());
}

TEST_F(EarthOnlyCenteredTest, GivenShipAtBodyCenter_WhenTicked_ThenNoCrash)
{
    // Ship placed exactly at Earth's center — division-by-zero guard
    spaceship::server::ShipSpawnRequest request {};
    request.transform.position = {0.0, 0.0, 0.0};
    server.spawnShip(request);
    EXPECT_NO_FATAL_FAILURE(server.tick());
}

TEST_F(EarthOnlyCenteredTest, GivenEarthAtOrigin_WhenManyTicksElapsed_ThenPositionUnchanged)
{
    for (int i = 0; i < 3600; ++i)
        server.tick();

    const auto& earth = server.world().massiveBodies[0];
    EXPECT_NEAR(earth.transform.position.x, 0.0, 1.0);
    EXPECT_NEAR(earth.transform.position.y, 0.0, 1.0);
    EXPECT_NEAR(earth.transform.position.z, 0.0, 1.0);
}

#include "server/simulation/gravity/gravity_system.hpp"
#include "server/simulation/simulation_math.hpp"
#include "server/spawning/spawning_system.hpp"

#include <gtest/gtest.h>

using namespace spaceship::server;

// ---------------------------------------------------------------------------
// SpawningSystemTest — direct unit tests for ship/projectile spawning
// ---------------------------------------------------------------------------

class SpawningSystemTest : public ::testing::Test
{
  protected:
    SpawningSystem system;
    SimulationConfig config {};
    std::vector<ShipState> ships;
    std::vector<ProjectileState> projectiles;
    std::vector<MassiveBodyState> massiveBodies;
};

TEST_F(SpawningSystemTest, GivenTwoSpawnCalls_WhenShipsSpawned_ThenIdsAreSequential)
{
    ShipSpawnRequest request {};
    request.transform.position = {100.0, 0.0, 0.0};

    const auto id1 = system.spawnShip(ships, request, config, massiveBodies);
    const auto id2 = system.spawnShip(ships, request, config, massiveBodies);

    EXPECT_EQ(id1, kFirstShipNetId);
    EXPECT_EQ(id2, kFirstShipNetId + 1);
    ASSERT_EQ(ships.size(), 2U);
}

TEST_F(SpawningSystemTest, GivenSpawnRequestWithTransform_WhenShipSpawned_ThenPositionAndVelocitySet)
{
    ShipSpawnRequest request {};
    request.transform.position = {1.0, 2.0, 3.0};
    request.velocity = {{4.0, 5.0, 6.0}};

    (void)system.spawnShip(ships, request, config, massiveBodies);

    ASSERT_EQ(ships.size(), 1U);
    EXPECT_DOUBLE_EQ(ships[0].transform.position.x, 1.0);
    EXPECT_DOUBLE_EQ(ships[0].transform.position.y, 2.0);
    EXPECT_DOUBLE_EQ(ships[0].velocity.linear.x, 4.0);
    EXPECT_DOUBLE_EQ(ships[0].velocity.linear.y, 5.0);
}

TEST_F(SpawningSystemTest, GivenDefaultConfig_WhenShipSpawned_ThenMassPropertiesFromConfig)
{
    ShipSpawnRequest request {};
    (void)system.spawnShip(ships, request, config, massiveBodies);

    EXPECT_DOUBLE_EQ(ships[0].massProperties.massKg, config.shipDefaultMassKg);
    EXPECT_NEAR(ships[0].massProperties.inverseMass, 1.0 / config.shipDefaultMassKg, 1e-15);
}

TEST_F(SpawningSystemTest, GivenBodyNearby_WhenShipSpawned_ThenAccelerationSeededFromGravity)
{
    // Place an Earth-like body at origin
    massiveBodies.push_back(MassiveBodyState {{0, "Earth", 3.986004418e14, 6.371e6}});

    ShipSpawnRequest request {};
    request.transform.position = {6.771e6, 0.0, 0.0};
    (void)system.spawnShip(ships, request, config, massiveBodies);

    // Acceleration should be seeded from gravity
    EXPECT_LT(ships[0].acceleration.x, 0.0);
}

TEST_F(SpawningSystemTest, GivenCustomInitialAcceleration_WhenShipSpawned_ThenGravityOverridden)
{
    massiveBodies.push_back(MassiveBodyState {{0, "Earth", 3.986004418e14, 6.371e6}});

    ShipSpawnRequest request {};
    request.transform.position = {6.771e6, 0.0, 0.0};
    request.initialAcceleration = spaceship::shared::Vec3 {99.0, 88.0, 77.0};
    (void)system.spawnShip(ships, request, config, massiveBodies);

    EXPECT_DOUBLE_EQ(ships[0].acceleration.x, 99.0);
    EXPECT_DOUBLE_EQ(ships[0].acceleration.y, 88.0);
    EXPECT_DOUBLE_EQ(ships[0].acceleration.z, 77.0);
}

TEST_F(SpawningSystemTest, GivenShipWithFireFlagSet_WhenSystemUpdated_ThenProjectileSpawned)
{
    ShipSpawnRequest request {};
    request.transform.position = {100.0, 0.0, 0.0};
    (void)system.spawnShip(ships, request, config, massiveBodies);

    ships[0].control.fire = true;
    ships[0].control.desiredOrientation = {1.0, 0.0, 0.0, 0.0};

    system.update(ships, projectiles, massiveBodies, config);

    ASSERT_EQ(projectiles.size(), 1U);
    EXPECT_EQ(projectiles[0].params.ownerNetId, ships[0].netId);
    EXPECT_FALSE(ships[0].control.fire); // fire flag cleared
}

TEST_F(SpawningSystemTest, GivenShipWithoutFireFlag_WhenSystemUpdated_ThenNoProjectileSpawned)
{
    ShipSpawnRequest request {};
    (void)system.spawnShip(ships, request, config, massiveBodies);
    ships[0].control.fire = false;

    system.update(ships, projectiles, massiveBodies, config);

    EXPECT_TRUE(projectiles.empty());
}

TEST_F(SpawningSystemTest, GivenShipWithVelocity_WhenProjectileSpawned_ThenVelocityIsShipPlusMuzzleSpeed)
{
    ShipSpawnRequest request {};
    request.velocity = {{10.0, 0.0, 0.0}};
    (void)system.spawnShip(ships, request, config, massiveBodies);
    ships[0].transform.orientation = {1.0, 0.0, 0.0, 0.0}; // forward = +X

    (void)system.spawnProjectile(projectiles, ships[0], config, massiveBodies);

    ASSERT_EQ(projectiles.size(), 1U);
    EXPECT_NEAR(projectiles[0].velocity.linear.x,
                10.0 + config.projectileMuzzleSpeedMetersPerSecond, 1e-9);
}

TEST_F(SpawningSystemTest, GivenDefaultConfig_WhenProjectileSpawned_ThenTtlFromConfig)
{
    ShipSpawnRequest request {};
    (void)system.spawnShip(ships, request, config, massiveBodies);

    (void)system.spawnProjectile(projectiles, ships[0], config, massiveBodies);

    EXPECT_DOUBLE_EQ(projectiles[0].params.ttlSeconds, config.projectileDefaultTtlSeconds);
}

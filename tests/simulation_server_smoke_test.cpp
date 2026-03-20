#include "server/simulation_server.hpp"

#include <gtest/gtest.h>

class SimulationServerSmokeTest : public ::testing::Test
{
  protected:
    spaceship::server::SimulationServer server {};
};

TEST_F(SimulationServerSmokeTest, BootstrapsThreeMassiveBodies)
{

    ASSERT_EQ(server.world().massiveBodies.size(), 3U);
    EXPECT_EQ(server.world().massiveBodies[0].definition.name, "Sun");
    EXPECT_EQ(server.world().massiveBodies[1].definition.name, "Earth");
    EXPECT_EQ(server.world().massiveBodies[2].definition.name, "Moon");
    EXPECT_GT(server.world().massiveBodies[0].definition.muMetersCubedPerSecondSquared, 0.0);
    EXPECT_GT(server.world().massiveBodies[1].definition.radiusMeters, 0.0);
    EXPECT_DOUBLE_EQ(server.world().massiveBodies[0].transform.orientation.w, 1.0);
    EXPECT_NE(server.world().massiveBodies[1].velocity.linear.y, 0.0);
    EXPECT_NE(server.world().massiveBodies[2].velocity.linear.y, 0.0);
}

TEST_F(SimulationServerSmokeTest, TickAdvancesServerClock)
{
    server.tick();

    EXPECT_EQ(server.tickCount(), 1U);
    EXPECT_TRUE(server.lastSnapshotSummary().empty());
}

TEST_F(SimulationServerSmokeTest, SpawnShipAddsShipToWorld)
{
    constexpr spaceship::shared::NetId kExpectedFirstShipNetId = spaceship::server::kFirstShipNetId;

    const spaceship::server::ShipSpawnRequest request {
        {{10.0, 20.0, 30.0}, {1.0, 0.0, 0.0, 0.0}},
        {{100.0, 200.0, 300.0}},
    };

    const auto shipNetId = server.spawnShip(request);

    ASSERT_EQ(server.world().ships.size(), 1U);
    const auto& ship = server.world().ships.back();
    EXPECT_EQ(shipNetId, kExpectedFirstShipNetId);
    EXPECT_EQ(ship.netId, kExpectedFirstShipNetId);
    EXPECT_DOUBLE_EQ(ship.transform.position.x, 10.0);
    EXPECT_DOUBLE_EQ(ship.transform.position.y, 20.0);
    EXPECT_DOUBLE_EQ(ship.velocity.linear.z, 300.0);
    EXPECT_DOUBLE_EQ(ship.massProperties.massKg, 1'000.0);
}

TEST_F(SimulationServerSmokeTest, FireProjectileCreatesProjectileFromShipState)
{
    const spaceship::server::ShipSpawnRequest request {
        {{1.0, 2.0, 3.0}, {1.0, 0.0, 0.0, 0.0}},
        {{10.0, 20.0, 30.0}},
    };

    const auto shipNetId = server.spawnShip(request);
    const auto projectileNetId = server.fireProjectile(shipNetId);

    ASSERT_TRUE(projectileNetId.has_value());
    ASSERT_EQ(server.world().projectiles.size(), 1U);
    const auto& projectile = server.world().projectiles.back();
    EXPECT_EQ(projectileNetId.value(), projectile.netId);
    EXPECT_EQ(projectile.params.ownerNetId, shipNetId);
    EXPECT_DOUBLE_EQ(projectile.transform.position.x, 1.0);
    EXPECT_DOUBLE_EQ(projectile.transform.orientation.w, 1.0);
    EXPECT_DOUBLE_EQ(projectile.velocity.linear.x, 1'010.0);
    EXPECT_DOUBLE_EQ(projectile.velocity.linear.y, 20.0);
    EXPECT_DOUBLE_EQ(projectile.params.ttlSeconds, 10.0);
}

TEST_F(SimulationServerSmokeTest, SpawnShipAssignsSequentialShipIds)
{
    constexpr spaceship::shared::NetId kExpectedFirstShipNetId = spaceship::server::kFirstShipNetId;
    constexpr spaceship::shared::NetId kExpectedSecondShipNetId = spaceship::server::kFirstShipNetId + 1U;

    const spaceship::server::ShipSpawnRequest request {
        {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0, 0.0}},
        {{0.0, 0.0, 0.0}},
    };

    const auto firstShipNetId = server.spawnShip(request);
    const auto secondShipNetId = server.spawnShip(request);

    EXPECT_EQ(firstShipNetId, kExpectedFirstShipNetId);
    EXPECT_EQ(secondShipNetId, kExpectedSecondShipNetId);
    ASSERT_EQ(server.world().ships.size(), 2U);
    EXPECT_EQ(server.world().ships[0].netId, kExpectedFirstShipNetId);
    EXPECT_EQ(server.world().ships[1].netId, kExpectedSecondShipNetId);
}

TEST_F(SimulationServerSmokeTest, FireProjectileReturnsEmptyWhenShipDoesNotExist)
{
    constexpr spaceship::shared::NetId kMissingShipNetId = 99U;

    const auto projectileNetId = server.fireProjectile(kMissingShipNetId);

    EXPECT_FALSE(projectileNetId.has_value());
    EXPECT_TRUE(server.world().projectiles.empty());
}

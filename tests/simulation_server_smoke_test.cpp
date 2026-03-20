#include "server/simulation_server.hpp"

#include <gtest/gtest.h>

TEST(SimulationServerSmokeTest, BootstrapsThreeMassiveBodies)
{
    spaceship::server::SimulationServer server;

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

TEST(SimulationServerSmokeTest, TickAdvancesServerClock)
{
    spaceship::server::SimulationServer server;

    server.tick();

    EXPECT_EQ(server.tickCount(), 1U);
    EXPECT_TRUE(server.lastSnapshotSummary().empty());
}

TEST(SimulationServerSmokeTest, SpawnShipAddsShipToWorld)
{
    spaceship::server::SimulationServer server;

    const spaceship::server::ShipSpawnRequest request {
        42U,
        {{10.0, 20.0, 30.0}, {1.0, 0.0, 0.0, 0.0}},
        {{100.0, 200.0, 300.0}},
    };

    const auto& ship = server.spawnShip(request);

    ASSERT_EQ(server.world().ships.size(), 1U);
    EXPECT_EQ(ship.netId, 42U);
    EXPECT_DOUBLE_EQ(ship.transform.position.x, 10.0);
    EXPECT_DOUBLE_EQ(ship.transform.position.y, 20.0);
    EXPECT_DOUBLE_EQ(ship.velocity.linear.z, 300.0);
    EXPECT_DOUBLE_EQ(ship.massProperties.massKg, 1'000.0);
}

TEST(SimulationServerSmokeTest, FireProjectileCreatesProjectileFromShipState)
{
    spaceship::server::SimulationServer server;

    const spaceship::server::ShipSpawnRequest request {
        7U,
        {{1.0, 2.0, 3.0}, {1.0, 0.0, 0.0, 0.0}},
        {{10.0, 20.0, 30.0}},
    };

    server.spawnShip(request);
    const auto* projectile = server.fireProjectile(7U);

    ASSERT_NE(projectile, nullptr);
    ASSERT_EQ(server.world().projectiles.size(), 1U);
    EXPECT_EQ(projectile->params.ownerNetId, 7U);
    EXPECT_DOUBLE_EQ(projectile->transform.position.x, 1.0);
    EXPECT_DOUBLE_EQ(projectile->transform.orientation.w, 1.0);
    EXPECT_DOUBLE_EQ(projectile->velocity.linear.x, 1'010.0);
    EXPECT_DOUBLE_EQ(projectile->velocity.linear.y, 20.0);
    EXPECT_DOUBLE_EQ(projectile->params.ttlSeconds, 10.0);
}

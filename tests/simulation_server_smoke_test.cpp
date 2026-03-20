#include "server/simulation_server.hpp"

#include <gtest/gtest.h>
#include <numbers>

namespace
{

constexpr double kDefaultShipAccelerationMetersPerSecondSquared = 20'000.0 / 1'000.0;
constexpr double kExpectedVelocityDeltaPerTick =
    kDefaultShipAccelerationMetersPerSecondSquared * spaceship::shared::constants::kFixedDeltaSeconds;
constexpr double kQuarterTurnZAxisHalfAngleComponent = std::numbers::sqrt2_v<double> / 2.0;
constexpr double kDefaultProjectileMuzzleSpeedMetersPerSecond = 1'000.0;

} // namespace

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

TEST_F(SimulationServerSmokeTest, UpdateShipControlReturnsFalseWhenShipDoesNotExist)
{
    constexpr spaceship::shared::NetId kMissingShipNetId = 99U;

    const bool updated =
        server.updateShipControl(kMissingShipNetId, spaceship::shared::ShipControl {1.0, {1.0, 0.0, 0.0, 0.0}, true});

    EXPECT_FALSE(updated);
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

TEST_F(SimulationServerSmokeTest, SpawnShipAllowsDuplicateStateButKeepsUniqueAssignedIds)
{
    const spaceship::server::ShipSpawnRequest request {
        {{5.0, 6.0, 7.0}, {1.0, 0.0, 0.0, 0.0}},
        {{8.0, 9.0, 10.0}},
    };

    const auto firstShipNetId = server.spawnShip(request);
    const auto secondShipNetId = server.spawnShip(request);

    ASSERT_EQ(server.world().ships.size(), 2U);
    EXPECT_NE(firstShipNetId, secondShipNetId);
    EXPECT_EQ(server.world().ships[0].transform.position.x, server.world().ships[1].transform.position.x);
    EXPECT_EQ(server.world().ships[0].velocity.linear.y, server.world().ships[1].velocity.linear.y);
}

TEST_F(SimulationServerSmokeTest, UpdateShipControlReplacesExistingControlState)
{
    const spaceship::server::ShipSpawnRequest request {
        {{1.0, 2.0, 3.0}, {1.0, 0.0, 0.0, 0.0}},
        {{4.0, 5.0, 6.0}},
    };
    const spaceship::shared::ShipControl control {0.75, {0.0, 0.0, 1.0, 0.0}, true};

    const auto shipNetId = server.spawnShip(request);
    const bool updated = server.updateShipControl(shipNetId, control);

    ASSERT_TRUE(updated);
    ASSERT_EQ(server.world().ships.size(), 1U);
    const auto& ship = server.world().ships.front();
    EXPECT_DOUBLE_EQ(ship.control.throttle, 0.75);
    EXPECT_DOUBLE_EQ(ship.control.desiredOrientation.y, 1.0);
    EXPECT_TRUE(ship.control.fire);
}

TEST_F(SimulationServerSmokeTest, TickAppliesForwardThrustToShipVelocity)
{
    const spaceship::server::ShipSpawnRequest request {
        {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0, 0.0}},
        {{0.0, 0.0, 0.0}},
    };

    const auto shipNetId = server.spawnShip(request);
    ASSERT_TRUE(server.updateShipControl(
        shipNetId,
        spaceship::shared::ShipControl {1.0, {1.0, 0.0, 0.0, 0.0}, false}));

    server.tick();

    ASSERT_EQ(server.world().ships.size(), 1U);
    const auto& ship = server.world().ships.front();
    EXPECT_NEAR(ship.velocity.linear.x, kExpectedVelocityDeltaPerTick, 1e-9);
    EXPECT_DOUBLE_EQ(ship.velocity.linear.y, 0.0);
    EXPECT_DOUBLE_EQ(ship.velocity.linear.z, 0.0);
}

TEST_F(SimulationServerSmokeTest, TickAppliesDesiredOrientationBeforeThrust)
{
    const spaceship::server::ShipSpawnRequest request {
        {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0, 0.0}},
        {{0.0, 0.0, 0.0}},
    };

    const auto shipNetId = server.spawnShip(request);
    ASSERT_TRUE(server.updateShipControl(
        shipNetId,
        spaceship::shared::ShipControl {
            1.0,
            {kQuarterTurnZAxisHalfAngleComponent, 0.0, 0.0, kQuarterTurnZAxisHalfAngleComponent},
            false,
        }));

    server.tick();

    ASSERT_EQ(server.world().ships.size(), 1U);
    const auto& ship = server.world().ships.front();
    EXPECT_NEAR(ship.transform.orientation.w, kQuarterTurnZAxisHalfAngleComponent, 1e-12);
    EXPECT_NEAR(ship.velocity.linear.x, 0.0, 1e-9);
    EXPECT_NEAR(ship.velocity.linear.y, kExpectedVelocityDeltaPerTick, 1e-9);
}

TEST_F(SimulationServerSmokeTest, TickDrivenFireSpawnsProjectileAndClearsFireFlag)
{
    constexpr spaceship::shared::NetId kExpectedFirstProjectileNetId = spaceship::server::kFirstProjectileNetId;
    const spaceship::server::ShipSpawnRequest request {
        {{1.0, 2.0, 3.0}, {1.0, 0.0, 0.0, 0.0}},
        {{10.0, 20.0, 30.0}},
    };
    constexpr double kExpectedShipVelocityX = 10.0 + kExpectedVelocityDeltaPerTick;
    constexpr double kExpectedProjectileVelocityX =
        kExpectedShipVelocityX + kDefaultProjectileMuzzleSpeedMetersPerSecond;

    const auto shipNetId = server.spawnShip(request);
    ASSERT_TRUE(server.updateShipControl(
        shipNetId,
        spaceship::shared::ShipControl {1.0, {1.0, 0.0, 0.0, 0.0}, true}));

    EXPECT_TRUE(server.world().projectiles.empty());

    server.tick();

    ASSERT_EQ(server.world().projectiles.size(), 1U);
    ASSERT_EQ(server.world().ships.size(), 1U);
    const auto& projectile = server.world().projectiles.front();
    const auto& ship = server.world().ships.front();
    EXPECT_EQ(projectile.netId, kExpectedFirstProjectileNetId);
    EXPECT_EQ(projectile.params.ownerNetId, shipNetId);
    EXPECT_DOUBLE_EQ(projectile.transform.position.x, 1.0);
    EXPECT_DOUBLE_EQ(projectile.transform.orientation.w, 1.0);
    EXPECT_NEAR(projectile.velocity.linear.x, kExpectedProjectileVelocityX, 1e-9);
    EXPECT_DOUBLE_EQ(projectile.velocity.linear.y, 20.0);
    EXPECT_FALSE(ship.control.fire);

    server.tick();

    EXPECT_EQ(server.world().projectiles.size(), 1U);
}

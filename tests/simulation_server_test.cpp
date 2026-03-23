#include "test_helpers.hpp"

#include "server/spawning_system.hpp"

// ---------------------------------------------------------------------------
// SimulationServerSmokeTest — basic API tests, full world, no integration checks
// ---------------------------------------------------------------------------

class SimulationServerSmokeTest : public ::testing::Test
{
  protected:
    spaceship::server::SimulationServer server {};
};

TEST_F(SimulationServerSmokeTest, GivenDefaultWorld_WhenCreated_ThenThreeMassiveBodiesPresent)
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

TEST_F(SimulationServerSmokeTest, GivenFreshServer_WhenTicked_ThenClockAdvances)
{
    server.tick();

    EXPECT_EQ(server.tickCount(), 1U);
    EXPECT_TRUE(server.lastSnapshotSummary().empty());
}

TEST_F(SimulationServerSmokeTest, GivenSpawnRequest_WhenShipSpawned_ThenShipAddedToWorld)
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

TEST_F(SimulationServerSmokeTest, GivenUnknownShipId_WhenControlUpdated_ThenNoError)
{
    constexpr spaceship::shared::NetId kMissingShipNetId = 99U;
    // Stale/out-of-order packets for despawned ships must be silently discarded.
    EXPECT_NO_FATAL_FAILURE(
        server.updateShipControl(kMissingShipNetId, spaceship::shared::ShipControl {1.0, {1.0, 0.0, 0.0, 0.0}, true}));
}

TEST_F(SimulationServerSmokeTest, GivenTwoSpawnRequests_WhenShipsSpawned_ThenIdsAreSequential)
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

TEST_F(SimulationServerSmokeTest, GivenIdenticalSpawnRequests_WhenShipsSpawned_ThenIdsAreUnique)
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

TEST_F(SimulationServerSmokeTest, GivenShipWithExistingControl_WhenControlUpdated_ThenControlReplaced)
{
    const spaceship::server::ShipSpawnRequest request {
        {{1.0, 2.0, 3.0}, {1.0, 0.0, 0.0, 0.0}},
        {{4.0, 5.0, 6.0}},
    };
    const spaceship::shared::ShipControl control {0.75, {0.0, 0.0, 1.0, 0.0}, true};

    const auto shipNetId = server.spawnShip(request);
    server.updateShipControl(shipNetId, control);

    ASSERT_EQ(server.world().ships.size(), 1U);
    const auto& ship = server.world().ships.front();
    EXPECT_DOUBLE_EQ(ship.control.throttle, 0.75);
    EXPECT_DOUBLE_EQ(ship.control.desiredOrientation.y, 1.0);
    EXPECT_TRUE(ship.control.fire);
}

TEST_F(SimulationServerSmokeTest, GivenServerTickedAtInterval_WhenSnapshotIntervalReached_ThenSummaryGenerated)
{
    // Default snapshotIntervalTicks = 3
    server.tick(); // tick 1
    EXPECT_TRUE(server.lastSnapshotSummary().empty());

    server.tick(); // tick 2
    EXPECT_TRUE(server.lastSnapshotSummary().empty());

    server.tick(); // tick 3 — snapshot should fire
    EXPECT_FALSE(server.lastSnapshotSummary().empty());
    EXPECT_NE(server.lastSnapshotSummary().find("bodies=3"), std::string::npos);
}

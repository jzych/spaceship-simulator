#include "server/tests/test_helpers.hpp"

#include "server/spawning/spawning_system.hpp"

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
    // No snapshot has fired yet (interval = 3 ticks) — serverTick stays at default 0.
    EXPECT_EQ(server.lastSnapshot().serverTick, 0U);
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
    EXPECT_EQ(server.lastSnapshot().serverTick, 0U);  // snapshot not yet fired

    server.tick(); // tick 2
    EXPECT_EQ(server.lastSnapshot().serverTick, 0U);

    server.tick(); // tick 3 — snapshot should fire
    EXPECT_EQ(server.lastSnapshot().serverTick, 3U);
    EXPECT_EQ(server.lastSnapshot().massiveBodies.size(), 3U);
}

TEST(SimulationServerSnapshotTest, GivenSnapshotEveryTick_WhenFixedStepServerTicked_ThenSnapshotTimeMatchesWorldState)
{
    spaceship::server::SimulationConfig cfg;
    cfg.snapshotIntervalTicks = 1U;

    spaceship::server::SimulationServer srv {cfg};
    const double initialEarthY = srv.world().massiveBodies[1].transform.position.y;

    srv.tick();

    const auto& snapshot = srv.lastSnapshot();
    ASSERT_EQ(snapshot.serverTick, 1U);
    ASSERT_EQ(snapshot.massiveBodies.size(), 3U);
    EXPECT_DOUBLE_EQ(snapshot.elapsedSeconds, cfg.fixedDeltaSeconds);
    EXPECT_GT(snapshot.massiveBodies[1].position.y, initialEarthY);
    EXPECT_DOUBLE_EQ(snapshot.massiveBodies[1].position.y, srv.world().massiveBodies[1].transform.position.y);
}

TEST(SimulationServerSnapshotTest, GivenSnapshotDisabled_WhenServerTicked_ThenLastSnapshotRemainsUnset)
{
    spaceship::server::SimulationConfig cfg;
    cfg.snapshotIntervalTicks = 0U;

    spaceship::server::SimulationServer srv {cfg};

    srv.tick();
    srv.tick();

    EXPECT_EQ(srv.tickCount(), 2U);
    EXPECT_EQ(srv.lastSnapshot().serverTick, 0U);
    EXPECT_TRUE(srv.lastSnapshot().massiveBodies.empty());
    EXPECT_TRUE(srv.lastSnapshot().ships.empty());
    EXPECT_TRUE(srv.lastSnapshot().projectiles.empty());
}

TEST(SimulationServerSnapshotTest, GivenAdaptiveTimestep_WhenSnapshotIntervalReached_ThenSnapshotCaptured)
{
    spaceship::server::SimulationConfig cfg;
    cfg.useAdaptiveTimestep   = true;
    cfg.snapshotIntervalTicks = 1U;

    spaceship::server::SimulationServer srv {cfg};
    const double initialEarthY = srv.world().massiveBodies[1].transform.position.y;

    srv.tick();

    const auto& snapshot = srv.lastSnapshot();
    ASSERT_EQ(snapshot.serverTick, 1U);
    ASSERT_EQ(snapshot.massiveBodies.size(), 3U);
    EXPECT_DOUBLE_EQ(snapshot.elapsedSeconds, cfg.fixedDeltaSeconds);
    EXPECT_GT(snapshot.massiveBodies[1].position.y, initialEarthY);
    EXPECT_DOUBLE_EQ(snapshot.massiveBodies[1].position.y, srv.world().massiveBodies[1].transform.position.y);
}

TEST(SimulationServerSnapshotTest, GivenAdaptiveProjectile_WhenTicked_ThenProjectileSnapshotAndDiagnosticsCaptured)
{
    spaceship::server::SimulationConfig cfg;
    cfg.useAdaptiveTimestep   = true;
    cfg.snapshotIntervalTicks = 1U;

    spaceship::server::SimulationServer srv {spaceship::server::SimulationWorld {}, cfg};

    spaceship::server::ProjectileState projectile;
    projectile.netId                 = 10'000U;
    projectile.transform.position    = {10.0, 0.0, 0.0};
    projectile.velocity.linear       = {0.0, 50.0, 0.0};
    projectile.collider.radiusMeters = cfg.projectileRadiusMeters;
    projectile.massProperties        = {cfg.projectileMassKg, 1.0 / cfg.projectileMassKg};
    projectile.params                = {10.0, 42U};
    srv.injectProjectile(projectile);

    srv.tick();

    const auto& snapshot = srv.lastSnapshot();
    ASSERT_EQ(snapshot.serverTick, 1U);
    ASSERT_EQ(snapshot.projectiles.size(), 1U);
    EXPECT_EQ(snapshot.projectiles[0].netId, projectile.netId);
    EXPECT_EQ(snapshot.projectiles[0].ownerNetId, projectile.params.ownerNetId);

    const auto& diagnostics = srv.timestepDiagnostics();
    ASSERT_EQ(diagnostics.size(), 1U);
    EXPECT_EQ(diagnostics[0].netId, projectile.netId);
    EXPECT_GT(diagnostics[0].dtApplied, 0.0);
    EXPECT_GE(diagnostics[0].substepCount, 1);

    ASSERT_EQ(srv.world().projectiles.size(), 1U);
    EXPECT_DOUBLE_EQ(srv.world().projectiles[0].timestepState.dtPrev, diagnostics[0].dtApplied);
}

// ---------------------------------------------------------------------------
// Ship despawn + shipIndex maintenance
// ---------------------------------------------------------------------------

TEST_F(SimulationServerSmokeTest,
       GivenShipDespawnedByCollision_WhenUpdateShipControlCalledWithStaleId_ThenNoError)
{
    // Use an empty world (no massive bodies) so gravity doesn't pull ships.
    spaceship::server::SimulationServer zeroGravServer {spaceship::server::SimulationWorld {}};

    // Spawn two ships at the same position (already overlapping → toi=0 → both despawned
    // if E_rel > shipShipDestroyBothEnergyJoules, but here they're stationary so E_rel=0).
    // Instead, spawn one ship at a position already inside a "fake" projectile.
    // Easier approach: spawn a ship and a fast projectile that will definitely destroy the ship.
    spaceship::server::SimulationConfig cfg;
    cfg.shipProjectileDestroyShipEnergyJoules = 0.0;  // always destroy ship on any hit

    spaceship::server::SimulationServer srv {spaceship::server::SimulationWorld {}, cfg};

    // Ship (identity orientation → forward = +x) at origin, stationary
    const auto shipId = srv.spawnShip(spaceship::server::ShipSpawnRequest {
        {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0, 0.0}}, {{0.0, 0.0, 0.0}}});

    // Position projectile just inside the ship hull so toi=0 (already overlapping)
    spaceship::server::ProjectileState proj;
    proj.netId = 10'000U;
    proj.params = {10.0, shipId};
    proj.transform.position = {0.0, 0.0, 0.0};   // same as ship → overlapping → toi=0
    proj.velocity.linear    = {-1.0, 0.0, 0.0};  // nonzero relative velocity → E_rel > 0 > threshold
    proj.collider.radiusMeters = cfg.projectileRadiusMeters;
    proj.massProperties = {cfg.projectileMassKg, 1.0 / cfg.projectileMassKg};
    srv.injectProjectile(proj);

    srv.tick();

    // Ship should be despawned (E_rel > 0 > shipProjectileDestroyShipEnergyJoules=0)
    EXPECT_TRUE(srv.world().ships.empty());
    // Stale control update for the now-despawned ship must not crash or assert
    EXPECT_NO_FATAL_FAILURE(
        srv.updateShipControl(shipId, spaceship::shared::ShipControl {1.0, {1.0, 0.0, 0.0, 0.0}, false}));
}

TEST_F(SimulationServerSmokeTest,
       GivenShipDespawnedByCollision_WhenSecondShipStillPresent_ThenShipIndexRemainsCorrect)
{
    spaceship::server::SimulationConfig cfg;
    cfg.shipProjectileDestroyShipEnergyJoules = 0.0;

    spaceship::server::SimulationServer srv {spaceship::server::SimulationWorld {}, cfg};

    const auto shipIdA = srv.spawnShip(spaceship::server::ShipSpawnRequest {
        {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0, 0.0}}, {{0.0, 0.0, 0.0}}});
    const auto shipIdB = srv.spawnShip(spaceship::server::ShipSpawnRequest {
        {{1000.0, 0.0, 0.0}, {1.0, 0.0, 0.0, 0.0}}, {{0.0, 0.0, 0.0}}});

    // Add projectile overlapping ship A only
    spaceship::server::ProjectileState proj;
    proj.netId = 10'000U;
    proj.params = {10.0, shipIdA};
    proj.transform.position = {0.0, 0.0, 0.0};
    proj.velocity.linear    = {-1.0, 0.0, 0.0};  // nonzero relative velocity → E_rel > 0
    proj.collider.radiusMeters = cfg.projectileRadiusMeters;
    proj.massProperties = {cfg.projectileMassKg, 1.0 / cfg.projectileMassKg};
    srv.injectProjectile(proj);

    srv.tick();

    // Ship A should be despawned; ship B should remain
    ASSERT_EQ(srv.world().ships.size(), 1U);
    EXPECT_EQ(srv.world().ships[0].netId, shipIdB);

    // Control update for surviving ship B must succeed
    EXPECT_NO_FATAL_FAILURE(
        srv.updateShipControl(shipIdB, spaceship::shared::ShipControl {1.0, {1.0, 0.0, 0.0, 0.0}, false}));
    EXPECT_DOUBLE_EQ(srv.world().ships[0].control.throttle, 1.0);
}

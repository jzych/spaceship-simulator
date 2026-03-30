#include "server/snapshot/snapshot_system.hpp"
#include "server/snapshot/snapshot_types.hpp"

#include <gtest/gtest.h>

using namespace spaceship::server;
using namespace spaceship::shared;

// ---------------------------------------------------------------------------
// SnapshotSystemTest — unit tests for structured snapshot building
// ---------------------------------------------------------------------------

TEST(SnapshotSystemTest, GivenEmptyWorld_WhenSnapshotBuilt_ThenAllVectorsAreEmpty)
{
    SnapshotSystem system;
    SimulationWorld world {};

    const auto snapshot = system.buildSnapshot(world, /*serverTick=*/0, /*elapsedSeconds=*/0.0);

    EXPECT_TRUE(snapshot.massiveBodies.empty());
    EXPECT_TRUE(snapshot.ships.empty());
    EXPECT_TRUE(snapshot.projectiles.empty());
    EXPECT_TRUE(snapshot.collisionEvents.empty());
}

TEST(SnapshotSystemTest, GivenWorldWithEntities_WhenSnapshotBuilt_ThenCountsMatchContents)
{
    SnapshotSystem system;
    SimulationWorld world {};

    world.massiveBodies.push_back(MassiveBodyState {{0, "Sun", 1e20, 1e8}});
    world.massiveBodies.push_back(MassiveBodyState {{1, "Earth", 1e14, 1e6}});
    world.ships.push_back(ShipState {100});
    world.ships.push_back(ShipState {101});
    world.ships.push_back(ShipState {102});

    const auto snapshot = system.buildSnapshot(world, 5, 0.1);

    EXPECT_EQ(snapshot.massiveBodies.size(), 2U);
    EXPECT_EQ(snapshot.ships.size(), 3U);
    EXPECT_EQ(snapshot.projectiles.size(), 0U);
}

TEST(SnapshotSystemTest, GivenWorldWithProjectiles_WhenSnapshotBuilt_ThenProjectilesCountedCorrectly)
{
    SnapshotSystem system;
    SimulationWorld world {};

    world.projectiles.push_back(ProjectileState {10'000});
    world.projectiles.push_back(ProjectileState {10'001});

    const auto snapshot = system.buildSnapshot(world, 1, 0.0);

    EXPECT_EQ(snapshot.projectiles.size(), 2U);
}

TEST(SnapshotSystemTest, GivenTickAndElapsed_WhenSnapshotBuilt_ThenTickAndElapsedAreCaptured)
{
    SnapshotSystem system;
    SimulationWorld world {};

    const auto snapshot = system.buildSnapshot(world, /*serverTick=*/42, /*elapsedSeconds=*/0.7);

    EXPECT_EQ(snapshot.serverTick, 42U);
    EXPECT_DOUBLE_EQ(snapshot.elapsedSeconds, 0.7);
}

TEST(SnapshotSystemTest, GivenWorldWithOneMassiveBody_WhenSnapshotBuilt_ThenMassiveBodyFieldsAreCopied)
{
    SnapshotSystem system;
    SimulationWorld world {};

    MassiveBodyState body {};
    body.definition.netId        = 1U;
    body.definition.radiusMeters = 6.371e6;
    body.transform.position      = {1.5e11, 0.0, 0.0};
    body.velocity.linear         = {0.0, 29'783.0, 0.0};
    world.massiveBodies.push_back(body);

    const auto snapshot = system.buildSnapshot(world, 1, 0.0);

    ASSERT_EQ(snapshot.massiveBodies.size(), 1U);
    const auto& mb = snapshot.massiveBodies[0];
    EXPECT_EQ(mb.netId, 1U);
    EXPECT_DOUBLE_EQ(mb.radiusMeters, 6.371e6);
    EXPECT_DOUBLE_EQ(mb.position.x, 1.5e11);
    EXPECT_DOUBLE_EQ(mb.velocity.y, 29'783.0);
}

TEST(SnapshotSystemTest, GivenWorldWithOneShip_WhenSnapshotBuilt_ThenShipFieldsAreCopied)
{
    SnapshotSystem system;
    SimulationWorld world {};

    ShipState ship {};
    ship.netId                    = 100U;
    ship.transform.position       = {1.0, 2.0, 3.0};
    ship.velocity.linear          = {4.0, 5.0, 6.0};
    ship.transform.orientation    = {0.707, 0.0, 0.707, 0.0};
    ship.collider.radiusMeters    = 12.5;
    ship.control.throttle         = 0.75;
    world.ships.push_back(ship);

    const auto snapshot = system.buildSnapshot(world, 1, 0.0);

    ASSERT_EQ(snapshot.ships.size(), 1U);
    const auto& ss = snapshot.ships[0];
    EXPECT_EQ(ss.netId, 100U);
    EXPECT_DOUBLE_EQ(ss.position.x, 1.0);
    EXPECT_DOUBLE_EQ(ss.position.y, 2.0);
    EXPECT_DOUBLE_EQ(ss.position.z, 3.0);
    EXPECT_DOUBLE_EQ(ss.velocity.x, 4.0);
    EXPECT_DOUBLE_EQ(ss.velocity.y, 5.0);
    EXPECT_DOUBLE_EQ(ss.velocity.z, 6.0);
    EXPECT_DOUBLE_EQ(ss.orientation.w, 0.707);
    EXPECT_DOUBLE_EQ(ss.orientation.y, 0.707);
    EXPECT_DOUBLE_EQ(ss.radiusMeters, 12.5);
    EXPECT_DOUBLE_EQ(ss.throttle, 0.75);
}

TEST(SnapshotSystemTest, GivenWorldWithOneProjectile_WhenSnapshotBuilt_ThenProjectileFieldsAreCopied)
{
    SnapshotSystem system;
    SimulationWorld world {};

    ProjectileState proj {};
    proj.netId                 = 10'000U;
    proj.transform.position    = {7.0, 8.0, 9.0};
    proj.velocity.linear       = {-1.0, 0.0, 0.0};
    proj.collider.radiusMeters = 0.05;
    proj.params.ownerNetId     = 100U;
    world.projectiles.push_back(proj);

    const auto snapshot = system.buildSnapshot(world, 1, 0.0);

    ASSERT_EQ(snapshot.projectiles.size(), 1U);
    const auto& ps = snapshot.projectiles[0];
    EXPECT_EQ(ps.netId, 10'000U);
    EXPECT_DOUBLE_EQ(ps.position.x, 7.0);
    EXPECT_DOUBLE_EQ(ps.velocity.x, -1.0);
    EXPECT_DOUBLE_EQ(ps.radiusMeters, 0.05);
    EXPECT_EQ(ps.ownerNetId, 100U);
}

TEST(SnapshotSystemTest, GivenWorldWithCollisionEvent_WhenSnapshotBuilt_ThenCollisionEventCopied)
{
    SnapshotSystem system;
    SimulationWorld world {};

    CollisionEvent event {};
    event.netIdA      = 100U;
    event.netIdB      = 10'000U;
    event.kindA       = EntityKind::Ship;
    event.kindB       = EntityKind::Projectile;
    event.eRelJoules  = 500.0;
    event.outcome     = CollisionOutcome::ADespawned;
    world.collisionEvents.push_back(event);

    const auto snapshot = system.buildSnapshot(world, 1, 0.0);

    ASSERT_EQ(snapshot.collisionEvents.size(), 1U);
    const auto& ce = snapshot.collisionEvents[0];
    EXPECT_EQ(ce.netIdA, 100U);
    EXPECT_EQ(ce.netIdB, 10'000U);
    EXPECT_DOUBLE_EQ(ce.eRelJoules, 500.0);
    EXPECT_EQ(ce.outcome, CollisionOutcome::ADespawned);
}

TEST(SnapshotSystemTest, GivenWorldSnapshot_WhenDebugSummaryProduced_ThenCountsMatch)
{
    WorldSnapshot snapshot;
    snapshot.massiveBodies.push_back({});
    snapshot.massiveBodies.push_back({});
    snapshot.ships.push_back({});
    snapshot.projectiles.push_back({});
    snapshot.projectiles.push_back({});
    snapshot.projectiles.push_back({});

    const auto summary = SnapshotSystem::debugSummary(snapshot);

    EXPECT_NE(summary.find("bodies=2"), std::string::npos);
    EXPECT_NE(summary.find("ships=1"), std::string::npos);
    EXPECT_NE(summary.find("projectiles=3"), std::string::npos);
    EXPECT_NE(summary.find("collisions=0"), std::string::npos);
    EXPECT_EQ(summary.substr(0, 8), "snapshot");
}

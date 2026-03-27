#include "client/client_snapshot_buffer.hpp"
#include "server/snapshot/snapshot_types.hpp"

#include <gtest/gtest.h>

using namespace spaceship::client;
using namespace spaceship::server;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

static WorldSnapshot makeSnap(spaceship::shared::Tick tick,
                               double elapsedSeconds,
                               spaceship::shared::Vec3 pos = {})
{
    WorldSnapshot snap;
    snap.serverTick     = tick;
    snap.elapsedSeconds = elapsedSeconds;
    ShipSnapshot ship;
    ship.netId    = 100;
    ship.position = pos;
    snap.ships.push_back(ship);
    return snap;
}

// ---------------------------------------------------------------------------
// Backward-compatible latest() tests (original 4)
// ---------------------------------------------------------------------------

TEST(ClientSnapshotBufferTest, GivenNoSnapshot_WhenLatestQueried_ThenOptionalIsEmpty)
{
    ClientSnapshotBuffer buffer;

    EXPECT_FALSE(buffer.latest().has_value());
}

TEST(ClientSnapshotBufferTest, GivenPushedSnapshot_WhenLatestQueried_ThenSnapshotReturned)
{
    ClientSnapshotBuffer buffer;

    WorldSnapshot snapshot;
    snapshot.serverTick     = 5U;
    snapshot.elapsedSeconds = 0.25;
    buffer.push(std::move(snapshot));

    ASSERT_TRUE(buffer.latest().has_value());
    EXPECT_EQ(buffer.latest()->serverTick, 5U);
    EXPECT_DOUBLE_EQ(buffer.latest()->elapsedSeconds, 0.25);
}

TEST(ClientSnapshotBufferTest, GivenTwoSnapshots_WhenBothPushed_ThenLatestReturnsSecond)
{
    ClientSnapshotBuffer buffer;

    WorldSnapshot first;
    first.serverTick     = 5U;
    first.elapsedSeconds = 0.1;
    buffer.push(std::move(first));

    WorldSnapshot second;
    second.serverTick     = 10U;
    second.elapsedSeconds = 0.2;
    buffer.push(std::move(second));

    ASSERT_TRUE(buffer.latest().has_value());
    EXPECT_EQ(buffer.latest()->serverTick, 10U);
}

TEST(ClientSnapshotBufferTest, GivenPopulatedSnapshot_WhenPushed_ThenNestedPayloadPreserved)
{
    ClientSnapshotBuffer buffer;

    WorldSnapshot snapshot;
    snapshot.serverTick     = 7U;
    snapshot.elapsedSeconds = 0.1;
    snapshot.ships.push_back({
        .netId        = 100U,
        .position     = {1.0, 2.0, 3.0},
        .velocity     = {4.0, 5.0, 6.0},
        .orientation  = {1.0, 0.0, 0.0, 0.0},
        .radiusMeters = 12.5,
        .throttle     = 0.5,
    });
    snapshot.collisionEvents.push_back({
        .netIdA      = 100U,
        .netIdB      = 10'000U,
        .kindA       = spaceship::shared::EntityKind::Ship,
        .kindB       = spaceship::shared::EntityKind::Projectile,
        .toi         = 0.01,
        .eRelJoules  = 250.0,
        .outcome     = CollisionOutcome::ADespawned,
    });

    buffer.push(std::move(snapshot));

    ASSERT_TRUE(buffer.latest().has_value());
    ASSERT_EQ(buffer.latest()->ships.size(), 1U);
    ASSERT_EQ(buffer.latest()->collisionEvents.size(), 1U);
    EXPECT_EQ(buffer.latest()->ships[0].netId, 100U);
    EXPECT_DOUBLE_EQ(buffer.latest()->ships[0].position.z, 3.0);
    EXPECT_EQ(buffer.latest()->collisionEvents[0].outcome, CollisionOutcome::ADespawned);
}

// ---------------------------------------------------------------------------
// Ring-buffer structural tests
// ---------------------------------------------------------------------------

TEST(ClientSnapshotBufferTest, GivenNewBuffer_WhenSizeQueried_ThenZero)
{
    ClientSnapshotBuffer buffer;
    EXPECT_EQ(buffer.size(), 0u);
    EXPECT_TRUE(buffer.empty());
}

TEST(ClientSnapshotBufferTest, GivenFullBuffer_WhenPushNewSnapshot_ThenOldestIsDiscarded)
{
    ClientSnapshotBuffer buffer{4};   // tiny capacity for testing

    buffer.push(makeSnap(1, 1.0));
    buffer.push(makeSnap(2, 2.0));
    buffer.push(makeSnap(3, 3.0));
    buffer.push(makeSnap(4, 4.0));
    ASSERT_EQ(buffer.size(), 4u);

    // One more push should evict tick=1 / elapsed=1.0
    buffer.push(makeSnap(5, 5.0));
    ASSERT_EQ(buffer.size(), 4u);

    // Oldest surviving snap has elapsed=2.0 → interpolate at 1.5 should return nullopt
    EXPECT_FALSE(buffer.interpolate(1.5).has_value());
    // But interpolate at 2.5 (between 2.0 and 3.0) should succeed
    EXPECT_TRUE(buffer.interpolate(2.5).has_value());
}

TEST(ClientSnapshotBufferTest, GivenOutOfOrderSnapshot_WhenPushed_ThenDiscarded)
{
    ClientSnapshotBuffer buffer;
    buffer.push(makeSnap(10, 10.0));
    buffer.push(makeSnap(5,   5.0));   // older time — must be discarded

    EXPECT_EQ(buffer.size(), 1u);
    EXPECT_EQ(buffer.latest()->serverTick, 10U);
}

// ---------------------------------------------------------------------------
// interpolate() — edge cases
// ---------------------------------------------------------------------------

TEST(ClientSnapshotBufferTest, GivenEmptyBuffer_WhenInterpolate_ThenNullopt)
{
    ClientSnapshotBuffer buffer;
    EXPECT_FALSE(buffer.interpolate(0.0).has_value());
}

TEST(ClientSnapshotBufferTest, GivenSingleSnapshot_WhenInterpolateAtExactTime_ThenEntityStateMatches)
{
    ClientSnapshotBuffer buffer;
    buffer.push(makeSnap(1, 1.0, {10.0, 20.0, 30.0}));

    const auto result = buffer.interpolate(1.0);

    ASSERT_TRUE(result.has_value());
    ASSERT_EQ(result->ships.size(), 1u);
    EXPECT_DOUBLE_EQ(result->ships[0].position.x, 10.0);
    EXPECT_DOUBLE_EQ(result->ships[0].position.y, 20.0);
    EXPECT_DOUBLE_EQ(result->ships[0].position.z, 30.0);
}

TEST(ClientSnapshotBufferTest, GivenSingleSnapshot_WhenInterpolateBeforeRange_ThenNullopt)
{
    ClientSnapshotBuffer buffer;
    buffer.push(makeSnap(1, 5.0));

    EXPECT_FALSE(buffer.interpolate(4.9).has_value());
}

TEST(ClientSnapshotBufferTest, GivenSingleSnapshot_WhenInterpolateAfterRange_ThenNullopt)
{
    ClientSnapshotBuffer buffer;
    buffer.push(makeSnap(1, 5.0));

    EXPECT_FALSE(buffer.interpolate(5.1).has_value());
}

TEST(ClientSnapshotBufferTest, GivenTwoSnapshots_WhenInterpolateBeforeRange_ThenNullopt)
{
    ClientSnapshotBuffer buffer;
    buffer.push(makeSnap(1, 1.0));
    buffer.push(makeSnap(2, 2.0));

    EXPECT_FALSE(buffer.interpolate(0.5).has_value());
}

TEST(ClientSnapshotBufferTest, GivenTwoSnapshots_WhenInterpolateAfterRange_ThenNullopt)
{
    ClientSnapshotBuffer buffer;
    buffer.push(makeSnap(1, 1.0));
    buffer.push(makeSnap(2, 2.0));

    EXPECT_FALSE(buffer.interpolate(2.5).has_value());
}

// ---------------------------------------------------------------------------
// interpolate() — interpolation correctness
// ---------------------------------------------------------------------------

TEST(ClientSnapshotBufferTest, GivenTwoSnapshots_WhenInterpolateAtT0_ThenReturnsFirstShipPosition)
{
    ClientSnapshotBuffer buffer;
    buffer.push(makeSnap(1, 0.0, {0.0, 0.0, 0.0}));
    buffer.push(makeSnap(2, 1.0, {100.0, 0.0, 0.0}));

    const auto result = buffer.interpolate(0.0);

    ASSERT_TRUE(result.has_value());
    ASSERT_EQ(result->ships.size(), 1u);
    EXPECT_DOUBLE_EQ(result->ships[0].position.x, 0.0);
}

TEST(ClientSnapshotBufferTest, GivenTwoSnapshots_WhenInterpolateAtT1_ThenReturnsSecondShipPosition)
{
    ClientSnapshotBuffer buffer;
    buffer.push(makeSnap(1, 0.0, {0.0, 0.0, 0.0}));
    buffer.push(makeSnap(2, 1.0, {100.0, 0.0, 0.0}));

    const auto result = buffer.interpolate(1.0);

    ASSERT_TRUE(result.has_value());
    ASSERT_EQ(result->ships.size(), 1u);
    EXPECT_DOUBLE_EQ(result->ships[0].position.x, 100.0);
}

TEST(ClientSnapshotBufferTest, GivenTwoSnapshots_WhenInterpolateAtMidpoint_ThenPositionIsAveraged)
{
    ClientSnapshotBuffer buffer;
    buffer.push(makeSnap(1, 0.0, {0.0, 0.0, 0.0}));
    buffer.push(makeSnap(2, 1.0, {200.0, 400.0, 600.0}));

    const auto result = buffer.interpolate(0.5);

    ASSERT_TRUE(result.has_value());
    ASSERT_EQ(result->ships.size(), 1u);
    EXPECT_DOUBLE_EQ(result->ships[0].position.x, 100.0);
    EXPECT_DOUBLE_EQ(result->ships[0].position.y, 200.0);
    EXPECT_DOUBLE_EQ(result->ships[0].position.z, 300.0);
}

TEST(ClientSnapshotBufferTest, GivenTwoSnapshots_WhenInterpolateAtMidpoint_ThenRenderTimeIsSet)
{
    ClientSnapshotBuffer buffer;
    buffer.push(makeSnap(1, 0.0));
    buffer.push(makeSnap(2, 2.0));

    const auto result = buffer.interpolate(1.0);

    ASSERT_TRUE(result.has_value());
    EXPECT_DOUBLE_EQ(result->renderTime, 1.0);
}

#include "server/collision/broad_phase.hpp"

#include <gtest/gtest.h>

using namespace spaceship::server;
using namespace spaceship::shared;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

static IntervalSnapshot makeSnapshot(
    NetId id, EntityKind kind,
    Vec3 posStart, Vec3 posEnd,
    double radius, double massKg = 1.0)
{
    IntervalSnapshot s;
    s.netId    = id;
    s.kind     = kind;
    s.posStart = posStart;
    s.posEnd   = posEnd;
    s.radius   = radius;
    s.massKg   = massKg;
    return s;
}

// ---------------------------------------------------------------------------
// Empty / trivial inputs
// ---------------------------------------------------------------------------

TEST(BroadPhaseTest,
     GivenNoSnapshots_WhenBroadPhaseRun_ThenNoPairsReturned)
{
    const auto pairs = broadPhaseSmallObjects({});
    EXPECT_TRUE(pairs.empty());
}

TEST(BroadPhaseTest,
     GivenSingleSnapshot_WhenBroadPhaseRun_ThenNoPairsReturned)
{
    const auto s = makeSnapshot(10000, EntityKind::Projectile,
                                {0, 0, 0}, {1, 0, 0}, 0.1);
    const auto pairs = broadPhaseSmallObjects(std::span{&s, 1});
    EXPECT_TRUE(pairs.empty());
}

// ---------------------------------------------------------------------------
// Stationary, clearly separated → no overlap
// ---------------------------------------------------------------------------

TEST(BroadPhaseTest,
     GivenTwoDistantStationaryObjects_WhenBroadPhaseRun_ThenNoPairReturned)
{
    // Two spheres 1000 m apart, radius 1 m each → swept AABBs don't touch
    const IntervalSnapshot snaps[2] = {
        makeSnapshot(10000, EntityKind::Projectile, {0,0,0},    {0,0,0},    1.0),
        makeSnapshot(10001, EntityKind::Projectile, {1000,0,0}, {1000,0,0}, 1.0),
    };
    const auto pairs = broadPhaseSmallObjects(snaps);
    EXPECT_TRUE(pairs.empty());
}

// ---------------------------------------------------------------------------
// Overlapping swept AABBs → pair returned
// ---------------------------------------------------------------------------

TEST(BroadPhaseTest,
     GivenTwoSpheresWithOverlappingSweptAabbs_WhenBroadPhaseRun_ThenPairReturned)
{
    // A moves right from 0→1; B stationary at x=1.5; radii 1 each → AABBs overlap
    const IntervalSnapshot snaps[2] = {
        makeSnapshot(10000, EntityKind::Projectile, {0,0,0},   {1,0,0},   1.0),
        makeSnapshot(10001, EntityKind::Projectile, {1.5,0,0}, {1.5,0,0}, 1.0),
    };
    const auto pairs = broadPhaseSmallObjects(snaps);
    ASSERT_EQ(pairs.size(), 1u);
    EXPECT_EQ(pairs[0].idxA, 0u);
    EXPECT_EQ(pairs[0].idxB, 1u);
}

// ---------------------------------------------------------------------------
// Moving into each other → pair returned
// ---------------------------------------------------------------------------

TEST(BroadPhaseTest,
     GivenTwoSpheresMovingTowardEachOther_WhenBroadPhaseRun_ThenPairReturned)
{
    // A: 0→3, B: 5→2; radii 0.1 each; swept ranges [−0.1,3.1] and [1.9,5.1] → overlap
    const IntervalSnapshot snaps[2] = {
        makeSnapshot(100,   EntityKind::Ship,       {0,0,0}, {3,0,0}, 0.1),
        makeSnapshot(10000, EntityKind::Projectile, {5,0,0}, {2,0,0}, 0.1),
    };
    const auto pairs = broadPhaseSmallObjects(snaps);
    ASSERT_EQ(pairs.size(), 1u);
}

// ---------------------------------------------------------------------------
// Parallel paths, never close → no overlap
// ---------------------------------------------------------------------------

TEST(BroadPhaseTest,
     GivenTwoSpheresOnParallelPaths_WhenBroadPhaseRun_ThenNoPairReturned)
{
    // A moves along x-axis; B is 10 m above; radii 1 each → y-ranges don't overlap
    const IntervalSnapshot snaps[2] = {
        makeSnapshot(100,   EntityKind::Ship,       {0,0,0},  {5,0,0},  1.0),
        makeSnapshot(10000, EntityKind::Projectile, {0,10,0}, {5,10,0}, 1.0),
    };
    const auto pairs = broadPhaseSmallObjects(snaps);
    EXPECT_TRUE(pairs.empty());
}

// ---------------------------------------------------------------------------
// Three objects, two pairs overlap → both returned
// ---------------------------------------------------------------------------

TEST(BroadPhaseTest,
     GivenThreeObjectsWithOnePairOverlapping_WhenBroadPhaseRun_ThenOnePairReturned)
{
    // A at origin, B very close to A, C far away
    // A-B overlap, A-C don't, B-C don't
    const IntervalSnapshot snaps[3] = {
        makeSnapshot(100,   EntityKind::Ship,       {0,0,0},   {0,0,0},   1.0),
        makeSnapshot(101,   EntityKind::Ship,       {1,0,0},   {1,0,0},   1.0),
        makeSnapshot(10000, EntityKind::Projectile, {100,0,0}, {100,0,0}, 1.0),
    };
    const auto pairs = broadPhaseSmallObjects(snaps);
    ASSERT_EQ(pairs.size(), 1u);
    EXPECT_EQ(pairs[0].idxA, 0u);
    EXPECT_EQ(pairs[0].idxB, 1u);
}

// ---------------------------------------------------------------------------
// Pair indices are always (smaller, larger)
// ---------------------------------------------------------------------------

TEST(BroadPhaseTest,
     GivenOverlappingPair_WhenBroadPhaseRun_ThenIdxALessThanIdxB)
{
    const IntervalSnapshot snaps[2] = {
        makeSnapshot(10001, EntityKind::Projectile, {0,0,0}, {0,0,0}, 2.0),
        makeSnapshot(10000, EntityKind::Projectile, {1,0,0}, {1,0,0}, 2.0),
    };
    const auto pairs = broadPhaseSmallObjects(snaps);
    ASSERT_EQ(pairs.size(), 1u);
    EXPECT_LT(pairs[0].idxA, pairs[0].idxB);
}

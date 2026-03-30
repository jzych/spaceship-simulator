// Unit tests for the pure math functions in snapshot_interpolator.
// These test lerp, slerp, and interpolateWorldState in isolation.

#include "client/snapshot_interpolator.hpp"
#include "server/snapshot/snapshot_types.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace spaceship::client::interpolator
{

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

static double quatDot(const shared::Quaternion& a, const shared::Quaternion& b)
{
    return a.w*b.w + a.x*b.x + a.y*b.y + a.z*b.z;
}

static double quatNorm(const shared::Quaternion& q)
{
    return std::sqrt(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
}

static server::WorldSnapshot makeSnapshot(double elapsedSeconds,
                                          shared::Vec3 shipPos  = {},
                                          shared::Vec3 shipVel  = {},
                                          shared::Quaternion shipOri = {1,0,0,0})
{
    server::WorldSnapshot snap;
    snap.serverTick     = 0;
    snap.elapsedSeconds = elapsedSeconds;
    server::ShipSnapshot ship;
    ship.netId       = 100;
    ship.position    = shipPos;
    ship.velocity    = shipVel;
    ship.orientation = shipOri;
    snap.ships.push_back(ship);
    return snap;
}

static server::MassiveBodySnapshot makeBody(shared::NetId id, double px)
{
    server::MassiveBodySnapshot body;
    body.netId        = id;
    body.position     = {px, 0.0, 0.0};
    body.velocity     = {px / 10.0, 0.0, 0.0};
    body.radiusMeters = 1000.0 + px;
    return body;
}

static server::ProjectileSnapshot makeProjectile(shared::NetId id, double px)
{
    server::ProjectileSnapshot projectile;
    projectile.netId        = id;
    projectile.position     = {px, 1.0, 2.0};
    projectile.velocity     = {px / 5.0, 0.0, 0.0};
    projectile.radiusMeters = 0.5 + px;
    projectile.ownerNetId   = 42U;
    return projectile;
}

// ---------------------------------------------------------------------------
// lerp — scalar
// ---------------------------------------------------------------------------

TEST(SnapshotInterpolatorTest, GivenTwoDoubles_WhenLerpAtZero_ThenReturnsA)
{
    EXPECT_DOUBLE_EQ(lerp(3.0, 7.0, 0.0), 3.0);
}

TEST(SnapshotInterpolatorTest, GivenTwoDoubles_WhenLerpAtOne_ThenReturnsB)
{
    EXPECT_DOUBLE_EQ(lerp(3.0, 7.0, 1.0), 7.0);
}

TEST(SnapshotInterpolatorTest, GivenTwoDoubles_WhenLerpAtMidpoint_ThenReturnsMidpoint)
{
    EXPECT_DOUBLE_EQ(lerp(0.0, 10.0, 0.5), 5.0);
}

// ---------------------------------------------------------------------------
// lerp — Vec3
// ---------------------------------------------------------------------------

TEST(SnapshotInterpolatorTest, GivenTwoVec3s_WhenLerpAtMidpoint_ThenResultIsAverage)
{
    const shared::Vec3 a{0.0, 0.0,  0.0};
    const shared::Vec3 b{4.0, 6.0, 10.0};
    const auto result = lerp(a, b, 0.5);

    EXPECT_DOUBLE_EQ(result.x, 2.0);
    EXPECT_DOUBLE_EQ(result.y, 3.0);
    EXPECT_DOUBLE_EQ(result.z, 5.0);
}

// ---------------------------------------------------------------------------
// slerp
// ---------------------------------------------------------------------------

TEST(SnapshotInterpolatorTest, GivenIdenticalQuaternions_WhenSlerpAtAnyT_ThenResultIsUnchanged)
{
    // 45° around Z
    const shared::Quaternion q{0.9238795325112867, 0.0, 0.0, 0.3826834323650898};
    const auto result = slerp(q, q, 0.5);

    EXPECT_NEAR(result.w, q.w, 1e-12);
    EXPECT_NEAR(result.x, q.x, 1e-12);
    EXPECT_NEAR(result.y, q.y, 1e-12);
    EXPECT_NEAR(result.z, q.z, 1e-12);
}

TEST(SnapshotInterpolatorTest, GivenQuaternionAndIdentity_WhenSlerpAtHalf_ThenResultIsNormalised)
{
    const shared::Quaternion q0{1.0, 0.0, 0.0, 0.0};         // identity
    const shared::Quaternion q1{0.0, 0.0, 1.0, 0.0};         // 180° around Y
    const auto result = slerp(q0, q1, 0.5);

    EXPECT_NEAR(quatNorm(result), 1.0, 1e-12);
}

TEST(SnapshotInterpolatorTest, GivenOppositeHemisphereQuaternions_WhenSlerp_ThenShortArcTaken)
{
    // A 90° rotation and its antipodal (-q) represent the same orientation.
    // Slerp should choose the short arc (dot > 0 after negation).
    const shared::Quaternion q0{1.0, 0.0, 0.0, 0.0};         // identity
    // 90° around Y, expressed in the opposite hemisphere
    const shared::Quaternion q1{-0.7071067811865476, 0.0, -0.7071067811865476, 0.0};

    const auto resultShort   = slerp(q0, {0.7071067811865476, 0.0, 0.7071067811865476, 0.0}, 0.5);
    const auto resultCorrected = slerp(q0, q1, 0.5);

    // Both should produce the same midpoint rotation (short-arc correction applied)
    EXPECT_NEAR(std::abs(quatDot(resultShort, resultCorrected)), 1.0, 1e-12);
}

TEST(SnapshotInterpolatorTest, GivenQuaternionAtT0_WhenSlerpAtZero_ThenReturnsQ0)
{
    const shared::Quaternion q0{1.0, 0.0, 0.0, 0.0};
    const shared::Quaternion q1{0.7071067811865476, 0.0, 0.7071067811865476, 0.0};
    const auto result = slerp(q0, q1, 0.0);

    EXPECT_NEAR(result.w, q0.w, 1e-12);
    EXPECT_NEAR(result.x, q0.x, 1e-12);
    EXPECT_NEAR(result.y, q0.y, 1e-12);
    EXPECT_NEAR(result.z, q0.z, 1e-12);
}

TEST(SnapshotInterpolatorTest, GivenQuaternionAtT1_WhenSlerpAtOne_ThenReturnsQ1)
{
    const shared::Quaternion q0{1.0, 0.0, 0.0, 0.0};
    const shared::Quaternion q1{0.7071067811865476, 0.0, 0.7071067811865476, 0.0};
    const auto result = slerp(q0, q1, 1.0);

    EXPECT_NEAR(result.w, q1.w, 1e-12);
    EXPECT_NEAR(result.x, q1.x, 1e-12);
    EXPECT_NEAR(result.y, q1.y, 1e-12);
    EXPECT_NEAR(result.z, q1.z, 1e-12);
}

TEST(SnapshotInterpolatorTest, GivenDegenerateQuaternions_WhenSlerped_ThenFirstQuaternionIsRetained)
{
    const shared::Quaternion q0{};
    const shared::Quaternion q1{};

    const auto result = slerp(q0, q1, 0.5);

    EXPECT_DOUBLE_EQ(result.w, q0.w);
    EXPECT_DOUBLE_EQ(result.x, q0.x);
    EXPECT_DOUBLE_EQ(result.y, q0.y);
    EXPECT_DOUBLE_EQ(result.z, q0.z);
}

// ---------------------------------------------------------------------------
// interpolateWorldState
// ---------------------------------------------------------------------------

TEST(SnapshotInterpolatorTest, GivenTwoSnapshots_WhenInterpolateAtMidpoint_ThenPositionIsAverage)
{
    const auto s0 = makeSnapshot(0.0, {0.0, 0.0, 0.0});
    const auto s1 = makeSnapshot(1.0, {100.0, 200.0, 300.0});

    const auto result = interpolateWorldState(s0, s1, 0.5);

    ASSERT_EQ(result.ships.size(), 1u);
    EXPECT_DOUBLE_EQ(result.ships[0].position.x, 50.0);
    EXPECT_DOUBLE_EQ(result.ships[0].position.y, 100.0);
    EXPECT_DOUBLE_EQ(result.ships[0].position.z, 150.0);
}

TEST(SnapshotInterpolatorTest, GivenEntityOnlyInS1_WhenInterpolated_ThenEntityAppearsWithS1State)
{
    // s0 has no ships; s1 has one — spawn case
    server::WorldSnapshot s0;
    s0.elapsedSeconds = 0.0;

    server::WorldSnapshot s1;
    s1.elapsedSeconds = 1.0;
    server::ShipSnapshot spawned;
    spawned.netId    = 200;
    spawned.position = {5.0, 0.0, 0.0};
    s1.ships.push_back(spawned);

    const auto result = interpolateWorldState(s0, s1, 0.5);

    ASSERT_EQ(result.ships.size(), 1u);
    EXPECT_EQ(result.ships[0].netId, 200u);
    EXPECT_DOUBLE_EQ(result.ships[0].position.x, 5.0);
}

TEST(SnapshotInterpolatorTest, GivenEntityOnlyInS0_WhenInterpolated_ThenEntityIsDropped)
{
    // s0 has one ship; s1 has none — despawn case
    server::WorldSnapshot s0;
    s0.elapsedSeconds = 0.0;
    server::ShipSnapshot despawned;
    despawned.netId = 300;
    s0.ships.push_back(despawned);

    server::WorldSnapshot s1;
    s1.elapsedSeconds = 1.0;

    const auto result = interpolateWorldState(s0, s1, 0.5);

    EXPECT_EQ(result.ships.size(), 0u);
}

TEST(SnapshotInterpolatorTest, GivenMassiveBodyAndProjectileInBothSnapshots_WhenInterpolated_ThenTheirFieldsAreInterpolated)
{
    server::WorldSnapshot s0;
    s0.elapsedSeconds = 0.0;
    s0.massiveBodies.push_back(makeBody(1U, 0.0));
    s0.projectiles.push_back(makeProjectile(10'000U, 0.0));

    server::WorldSnapshot s1;
    s1.elapsedSeconds = 2.0;
    s1.massiveBodies.push_back(makeBody(1U, 20.0));
    s1.projectiles.push_back(makeProjectile(10'000U, 10.0));

    const auto result = interpolateWorldState(s0, s1, 1.0);

    ASSERT_EQ(result.massiveBodies.size(), 1u);
    EXPECT_DOUBLE_EQ(result.massiveBodies[0].position.x, 10.0);
    EXPECT_DOUBLE_EQ(result.massiveBodies[0].velocity.x, 1.0);

    ASSERT_EQ(result.projectiles.size(), 1u);
    EXPECT_DOUBLE_EQ(result.projectiles[0].position.x, 5.0);
    EXPECT_DOUBLE_EQ(result.projectiles[0].velocity.x, 1.0);
}

TEST(SnapshotInterpolatorTest, GivenRenderTimeBeforeFirstSnapshot_WhenInterpolated_ThenFirstSnapshotStateIsReturned)
{
    const auto s0 = makeSnapshot(1.0, {10.0, 0.0, 0.0});
    const auto s1 = makeSnapshot(3.0, {30.0, 0.0, 0.0});

    const auto result = interpolateWorldState(s0, s1, 0.0);

    ASSERT_EQ(result.ships.size(), 1u);
    EXPECT_DOUBLE_EQ(result.ships[0].position.x, 10.0);
}

TEST(SnapshotInterpolatorTest, GivenRenderTimeAfterSecondSnapshot_WhenInterpolated_ThenSecondSnapshotStateIsReturned)
{
    const auto s0 = makeSnapshot(1.0, {10.0, 0.0, 0.0});
    const auto s1 = makeSnapshot(3.0, {30.0, 0.0, 0.0});

    const auto result = interpolateWorldState(s0, s1, 10.0);

    ASSERT_EQ(result.ships.size(), 1u);
    EXPECT_DOUBLE_EQ(result.ships[0].position.x, 30.0);
}

TEST(SnapshotInterpolatorTest, GivenEqualSnapshotTimes_WhenInterpolated_ThenFirstSnapshotStateIsUsedForMatchedEntities)
{
    const auto s0 = makeSnapshot(1.0, {10.0, 0.0, 0.0});
    const auto s1 = makeSnapshot(1.0, {30.0, 0.0, 0.0});

    const auto result = interpolateWorldState(s0, s1, 1.0);

    ASSERT_EQ(result.ships.size(), 1u);
    EXPECT_DOUBLE_EQ(result.ships[0].position.x, 10.0);
}

TEST(SnapshotInterpolatorTest, GivenSingleSnapshotWithBodiesAndProjectiles_WhenLifted_ThenAllEntityKindsAreCopied)
{
    server::WorldSnapshot snap;
    snap.elapsedSeconds = 4.0;
    snap.massiveBodies.push_back(makeBody(1U, 5.0));
    snap.ships.push_back(makeSnapshot(4.0, {7.0, 8.0, 9.0}).ships[0]);
    snap.projectiles.push_back(makeProjectile(10'000U, 3.0));

    const auto result = fromSnapshot(snap, 4.0);

    ASSERT_EQ(result.massiveBodies.size(), 1u);
    ASSERT_EQ(result.ships.size(), 1u);
    ASSERT_EQ(result.projectiles.size(), 1u);
    EXPECT_DOUBLE_EQ(result.massiveBodies[0].position.x, 5.0);
    EXPECT_DOUBLE_EQ(result.ships[0].position.y, 8.0);
    EXPECT_DOUBLE_EQ(result.projectiles[0].position.x, 3.0);
}

} // namespace spaceship::client::interpolator

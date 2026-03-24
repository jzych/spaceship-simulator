#include "server/collision/sweep_math.hpp"

#include <gtest/gtest.h>
#include <cmath>

using namespace spaceship::server;
using namespace spaceship::shared;

// ---------------------------------------------------------------------------
// Head-on collision
// ---------------------------------------------------------------------------

TEST(SweepMathTest,
     GivenTwoSpheresMovingTowardEachOther_WhenSweepComputed_ThenHitDetected)
{
    // A at x=0 moving right at 1 m/s; B at x=10 moving left at 1 m/s; r=1 each
    // Combined radius = 2; they close at 2 m/s; gap = 10-2 = 8 m → toi = 4 s
    const Vec3 posA {0.0, 0.0, 0.0};
    const Vec3 velA {1.0, 0.0, 0.0};
    const Vec3 posB {10.0, 0.0, 0.0};
    const Vec3 velB {-1.0, 0.0, 0.0};

    const auto toi = sphereSweepTOI(posA, velA, posB, velB, 1.0, 1.0, 10.0);
    ASSERT_TRUE(toi.has_value());
    EXPECT_NEAR(*toi, 4.0, 1e-9);
}

// ---------------------------------------------------------------------------
// Miss — spheres moving apart
// ---------------------------------------------------------------------------

TEST(SweepMathTest,
     GivenTwoSpheresMovingApart_WhenSweepComputed_ThenNoHit)
{
    const Vec3 posA {0.0, 0.0, 0.0};
    const Vec3 velA {-1.0, 0.0, 0.0};
    const Vec3 posB {10.0, 0.0, 0.0};
    const Vec3 velB {1.0, 0.0, 0.0};

    const auto toi = sphereSweepTOI(posA, velA, posB, velB, 1.0, 1.0, 10.0);
    EXPECT_FALSE(toi.has_value());
}

// ---------------------------------------------------------------------------
// Miss — parallel paths, always separated
// ---------------------------------------------------------------------------

TEST(SweepMathTest,
     GivenTwoSpheresOnParallelPaths_WhenSweepComputed_ThenNoHit)
{
    const Vec3 posA {0.0, 0.0, 0.0};
    const Vec3 velA {1.0, 0.0, 0.0};
    const Vec3 posB {0.0, 5.0, 0.0};   // 5 m apart laterally, radii=1 each → never touch
    const Vec3 velB {1.0, 0.0, 0.0};

    const auto toi = sphereSweepTOI(posA, velA, posB, velB, 1.0, 1.0, 100.0);
    EXPECT_FALSE(toi.has_value());
}

// ---------------------------------------------------------------------------
// Already overlapping → toi = 0
// ---------------------------------------------------------------------------

TEST(SweepMathTest,
     GivenAlreadyOverlappingSpheres_WhenSweepComputed_ThenToiIsZero)
{
    // Centres 1m apart, radii 1m each → already overlapping
    const Vec3 posA {0.0, 0.0, 0.0};
    const Vec3 velA {0.0, 0.0, 0.0};
    const Vec3 posB {1.0, 0.0, 0.0};
    const Vec3 velB {0.0, 0.0, 0.0};

    const auto toi = sphereSweepTOI(posA, velA, posB, velB, 1.0, 1.0, 1.0);
    ASSERT_TRUE(toi.has_value());
    EXPECT_DOUBLE_EQ(*toi, 0.0);
}

// ---------------------------------------------------------------------------
// Hit outside interval → nullopt (toi would be beyond dt)
// ---------------------------------------------------------------------------

TEST(SweepMathTest,
     GivenSpheresCollideAfterInterval_WhenSweepComputed_ThenNoHit)
{
    // Same head-on setup as first test but dt = 3.0 (toi would be 4.0)
    const Vec3 posA {0.0, 0.0, 0.0};
    const Vec3 velA {1.0, 0.0, 0.0};
    const Vec3 posB {10.0, 0.0, 0.0};
    const Vec3 velB {-1.0, 0.0, 0.0};

    const auto toi = sphereSweepTOI(posA, velA, posB, velB, 1.0, 1.0, 3.0);
    EXPECT_FALSE(toi.has_value());
}

// ---------------------------------------------------------------------------
// Tunneling — endpoint overlap would miss, CCD catches
// ---------------------------------------------------------------------------

TEST(SweepMathTest,
     GivenFastProjectilesThatTunnelAtEndpoints_WhenSweepComputed_ThenHitDetected)
{
    // Projectile A: starts at x=0, moves right at 100 m/s
    // Projectile B: starts at x=2, moves left at 100 m/s; combined radius = 0.2
    // First contact: t = (gap - R) / rel_speed = (2.0 - 0.2) / 200 = 0.009 s
    // dt = 1/60 ≈ 0.01667 s → contact at t=0.009 is within interval (tunneling caught)
    // Endpoint check misses it: at t=dt, posA≈1.667, posB≈0.333, dist≈1.334 > 0.2
    const double dt = 1.0 / 60.0;
    const Vec3 posA {0.0, 0.0, 0.0};
    const Vec3 velA {100.0, 0.0, 0.0};
    const Vec3 posB {2.0, 0.0, 0.0};
    const Vec3 velB {-100.0, 0.0, 0.0};

    const auto toi = sphereSweepTOI(posA, velA, posB, velB, 0.1, 0.1, dt);
    ASSERT_TRUE(toi.has_value());
    EXPECT_GE(*toi, 0.0);
    EXPECT_LE(*toi, dt);
}

// ---------------------------------------------------------------------------
// Stationary spheres separated → no hit
// ---------------------------------------------------------------------------

TEST(SweepMathTest,
     GivenStationaryNonOverlappingSpheres_WhenSweepComputed_ThenNoHit)
{
    const Vec3 pos {0.0, 0.0, 0.0};
    const Vec3 vel {};
    const Vec3 posB {10.0, 0.0, 0.0};

    const auto toi = sphereSweepTOI(pos, vel, posB, vel, 1.0, 1.0, 1.0);
    EXPECT_FALSE(toi.has_value());
}

// ---------------------------------------------------------------------------
// Glancing pass — spheres come very close but don't touch
// ---------------------------------------------------------------------------

TEST(SweepMathTest,
     GivenGlancingMiss_WhenSweepComputed_ThenNoHit)
{
    // A moves right; B is 2.01 m above (r=1 each, so min gap = 2.0 m → miss)
    const Vec3 posA {-10.0, 0.0, 0.0};
    const Vec3 velA {1.0, 0.0, 0.0};
    const Vec3 posB {0.0, 2.01, 0.0};
    const Vec3 velB {};

    const auto toi = sphereSweepTOI(posA, velA, posB, velB, 1.0, 1.0, 100.0);
    EXPECT_FALSE(toi.has_value());
}

// ---------------------------------------------------------------------------
// Direct hit at t=0 boundary (contact at start)
// ---------------------------------------------------------------------------

TEST(SweepMathTest,
     GivenSpheresTouchingAtIntervalStart_WhenSweepComputed_ThenToiIsZero)
{
    // Centres exactly 2m apart, radii 1 each → touching (c == 0)
    const Vec3 posA {0.0, 0.0, 0.0};
    const Vec3 velA {1.0, 0.0, 0.0};
    const Vec3 posB {2.0, 0.0, 0.0};
    const Vec3 velB {-1.0, 0.0, 0.0};

    const auto toi = sphereSweepTOI(posA, velA, posB, velB, 1.0, 1.0, 10.0);
    ASSERT_TRUE(toi.has_value());
    EXPECT_NEAR(*toi, 0.0, 1e-9);
}

#include "server/simulation/timestep/timescale_heuristics.hpp"
#include "server/simulation/simulation_world.hpp"

#include <gtest/gtest.h>
#include <cmath>

using namespace spaceship::server;
using namespace spaceship::shared;

namespace
{

constexpr double kEarthMu     = 3.986004418e14;
constexpr double kEarthRadius = 6.371e6;
constexpr double kLeoRadius   = kEarthRadius + 400'000.0;
constexpr double kLeoSpeed    = 7672.0;

MassiveBodyState makeEarthAtOrigin()
{
    MassiveBodyState earth {};
    earth.definition.muMetersCubedPerSecondSquared = kEarthMu;
    earth.transform.position = {};
    earth.velocity.linear    = {};
    return earth;
}

TimestepLadderConfig defaultCfg()
{
    return {};  // default-constructed
}

} // anonymous namespace

// ---------------------------------------------------------------------------
// Orbital heuristic
// ---------------------------------------------------------------------------

TEST(TimescaleHeuristics,
     GivenShipInCircularLEO_WhenOrbitalHeuristicComputed_ThenTimescaleIsReasonable)
{
    const Vec3 pos {kLeoRadius, 0.0, 0.0};
    const Vec3 vel {0.0, kLeoSpeed, 0.0};
    const Vec3 bodyPos {};

    const double dt = computeOrbitalTimescale(pos, vel, bodyPos, defaultCfg());
    EXPECT_GT(dt, 1.0);     // much longer than a tick
    EXPECT_LT(dt, 500.0);   // well below the orbital period
}

TEST(TimescaleHeuristics,
     GivenZeroVelocity_WhenOrbitalHeuristicComputed_ThenResultIsFinite)
{
    const Vec3 pos {kLeoRadius, 0.0, 0.0};
    const Vec3 vel {};
    const Vec3 bodyPos {};

    const double dt = computeOrbitalTimescale(pos, vel, bodyPos, defaultCfg());
    EXPECT_FALSE(std::isinf(dt));
    EXPECT_FALSE(std::isnan(dt));
}

// ---------------------------------------------------------------------------
// Acceleration heuristic
// ---------------------------------------------------------------------------

TEST(TimescaleHeuristics,
     GivenHighAcceleration_WhenAccelerationHeuristicComputed_ThenTimescaleIsShorter)
{
    const TimestepLadderConfig cfg = defaultCfg();
    const Vec3 rRel {kLeoRadius, 0.0, 0.0};

    const double dtLow  = computeAccelerationTimescale(rRel, {0.1, 0.0, 0.0}, cfg);
    const double dtHigh = computeAccelerationTimescale(rRel, {10.0, 0.0, 0.0}, cfg);

    EXPECT_GT(dtLow, dtHigh);
}

TEST(TimescaleHeuristics,
     GivenLEOGravity_WhenAccelerationHeuristicComputed_ThenTimescaleIsPositive)
{
    const double gLeo = kEarthMu / (kLeoRadius * kLeoRadius);
    const Vec3 rRel   {kLeoRadius, 0.0, 0.0};
    const Vec3 a      {-gLeo, 0.0, 0.0};

    const double dt = computeAccelerationTimescale(rRel, a, defaultCfg());
    EXPECT_GT(dt, 0.0);
    EXPECT_FALSE(std::isnan(dt));
}

// ---------------------------------------------------------------------------
// Jerk heuristic
// ---------------------------------------------------------------------------

TEST(TimescaleHeuristics,
     GivenNoPreviousAcceleration_WhenJerkHeuristicComputed_ThenReturnsMaxDt)
{
    const TimestepLadderConfig cfg = defaultCfg();
    const Vec3 a     {0.0, -9.0, 0.0};
    const Vec3 aPrev {0.0, 0.0, 0.0};

    const double dt = computeJerkTimescale(a, aPrev, 0.0, cfg);
    EXPECT_DOUBLE_EQ(dt, cfg.dt_max);
}

TEST(TimescaleHeuristics,
     GivenRapidlyChangingAcceleration_WhenJerkHeuristicComputed_ThenTimescaleIsShorterThanSmallJerk)
{
    const TimestepLadderConfig cfg = defaultCfg();
    const double dtPrev = 1.0 / 60.0;

    const Vec3 aPrev {0.0, -9.0, 0.0};
    const Vec3 aSmallChange  {0.0, -9.5, 0.0};
    const Vec3 aLargeChange  {0.0, -18.0, 0.0};

    const double dtSmallJerk = computeJerkTimescale(aSmallChange, aPrev, dtPrev, cfg);
    const double dtLargeJerk = computeJerkTimescale(aLargeChange, aPrev, dtPrev, cfg);

    EXPECT_GT(dtSmallJerk, dtLargeJerk);
}

// ---------------------------------------------------------------------------
// Close-approach heuristic
// ---------------------------------------------------------------------------

// The close-approach heuristic computes dt ≈ alpha * distance_to_centre / relative_speed.
// Distance is measured to the body's centre of mass, so even at low altitude above Earth
// the result is on the order of minutes.  The key property is monotone ordering: a faster
// or closer approach gives a shorter dt than a slower or farther one.

TEST(TimescaleHeuristics,
     GivenFastApproach_WhenCloseApproachHeuristicComputed_ThenTimescaleShorterThanSlowApproach)
{
    const TimestepLadderConfig cfg = defaultCfg();
    std::vector<MassiveBodyState> bodies = {makeEarthAtOrigin()};

    // Same position; fast approach vs slow approach.
    const Vec3 pos  {kLeoRadius, 0.0, 0.0};
    const Vec3 velFast {-7'000.0, 0.0, 0.0};   // ~escape speed component
    const Vec3 velSlow {-100.0,   0.0, 0.0};

    const double dtFast = computeCloseApproachTimescale(pos, velFast, bodies, cfg);
    const double dtSlow = computeCloseApproachTimescale(pos, velSlow, bodies, cfg);

    EXPECT_LT(dtFast, dtSlow);
    EXPECT_GT(dtFast, 0.0);
}

TEST(TimescaleHeuristics,
     GivenFarFromBodies_WhenCloseApproachHeuristicComputed_ThenTimescaleLargerThanNearApproach)
{
    const TimestepLadderConfig cfg = defaultCfg();
    std::vector<MassiveBodyState> bodies = {makeEarthAtOrigin()};

    const Vec3 vel   {-1'000.0, 0.0, 0.0};   // same speed, different distance
    const Vec3 posNear{kLeoRadius,  0.0, 0.0};
    const Vec3 posFar {1.0e11,      0.0, 0.0};

    const double dtNear = computeCloseApproachTimescale(posNear, vel, bodies, cfg);
    const double dtFar  = computeCloseApproachTimescale(posFar,  vel, bodies, cfg);

    EXPECT_LT(dtNear, dtFar);
    EXPECT_GT(dtNear, 0.0);
}

// ---------------------------------------------------------------------------
// Combined target timestep
// ---------------------------------------------------------------------------

TEST(TimescaleHeuristics,
     GivenCombinedHeuristics_WhenTargetComputed_ThenResultClamped)
{
    const TimestepLadderConfig cfg = defaultCfg();
    std::vector<MassiveBodyState> bodies = {makeEarthAtOrigin()};

    const Vec3 pos  {kLeoRadius, 0.0, 0.0};
    const Vec3 vel  {0.0, kLeoSpeed, 0.0};
    const double gLeo = kEarthMu / (kLeoRadius * kLeoRadius);
    const Vec3 a    {0.0, -gLeo, 0.0};

    const double dt = computeTargetTimestep(pos, vel, a, {}, 0.0, bodies, cfg);
    EXPECT_GE(dt, cfg.dt_min());
    EXPECT_LE(dt, cfg.dt_max);
}

TEST(TimescaleHeuristics,
     GivenEmptyBodiesList_WhenTargetComputed_ThenNoNanOrInf)
{
    const TimestepLadderConfig cfg = defaultCfg();
    std::vector<MassiveBodyState> bodies {};

    const Vec3 pos {kLeoRadius, 0.0, 0.0};
    const Vec3 vel {0.0, kLeoSpeed, 0.0};
    const Vec3 a   {0.0, -9.0, 0.0};

    const double dt = computeTargetTimestep(pos, vel, a, {}, 0.0, bodies, cfg);
    EXPECT_FALSE(std::isnan(dt));
    EXPECT_FALSE(std::isinf(dt));
    EXPECT_GE(dt, cfg.dt_min());
    EXPECT_LE(dt, cfg.dt_max);
}

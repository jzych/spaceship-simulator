#include "test_helpers.hpp"
#include "server/simulation/timestep/timestep_controller.hpp"
#include "server/simulation/timestep/timestep_types.hpp"

#include <gtest/gtest.h>
#include <cmath>

using namespace spaceship::test;
using namespace spaceship::server;
using namespace spaceship::shared;

namespace
{

constexpr double kLeoSpeed = 7672.0;

SimulationConfig adaptiveCfg()
{
    SimulationConfig cfg {};
    cfg.useAdaptiveTimestep              = true;
    cfg.timestepLadder.dt_max            = cfg.fixedDeltaSeconds;
    cfg.timestepLadder.k_max             = 4;
    cfg.timestepLadder.tau_raise_seconds = 0.1;
    return cfg;
}

} // anonymous namespace

// ---------------------------------------------------------------------------
// Flag disabled — behaviour unchanged
// ---------------------------------------------------------------------------

TEST(TimestepControllerIntegration,
     GivenAdaptiveTimestepDisabled_WhenTickCalled_ThenBehaviourMatchesFixedStep)
{
    SimulationConfig fixedCfg {};
    fixedCfg.useAdaptiveTimestep = false;

    SimulationServer fixedServer  {createEarthOnlyAtOriginWorld(), fixedCfg};
    SimulationServer fixedServer2 {createEarthOnlyAtOriginWorld(), fixedCfg};

    ShipSpawnRequest req {};
    req.transform.position = {kLeoRadius, 0.0, 0.0};
    req.velocity.linear    = {0.0, kLeoSpeed, 0.0};
    fixedServer.spawnShip(req);
    fixedServer2.spawnShip(req);

    for (int i = 0; i < 10; ++i)
    {
        fixedServer.tick();
        fixedServer2.tick();
    }

    const Vec3& p1 = fixedServer.world().ships.front().transform.position;
    const Vec3& p2 = fixedServer2.world().ships.front().transform.position;
    EXPECT_NEAR(p1.x, p2.x, 1e-9);
    EXPECT_NEAR(p1.y, p2.y, 1e-9);
}

// ---------------------------------------------------------------------------
// Adaptive path executes substeps and records diagnostics
// ---------------------------------------------------------------------------

TEST(TimestepControllerIntegration,
     GivenAdaptiveTimestepEnabled_WhenTickCalled_ThenSubstepsDiagnosticsRecorded)
{
    auto cfg = adaptiveCfg();
    SimulationServer srv {createEarthOnlyAtOriginWorld(), cfg};
    ShipSpawnRequest req {};
    req.transform.position = {kLeoRadius, 0.0, 0.0};
    req.velocity.linear    = {0.0, kLeoSpeed, 0.0};
    srv.spawnShip(req);

    srv.tick();

    const auto& diag = srv.timestepDiagnostics();
    ASSERT_EQ(diag.size(), 1U);
    EXPECT_GE(diag.front().substepCount, 1);
    EXPECT_GT(diag.front().dtApplied, 0.0);
}

// ---------------------------------------------------------------------------
// Elapsed time advances correctly
// ---------------------------------------------------------------------------

TEST(TimestepControllerIntegration,
     GivenAdaptive_WhenFullFrameCompleted_ThenTickCountAdvances)
{
    SimulationServer srv {createEarthOnlyAtOriginWorld(), adaptiveCfg()};
    ShipSpawnRequest req {};
    req.transform.position = {kLeoRadius, 0.0, 0.0};
    req.velocity.linear    = {0.0, kLeoSpeed, 0.0};
    srv.spawnShip(req);

    constexpr int kTicks = 60;
    for (int i = 0; i < kTicks; ++i)
        srv.tick();

    EXPECT_EQ(srv.tickCount(), static_cast<Tick>(kTicks));
}

// ---------------------------------------------------------------------------
// Circular LEO orbit stability
// ---------------------------------------------------------------------------

TEST(TimestepControllerIntegration,
     GivenShipInCircularLEO_WhenAdaptive600Ticks_ThenRadiusStable)
{
    SimulationServer srv {createEarthOnlyAtOriginWorld(), adaptiveCfg()};
    ShipSpawnRequest req {};
    req.transform.position = {kLeoRadius, 0.0, 0.0};
    req.velocity.linear    = {0.0, kLeoSpeed, 0.0};
    srv.spawnShip(req);

    for (int i = 0; i < 600; ++i)
        srv.tick();

    const auto& ship = srv.world().ships.front();
    const double r = std::sqrt(
        ship.transform.position.x * ship.transform.position.x +
        ship.transform.position.y * ship.transform.position.y +
        ship.transform.position.z * ship.transform.position.z);

    EXPECT_NEAR(r, kLeoRadius, kLeoRadius * 0.005);
}

// ---------------------------------------------------------------------------
// Diagnostics ladder level in valid range
// ---------------------------------------------------------------------------

TEST(TimestepControllerIntegration,
     GivenShipCoasting_WhenAdaptiveMultipleTicks_ThenLadderLevelInValidRange)
{
    auto cfg = adaptiveCfg();
    SimulationServer srv {createEarthOnlyAtOriginWorld(), cfg};
    ShipSpawnRequest req {};
    req.transform.position = {kLeoRadius, 0.0, 0.0};
    req.velocity.linear    = {0.0, kLeoSpeed, 0.0};
    srv.spawnShip(req);

    for (int i = 0; i < 120; ++i)
        srv.tick();

    const auto& diag = srv.timestepDiagnostics();
    ASSERT_FALSE(diag.empty());
    EXPECT_GE(diag.front().ladderLevel, 0);
    EXPECT_LE(diag.front().ladderLevel, cfg.timestepLadder.k_max);
}

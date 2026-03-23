#include "server/tests/test_helpers.hpp"
#include "server/simulation/simulation_math.hpp"

#include <gtest/gtest.h>
#include <cmath>

using namespace spaceship::test;
using namespace spaceship::server;
using namespace spaceship::shared;

namespace
{

constexpr double kLeoSpeed = 7672.0;

double specificEnergy(const Vec3& pos, const Vec3& vel)
{
    const double r  = length(pos);
    const double v2 = dot(vel, vel);
    return 0.5 * v2 - kEarthMu / r;
}

double relEnergyDrift(double e0, double e)
{
    if (std::abs(e0) < 1e-30)
        return 0.0;
    return std::abs(e - e0) / std::abs(e0);
}

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
// Circular LEO — energy bounded after 1000 ticks
// ---------------------------------------------------------------------------

TEST(EnergyRegression,
     GivenCircularLEO_WhenAdaptive1000Ticks_ThenEnergyDriftBounded)
{
    SimulationServer srv {createEarthOnlyAtOriginWorld(), adaptiveCfg()};
    ShipSpawnRequest req {};
    req.transform.position = {kLeoRadius, 0.0, 0.0};
    req.velocity.linear    = {0.0, kLeoSpeed, 0.0};
    srv.spawnShip(req);

    srv.tick();  // seed initial acceleration
    const auto& ship = srv.world().ships.front();
    const double e0  = specificEnergy(ship.transform.position, ship.velocity.linear);

    for (int i = 0; i < 999; ++i)
        srv.tick();

    const double e1    = specificEnergy(ship.transform.position, ship.velocity.linear);
    const double drift = relEnergyDrift(e0, e1);

    EXPECT_LT(drift, 1e-3) << "Energy drift too large: " << drift;
}

// ---------------------------------------------------------------------------
// Adaptive vs naive very-large fixed step over 1000 simulated seconds.
// A 10-second fixed step is clearly too coarse for LEO (orbital period ~5400s,
// recommended max step ~60s). The adaptive scheme (max 1/60 s) should show
// materially smaller energy drift than the 10-second naive baseline.
// ---------------------------------------------------------------------------

TEST(EnergyRegression,
     GivenLEOOrbit_WhenAdaptiveVsNaive10sStep_ThenAdaptiveEnergyDriftIsSmaller)
{
    // Fine reference: 60 Hz fixed step
    SimulationConfig fineCfg {};

    // Naive large-step baseline (10 second step)
    SimulationConfig coarseCfg {};
    coarseCfg.fixedDeltaSeconds = 10.0;

    ShipSpawnRequest req {};
    req.transform.position = {kLeoRadius, 0.0, 0.0};
    req.velocity.linear    = {0.0, kLeoSpeed, 0.0};

    SimulationServer fine    {createEarthOnlyAtOriginWorld(), fineCfg};
    SimulationServer coarse  {createEarthOnlyAtOriginWorld(), coarseCfg};
    SimulationServer adaptive{createEarthOnlyAtOriginWorld(), adaptiveCfg()};

    fine.spawnShip(req);
    coarse.spawnShip(req);
    adaptive.spawnShip(req);

    // Simulate 1000 seconds (enough for coarse error to accumulate)
    constexpr int kFineSteps   = 1000 * 60;   // 1000 s at 60 Hz
    constexpr int kCoarseSteps = 100;          // 1000 s at 10 s/step
    constexpr int kAdaptSteps  = 1000 * 60;   // 1000 s at 60 Hz outer tick

    for (int i = 0; i < kFineSteps;   ++i) fine.tick();
    for (int i = 0; i < kCoarseSteps; ++i) coarse.tick();
    for (int i = 0; i < kAdaptSteps;  ++i) adaptive.tick();

    const auto& shipFine   = fine.world().ships.front();
    const auto& shipCoarse = coarse.world().ships.front();
    const auto& shipAdapt  = adaptive.world().ships.front();

    const double e0      = specificEnergy(shipFine.transform.position,   shipFine.velocity.linear);
    const double eCoarse = specificEnergy(shipCoarse.transform.position, shipCoarse.velocity.linear);
    const double eAdapt  = specificEnergy(shipAdapt.transform.position,  shipAdapt.velocity.linear);

    const double driftCoarse = relEnergyDrift(e0, eCoarse);
    const double driftAdapt  = relEnergyDrift(e0, eAdapt);

    EXPECT_LT(driftAdapt, driftCoarse)
        << "Adaptive drift=" << driftAdapt << " should be less than coarse drift=" << driftCoarse;
}

// ---------------------------------------------------------------------------
// Adaptive vs fixed small step produce comparable orbits
// ---------------------------------------------------------------------------

TEST(EnergyRegression,
     GivenCircularLEO_WhenAdaptiveVsFixedSmallDt_ThenResultsComparable)
{
    SimulationConfig fixedCfg {};
    fixedCfg.useAdaptiveTimestep = false;

    SimulationServer fixed    {createEarthOnlyAtOriginWorld(), fixedCfg};
    SimulationServer adaptive {createEarthOnlyAtOriginWorld(), adaptiveCfg()};

    ShipSpawnRequest req {};
    req.transform.position = {kLeoRadius, 0.0, 0.0};
    req.velocity.linear    = {0.0, kLeoSpeed, 0.0};
    fixed.spawnShip(req);
    adaptive.spawnShip(req);

    constexpr int kTicks = 600;
    for (int i = 0; i < kTicks; ++i)
    {
        fixed.tick();
        adaptive.tick();
    }

    const Vec3& pFixed = fixed.world().ships.front().transform.position;
    const Vec3& pAdapt = adaptive.world().ships.front().transform.position;

    const double dx   = pFixed.x - pAdapt.x;
    const double dy   = pFixed.y - pAdapt.y;
    const double dz   = pFixed.z - pAdapt.z;
    const double dist = std::sqrt(dx*dx + dy*dy + dz*dz);

    EXPECT_LT(dist, 100.0)
        << "Position divergence " << dist << " m after " << kTicks << " ticks";
}

// ---------------------------------------------------------------------------
// Thrust increases specific energy
// ---------------------------------------------------------------------------

TEST(EnergyRegression,
     GivenThrottlingShip_WhenAdaptiveTick_ThenEnergyIncreasesWithThrust)
{
    SimulationServer srv {createEarthOnlyAtOriginWorld(), adaptiveCfg()};
    ShipSpawnRequest req {};
    req.transform.position = {kLeoRadius, 0.0, 0.0};
    req.velocity.linear    = {0.0, kLeoSpeed, 0.0};
    srv.spawnShip(req);
    srv.tick();

    const auto& ship = srv.world().ships.front();
    const double e0  = specificEnergy(ship.transform.position, ship.velocity.linear);

    ShipControl ctrl {};
    ctrl.throttle = 1.0;
    srv.updateShipControl(srv.world().ships.front().netId, ctrl);

    for (int i = 0; i < 60; ++i)
        srv.tick();

    const double e1 = specificEnergy(ship.transform.position, ship.velocity.linear);
    EXPECT_GT(e1, e0) << "Expected energy to increase with thrust";
}

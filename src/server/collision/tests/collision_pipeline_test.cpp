#include "server/collision/collision_system.hpp"
#include "server/simulation/simulation_world.hpp"

#include <gtest/gtest.h>

using namespace spaceship::server;
using namespace spaceship::shared;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

static ShipState makeShip(
    NetId id, Vec3 pos, Vec3 vel, double radius = 5.0, double mass = 1000.0)
{
    ShipState s;
    s.netId = id;
    s.transform.position = pos;
    s.velocity.linear = vel;
    s.collider.radiusMeters = radius;
    s.massProperties = {mass, 1.0 / mass};
    return s;
}

static ProjectileState makeProjectile(
    NetId id, Vec3 pos, Vec3 vel,
    double radius = 0.1, double mass = 1.0, NetId owner = 100, double ttl = 10.0)
{
    ProjectileState p;
    p.netId = id;
    p.params = {ttl, owner};
    p.transform.position = pos;
    p.velocity.linear = vel;
    p.collider.radiusMeters = radius;
    p.massProperties = {mass, 1.0 / mass};
    return p;
}

static MassiveBodyState makeMassiveBody(NetId id, Vec3 pos, double radius, Vec3 vel = {})
{
    MassiveBodyState b;
    b.definition.netId = id;
    b.definition.radiusMeters = radius;
    b.definition.muMetersCubedPerSecondSquared = 3.986e14;
    b.transform.position = pos;
    b.velocity.linear = vel;
    return b;
}

// ---------------------------------------------------------------------------
// Small vs massive body
// ---------------------------------------------------------------------------

TEST(CollisionPipelineTest,
     GivenProjectileInsideMassiveBody_WhenDetectAndResolve_ThenProjectileDespawnedAndEventEmitted)
{
    // Projectile centre at x=500, body radius=1000 → projectile is inside body → toi=0
    const MassiveBodyState body = makeMassiveBody(0, {0, 0, 0}, 1000.0);
    auto proj = makeProjectile(10000, {500, 0, 0}, {0, 0, 0});

    std::vector<ShipState> ships;
    std::vector<ProjectileState> projectiles = {proj};
    std::vector<CollisionEvent> events;
    SimulationConfig config;

    CollisionSystem system;
    system.detectAndResolve(ships, projectiles, std::span{&body, 1}, events, config, 1.0 / 60.0);

    EXPECT_TRUE(projectiles.empty());
    ASSERT_EQ(events.size(), 1U);
    EXPECT_EQ(events[0].outcome, CollisionOutcome::SmallDespawned);
}

TEST(CollisionPipelineTest,
     GivenShipApproachingMassiveBody_WhenDetectAndResolve_ThenShipDespawnedAndEventEmitted)
{
    // Ship approaching massive body's surface
    // Body at origin, radius=1000. Ship at x=1006, radius=5, moving left at 1000 m/s
    // Combined radius = 1005. Gap = 1006 - 1005 = 1. toi = 1/1000 = 0.001 s < 1/60
    const MassiveBodyState body = makeMassiveBody(1, {0, 0, 0}, 1000.0);
    auto ship = makeShip(100, {1006, 0, 0}, {-1000, 0, 0});

    std::vector<ShipState> ships = {ship};
    std::vector<ProjectileState> projectiles;
    std::vector<CollisionEvent> events;
    SimulationConfig config;

    CollisionSystem system;
    system.detectAndResolve(ships, projectiles, std::span{&body, 1}, events, config, 1.0 / 60.0);

    EXPECT_TRUE(ships.empty());
    ASSERT_EQ(events.size(), 1U);
    EXPECT_EQ(events[0].outcome, CollisionOutcome::SmallDespawned);
}

// ---------------------------------------------------------------------------
// Projectile vs projectile
// ---------------------------------------------------------------------------

TEST(CollisionPipelineTest,
     GivenTwoProjectilesWithHighEnergyImpact_WhenDetectAndResolve_ThenBothDespawned)
{
    // 1-kg projectiles, head-on at ±1000 m/s, gap=0.3 m after radii
    // mu_r=0.5, |dv|=2000, E_rel=0.5*0.5*4e6=1e6 J >> 1000 J default threshold
    // toi = (0.5 - 0.2) / 2000 = 0.00015 s < 1/60
    auto pA = makeProjectile(10000, {0.0, 0, 0}, { 1000, 0, 0});
    auto pB = makeProjectile(10001, {0.5, 0, 0}, {-1000, 0, 0});

    std::vector<ShipState> ships;
    std::vector<ProjectileState> projectiles = {pA, pB};
    std::vector<CollisionEvent> events;
    SimulationConfig config;  // default threshold = 1000 J

    CollisionSystem system;
    system.detectAndResolve(ships, projectiles, {}, events, config, 1.0 / 60.0);

    EXPECT_TRUE(projectiles.empty());
    ASSERT_EQ(events.size(), 1U);
    EXPECT_EQ(events[0].outcome, CollisionOutcome::BothDespawned);
}

TEST(CollisionPipelineTest,
     GivenTwoProjectilesWithLowEnergyImpact_WhenDetectAndResolve_ThenOneDespawnedWithVcm)
{
    // pA at x=0 vel=(1,0,0), pB at x=0.19 (overlapping since R=0.2) vel=(0,0,0)
    // mu_r=0.5, |dv|=1, E_rel=0.25 J << 1000 J → one survives
    // v_cm = (1*1 + 1*0) / 2 = 0.5 m/s (both mass=1)
    auto pA = makeProjectile(10000, {0.00, 0, 0}, {1, 0, 0});
    auto pB = makeProjectile(10001, {0.19, 0, 0}, {0, 0, 0});

    std::vector<ShipState> ships;
    std::vector<ProjectileState> projectiles = {pA, pB};
    std::vector<CollisionEvent> events;
    SimulationConfig config;  // threshold = 1000 J

    CollisionSystem system;
    system.detectAndResolve(ships, projectiles, {}, events, config, 1.0 / 60.0);

    ASSERT_EQ(projectiles.size(), 1U);  // exactly one survives
    ASSERT_EQ(events.size(), 1U);
    EXPECT_NEAR(projectiles[0].velocity.linear.x, 0.5, 1e-9);  // v_cm
}

// ---------------------------------------------------------------------------
// Ship vs projectile
// ---------------------------------------------------------------------------

TEST(CollisionPipelineTest,
     GivenProjectileHitsShipBelowDestroyThreshold_WhenDetectAndResolve_ThenProjectileDespawnedShipSurvivesWithVcm)
{
    // Ship (1000 kg) stationary; projectile (1 kg) moving left at -1 m/s
    // Combined radius = 5.0 + 0.1 = 5.1 m; start gap = 5.11 - 5.1 = 0.01 m
    // toi = 0.01 / 1 = 0.01 s < 1/60
    // mu_r ≈ 0.999 kg; E_rel ≈ 0.5 J << 50000 J threshold → ship survives
    auto ship = makeShip(100, {0.0, 0, 0}, {0, 0, 0});
    auto proj = makeProjectile(10000, {5.11, 0, 0}, {-1, 0, 0});

    std::vector<ShipState> ships = {ship};
    std::vector<ProjectileState> projectiles = {proj};
    std::vector<CollisionEvent> events;
    SimulationConfig config;

    CollisionSystem system;
    system.detectAndResolve(ships, projectiles, {}, events, config, 1.0 / 60.0);

    ASSERT_EQ(ships.size(), 1U);
    EXPECT_TRUE(projectiles.empty());
    ASSERT_EQ(events.size(), 1U);
    EXPECT_EQ(events[0].outcome, CollisionOutcome::BDespawned);
    // Ship gets v_cm ≈ (0*1000 + (-1)*1) / 1001 ≈ -0.000999 m/s
    EXPECT_LT(ships[0].velocity.linear.x, 0.0);
}

TEST(CollisionPipelineTest,
     GivenProjectileHitsShipAboveDestroyThreshold_WhenDetectAndResolve_ThenBothDespawned)
{
    // Ship (1000 kg) stationary; projectile (1 kg) moving left at -500 m/s
    // mu_r ≈ 1 kg; E_rel ≈ 0.5 * 1 * 500^2 = 125000 J > 50000 J threshold → both destroyed
    auto ship = makeShip(100, {0.0, 0, 0}, {0, 0, 0});
    auto proj = makeProjectile(10000, {5.11, 0, 0}, {-500, 0, 0});

    std::vector<ShipState> ships = {ship};
    std::vector<ProjectileState> projectiles = {proj};
    std::vector<CollisionEvent> events;
    SimulationConfig config;

    CollisionSystem system;
    system.detectAndResolve(ships, projectiles, {}, events, config, 1.0 / 60.0);

    EXPECT_TRUE(ships.empty());
    EXPECT_TRUE(projectiles.empty());
    ASSERT_EQ(events.size(), 1U);
    EXPECT_EQ(events[0].outcome, CollisionOutcome::BothDespawned);
}

// ---------------------------------------------------------------------------
// Tunneling — CCD catches what endpoint check would miss
// ---------------------------------------------------------------------------

TEST(CollisionPipelineTest,
     GivenFastProjectilesThatTunnelAtEndpoints_WhenDetectAndResolve_ThenCollisionDetected)
{
    // Gap=2 m (from centres), radii=0.1 each, speeds=±100 m/s
    // toi = (2.0 - 0.2) / 200 = 0.009 s < 1/60 ≈ 0.0167 s
    // At dt: posA=1.667, posB=0.333, dist=1.334 > 0.2 → endpoint check misses it
    auto pA = makeProjectile(10000, {0.0, 0, 0}, { 100, 0, 0});
    auto pB = makeProjectile(10001, {2.0, 0, 0}, {-100, 0, 0});

    std::vector<ShipState> ships;
    std::vector<ProjectileState> projectiles = {pA, pB};
    std::vector<CollisionEvent> events;
    SimulationConfig config;
    config.projectileProjectileDestroyEnergyJoules = 0.0;  // always destroy both

    CollisionSystem system;
    system.detectAndResolve(ships, projectiles, {}, events, config, 1.0 / 60.0);

    EXPECT_TRUE(projectiles.empty());
    ASSERT_EQ(events.size(), 1U);
}

// ---------------------------------------------------------------------------
// Determinism — already-destroyed entity skips subsequent hits
// ---------------------------------------------------------------------------

TEST(CollisionPipelineTest,
     GivenEntityDestroyedInEarlierHit_WhenDetectAndResolve_ThenLaterHitWithSameEntitySkipped)
{
    // pA and pB overlap (toi=0); pA and pC collide later (toi=0.004 s).
    // After pA-pB resolves (both destroyed), pA-pC should be skipped so pC survives.
    auto pA = makeProjectile(10000, {0.00, 0, 0}, { 100, 0, 0});
    auto pB = makeProjectile(10001, {0.15, 0, 0}, {-100, 0, 0}); // overlapping → toi=0
    auto pC = makeProjectile(10002, {1.00, 0, 0}, {-100, 0, 0}); // toi=0.004 s

    std::vector<ShipState> ships;
    std::vector<ProjectileState> projectiles = {pA, pB, pC};
    std::vector<CollisionEvent> events;
    SimulationConfig config;
    config.projectileProjectileDestroyEnergyJoules = 0.0;  // always destroy both

    CollisionSystem system;
    system.detectAndResolve(ships, projectiles, {}, events, config, 1.0 / 60.0);

    // pA and pB destroyed (first hit); pC survives (second hit skipped)
    ASSERT_EQ(projectiles.size(), 1U);
    EXPECT_EQ(projectiles[0].netId, 10002U);
    ASSERT_EQ(events.size(), 1U);  // only the pA-pB event
}

// ---------------------------------------------------------------------------
// No false positives — separated objects produce no events
// ---------------------------------------------------------------------------

TEST(CollisionPipelineTest,
     GivenAllObjectsWellSeparated_WhenDetectAndResolve_ThenNoEventsAndNoRemovals)
{
    auto ship = makeShip(100, {0, 0, 0}, {0, 0, 0});
    auto proj = makeProjectile(10000, {1000, 0, 0}, {0, 0, 0});

    std::vector<ShipState> ships = {ship};
    std::vector<ProjectileState> projectiles = {proj};
    std::vector<CollisionEvent> events;
    SimulationConfig config;

    CollisionSystem system;
    system.detectAndResolve(ships, projectiles, {}, events, config, 1.0 / 60.0);

    EXPECT_EQ(ships.size(), 1U);
    EXPECT_EQ(projectiles.size(), 1U);
    EXPECT_TRUE(events.empty());
}

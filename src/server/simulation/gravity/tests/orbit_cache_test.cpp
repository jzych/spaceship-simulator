#include "server/tests/test_helpers.hpp"

#include "server/spawning/spawning_system.hpp"

#include <numbers>

using namespace spaceship::test;

// ---------------------------------------------------------------------------
// OrbitCacheIntegrationTest — OrbitCache on ships via the tick pipeline
// ---------------------------------------------------------------------------

TEST_F(EarthOnlyCenteredTest, GivenShipInCircularLEO_WhenOneTick_ThenOrbitCacheIsElliptic)
{
    const double vOrbit = std::sqrt(kEarthMu / kLeoRadius);
    spaceship::server::ShipSpawnRequest request {};
    request.transform.position = {kLeoRadius, 0.0, 0.0};
    request.velocity = {{0.0, vOrbit, 0.0}};
    server.spawnShip(request);

    server.tick();

    const auto& cache = server.world().ships.front().orbitCache;
    EXPECT_TRUE(cache.isElliptic);
    EXPECT_NEAR(cache.eccentricity, 0.0, 1e-3);
    EXPECT_NEAR(cache.semiMajorAxis, kLeoRadius, 100.0);
    EXPECT_EQ(cache.referenceBodyId, 1U); // Earth
}

TEST_F(EarthOnlyCenteredTest, GivenShipInCircularLEO_WhenOneTick_ThenCacheAltitudeMatchesLEO)
{
    const double vOrbit = std::sqrt(kEarthMu / kLeoRadius);
    spaceship::server::ShipSpawnRequest request {};
    request.transform.position = {kLeoRadius, 0.0, 0.0};
    request.velocity = {{0.0, vOrbit, 0.0}};
    server.spawnShip(request);

    server.tick();

    const auto& cache = server.world().ships.front().orbitCache;
    EXPECT_NEAR(cache.altitudeMeters, kLeoAltitude, 100.0);
}

TEST_F(EarthOnlyCenteredTest, GivenThrustingShip_WhenTwoTicks_ThenCacheEpochAdvancesEachTick)
{
    const double vOrbit = std::sqrt(kEarthMu / kLeoRadius);
    spaceship::server::ShipSpawnRequest request {};
    request.transform.position = {kLeoRadius, 0.0, 0.0};
    request.velocity = {{0.0, vOrbit, 0.0}};
    const auto shipId = server.spawnShip(request);
    server.updateShipControl(shipId, {1.0, {1.0, 0.0, 0.0, 0.0}, false});

    server.tick();
    const auto epoch1 = server.world().ships.front().orbitCache.epoch;

    server.tick();
    const auto epoch2 = server.world().ships.front().orbitCache.epoch;

    EXPECT_EQ(epoch2, epoch1 + 1);
}

TEST_F(EarthOnlyCenteredTest, GivenCoastingShip_WhenTwoTicks_ThenCacheEpochUnchangedOnSecondTick)
{
    const double vOrbit = std::sqrt(kEarthMu / kLeoRadius);
    spaceship::server::ShipSpawnRequest request {};
    request.transform.position = {kLeoRadius, 0.0, 0.0};
    request.velocity = {{0.0, vOrbit, 0.0}};
    server.spawnShip(request);

    // First tick always computes (dirty)
    server.tick();
    const auto epoch1 = server.world().ships.front().orbitCache.epoch;

    // Second tick — inactive ship → cache should still be valid (TTL not expired)
    server.tick();
    const auto epoch2 = server.world().ships.front().orbitCache.epoch;

    // epoch should NOT have changed (inactive ship, TTL ~60 ticks)
    EXPECT_EQ(epoch2, epoch1);
}

TEST_F(EarthOnlyCenteredTest, GivenCoastingShipThatStartsThrusting_WhenTicked_ThenCacheInvalidated)
{
    const double vOrbit = std::sqrt(kEarthMu / kLeoRadius);
    spaceship::server::ShipSpawnRequest request {};
    request.transform.position = {kLeoRadius, 0.0, 0.0};
    request.velocity = {{0.0, vOrbit, 0.0}};
    const auto shipId = server.spawnShip(request);

    // Tick 1: coasting
    server.tick();
    const auto epoch1 = server.world().ships.front().orbitCache.epoch;

    // Tick 2: still coasting, cache not refreshed
    server.tick();
    EXPECT_EQ(server.world().ships.front().orbitCache.epoch, epoch1);

    // Tick 3: start thrust → cache must invalidate and refresh
    server.updateShipControl(shipId, {1.0, {1.0, 0.0, 0.0, 0.0}, false});
    server.tick();
    const auto epoch3 = server.world().ships.front().orbitCache.epoch;
    EXPECT_GT(epoch3, epoch1);
}

TEST_F(EarthOnlyCenteredTest, GivenShipInPureTwoBodyOrbit_WhenOneTick_ThenQualityScoreIsHigh)
{
    const double vOrbit = std::sqrt(kEarthMu / kLeoRadius);
    spaceship::server::ShipSpawnRequest request {};
    request.transform.position = {kLeoRadius, 0.0, 0.0};
    request.velocity = {{0.0, vOrbit, 0.0}};
    server.spawnShip(request);

    server.tick();

    // Single body → pure two-body → high quality score
    EXPECT_GT(server.world().ships.front().orbitCache.qualityScore, 0.9);
}

// ---------------------------------------------------------------------------
// Geographic telemetry via OrbitCache pipeline
// ---------------------------------------------------------------------------

TEST_F(EarthOnlyCenteredTest, GivenCoastingShipAndRotatingEarth_WhenManyTicks_ThenLongitudeChanges)
{
    // Even when orbit fit is skipped (inactive), geographic coords should update
    // because the body rotates. After multiple ticks, longitude should have changed.
    const double vOrbit = std::sqrt(kEarthMu / kLeoRadius);
    spaceship::server::ShipSpawnRequest request {};
    request.transform.position = {kLeoRadius, 0.0, 0.0};
    request.velocity = {{0.0, vOrbit, 0.0}};
    server.spawnShip(request);

    server.tick();
    const double lon1 = server.world().ships.front().orbitCache.longitudeRadians;

    // Run enough ticks for visible rotation (Earth rotates ~0.004°/tick at 60Hz)
    for (int i = 0; i < 60; ++i)
        server.tick();

    const double lon2 = server.world().ships.front().orbitCache.longitudeRadians;

    // Longitude should have changed due to Earth rotation
    EXPECT_NE(lon1, lon2);
}

TEST_F(EarthOnlyCenteredTest, GivenShipInCircularLEO_WhenOneTick_ThenGeographicAltitudeMatchesLEO)
{
    const double vOrbit = std::sqrt(kEarthMu / kLeoRadius);
    spaceship::server::ShipSpawnRequest request {};
    request.transform.position = {kLeoRadius, 0.0, 0.0};
    request.velocity = {{0.0, vOrbit, 0.0}};
    server.spawnShip(request);

    server.tick();

    EXPECT_NEAR(server.world().ships.front().orbitCache.altitudeMeters, kLeoAltitude, 200.0);
}

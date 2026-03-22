#include "server/bootstrap.hpp"
#include "server/orbit_fitting.hpp"
#include "server/reference_body_selector.hpp"
#include "server/simulation_math.hpp"
#include "server/simulation_server.hpp"
#include "server/simulation_world.hpp"

#include <gtest/gtest.h>
#include <cmath>
#include <numbers>

namespace
{

constexpr double kDefaultShipAccelerationMetersPerSecondSquared = 20'000.0 / 1'000.0;
constexpr double kExpectedVelocityDeltaPerTick =
    kDefaultShipAccelerationMetersPerSecondSquared * spaceship::shared::constants::kFixedDeltaSeconds;
constexpr double kQuarterTurnZAxisHalfAngleComponent = std::numbers::sqrt2_v<double> / 2.0;
constexpr double kDefaultProjectileMuzzleSpeedMetersPerSecond = 1'000.0;

} // namespace

// ---------------------------------------------------------------------------
// SimulationServerSmokeTest — basic API tests, full world, no integration checks
// ---------------------------------------------------------------------------

class SimulationServerSmokeTest : public ::testing::Test
{
  protected:
    spaceship::server::SimulationServer server {};
};

TEST_F(SimulationServerSmokeTest, BootstrapsThreeMassiveBodies)
{

    ASSERT_EQ(server.world().massiveBodies.size(), 3U);
    EXPECT_EQ(server.world().massiveBodies[0].definition.name, "Sun");
    EXPECT_EQ(server.world().massiveBodies[1].definition.name, "Earth");
    EXPECT_EQ(server.world().massiveBodies[2].definition.name, "Moon");
    EXPECT_GT(server.world().massiveBodies[0].definition.muMetersCubedPerSecondSquared, 0.0);
    EXPECT_GT(server.world().massiveBodies[1].definition.radiusMeters, 0.0);
    EXPECT_DOUBLE_EQ(server.world().massiveBodies[0].transform.orientation.w, 1.0);
    EXPECT_NE(server.world().massiveBodies[1].velocity.linear.y, 0.0);
    EXPECT_NE(server.world().massiveBodies[2].velocity.linear.y, 0.0);
}

TEST_F(SimulationServerSmokeTest, TickAdvancesServerClock)
{
    server.tick();

    EXPECT_EQ(server.tickCount(), 1U);
    EXPECT_TRUE(server.lastSnapshotSummary().empty());
}

TEST_F(SimulationServerSmokeTest, SpawnShipAddsShipToWorld)
{
    constexpr spaceship::shared::NetId kExpectedFirstShipNetId = spaceship::server::kFirstShipNetId;

    const spaceship::server::ShipSpawnRequest request {
        {{10.0, 20.0, 30.0}, {1.0, 0.0, 0.0, 0.0}},
        {{100.0, 200.0, 300.0}},
    };

    const auto shipNetId = server.spawnShip(request);

    ASSERT_EQ(server.world().ships.size(), 1U);
    const auto& ship = server.world().ships.back();
    EXPECT_EQ(shipNetId, kExpectedFirstShipNetId);
    EXPECT_EQ(ship.netId, kExpectedFirstShipNetId);
    EXPECT_DOUBLE_EQ(ship.transform.position.x, 10.0);
    EXPECT_DOUBLE_EQ(ship.transform.position.y, 20.0);
    EXPECT_DOUBLE_EQ(ship.velocity.linear.z, 300.0);
    EXPECT_DOUBLE_EQ(ship.massProperties.massKg, 1'000.0);
}

TEST_F(SimulationServerSmokeTest, UpdateShipControlForUnknownNetIdIsNoOp)
{
    constexpr spaceship::shared::NetId kMissingShipNetId = 99U;
    // Stale/out-of-order packets for despawned ships must be silently discarded.
    EXPECT_NO_FATAL_FAILURE(
        server.updateShipControl(kMissingShipNetId, spaceship::shared::ShipControl {1.0, {1.0, 0.0, 0.0, 0.0}, true}));
}

TEST_F(SimulationServerSmokeTest, SpawnShipAssignsSequentialShipIds)
{
    constexpr spaceship::shared::NetId kExpectedFirstShipNetId = spaceship::server::kFirstShipNetId;
    constexpr spaceship::shared::NetId kExpectedSecondShipNetId = spaceship::server::kFirstShipNetId + 1U;

    const spaceship::server::ShipSpawnRequest request {
        {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0, 0.0}},
        {{0.0, 0.0, 0.0}},
    };

    const auto firstShipNetId = server.spawnShip(request);
    const auto secondShipNetId = server.spawnShip(request);

    EXPECT_EQ(firstShipNetId, kExpectedFirstShipNetId);
    EXPECT_EQ(secondShipNetId, kExpectedSecondShipNetId);
    ASSERT_EQ(server.world().ships.size(), 2U);
    EXPECT_EQ(server.world().ships[0].netId, kExpectedFirstShipNetId);
    EXPECT_EQ(server.world().ships[1].netId, kExpectedSecondShipNetId);
}

TEST_F(SimulationServerSmokeTest, SpawnShipAllowsDuplicateStateButKeepsUniqueAssignedIds)
{
    const spaceship::server::ShipSpawnRequest request {
        {{5.0, 6.0, 7.0}, {1.0, 0.0, 0.0, 0.0}},
        {{8.0, 9.0, 10.0}},
    };

    const auto firstShipNetId = server.spawnShip(request);
    const auto secondShipNetId = server.spawnShip(request);

    ASSERT_EQ(server.world().ships.size(), 2U);
    EXPECT_NE(firstShipNetId, secondShipNetId);
    EXPECT_EQ(server.world().ships[0].transform.position.x, server.world().ships[1].transform.position.x);
    EXPECT_EQ(server.world().ships[0].velocity.linear.y, server.world().ships[1].velocity.linear.y);
}

TEST_F(SimulationServerSmokeTest, UpdateShipControlReplacesExistingControlState)
{
    const spaceship::server::ShipSpawnRequest request {
        {{1.0, 2.0, 3.0}, {1.0, 0.0, 0.0, 0.0}},
        {{4.0, 5.0, 6.0}},
    };
    const spaceship::shared::ShipControl control {0.75, {0.0, 0.0, 1.0, 0.0}, true};

    const auto shipNetId = server.spawnShip(request);
    server.updateShipControl(shipNetId, control);

    ASSERT_EQ(server.world().ships.size(), 1U);
    const auto& ship = server.world().ships.front();
    EXPECT_DOUBLE_EQ(ship.control.throttle, 0.75);
    EXPECT_DOUBLE_EQ(ship.control.desiredOrientation.y, 1.0);
    EXPECT_TRUE(ship.control.fire);
}

// ---------------------------------------------------------------------------
// ZeroGravityShipBehaviorTest — Verlet integration and thrust tests (no gravity)
// ---------------------------------------------------------------------------

class ZeroGravityShipBehaviorTest : public ::testing::Test
{
  protected:
    spaceship::server::SimulationServer server {spaceship::server::SimulationWorld {}};
};

TEST_F(ZeroGravityShipBehaviorTest, TickAppliesForwardThrustToShipVelocity)
{
    const spaceship::server::ShipSpawnRequest request {
        {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0, 0.0}},
        {{0.0, 0.0, 0.0}},
    };

    const auto shipNetId = server.spawnShip(request);
    server.updateShipControl(shipNetId, spaceship::shared::ShipControl {1.0, {1.0, 0.0, 0.0, 0.0}, false});

    server.tick();

    ASSERT_EQ(server.world().ships.size(), 1U);
    const auto& ship = server.world().ships.front();
    EXPECT_NEAR(ship.velocity.linear.x, kExpectedVelocityDeltaPerTick, 1e-9);
    EXPECT_DOUBLE_EQ(ship.velocity.linear.y, 0.0);
    EXPECT_DOUBLE_EQ(ship.velocity.linear.z, 0.0);
}

TEST_F(ZeroGravityShipBehaviorTest, TickAppliesDesiredOrientationBeforeThrust)
{
    const spaceship::server::ShipSpawnRequest request {
        {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0, 0.0}},
        {{0.0, 0.0, 0.0}},
    };

    const auto shipNetId = server.spawnShip(request);
    server.updateShipControl(
        shipNetId,
        spaceship::shared::ShipControl {
            1.0,
            {kQuarterTurnZAxisHalfAngleComponent, 0.0, 0.0, kQuarterTurnZAxisHalfAngleComponent},
            false,
        });

    server.tick();

    ASSERT_EQ(server.world().ships.size(), 1U);
    const auto& ship = server.world().ships.front();
    EXPECT_NEAR(ship.transform.orientation.w, kQuarterTurnZAxisHalfAngleComponent, 1e-12);
    EXPECT_NEAR(ship.velocity.linear.x, 0.0, 1e-9);
    EXPECT_NEAR(ship.velocity.linear.y, kExpectedVelocityDeltaPerTick, 1e-9);
}

TEST_F(ZeroGravityShipBehaviorTest, TickDrivenFireSpawnsProjectileAndClearsFireFlag)
{
    constexpr spaceship::shared::NetId kExpectedFirstProjectileNetId = spaceship::server::kFirstProjectileNetId;
    const spaceship::server::ShipSpawnRequest request {
        {{1.0, 2.0, 3.0}, {1.0, 0.0, 0.0, 0.0}},
        {{10.0, 20.0, 30.0}},
    };
    // Projectile spawns before ship thrust is applied, so it takes the ship's
    // pre-tick velocity (10.0) plus muzzle speed.
    constexpr double kExpectedProjectileVelocityX =
        10.0 + kDefaultProjectileMuzzleSpeedMetersPerSecond;
    const double kExpectedProjectilePositionX =
        1.0 + kExpectedProjectileVelocityX * spaceship::shared::constants::kFixedDeltaSeconds;

    const auto shipNetId = server.spawnShip(request);
    server.updateShipControl(shipNetId, spaceship::shared::ShipControl {1.0, {1.0, 0.0, 0.0, 0.0}, true});

    EXPECT_TRUE(server.world().projectiles.empty());

    server.tick();

    ASSERT_EQ(server.world().projectiles.size(), 1U);
    ASSERT_EQ(server.world().ships.size(), 1U);
    const auto& projectile = server.world().projectiles.front();
    const auto& ship = server.world().ships.front();
    EXPECT_EQ(projectile.netId, kExpectedFirstProjectileNetId);
    EXPECT_EQ(projectile.params.ownerNetId, shipNetId);
    EXPECT_NEAR(projectile.transform.position.x, kExpectedProjectilePositionX, 1e-6);
    EXPECT_DOUBLE_EQ(projectile.transform.orientation.w, 1.0);
    EXPECT_NEAR(projectile.velocity.linear.x, kExpectedProjectileVelocityX, 1e-9);
    EXPECT_DOUBLE_EQ(projectile.velocity.linear.y, 20.0);
    EXPECT_FALSE(ship.control.fire);

    server.tick();

    EXPECT_EQ(server.world().projectiles.size(), 1U);
}

TEST_F(ZeroGravityShipBehaviorTest, VerletPositionAdvancesFromInitialVelocity)
{
    const spaceship::server::ShipSpawnRequest request {
        {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0, 0.0}},
        {{10.0, 0.0, 0.0}},
    };
    server.spawnShip(request);

    server.tick();

    const auto& ship = server.world().ships.front();
    const double expectedX = 10.0 * spaceship::shared::constants::kFixedDeltaSeconds;
    EXPECT_NEAR(ship.transform.position.x, expectedX, 1e-9);
}

TEST_F(ZeroGravityShipBehaviorTest, VerletPositionIncludesAccelerationTerm)
{
    // Ship at rest with full throttle — position advances by 0.5*a*dt²
    const spaceship::server::ShipSpawnRequest request {
        {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0, 0.0}},
        {{0.0, 0.0, 0.0}},
    };
    const auto shipNetId = server.spawnShip(request);
    server.updateShipControl(shipNetId, spaceship::shared::ShipControl {1.0, {1.0, 0.0, 0.0, 0.0}, false});

    server.tick();

    const auto& ship = server.world().ships.front();
    const double dt = spaceship::shared::constants::kFixedDeltaSeconds;
    const double expectedX = 0.5 * kDefaultShipAccelerationMetersPerSecondSquared * dt * dt;
    EXPECT_NEAR(ship.transform.position.x, expectedX, 1e-9);
}

TEST_F(ZeroGravityShipBehaviorTest, ShipControlWritesToAccelerationNotVelocityDirectly)
{
    // Before any tick, thrust must not have modified velocity
    const spaceship::server::ShipSpawnRequest request {
        {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0, 0.0}},
        {{0.0, 0.0, 0.0}},
    };
    const auto shipNetId = server.spawnShip(request);
    server.updateShipControl(shipNetId, spaceship::shared::ShipControl {1.0, {1.0, 0.0, 0.0, 0.0}, false});

    // Velocity unchanged before tick — only acceleration changes during tick
    const auto& ship = server.world().ships.front();
    EXPECT_DOUBLE_EQ(ship.velocity.linear.x, 0.0);
}

// --- Math utility tests ---

TEST(SimulationMathTest, SubtractVec3ReturnsComponentWiseDifference)
{
    const spaceship::shared::Vec3 a {3.0, 2.0, 1.0};
    const spaceship::shared::Vec3 b {1.0, 1.0, 1.0};
    const auto result = spaceship::server::subtract(a, b);
    EXPECT_DOUBLE_EQ(result.x, 2.0);
    EXPECT_DOUBLE_EQ(result.y, 1.0);
    EXPECT_DOUBLE_EQ(result.z, 0.0);
}

TEST(SimulationMathTest, SubtractVec3WithNegativeResult)
{
    const spaceship::shared::Vec3 a {1.0, 2.0, 3.0};
    const spaceship::shared::Vec3 b {4.0, 5.0, 6.0};
    const auto result = spaceship::server::subtract(a, b);
    EXPECT_DOUBLE_EQ(result.x, -3.0);
    EXPECT_DOUBLE_EQ(result.y, -3.0);
    EXPECT_DOUBLE_EQ(result.z, -3.0);
}

TEST(SimulationMathTest, DotProductOfOrthogonalVectorsIsZero)
{
    const spaceship::shared::Vec3 x {1.0, 0.0, 0.0};
    const spaceship::shared::Vec3 y {0.0, 1.0, 0.0};
    EXPECT_DOUBLE_EQ(spaceship::server::dot(x, y), 0.0);
}

TEST(SimulationMathTest, DotProductOfParallelVectorsMatchesMagnitudeSquared)
{
    const spaceship::shared::Vec3 v {3.0, 0.0, 0.0};
    EXPECT_DOUBLE_EQ(spaceship::server::dot(v, v), 9.0);
}

TEST(SimulationMathTest, LengthSquaredMatchesDotProductWithSelf)
{
    const spaceship::shared::Vec3 v {3.0, 4.0, 0.0};
    EXPECT_DOUBLE_EQ(spaceship::server::lengthSquared(v), 25.0);
}

TEST(SimulationMathTest, LengthOfKnownVector)
{
    const spaceship::shared::Vec3 v {3.0, 4.0, 0.0};
    EXPECT_DOUBLE_EQ(spaceship::server::length(v), 5.0);
}

TEST(SimulationMathTest, LengthOfZeroVectorIsZero)
{
    const spaceship::shared::Vec3 zero {0.0, 0.0, 0.0};
    EXPECT_DOUBLE_EQ(spaceship::server::length(zero), 0.0);
}

TEST(SimulationMathTest, ShipSpawnedInZeroGravityHasZeroAcceleration)
{
    spaceship::server::SimulationServer server {spaceship::server::SimulationWorld {}};
    const spaceship::server::ShipSpawnRequest request {
        {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0, 0.0}},
        {{0.0, 0.0, 0.0}},
    };
    server.spawnShip(request);

    const auto& ship = server.world().ships.front();
    EXPECT_DOUBLE_EQ(ship.acceleration.x, 0.0);
    EXPECT_DOUBLE_EQ(ship.acceleration.y, 0.0);
    EXPECT_DOUBLE_EQ(ship.acceleration.z, 0.0);
}

TEST(SimulationMathTest, ProjectileSpawnedInZeroGravityHasZeroAcceleration)
{
    spaceship::server::SimulationServer server {spaceship::server::SimulationWorld {}};
    const spaceship::server::ShipSpawnRequest request {
        {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0, 0.0}},
        {{0.0, 0.0, 0.0}},
    };
    const auto shipNetId = server.spawnShip(request);
    server.updateShipControl(shipNetId, spaceship::shared::ShipControl {0.0, {1.0, 0.0, 0.0, 0.0}, true});
    server.tick();

    ASSERT_EQ(server.world().projectiles.size(), 1U);
    const auto& projectile = server.world().projectiles.front();
    EXPECT_DOUBLE_EQ(projectile.acceleration.x, 0.0);
    EXPECT_DOUBLE_EQ(projectile.acceleration.y, 0.0);
    EXPECT_DOUBLE_EQ(projectile.acceleration.z, 0.0);
}

// ---------------------------------------------------------------------------
// MassiveBodyMotionTest — full Sun/Earth/Moon world
// ---------------------------------------------------------------------------

class MassiveBodyMotionTest : public ::testing::Test
{
  protected:
    spaceship::server::SimulationServer server {};
};

TEST_F(MassiveBodyMotionTest, SunRemainsAtOriginAfterManyTicks)
{
    for (int i = 0; i < 3600; ++i)
        server.tick();

    const auto& sun = server.world().massiveBodies[0];
    EXPECT_NEAR(sun.transform.position.x, 0.0, 1.0);
    EXPECT_NEAR(sun.transform.position.y, 0.0, 1.0);
    EXPECT_NEAR(sun.transform.position.z, 0.0, 1.0);
}

TEST_F(MassiveBodyMotionTest, EarthAngularPositionAdvancesAfterOneSimulatedMinute)
{
    // t=0: Earth at angle 0 (on +x axis)
    const double initialAngle =
        std::atan2(server.world().massiveBodies[1].transform.position.y,
                   server.world().massiveBodies[1].transform.position.x);

    for (int i = 0; i < 3600; ++i)
        server.tick();

    const auto& earth = server.world().massiveBodies[1];
    const double finalAngle = std::atan2(earth.transform.position.y, earth.transform.position.x);

    EXPECT_GT(finalAngle, initialAngle); // angle increases (counter-clockwise orbit)
}

TEST_F(MassiveBodyMotionTest, EarthOrbitalRadiusUnchangedAfterOneSimulatedMinute)
{
    const double initialRadius = spaceship::server::length(
        server.world().massiveBodies[1].transform.position);

    for (int i = 0; i < 3600; ++i)
        server.tick();

    const auto& earth = server.world().massiveBodies[1];
    const double finalRadius = spaceship::server::length(earth.transform.position);

    EXPECT_NEAR(finalRadius, initialRadius, 1.0); // radius stable to 1 m
}

TEST_F(MassiveBodyMotionTest, MoonEarthDistanceStableOver3600Ticks)
{
    constexpr double kMoonDistanceMeters = 384'400'000.0;
    constexpr double kTolerance = kMoonDistanceMeters * 0.01; // 1%

    for (int i = 0; i < 3600; ++i)
        server.tick();

    const auto& earth = server.world().massiveBodies[1];
    const auto& moon = server.world().massiveBodies[2];
    const auto diff = spaceship::server::subtract(moon.transform.position, earth.transform.position);

    EXPECT_NEAR(spaceship::server::length(diff), kMoonDistanceMeters, kTolerance);
}

TEST_F(MassiveBodyMotionTest, MoonAngularPositionRelativeToEarthAdvances)
{
    const auto& earth0 = server.world().massiveBodies[1];
    const auto& moon0 = server.world().massiveBodies[2];
    const auto diff0 = spaceship::server::subtract(moon0.transform.position, earth0.transform.position);
    const double initialAngle = std::atan2(diff0.y, diff0.x);

    for (int i = 0; i < 3600; ++i)
        server.tick();

    const auto& earth = server.world().massiveBodies[1];
    const auto& moon = server.world().massiveBodies[2];
    const auto diff = spaceship::server::subtract(moon.transform.position, earth.transform.position);
    const double finalAngle = std::atan2(diff.y, diff.x);

    EXPECT_GT(finalAngle, initialAngle);
}

TEST_F(MassiveBodyMotionTest, EarthVelocityIsTangentialToOrbitAroundSun)
{
    server.tick();

    const auto& earth = server.world().massiveBodies[1];
    // Sun at origin → position IS the radius vector
    const double dotResult = spaceship::server::dot(earth.transform.position, earth.velocity.linear);
    const double posMag = spaceship::server::length(earth.transform.position);
    const double velMag = spaceship::server::length(earth.velocity.linear);

    EXPECT_NEAR(dotResult / (posMag * velMag), 0.0, 1e-6);
}

// ---------------------------------------------------------------------------
// EarthOnlyCenteredTest — single Earth at origin, for precise gravity checks
// ---------------------------------------------------------------------------

class EarthOnlyCenteredTest : public ::testing::Test
{
  protected:
    spaceship::server::SimulationServer server {spaceship::server::createEarthOnlyAtOriginWorld()};

    static constexpr double kEarthMu = 3.986004418e14;
    static constexpr double kEarthRadius = 6.371e6;
    static constexpr double kLeoAltitude = 400'000.0;
    static constexpr double kLeoRadius = kEarthRadius + kLeoAltitude;
};

TEST_F(EarthOnlyCenteredTest, WorldContainsOnlyEarth)
{
    ASSERT_EQ(server.world().massiveBodies.size(), 1U);
    EXPECT_EQ(server.world().massiveBodies[0].definition.name, "Earth");
    EXPECT_NEAR(server.world().massiveBodies[0].transform.position.x, 0.0, 1e-9);
}

TEST_F(EarthOnlyCenteredTest, EarthRemainsAtOriginAfterManyTicks)
{
    for (int i = 0; i < 3600; ++i)
        server.tick();

    const auto& earth = server.world().massiveBodies[0];
    EXPECT_NEAR(earth.transform.position.x, 0.0, 1.0);
    EXPECT_NEAR(earth.transform.position.y, 0.0, 1.0);
    EXPECT_NEAR(earth.transform.position.z, 0.0, 1.0);
}

// --- Spawn gravity seeding ---

TEST_F(EarthOnlyCenteredTest, ShipSpawnSeededWithGravityAcceleration)
{
    // Gravity at spawn position is immediately available (no tick needed)
    spaceship::server::ShipSpawnRequest request {};
    request.transform.position = {kLeoRadius, 0.0, 0.0};
    server.spawnShip(request);

    const auto& ship = server.world().ships.front();
    EXPECT_LT(ship.acceleration.x, 0.0); // points toward Earth at origin
    EXPECT_NEAR(ship.acceleration.y, 0.0, 1e-6);
}

TEST_F(EarthOnlyCenteredTest, ShipSpawnWithInjectedAccelerationOverridesGravity)
{
    const spaceship::shared::Vec3 kCustomAccel {1.0, 2.0, 3.0};
    spaceship::server::ShipSpawnRequest request {};
    request.transform.position = {kLeoRadius, 0.0, 0.0};
    request.initialAcceleration = kCustomAccel;
    server.spawnShip(request);

    const auto& ship = server.world().ships.front();
    EXPECT_DOUBLE_EQ(ship.acceleration.x, 1.0);
    EXPECT_DOUBLE_EQ(ship.acceleration.y, 2.0);
    EXPECT_DOUBLE_EQ(ship.acceleration.z, 3.0);
}

// --- Gravity acceleration tests ---

TEST_F(EarthOnlyCenteredTest, ShipAtLeoReceivesCorrectGravityMagnitude)
{
    // g at LEO = μ / r² ≈ 8.67 m/s²
    const double expectedG = kEarthMu / (kLeoRadius * kLeoRadius);

    spaceship::server::ShipSpawnRequest request {};
    request.transform.position = {kLeoRadius, 0.0, 0.0};
    server.spawnShip(request);
    server.tick();

    const auto& ship = server.world().ships.front();
    const double aMag = spaceship::server::length(ship.acceleration);
    EXPECT_NEAR(aMag, expectedG, 0.01);
}

TEST_F(EarthOnlyCenteredTest, ShipOnPositiveXAxisGravityPointsNegativeX)
{
    spaceship::server::ShipSpawnRequest request {};
    request.transform.position = {kLeoRadius, 0.0, 0.0};
    server.spawnShip(request);
    server.tick();

    const auto& ship = server.world().ships.front();
    EXPECT_LT(ship.acceleration.x, 0.0);           // points toward Earth at origin
    EXPECT_NEAR(ship.acceleration.y, 0.0, 1e-6);
    EXPECT_NEAR(ship.acceleration.z, 0.0, 1e-6);
}

TEST_F(EarthOnlyCenteredTest, ShipOnNegativeYAxisGravityPointsPositiveY)
{
    spaceship::server::ShipSpawnRequest request {};
    request.transform.position = {0.0, -kLeoRadius, 0.0};
    server.spawnShip(request);
    server.tick();

    const auto& ship = server.world().ships.front();
    EXPECT_NEAR(ship.acceleration.x, 0.0, 1e-6);
    EXPECT_GT(ship.acceleration.y, 0.0);            // points toward Earth at origin
    EXPECT_NEAR(ship.acceleration.z, 0.0, 1e-6);
}

TEST_F(EarthOnlyCenteredTest, ProjectileReceivesGravityAfterTick)
{
    spaceship::server::ShipSpawnRequest request {};
    request.transform.position = {kLeoRadius, 0.0, 0.0};
    const auto shipId = server.spawnShip(request);
    server.updateShipControl(shipId, {0.0, {1.0, 0.0, 0.0, 0.0}, true});
    server.tick();

    ASSERT_EQ(server.world().projectiles.size(), 1U);
    const auto& proj = server.world().projectiles.front();
    const double aMag = spaceship::server::length(proj.acceleration);
    EXPECT_GT(aMag, 0.0);
}

TEST_F(EarthOnlyCenteredTest, GravityNotAppliedToMassiveBodies)
{
    spaceship::server::ShipSpawnRequest request {};
    request.transform.position = {kLeoRadius, 0.0, 0.0};
    server.spawnShip(request);
    server.tick();

    // Earth should have no acceleration field (OrbitalParams drives its motion)
    const auto& earth = server.world().massiveBodies[0];
    EXPECT_DOUBLE_EQ(earth.velocity.linear.x, 0.0); // Earth at origin has zero velocity
}

TEST_F(EarthOnlyCenteredTest, GravityEventsNoLongerPushed)
{
    spaceship::server::ShipSpawnRequest request {};
    request.transform.position = {kLeoRadius, 0.0, 0.0};
    server.spawnShip(request);
    server.tick();

    EXPECT_TRUE(server.world().events.empty());
}

// --- Edge cases ---

TEST_F(EarthOnlyCenteredTest, ShipExactlyAtMassiveBodyCenterDoesNotCrash)
{
    // Ship placed exactly at Earth's center — division-by-zero guard
    spaceship::server::ShipSpawnRequest request {};
    request.transform.position = {0.0, 0.0, 0.0};
    server.spawnShip(request);
    EXPECT_NO_FATAL_FAILURE(server.tick());
}

TEST_F(EarthOnlyCenteredTest, ShipWithZeroVelocityFallsAlongRadialLine)
{
    // No velocity, no angular momentum → falls straight toward Earth core along x-axis
    spaceship::server::ShipSpawnRequest request {};
    request.transform.position = {kLeoRadius, 0.0, 0.0};
    // zero velocity (default)
    server.spawnShip(request);

    for (int i = 0; i < 600; ++i)
        server.tick();

    const auto& ship = server.world().ships.front();
    EXPECT_NEAR(ship.transform.position.y, 0.0, 1.0); // no lateral drift
    EXPECT_NEAR(ship.transform.position.z, 0.0, 1.0);
    EXPECT_LT(ship.transform.position.x, kLeoRadius);  // moved toward Earth
}

TEST_F(EarthOnlyCenteredTest, VerletVelocityUsesAverageOfOldAndNewAcceleration)
{
    // Verlet: v_{n+1} = v_n + 0.5*(a_n + a_{n+1})*dt
    // Ship falls from rest at LEO. a_n = -g(r_n), a_{n+1} = -g(r_{n+1}).
    // After one tick r_{n+1} < r_n, so |a_{n+1}| > |a_n| (closer to Earth).
    // Verlet velocity must be strictly larger than simple Euler v = v_n + a_n*dt.
    const double gAtLeo = kEarthMu / (kLeoRadius * kLeoRadius);
    const double dt = spaceship::shared::constants::kFixedDeltaSeconds;

    spaceship::server::ShipSpawnRequest request {};
    request.transform.position = {kLeoRadius, 0.0, 0.0}; // zero velocity
    server.spawnShip(request);

    server.tick();

    const auto& ship = server.world().ships.front();
    // Velocity gained in -x (toward Earth). Magnitude must exceed simple Euler a_n*dt
    // because a_{n+1} > a_n (ship moved closer to Earth during phase 1).
    const double eulerVelocity = gAtLeo * dt; // lower bound
    EXPECT_GT(std::abs(ship.velocity.linear.x), eulerVelocity);
    // And it must be less than 2*a_n*dt (strict upper bound — acceleration cannot double)
    EXPECT_LT(std::abs(ship.velocity.linear.x), 2.0 * eulerVelocity);
    // No lateral velocity generated for a purely radial fall
    EXPECT_NEAR(ship.velocity.linear.y, 0.0, 1e-9);
}

TEST_F(EarthOnlyCenteredTest, LEOCircularOrbitMaintainsRadius600Ticks)
{
    // Circular orbit at LEO: v = sqrt(μ/r) ≈ 7667 m/s tangential (+y direction)
    const double vOrbit = std::sqrt(kEarthMu / kLeoRadius);

    spaceship::server::ShipSpawnRequest request {};
    request.transform.position = {kLeoRadius, 0.0, 0.0};
    request.velocity = {{0.0, vOrbit, 0.0}};
    server.spawnShip(request);

    for (int i = 0; i < 600; ++i)
        server.tick();

    const auto& ship = server.world().ships.front();
    const double radius = spaceship::server::length(ship.transform.position);
    EXPECT_NEAR(radius, kLeoRadius, 200.0); // ±200 m tolerance
}

// ---------------------------------------------------------------------------
// SimulationMathTest — cross, normalize, negate, projectOntoPlane
// ---------------------------------------------------------------------------

TEST(SimulationMathTest, CrossProductOfXAndYIsZ)
{
    const spaceship::shared::Vec3 x {1.0, 0.0, 0.0};
    const spaceship::shared::Vec3 y {0.0, 1.0, 0.0};
    const auto result = spaceship::server::cross(x, y);
    EXPECT_DOUBLE_EQ(result.x, 0.0);
    EXPECT_DOUBLE_EQ(result.y, 0.0);
    EXPECT_DOUBLE_EQ(result.z, 1.0);
}

TEST(SimulationMathTest, CrossProductIsAntiCommutative)
{
    const spaceship::shared::Vec3 a {1.0, 2.0, 3.0};
    const spaceship::shared::Vec3 b {4.0, 5.0, 6.0};
    const auto ab = spaceship::server::cross(a, b);
    const auto ba = spaceship::server::cross(b, a);
    EXPECT_DOUBLE_EQ(ab.x, -ba.x);
    EXPECT_DOUBLE_EQ(ab.y, -ba.y);
    EXPECT_DOUBLE_EQ(ab.z, -ba.z);
}

TEST(SimulationMathTest, CrossProductOfParallelVectorsIsZero)
{
    const spaceship::shared::Vec3 a {3.0, 0.0, 0.0};
    const spaceship::shared::Vec3 b {7.0, 0.0, 0.0};
    const auto result = spaceship::server::cross(a, b);
    EXPECT_DOUBLE_EQ(result.x, 0.0);
    EXPECT_DOUBLE_EQ(result.y, 0.0);
    EXPECT_DOUBLE_EQ(result.z, 0.0);
}

TEST(SimulationMathTest, NormalizeUnitVector)
{
    const spaceship::shared::Vec3 v {0.0, 5.0, 0.0};
    const auto result = spaceship::server::normalize(v);
    EXPECT_DOUBLE_EQ(result.x, 0.0);
    EXPECT_DOUBLE_EQ(result.y, 1.0);
    EXPECT_DOUBLE_EQ(result.z, 0.0);
}

TEST(SimulationMathTest, NormalizeZeroVectorReturnsZero)
{
    const spaceship::shared::Vec3 zero {};
    const auto result = spaceship::server::normalize(zero);
    EXPECT_DOUBLE_EQ(result.x, 0.0);
    EXPECT_DOUBLE_EQ(result.y, 0.0);
    EXPECT_DOUBLE_EQ(result.z, 0.0);
}

TEST(SimulationMathTest, NormalizeProducesUnitLength)
{
    const spaceship::shared::Vec3 v {3.0, 4.0, 12.0};
    const auto result = spaceship::server::normalize(v);
    EXPECT_NEAR(spaceship::server::length(result), 1.0, 1e-15);
}

TEST(SimulationMathTest, NegateVec3FlipsSigns)
{
    const spaceship::shared::Vec3 v {1.0, -2.0, 3.0};
    const auto result = spaceship::server::negate(v);
    EXPECT_DOUBLE_EQ(result.x, -1.0);
    EXPECT_DOUBLE_EQ(result.y, 2.0);
    EXPECT_DOUBLE_EQ(result.z, -3.0);
}

TEST(SimulationMathTest, ProjectOntoPlaneRemovesNormalComponent)
{
    const spaceship::shared::Vec3 v {1.0, 2.0, 3.0};
    const spaceship::shared::Vec3 n {0.0, 1.0, 0.0}; // y-normal plane
    const auto result = spaceship::server::projectOntoPlane(v, n);
    EXPECT_DOUBLE_EQ(result.x, 1.0);
    EXPECT_NEAR(result.y, 0.0, 1e-15);
    EXPECT_DOUBLE_EQ(result.z, 3.0);
}

// ---------------------------------------------------------------------------
// OrbitFittingTest — Keplerian element computation
// ---------------------------------------------------------------------------

class OrbitFittingTest : public ::testing::Test
{
  protected:
    static constexpr double kEarthMu = 3.986004418e14;
    static constexpr double kEarthRadius = 6.371e6;
    static constexpr double kLeoRadius = kEarthRadius + 400'000.0;
    static constexpr spaceship::shared::NetId kEarthNetId = 1U;
    static constexpr spaceship::shared::Tick kTick = 42U;
};

TEST_F(OrbitFittingTest, CircularOrbitProducesZeroEccentricity)
{
    // Ship at (r, 0, 0) with v = (0, v_circ, 0) → circular orbit
    const double vCirc = std::sqrt(kEarthMu / kLeoRadius);
    const spaceship::shared::Vec3 r {kLeoRadius, 0.0, 0.0};
    const spaceship::shared::Vec3 v {0.0, vCirc, 0.0};

    const auto cache = spaceship::server::fitOrbit(r, v, kEarthMu, kEarthRadius, kEarthNetId, kTick);

    EXPECT_TRUE(cache.isElliptic);
    EXPECT_NEAR(cache.eccentricity, 0.0, 1e-6);
    EXPECT_NEAR(cache.semiMajorAxis, kLeoRadius, 1.0);
    EXPECT_NEAR(cache.semiMinorAxis, kLeoRadius, 1.0);
    EXPECT_NEAR(cache.periapsisRadius, kLeoRadius, 1.0);
    EXPECT_NEAR(cache.apoapsisRadius, kLeoRadius, 1.0);
}

TEST_F(OrbitFittingTest, CircularOrbitNormalPointsAlongAngularMomentum)
{
    // Orbit in x-y plane → angular momentum along +z
    const double vCirc = std::sqrt(kEarthMu / kLeoRadius);
    const spaceship::shared::Vec3 r {kLeoRadius, 0.0, 0.0};
    const spaceship::shared::Vec3 v {0.0, vCirc, 0.0};

    const auto cache = spaceship::server::fitOrbit(r, v, kEarthMu, kEarthRadius, kEarthNetId, kTick);

    EXPECT_NEAR(cache.orbitNormal.z, 1.0, 1e-10);
    EXPECT_NEAR(cache.orbitNormal.x, 0.0, 1e-10);
    EXPECT_NEAR(cache.orbitNormal.y, 0.0, 1e-10);
}

TEST_F(OrbitFittingTest, EllipticalOrbitCorrectSemiMajorAxisAndEccentricity)
{
    // Ship at periapsis: r = r_p, v = v_p (tangential)
    // Choose e = 0.1, a = kLeoRadius / (1 - e)
    constexpr double kEccentricity = 0.1;
    const double a = kLeoRadius / (1.0 - kEccentricity);
    // v at periapsis: v_p = sqrt(μ * (2/r - 1/a))
    const double vP = std::sqrt(kEarthMu * (2.0 / kLeoRadius - 1.0 / a));

    const spaceship::shared::Vec3 r {kLeoRadius, 0.0, 0.0};
    const spaceship::shared::Vec3 v {0.0, vP, 0.0};

    const auto cache = spaceship::server::fitOrbit(r, v, kEarthMu, kEarthRadius, kEarthNetId, kTick);

    EXPECT_TRUE(cache.isElliptic);
    EXPECT_NEAR(cache.eccentricity, kEccentricity, 1e-6);
    EXPECT_NEAR(cache.semiMajorAxis, a, 10.0);
    EXPECT_NEAR(cache.periapsisRadius, kLeoRadius, 10.0);
    EXPECT_NEAR(cache.apoapsisRadius, a * (1.0 + kEccentricity), 10.0);
}

TEST_F(OrbitFittingTest, EllipticalOrbitTrueAnomalyAtPeriapsisIsZero)
{
    constexpr double kEccentricity = 0.1;
    const double a = kLeoRadius / (1.0 - kEccentricity);
    const double vP = std::sqrt(kEarthMu * (2.0 / kLeoRadius - 1.0 / a));

    const spaceship::shared::Vec3 r {kLeoRadius, 0.0, 0.0};
    const spaceship::shared::Vec3 v {0.0, vP, 0.0};

    const auto cache = spaceship::server::fitOrbit(r, v, kEarthMu, kEarthRadius, kEarthNetId, kTick);

    EXPECT_NEAR(cache.trueAnomaly, 0.0, 1e-6);
    EXPECT_NEAR(cache.eccentricAnomaly, 0.0, 1e-6);
    EXPECT_NEAR(cache.meanAnomaly, 0.0, 1e-6);
}

TEST_F(OrbitFittingTest, EllipseGeometryIsConsistent)
{
    constexpr double kEccentricity = 0.3;
    const double a = kLeoRadius / (1.0 - kEccentricity);
    const double vP = std::sqrt(kEarthMu * (2.0 / kLeoRadius - 1.0 / a));

    const spaceship::shared::Vec3 r {kLeoRadius, 0.0, 0.0};
    const spaceship::shared::Vec3 v {0.0, vP, 0.0};

    const auto cache = spaceship::server::fitOrbit(r, v, kEarthMu, kEarthRadius, kEarthNetId, kTick);

    // b = a * sqrt(1 - e²)
    const double expectedB = cache.semiMajorAxis * std::sqrt(1.0 - cache.eccentricity * cache.eccentricity);
    EXPECT_NEAR(cache.semiMinorAxis, expectedB, 1.0);

    // Ellipse center = -a*e * p_hat
    const double centerDist = spaceship::server::length(cache.ellipseCenter);
    EXPECT_NEAR(centerDist, cache.semiMajorAxis * cache.eccentricity, 10.0);

    // p_hat and q_hat are orthonormal
    EXPECT_NEAR(spaceship::server::dot(cache.periapsisDirection, cache.sideDirection), 0.0, 1e-10);
    EXPECT_NEAR(spaceship::server::length(cache.periapsisDirection), 1.0, 1e-10);
    EXPECT_NEAR(spaceship::server::length(cache.sideDirection), 1.0, 1e-10);

    // h_hat is perpendicular to both
    EXPECT_NEAR(spaceship::server::dot(cache.orbitNormal, cache.periapsisDirection), 0.0, 1e-10);
    EXPECT_NEAR(spaceship::server::dot(cache.orbitNormal, cache.sideDirection), 0.0, 1e-10);
}

TEST_F(OrbitFittingTest, HyperbolicStateIsNotElliptic)
{
    // Escape velocity at LEO: v > sqrt(2μ/r)
    const double vEscape = std::sqrt(2.0 * kEarthMu / kLeoRadius);
    const spaceship::shared::Vec3 r {kLeoRadius, 0.0, 0.0};
    const spaceship::shared::Vec3 v {0.0, vEscape * 1.5, 0.0}; // well above escape

    const auto cache = spaceship::server::fitOrbit(r, v, kEarthMu, kEarthRadius, kEarthNetId, kTick);

    EXPECT_FALSE(cache.isElliptic);
}

TEST_F(OrbitFittingTest, ZeroVelocityIsNotElliptic)
{
    // Zero velocity → degenerate (collinear r, v; zero angular momentum)
    const spaceship::shared::Vec3 r {kLeoRadius, 0.0, 0.0};
    const spaceship::shared::Vec3 v {0.0, 0.0, 0.0};

    const auto cache = spaceship::server::fitOrbit(r, v, kEarthMu, kEarthRadius, kEarthNetId, kTick);

    EXPECT_FALSE(cache.isElliptic);
}

TEST_F(OrbitFittingTest, RadialVelocityIsNotElliptic)
{
    // Purely radial velocity → zero angular momentum
    const spaceship::shared::Vec3 r {kLeoRadius, 0.0, 0.0};
    const spaceship::shared::Vec3 v {100.0, 0.0, 0.0}; // along r

    const auto cache = spaceship::server::fitOrbit(r, v, kEarthMu, kEarthRadius, kEarthNetId, kTick);

    EXPECT_FALSE(cache.isElliptic);
}

TEST_F(OrbitFittingTest, NonEllipticStillStoresRelativeStateAndAltitude)
{
    const spaceship::shared::Vec3 r {kLeoRadius, 0.0, 0.0};
    const spaceship::shared::Vec3 v {0.0, 0.0, 0.0};

    const auto cache = spaceship::server::fitOrbit(r, v, kEarthMu, kEarthRadius, kEarthNetId, kTick);

    EXPECT_DOUBLE_EQ(cache.relativePosition.x, kLeoRadius);
    EXPECT_DOUBLE_EQ(cache.relativeVelocity.x, 0.0);
    EXPECT_NEAR(cache.altitudeMeters, 400'000.0, 1.0);
    EXPECT_EQ(cache.referenceBodyId, kEarthNetId);
    EXPECT_EQ(cache.epoch, kTick);
}

TEST_F(OrbitFittingTest, CircularOrbitPeriapsisDirectionFallsBackToPositionVector)
{
    // Near-circular orbit (e ≈ 0) → periapsis direction undefined.
    // Fallback should use normalized position projected onto orbital plane.
    const double vCirc = std::sqrt(kEarthMu / kLeoRadius);
    const spaceship::shared::Vec3 r {kLeoRadius, 0.0, 0.0};
    const spaceship::shared::Vec3 v {0.0, vCirc, 0.0};

    const auto cache = spaceship::server::fitOrbit(r, v, kEarthMu, kEarthRadius, kEarthNetId, kTick);

    // p_hat should point along +x (direction of r projected onto orbit plane)
    EXPECT_NEAR(cache.periapsisDirection.x, 1.0, 1e-6);
    EXPECT_NEAR(cache.periapsisDirection.y, 0.0, 1e-6);
}

// ---------------------------------------------------------------------------
// QualityScoreTest
// ---------------------------------------------------------------------------

TEST(QualityScoreTest, PureTwoBodyGivesScoreNearOne)
{
    // Acceleration is exactly the two-body acceleration → no perturbation
    constexpr double kMu = 3.986004418e14;
    constexpr double kR = 6.771e6;
    const spaceship::shared::Vec3 r {kR, 0.0, 0.0};
    // Two-body accel: a = -μ/r² in -x direction
    const spaceship::shared::Vec3 a {-kMu / (kR * kR), 0.0, 0.0};

    const double score = spaceship::server::computeQualityScore(a, r, kMu);

    EXPECT_GT(score, 0.95);
    EXPECT_LE(score, 1.0);
}

TEST(QualityScoreTest, LargePerturbationReducesScore)
{
    constexpr double kMu = 3.986004418e14;
    constexpr double kR = 6.771e6;
    const spaceship::shared::Vec3 r {kR, 0.0, 0.0};
    // Add a large perpendicular perturbation
    const double twoBodyAccel = kMu / (kR * kR);
    const spaceship::shared::Vec3 a {-twoBodyAccel, twoBodyAccel * 0.5, 0.0};

    const double score = spaceship::server::computeQualityScore(a, r, kMu);

    EXPECT_LT(score, 0.8);
    EXPECT_GE(score, 0.0);
}

TEST(QualityScoreTest, ScoreIsClampedBetweenZeroAndOne)
{
    constexpr double kMu = 3.986004418e14;
    constexpr double kR = 6.771e6;
    const spaceship::shared::Vec3 r {kR, 0.0, 0.0};
    // Extreme perturbation
    const spaceship::shared::Vec3 a {1e10, 1e10, 1e10};

    const double score = spaceship::server::computeQualityScore(a, r, kMu);

    EXPECT_GE(score, 0.0);
    EXPECT_LE(score, 1.0);
}

// ---------------------------------------------------------------------------
// ReferenceBodySelectorTest — hysteresis-based body selection
// ---------------------------------------------------------------------------

class ReferenceBodySelectorTest : public ::testing::Test
{
  protected:
    // Two bodies: Sun at origin, Earth at 1 AU on +x
    static constexpr double kSunMu = 1.32712440018e20;
    static constexpr double kEarthMu = 3.986004418e14;
    static constexpr double kAU = 149'597'870'700.0;
    static constexpr spaceship::shared::NetId kSunNetId = 0U;
    static constexpr spaceship::shared::NetId kEarthNetId = 1U;

    spaceship::server::SimulationConfig config {};

    std::vector<spaceship::server::MassiveBodyState> makeSunEarth() const
    {
        return {
            spaceship::server::MassiveBodyState {
                {kSunNetId, "Sun", kSunMu, 6.9634e8}, {{0.0, 0.0, 0.0}}, {}, {}},
            spaceship::server::MassiveBodyState {
                {kEarthNetId, "Earth", kEarthMu, 6.371e6}, {{kAU, 0.0, 0.0}}, {}, {}},
        };
    }
};

TEST_F(ReferenceBodySelectorTest, FirstUpdateSelectsDominantBody)
{
    auto bodies = makeSunEarth();
    spaceship::server::ReferenceBodySelector selector;

    // Ship in LEO around Earth — Earth dominates
    const spaceship::shared::Vec3 shipPos {kAU + 6.771e6, 0.0, 0.0};
    const auto result = selector.update(shipPos, bodies, config, 1.0 / 60.0);

    EXPECT_EQ(result.bodyId, kEarthNetId);
    EXPECT_GT(result.score, 0.5);
}

TEST_F(ReferenceBodySelectorTest, ShipInDeepSpaceSelectsSun)
{
    auto bodies = makeSunEarth();
    spaceship::server::ReferenceBodySelector selector;

    // Ship far from both, but closer to Sun in gravity terms (midway)
    const spaceship::shared::Vec3 shipPos {kAU * 0.5, 0.0, 0.0};
    const auto result = selector.update(shipPos, bodies, config, 1.0 / 60.0);

    EXPECT_EQ(result.bodyId, kSunNetId);
}

TEST_F(ReferenceBodySelectorTest, HysteresisPreventsImmediateSwitch)
{
    auto bodies = makeSunEarth();
    spaceship::server::ReferenceBodySelector selector;
    const double dt = 1.0 / 60.0;

    // Start in LEO — locks to Earth
    const spaceship::shared::Vec3 leoPos {kAU + 6.771e6, 0.0, 0.0};
    (void)selector.update(leoPos, bodies, config, dt);

    // Move to a position where Sun marginally dominates (just past SOI boundary)
    // Earth SOI ≈ 0.929e9 m. Place ship at kAU - 1e9 (just outside Earth SOI)
    const spaceship::shared::Vec3 boundaryPos {kAU - 1.0e9, 0.0, 0.0};

    // Single update should NOT switch (hysteresis dwell not met)
    const auto result = selector.update(boundaryPos, bodies, config, dt);

    EXPECT_EQ(result.bodyId, kEarthNetId); // stays with Earth
    EXPECT_FALSE(result.changed);
}

TEST_F(ReferenceBodySelectorTest, SwitchesAfterDwellTimeExceeded)
{
    auto bodies = makeSunEarth();
    spaceship::server::ReferenceBodySelector selector;
    const double dt = 1.0 / 60.0;

    // Start in LEO — locks to Earth
    const spaceship::shared::Vec3 leoPos {kAU + 6.771e6, 0.0, 0.0};
    (void)selector.update(leoPos, bodies, config, dt);

    // Move to deep space where Sun clearly dominates
    const spaceship::shared::Vec3 deepSpace {kAU * 0.5, 0.0, 0.0};

    // Accumulate dwell time over many updates
    bool switchOccurred = false;
    spaceship::server::ReferenceBodySelection lastResult {};
    const int dwellTicks = static_cast<int>(config.referenceBodyDwellTimeSeconds / dt) + 10;
    for (int i = 0; i < dwellTicks; ++i)
    {
        lastResult = selector.update(deepSpace, bodies, config, dt);
        if (lastResult.changed)
            switchOccurred = true;
    }

    EXPECT_EQ(lastResult.bodyId, kSunNetId);
    EXPECT_TRUE(switchOccurred);
}

TEST_F(ReferenceBodySelectorTest, DwellResetWhenAdvantageIsLost)
{
    auto bodies = makeSunEarth();
    spaceship::server::ReferenceBodySelector selector;
    const double dt = 1.0 / 60.0;

    // Start in LEO — locks to Earth
    const spaceship::shared::Vec3 leoPos {kAU + 6.771e6, 0.0, 0.0};
    (void)selector.update(leoPos, bodies, config, dt);

    // Move to deep space for partial dwell (less than dwellTime)
    const spaceship::shared::Vec3 deepSpace {kAU * 0.5, 0.0, 0.0};
    const int partialTicks = static_cast<int>(config.referenceBodyDwellTimeSeconds / dt) / 2;
    for (int i = 0; i < partialTicks; ++i)
        (void)selector.update(deepSpace, bodies, config, dt);

    // Return to LEO — dwell should reset
    (void)selector.update(leoPos, bodies, config, dt);

    // Go back to deep space — need full dwell again
    spaceship::server::ReferenceBodySelection result {};
    for (int i = 0; i < partialTicks; ++i)
        result = selector.update(deepSpace, bodies, config, dt);

    EXPECT_EQ(result.bodyId, kEarthNetId); // still Earth — dwell was reset
}

// ---------------------------------------------------------------------------
// OrbitCacheIntegrationTest — OrbitCache on ships via the tick pipeline
// ---------------------------------------------------------------------------

TEST_F(EarthOnlyCenteredTest, LEOShipOrbitCacheIsEllipticAfterTick)
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

TEST_F(EarthOnlyCenteredTest, OrbitCacheAltitudeMatchesLEO)
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

TEST_F(EarthOnlyCenteredTest, ActiveShipCacheRefreshesEveryTick)
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

TEST_F(EarthOnlyCenteredTest, InactiveShipCacheDoesNotRefreshEveryTick)
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

TEST_F(EarthOnlyCenteredTest, ThrustStartInvalidatesOrbitCache)
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

TEST_F(EarthOnlyCenteredTest, OrbitCacheQualityScoreIsHighForCircularOrbit)
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

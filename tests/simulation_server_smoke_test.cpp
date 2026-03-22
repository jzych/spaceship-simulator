#include "server/bootstrap.hpp"
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

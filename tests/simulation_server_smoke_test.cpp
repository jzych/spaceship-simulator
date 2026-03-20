#include "server/bootstrap.hpp"
#include "server/simulation_server.hpp"

#include <gtest/gtest.h>
#include <cmath>
#include <numbers>

namespace
{

constexpr double kOneTickSeconds = spaceship::shared::constants::kFixedDeltaSeconds;
constexpr double kDefaultShipAccelerationMetersPerSecondSquared = 20'000.0 / 1'000.0;
constexpr double kExpectedVelocityDeltaPerTick = kDefaultShipAccelerationMetersPerSecondSquared * kOneTickSeconds;
constexpr double kExpectedPositionDeltaPerTick = 0.5 * kDefaultShipAccelerationMetersPerSecondSquared * kOneTickSeconds *
    kOneTickSeconds;
constexpr double kQuarterTurnZAxisHalfAngleComponent = std::numbers::sqrt2_v<double> / 2.0;
constexpr double kDefaultProjectileMuzzleSpeedMetersPerSecond = 1'000.0;
constexpr double kFarFieldPositionXMeters = 2.0 * spaceship::server::kEarthOrbitRadiusMeters;
constexpr spaceship::shared::NetId kSingleMassNetId = 99U;
constexpr double kSingleMassMuMetersCubedPerSecondSquared = 3.986004418e14;
constexpr double kSingleMassRadiusMeters = 6.371e6;
constexpr double kSingleMassOrbitRadiusMeters = 1.0e7;
constexpr double kPositionTolerance = 1e-3;
constexpr double kVelocityTolerance = 1e-6;

double earthAngularSpeedRadiansPerSecond()
{
    return spaceship::server::kEarthOrbitalSpeedMetersPerSecond / spaceship::server::kEarthOrbitRadiusMeters;
}

double moonAngularSpeedRadiansPerSecond()
{
    return spaceship::server::kMoonOrbitalSpeedRelativeToEarthMetersPerSecond / spaceship::server::kMoonOrbitRadiusMeters;
}

spaceship::shared::Vec3 circularOrbitPosition(double radiusMeters, double angleRadians)
{
    return {radiusMeters * std::cos(angleRadians), radiusMeters * std::sin(angleRadians), 0.0};
}

spaceship::shared::Vec3 circularOrbitVelocity(double radiusMeters, double angularSpeedRadiansPerSecond, double angleRadians)
{
    return {
        -radiusMeters * angularSpeedRadiansPerSecond * std::sin(angleRadians),
        radiusMeters * angularSpeedRadiansPerSecond * std::cos(angleRadians),
        0.0,
    };
}

double circularOrbitSpeed(double muMetersCubedPerSecondSquared, double radiusMeters)
{
    return std::sqrt(muMetersCubedPerSecondSquared / radiusMeters);
}

double radialDistanceMeters(const spaceship::shared::Vec3& position)
{
    return std::sqrt(position.x * position.x + position.y * position.y + position.z * position.z);
}

double orbitalSpecificEnergy(
    const spaceship::shared::Vec3& position,
    const spaceship::shared::Vec3& velocity,
    double muMetersCubedPerSecondSquared)
{
    const double speedSquared = velocity.x * velocity.x + velocity.y * velocity.y + velocity.z * velocity.z;
    return 0.5 * speedSquared - muMetersCubedPerSecondSquared / radialDistanceMeters(position);
}

void runTicks(spaceship::server::SimulationServer& server, int tickCount)
{
    for (int tickIndex = 0; tickIndex < tickCount; ++tickIndex)
    {
        server.tick();
    }
}

} // namespace

class SimulationServerSmokeTest : public ::testing::Test
{
  protected:
    spaceship::server::SimulationServer server {};
};

class EarthOnlyServerTest : public ::testing::Test
{
  protected:
    spaceship::server::SimulationServer server {
        spaceship::server::SimulationConfig {},
        spaceship::server::BootstrapWorld::EarthOnly,
    };
};

TEST_F(SimulationServerSmokeTest, GivenDefaultServerWhenBootstrappedThenItContainsThreeMassiveBodies)
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

TEST(SimulationServerBootstrapTest, GivenEarthOnlyPresetWhenServerBootstrapsThenItContainsSingleEarthBody)
{
    const spaceship::server::SimulationServer earthOnlyServer {
        spaceship::server::SimulationConfig {},
        spaceship::server::BootstrapWorld::EarthOnly,
    };

    ASSERT_EQ(earthOnlyServer.world().massiveBodies.size(), 1U);
    EXPECT_EQ(earthOnlyServer.world().massiveBodies[0].definition.name, "Earth");
    EXPECT_DOUBLE_EQ(earthOnlyServer.world().massiveBodies[0].transform.position.x, 0.0);
    EXPECT_DOUBLE_EQ(earthOnlyServer.world().massiveBodies[0].velocity.linear.y, 0.0);
}

TEST(SimulationServerBootstrapTest, GivenSunEarthPresetWhenServerBootstrapsThenItContainsSunAndEarth)
{
    const spaceship::server::SimulationServer sunEarthServer {
        spaceship::server::SimulationConfig {},
        spaceship::server::BootstrapWorld::SunEarth,
    };

    ASSERT_EQ(sunEarthServer.world().massiveBodies.size(), 2U);
    EXPECT_EQ(sunEarthServer.world().massiveBodies[0].definition.name, "Sun");
    EXPECT_EQ(sunEarthServer.world().massiveBodies[1].definition.name, "Earth");
}

TEST_F(SimulationServerSmokeTest, GivenServerWhenTickRunsThenServerClockAdvances)
{
    server.tick();

    EXPECT_EQ(server.tickCount(), 1U);
    EXPECT_TRUE(server.lastSnapshotSummary().empty());
}

TEST_F(SimulationServerSmokeTest, GivenSpawnRequestWhenShipSpawnsThenWorldContainsShipState)
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
    EXPECT_DOUBLE_EQ(ship.acceleration.x, ship.gravityAcceleration.x);
    EXPECT_DOUBLE_EQ(ship.acceleration.y, ship.gravityAcceleration.y);
    EXPECT_DOUBLE_EQ(ship.acceleration.z, ship.gravityAcceleration.z);
    EXPECT_DOUBLE_EQ(ship.thrustAcceleration.x, 0.0);
    EXPECT_NE(ship.gravityAcceleration.x, 0.0);
    EXPECT_DOUBLE_EQ(ship.massProperties.massKg, 1'000.0);
}

TEST_F(SimulationServerSmokeTest, GivenMissingShipWhenControlUpdatesThenServerReturnsFalse)
{
    constexpr spaceship::shared::NetId kMissingShipNetId = 99U;

    const bool updated =
        server.updateShipControl(kMissingShipNetId, spaceship::shared::ShipControl {1.0, {1.0, 0.0, 0.0, 0.0}, true});

    EXPECT_FALSE(updated);
}

TEST_F(SimulationServerSmokeTest, GivenTwoShipSpawnsWhenIdsAreAssignedThenTheyAreSequential)
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

TEST_F(SimulationServerSmokeTest, GivenDuplicateShipRequestsWhenShipsSpawnThenAssignedIdsRemainUnique)
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

TEST_F(SimulationServerSmokeTest, GivenExistingShipWhenControlUpdatesThenStoredControlStateIsReplaced)
{
    const spaceship::server::ShipSpawnRequest request {
        {{1.0, 2.0, 3.0}, {1.0, 0.0, 0.0, 0.0}},
        {{4.0, 5.0, 6.0}},
    };
    const spaceship::shared::ShipControl control {0.75, {0.0, 0.0, 1.0, 0.0}, true};

    const auto shipNetId = server.spawnShip(request);
    const bool updated = server.updateShipControl(shipNetId, control);

    ASSERT_TRUE(updated);
    ASSERT_EQ(server.world().ships.size(), 1U);
    const auto& ship = server.world().ships.front();
    EXPECT_DOUBLE_EQ(ship.control.throttle, 0.75);
    EXPECT_DOUBLE_EQ(ship.control.desiredOrientation.y, 1.0);
    EXPECT_TRUE(ship.control.fire);
}

TEST_F(SimulationServerSmokeTest, GivenForwardThrottleWhenTickRunsThenShipAcceleratesForward)
{
    const spaceship::server::ShipSpawnRequest request {
        {{kFarFieldPositionXMeters, 0.0, 0.0}, {1.0, 0.0, 0.0, 0.0}},
        {{0.0, 0.0, 0.0}},
    };

    const auto shipNetId = server.spawnShip(request);
    ASSERT_TRUE(server.updateShipControl(
        shipNetId,
        spaceship::shared::ShipControl {1.0, {1.0, 0.0, 0.0, 0.0}, false}));

    server.tick();

    ASSERT_EQ(server.world().ships.size(), 1U);
    const auto& ship = server.world().ships.front();
    EXPECT_GT(ship.velocity.linear.x, 0.0);
    EXPECT_NEAR(ship.velocity.linear.x, kExpectedVelocityDeltaPerTick, 1e-4);
    EXPECT_GT(ship.transform.position.x, kFarFieldPositionXMeters);
    EXPECT_NEAR(ship.transform.position.x, kFarFieldPositionXMeters + kExpectedPositionDeltaPerTick, 1e-4);
    EXPECT_GT(ship.acceleration.x, 0.0);
    EXPECT_NEAR(ship.acceleration.x, kDefaultShipAccelerationMetersPerSecondSquared, 1e-2);
    EXPECT_NEAR(ship.velocity.linear.y, 0.0, 1e-6);
    EXPECT_DOUBLE_EQ(ship.velocity.linear.z, 0.0);
}

TEST_F(SimulationServerSmokeTest, GivenDesiredOrientationWhenTickRunsThenThrustUsesThatOrientation)
{
    const spaceship::server::ShipSpawnRequest request {
        {{kFarFieldPositionXMeters, 0.0, 0.0}, {1.0, 0.0, 0.0, 0.0}},
        {{0.0, 0.0, 0.0}},
    };

    const auto shipNetId = server.spawnShip(request);
    ASSERT_TRUE(server.updateShipControl(
        shipNetId,
        spaceship::shared::ShipControl {
            1.0,
            {kQuarterTurnZAxisHalfAngleComponent, 0.0, 0.0, kQuarterTurnZAxisHalfAngleComponent},
            false,
        }));

    server.tick();

    ASSERT_EQ(server.world().ships.size(), 1U);
    const auto& ship = server.world().ships.front();
    EXPECT_NEAR(ship.transform.orientation.w, kQuarterTurnZAxisHalfAngleComponent, 1e-12);
    EXPECT_LT(ship.velocity.linear.x, 0.0);
    EXPECT_GT(ship.velocity.linear.y, 0.0);
    EXPECT_NEAR(ship.velocity.linear.y, kExpectedVelocityDeltaPerTick, 1e-4);
    EXPECT_NEAR(ship.transform.position.y, kExpectedPositionDeltaPerTick, 1e-4);
}

TEST_F(SimulationServerSmokeTest, GivenDefaultWorldWhenTickRunsThenMassiveBodiesFollowPredefinedPaths)
{
    const double earthAngleRadians = earthAngularSpeedRadiansPerSecond() * kOneTickSeconds;
    const double moonAngleRadians = moonAngularSpeedRadiansPerSecond() * kOneTickSeconds;
    const auto expectedEarthPosition =
        circularOrbitPosition(spaceship::server::kEarthOrbitRadiusMeters, earthAngleRadians);
    const auto expectedEarthVelocity = circularOrbitVelocity(
        spaceship::server::kEarthOrbitRadiusMeters,
        earthAngularSpeedRadiansPerSecond(),
        earthAngleRadians);
    const auto expectedMoonRelativePosition =
        circularOrbitPosition(spaceship::server::kMoonOrbitRadiusMeters, moonAngleRadians);
    const auto expectedMoonRelativeVelocity = circularOrbitVelocity(
        spaceship::server::kMoonOrbitRadiusMeters,
        moonAngularSpeedRadiansPerSecond(),
        moonAngleRadians);

    server.tick();

    ASSERT_EQ(server.world().massiveBodies.size(), 3U);
    const auto& sun = server.world().massiveBodies[0];
    const auto& earth = server.world().massiveBodies[1];
    const auto& moon = server.world().massiveBodies[2];
    EXPECT_DOUBLE_EQ(sun.transform.position.x, 0.0);
    EXPECT_DOUBLE_EQ(sun.velocity.linear.y, 0.0);
    EXPECT_NEAR(earth.transform.position.x, expectedEarthPosition.x, kPositionTolerance);
    EXPECT_NEAR(earth.transform.position.y, expectedEarthPosition.y, kPositionTolerance);
    EXPECT_NEAR(earth.velocity.linear.x, expectedEarthVelocity.x, kVelocityTolerance);
    EXPECT_NEAR(earth.velocity.linear.y, expectedEarthVelocity.y, kVelocityTolerance);
    EXPECT_NEAR(
        moon.transform.position.x,
        expectedEarthPosition.x + expectedMoonRelativePosition.x,
        kPositionTolerance);
    EXPECT_NEAR(
        moon.transform.position.y,
        expectedEarthPosition.y + expectedMoonRelativePosition.y,
        kPositionTolerance);
    EXPECT_NEAR(
        moon.velocity.linear.x,
        expectedEarthVelocity.x + expectedMoonRelativeVelocity.x,
        kVelocityTolerance);
    EXPECT_NEAR(
        moon.velocity.linear.y,
        expectedEarthVelocity.y + expectedMoonRelativeVelocity.y,
        kVelocityTolerance);
}

TEST_F(EarthOnlyServerTest, GivenEarthOnlyWorldWhenShipStartsOnCircularOrbitThenOrbitRemainsApproximatelyStable)
{
    const spaceship::server::ShipSpawnRequest request {
        {{kSingleMassOrbitRadiusMeters, 0.0, 0.0}, {1.0, 0.0, 0.0, 0.0}},
        {{0.0, circularOrbitSpeed(kSingleMassMuMetersCubedPerSecondSquared, kSingleMassOrbitRadiusMeters), 0.0}},
    };

    const auto shipNetId = server.spawnShip(request);
    ASSERT_TRUE(server.updateShipControl(
        shipNetId,
        spaceship::shared::ShipControl {0.0, {1.0, 0.0, 0.0, 0.0}, false}));

    runTicks(server, 600);

    ASSERT_EQ(server.world().ships.size(), 1U);
    const auto& ship = server.world().ships.front();
    EXPECT_NEAR(radialDistanceMeters(ship.transform.position), kSingleMassOrbitRadiusMeters, 500.0);
    EXPECT_GT(ship.transform.position.y, 0.0);
}

TEST_F(EarthOnlyServerTest, GivenEarthOnlyWorldWhenShipStartsWithNoVelocityThenItFallsStraightTowardBody)
{
    const spaceship::server::ShipSpawnRequest request {
        {{kSingleMassOrbitRadiusMeters, 0.0, 0.0}, {1.0, 0.0, 0.0, 0.0}},
        {{0.0, 0.0, 0.0}},
    };

    const auto shipNetId = server.spawnShip(request);
    ASSERT_TRUE(server.updateShipControl(
        shipNetId,
        spaceship::shared::ShipControl {0.0, {1.0, 0.0, 0.0, 0.0}, false}));

    runTicks(server, 60);

    ASSERT_EQ(server.world().ships.size(), 1U);
    const auto& ship = server.world().ships.front();
    const auto& earth = server.world().massiveBodies.front();
    EXPECT_LT(ship.transform.position.x, kSingleMassOrbitRadiusMeters);
    EXPECT_LT(ship.velocity.linear.x, 0.0);
    EXPECT_NEAR(ship.transform.position.y, earth.transform.position.y, 1e-6);
    EXPECT_NEAR(ship.transform.position.z, earth.transform.position.z, 1e-6);
    EXPECT_NEAR(ship.velocity.linear.y, 0.0, 1e-6);
    EXPECT_NEAR(ship.velocity.linear.z, 0.0, 1e-6);
}

TEST(SimulationServerGravityTest, GivenCustomWorldWithPreloadedShipWhenFirstTickRunsThenGravityAffectsFirstIntegration)
{
    spaceship::server::SimulationWorld world = spaceship::server::createInitialWorld(spaceship::server::BootstrapWorld::EarthOnly);
    world.ships.push_back(spaceship::server::ShipState {
        500U,
        {{kSingleMassOrbitRadiusMeters, 0.0, 0.0}, {1.0, 0.0, 0.0, 0.0}},
        {{0.0, 0.0, 0.0}},
        {},
        {},
        {},
        {1'000.0, 1.0 / 1'000.0},
        {5.0},
        {0.0, {1.0, 0.0, 0.0, 0.0}, false},
    });

    spaceship::server::SimulationServer server {std::move(world)};

    server.tick();

    ASSERT_EQ(server.world().ships.size(), 1U);
    const auto& ship = server.world().ships.front();
    EXPECT_LT(ship.gravityAcceleration.x, 0.0);
    EXPECT_LT(ship.acceleration.x, 0.0);
    EXPECT_LT(ship.velocity.linear.x, 0.0);
}

TEST_F(EarthOnlyServerTest, GivenEarthOnlyWorldWhenShipAppliesProgradeThrustThenOrbitParametersChange)
{
    const double initialCircularSpeed =
        circularOrbitSpeed(kSingleMassMuMetersCubedPerSecondSquared, kSingleMassOrbitRadiusMeters);
    const double initialSpecificEnergy =
        orbitalSpecificEnergy(
            {kSingleMassOrbitRadiusMeters, 0.0, 0.0},
            {0.0, initialCircularSpeed, 0.0},
            kSingleMassMuMetersCubedPerSecondSquared);
    const spaceship::server::ShipSpawnRequest request {
        {{kSingleMassOrbitRadiusMeters, 0.0, 0.0}, {1.0, 0.0, 0.0, 0.0}},
        {{0.0, initialCircularSpeed, 0.0}},
    };

    const auto shipNetId = server.spawnShip(request);
    ASSERT_TRUE(server.updateShipControl(
        shipNetId,
        spaceship::shared::ShipControl {
            1.0,
            {kQuarterTurnZAxisHalfAngleComponent, 0.0, 0.0, kQuarterTurnZAxisHalfAngleComponent},
            false,
        }));

    runTicks(server, 300);

    ASSERT_EQ(server.world().ships.size(), 1U);
    const auto& ship = server.world().ships.front();
    const double speed = radialDistanceMeters(ship.velocity.linear);
    const double specificEnergy = orbitalSpecificEnergy(
        ship.transform.position,
        ship.velocity.linear,
        kSingleMassMuMetersCubedPerSecondSquared);

    EXPECT_GT(speed, initialCircularSpeed + 10.0);
    EXPECT_GT(specificEnergy, initialSpecificEnergy + 10.0);
}

TEST_F(SimulationServerSmokeTest, GivenFireControlWhenTickRunsThenProjectileSpawnsAfterShipIntegration)
{
    constexpr spaceship::shared::NetId kExpectedFirstProjectileNetId = spaceship::server::kFirstProjectileNetId;
    const spaceship::server::ShipSpawnRequest request {
        {{kFarFieldPositionXMeters, 0.0, 0.0}, {1.0, 0.0, 0.0, 0.0}},
        {{0.0, 0.0, 0.0}},
    };

    const auto shipNetId = server.spawnShip(request);
    ASSERT_TRUE(server.updateShipControl(
        shipNetId,
        spaceship::shared::ShipControl {0.0, {1.0, 0.0, 0.0, 0.0}, true}));

    server.tick();

    ASSERT_EQ(server.world().projectiles.size(), 1U);
    ASSERT_EQ(server.world().ships.size(), 1U);
    const auto& projectile = server.world().projectiles.front();
    const auto& ship = server.world().ships.front();
    EXPECT_EQ(projectile.netId, kExpectedFirstProjectileNetId);
    EXPECT_EQ(projectile.params.ownerNetId, shipNetId);
    EXPECT_DOUBLE_EQ(projectile.transform.position.x, ship.transform.position.x);
    EXPECT_DOUBLE_EQ(projectile.transform.orientation.w, 1.0);
    EXPECT_GT(projectile.velocity.linear.x, kDefaultProjectileMuzzleSpeedMetersPerSecond * 0.9);
    EXPECT_FALSE(ship.control.fire);
}

TEST_F(SimulationServerSmokeTest, GivenSpawnedProjectileWhenNextTickRunsThenItMovesAndLosesTtl)
{
    const spaceship::server::ShipSpawnRequest request {
        {{kFarFieldPositionXMeters, 0.0, 0.0}, {1.0, 0.0, 0.0, 0.0}},
        {{0.0, 0.0, 0.0}},
    };

    const auto shipNetId = server.spawnShip(request);
    ASSERT_TRUE(server.updateShipControl(
        shipNetId,
        spaceship::shared::ShipControl {0.0, {1.0, 0.0, 0.0, 0.0}, true}));

    server.tick();

    ASSERT_EQ(server.world().projectiles.size(), 1U);
    const double projectileSpawnPositionX = server.world().projectiles.front().transform.position.x;

    server.tick();

    ASSERT_EQ(server.world().projectiles.size(), 1U);
    const auto& projectile = server.world().projectiles.front();
    EXPECT_GT(projectile.transform.position.x, projectileSpawnPositionX);
    EXPECT_LT(projectile.acceleration.x, 0.0);
    EXPECT_DOUBLE_EQ(projectile.params.ttlSeconds, 10.0 - kOneTickSeconds);
}

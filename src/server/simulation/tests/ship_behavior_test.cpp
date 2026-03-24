#include "server/tests/test_helpers.hpp"

#include "server/spawning/spawning_system.hpp"

using namespace spaceship::test;

// ---------------------------------------------------------------------------
// ZeroGravityShipBehaviorTest — Verlet integration and thrust tests (no gravity)
// ---------------------------------------------------------------------------

class ZeroGravityShipBehaviorTest : public ::testing::Test
{
  protected:
    spaceship::server::SimulationServer server {spaceship::server::SimulationWorld {}};
};

TEST_F(ZeroGravityShipBehaviorTest, GivenShipWithFullThrottle_WhenTicked_ThenVelocityAdvancesForward)
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

TEST_F(ZeroGravityShipBehaviorTest, GivenShipWithRotatedOrientation_WhenTicked_ThenThrustInNewDirection)
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

TEST_F(ZeroGravityShipBehaviorTest, GivenShipWithFireFlagSet_WhenTicked_ThenProjectileSpawnedAndFlagCleared)
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
    // Spawn position is offset along ship forward (+x): ship_x + ship_radius + proj_radius + skin
    // = 1.0 + 5.0 + 0.1 + 0.1 = 6.2 m
    constexpr double kSpawnOffsetX = 5.0 + 0.1 + 0.1;  // ship_radius + proj_radius + skin
    const double kExpectedProjectilePositionX =
        1.0 + kSpawnOffsetX + kExpectedProjectileVelocityX * spaceship::shared::constants::kFixedDeltaSeconds;

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

TEST_F(ZeroGravityShipBehaviorTest, GivenShipWithInitialVelocity_WhenTicked_ThenPositionAdvancesFromVelocity)
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

TEST_F(ZeroGravityShipBehaviorTest, GivenShipAtRestWithThrust_WhenTicked_ThenPositionIncludesAccelerationTerm)
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

TEST_F(ZeroGravityShipBehaviorTest, GivenShipWithThrottle_WhenControlUpdated_ThenVelocityUnchangedBeforeTick)
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

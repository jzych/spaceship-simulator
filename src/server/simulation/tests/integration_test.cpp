#include "server/tests/test_helpers.hpp"

#include "server/spawning/spawning_system.hpp"

using namespace spaceship::test;

// ---------------------------------------------------------------------------
// Verlet integration tests — using single Earth at origin
// ---------------------------------------------------------------------------

TEST_F(EarthOnlyCenteredTest, GivenEarthOnlyWorld_WhenCreated_ThenContainsOnlyEarth)
{
    ASSERT_EQ(server.world().massiveBodies.size(), 1U);
    EXPECT_EQ(server.world().massiveBodies[0].definition.name, "Earth");
    EXPECT_NEAR(server.world().massiveBodies[0].transform.position.x, 0.0, 1e-9);
}

TEST_F(EarthOnlyCenteredTest, GivenShipAtRestNearEarth_WhenManyTicksElapsed_ThenFallsAlongRadialLine)
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

TEST_F(EarthOnlyCenteredTest, GivenFallingShip_WhenOneTick_ThenVerletVelocityExceedsEuler)
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

TEST_F(EarthOnlyCenteredTest, GivenShipInCircularLEO_WhenManyTicksElapsed_ThenRadiusStable)
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

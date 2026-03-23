#include "server/simulation/simulation_math.hpp"
#include "server/simulation/simulation_server.hpp"
#include "server/spawning/spawning_system.hpp"

#include <gtest/gtest.h>
#include <numbers>

// ---------------------------------------------------------------------------
// SimulationMathTest — vector arithmetic, cross, normalize, negate, projectOntoPlane
// ---------------------------------------------------------------------------

TEST(SimulationMathTest, GivenTwoVectors_WhenSubtracted_ThenComponentWiseDifference)
{
    const spaceship::shared::Vec3 a {3.0, 2.0, 1.0};
    const spaceship::shared::Vec3 b {1.0, 1.0, 1.0};
    const auto result = spaceship::server::subtract(a, b);
    EXPECT_DOUBLE_EQ(result.x, 2.0);
    EXPECT_DOUBLE_EQ(result.y, 1.0);
    EXPECT_DOUBLE_EQ(result.z, 0.0);
}

TEST(SimulationMathTest, GivenFirstVectorSmallerThanSecond_WhenSubtracted_ThenNegativeResult)
{
    const spaceship::shared::Vec3 a {1.0, 2.0, 3.0};
    const spaceship::shared::Vec3 b {4.0, 5.0, 6.0};
    const auto result = spaceship::server::subtract(a, b);
    EXPECT_DOUBLE_EQ(result.x, -3.0);
    EXPECT_DOUBLE_EQ(result.y, -3.0);
    EXPECT_DOUBLE_EQ(result.z, -3.0);
}

TEST(SimulationMathTest, GivenOrthogonalVectors_WhenDotted_ThenResultIsZero)
{
    const spaceship::shared::Vec3 x {1.0, 0.0, 0.0};
    const spaceship::shared::Vec3 y {0.0, 1.0, 0.0};
    EXPECT_DOUBLE_EQ(spaceship::server::dot(x, y), 0.0);
}

TEST(SimulationMathTest, GivenParallelVectors_WhenDotted_ThenResultIsMagnitudeSquared)
{
    const spaceship::shared::Vec3 v {3.0, 0.0, 0.0};
    EXPECT_DOUBLE_EQ(spaceship::server::dot(v, v), 9.0);
}

TEST(SimulationMathTest, GivenVector_WhenLengthSquaredComputed_ThenMatchesDotProductWithSelf)
{
    const spaceship::shared::Vec3 v {3.0, 4.0, 0.0};
    EXPECT_DOUBLE_EQ(spaceship::server::lengthSquared(v), 25.0);
}

TEST(SimulationMathTest, GivenKnownVector_WhenLengthComputed_ThenCorrectMagnitude)
{
    const spaceship::shared::Vec3 v {3.0, 4.0, 0.0};
    EXPECT_DOUBLE_EQ(spaceship::server::length(v), 5.0);
}

TEST(SimulationMathTest, GivenZeroVector_WhenLengthComputed_ThenResultIsZero)
{
    const spaceship::shared::Vec3 zero {0.0, 0.0, 0.0};
    EXPECT_DOUBLE_EQ(spaceship::server::length(zero), 0.0);
}

TEST(SimulationMathTest, GivenShipInEmptyWorld_WhenSpawned_ThenAccelerationIsZero)
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

TEST(SimulationMathTest, GivenProjectileInEmptyWorld_WhenSpawned_ThenAccelerationIsZero)
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

TEST(SimulationMathTest, GivenXAndYBasisVectors_WhenCrossed_ThenResultIsZ)
{
    const spaceship::shared::Vec3 x {1.0, 0.0, 0.0};
    const spaceship::shared::Vec3 y {0.0, 1.0, 0.0};
    const auto result = spaceship::server::cross(x, y);
    EXPECT_DOUBLE_EQ(result.x, 0.0);
    EXPECT_DOUBLE_EQ(result.y, 0.0);
    EXPECT_DOUBLE_EQ(result.z, 1.0);
}

TEST(SimulationMathTest, GivenTwoVectors_WhenCrossedInBothOrders_ThenResultsAreNegatedEachOther)
{
    const spaceship::shared::Vec3 a {1.0, 2.0, 3.0};
    const spaceship::shared::Vec3 b {4.0, 5.0, 6.0};
    const auto ab = spaceship::server::cross(a, b);
    const auto ba = spaceship::server::cross(b, a);
    EXPECT_DOUBLE_EQ(ab.x, -ba.x);
    EXPECT_DOUBLE_EQ(ab.y, -ba.y);
    EXPECT_DOUBLE_EQ(ab.z, -ba.z);
}

TEST(SimulationMathTest, GivenParallelVectors_WhenCrossed_ThenResultIsZeroVector)
{
    const spaceship::shared::Vec3 a {3.0, 0.0, 0.0};
    const spaceship::shared::Vec3 b {7.0, 0.0, 0.0};
    const auto result = spaceship::server::cross(a, b);
    EXPECT_DOUBLE_EQ(result.x, 0.0);
    EXPECT_DOUBLE_EQ(result.y, 0.0);
    EXPECT_DOUBLE_EQ(result.z, 0.0);
}

TEST(SimulationMathTest, GivenNonUnitVector_WhenNormalized_ThenResultHasUnitLength)
{
    const spaceship::shared::Vec3 v {0.0, 5.0, 0.0};
    const auto result = spaceship::server::normalize(v);
    EXPECT_DOUBLE_EQ(result.x, 0.0);
    EXPECT_DOUBLE_EQ(result.y, 1.0);
    EXPECT_DOUBLE_EQ(result.z, 0.0);
}

TEST(SimulationMathTest, GivenZeroVector_WhenNormalized_ThenResultIsZeroVector)
{
    const spaceship::shared::Vec3 zero {};
    const auto result = spaceship::server::normalize(zero);
    EXPECT_DOUBLE_EQ(result.x, 0.0);
    EXPECT_DOUBLE_EQ(result.y, 0.0);
    EXPECT_DOUBLE_EQ(result.z, 0.0);
}

TEST(SimulationMathTest, GivenArbitraryVector_WhenNormalized_ThenLengthIsOne)
{
    const spaceship::shared::Vec3 v {3.0, 4.0, 12.0};
    const auto result = spaceship::server::normalize(v);
    EXPECT_NEAR(spaceship::server::length(result), 1.0, 1e-15);
}

TEST(SimulationMathTest, GivenVector_WhenNegated_ThenSignsFlipped)
{
    const spaceship::shared::Vec3 v {1.0, -2.0, 3.0};
    const auto result = spaceship::server::negate(v);
    EXPECT_DOUBLE_EQ(result.x, -1.0);
    EXPECT_DOUBLE_EQ(result.y, 2.0);
    EXPECT_DOUBLE_EQ(result.z, -3.0);
}

TEST(SimulationMathTest, GivenVectorAndPlane_WhenProjected_ThenNormalComponentRemoved)
{
    const spaceship::shared::Vec3 v {1.0, 2.0, 3.0};
    const spaceship::shared::Vec3 n {0.0, 1.0, 0.0}; // y-normal plane
    const auto result = spaceship::server::projectOntoPlane(v, n);
    EXPECT_DOUBLE_EQ(result.x, 1.0);
    EXPECT_NEAR(result.y, 0.0, 1e-15);
    EXPECT_DOUBLE_EQ(result.z, 3.0);
}

TEST(SimulationMathTest, GivenTwoVectors_WhenAdded_ThenComponentWiseSum)
{
    const spaceship::shared::Vec3 a {1.0, 2.0, 3.0};
    const spaceship::shared::Vec3 b {4.0, 5.0, 6.0};
    const auto result = spaceship::server::add(a, b);
    EXPECT_DOUBLE_EQ(result.x, 5.0);
    EXPECT_DOUBLE_EQ(result.y, 7.0);
    EXPECT_DOUBLE_EQ(result.z, 9.0);
}

TEST(SimulationMathTest, GivenVectorAndScalar_WhenScaled_ThenComponentsMultiplied)
{
    const spaceship::shared::Vec3 v {1.0, 2.0, 3.0};
    const auto result = spaceship::server::scale(v, 2.5);
    EXPECT_DOUBLE_EQ(result.x, 2.5);
    EXPECT_DOUBLE_EQ(result.y, 5.0);
    EXPECT_DOUBLE_EQ(result.z, 7.5);
}

TEST(SimulationMathTest, GivenIdentityQuaternion_WhenForwardDirectionComputed_ThenResultIsPositiveX)
{
    const spaceship::shared::Quaternion identity {1.0, 0.0, 0.0, 0.0};
    const auto fwd = spaceship::server::forwardDirection(identity);
    EXPECT_NEAR(fwd.x, 1.0, 1e-12);
    EXPECT_NEAR(fwd.y, 0.0, 1e-12);
    EXPECT_NEAR(fwd.z, 0.0, 1e-12);
}

TEST(SimulationMathTest, GivenQuaternion_WhenConjugated_ThenImaginaryComponentsNegated)
{
    const spaceship::shared::Quaternion q {1.0, 2.0, 3.0, 4.0};
    const auto c = spaceship::server::conjugate(q);
    EXPECT_DOUBLE_EQ(c.w, 1.0);
    EXPECT_DOUBLE_EQ(c.x, -2.0);
    EXPECT_DOUBLE_EQ(c.y, -3.0);
    EXPECT_DOUBLE_EQ(c.z, -4.0);
}

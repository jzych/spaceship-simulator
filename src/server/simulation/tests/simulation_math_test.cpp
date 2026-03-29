#include "server/simulation/simulation_math.hpp"
#include "server/simulation/simulation_server.hpp"
#include "server/spawning/spawning_system.hpp"

#include <gtest/gtest.h>
#include <numbers>

// ---------------------------------------------------------------------------
// SimulationMathTest — vector arithmetic, cross, normalize, negate, projectOntoPlane,
//                      bodySpinAngle, toBodyFixedGeographic, conjugate
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
    // Non-unit quaternion is intentional: conjugate is defined for all quaternions,
    // and this verifies the sign-flip contract without masking errors via normalization.
    const spaceship::shared::Quaternion q {1.0, 2.0, 3.0, 4.0};
    const auto c = spaceship::server::conjugate(q);
    EXPECT_DOUBLE_EQ(c.w, 1.0);
    EXPECT_DOUBLE_EQ(c.x, -2.0);
    EXPECT_DOUBLE_EQ(c.y, -3.0);
    EXPECT_DOUBLE_EQ(c.z, -4.0);
}

// ---------------------------------------------------------------------------
// bodySpinAngle
// ---------------------------------------------------------------------------

TEST(SimulationMathTest, GivenPositivePeriodAndZeroPhase_WhenHalfPeriodElapsed_ThenAngleIsPi)
{
    constexpr double periodSeconds = 10.0;
    constexpr double initialPhase  = 0.0;
    constexpr double elapsed       = 5.0; // half period

    const double angle = spaceship::server::bodySpinAngle(periodSeconds, initialPhase, elapsed);

    EXPECT_NEAR(angle, std::numbers::pi, 1e-12);
}

TEST(SimulationMathTest, GivenNonZeroInitialPhase_WhenNoTimeElapsed_ThenAngleIsInitialPhase)
{
    constexpr double initialPhase = 1.23;

    const double angle = spaceship::server::bodySpinAngle(100.0, initialPhase, 0.0);

    EXPECT_DOUBLE_EQ(angle, initialPhase);
}

TEST(SimulationMathTest, GivenZeroPeriod_WhenTimeElapsed_ThenAngleIsInitialPhase)
{
    // Guard: non-positive period returns initialPhase unchanged (no division by zero).
    constexpr double initialPhase = 0.5;

    const double angle = spaceship::server::bodySpinAngle(0.0, initialPhase, 999.0);

    EXPECT_DOUBLE_EQ(angle, initialPhase);
}

// ---------------------------------------------------------------------------
// toBodyFixedGeographic
// ---------------------------------------------------------------------------

TEST(SimulationMathTest, GivenPointOnXAxisWithZeroSpin_WhenTransformed_ThenLongitudeIsZero)
{
    // With spinAngle = 0 the body-fixed and inertial frames coincide.
    // A point on the +X axis should map to longitude = 0, latitude = 0.
    const spaceship::shared::Vec3 relativePosition {1000.0, 0.0, 0.0};
    constexpr double bodyRadius = 500.0;

    const auto geo = spaceship::server::toBodyFixedGeographic(relativePosition, bodyRadius, 0.0);

    EXPECT_NEAR(geo.longitudeRadians, 0.0,   1e-12);
    EXPECT_NEAR(geo.latitudeRadians,  0.0,   1e-12);
    EXPECT_NEAR(geo.altitudeMeters,   500.0, 1e-9);
}

TEST(SimulationMathTest, GivenPointOnYAxis_WhenTransformed_ThenLatitudeIsHalfPi)
{
    // A point directly above the north pole (+Y) should have latitude = π/2.
    const spaceship::shared::Vec3 relativePosition {0.0, 1000.0, 0.0};
    constexpr double bodyRadius = 500.0;

    const auto geo = spaceship::server::toBodyFixedGeographic(relativePosition, bodyRadius, 0.0);

    EXPECT_NEAR(geo.latitudeRadians, std::numbers::pi / 2.0, 1e-12);
    EXPECT_NEAR(geo.altitudeMeters, 500.0, 1e-9);
}

TEST(SimulationMathTest, GivenPointBelowMinRadius_WhenTransformed_ThenOnlyAltitudeIsSet)
{
    // Position closer to origin than kMinRadius: longitude and latitude remain zero-initialised.
    const spaceship::shared::Vec3 relativePosition {0.5, 0.0, 0.0};
    constexpr double bodyRadius = 0.0;

    const auto geo = spaceship::server::toBodyFixedGeographic(relativePosition, bodyRadius, 0.0);

    EXPECT_NEAR(geo.altitudeMeters,    0.5, 1e-9);
    EXPECT_DOUBLE_EQ(geo.longitudeRadians, 0.0);
    EXPECT_DOUBLE_EQ(geo.latitudeRadians,  0.0);
}

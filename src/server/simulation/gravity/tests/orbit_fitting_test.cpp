#include "server/simulation/gravity/orbit_fitting.hpp"
#include "server/simulation/simulation_math.hpp"

#include <gtest/gtest.h>
#include <cmath>
#include <numbers>

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

TEST_F(OrbitFittingTest, GivenCircularOrbitStateVectors_WhenOrbitFit_ThenEccentricityIsZero)
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

TEST_F(OrbitFittingTest, GivenOrbitInXYPlane_WhenOrbitFit_ThenNormalPointsAlongPlusZ)
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

TEST_F(OrbitFittingTest, GivenEllipticalOrbitAtPeriapsis_WhenOrbitFit_ThenSemiMajorAxisAndEccentricityCorrect)
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

TEST_F(OrbitFittingTest, GivenOrbitAtPeriapsis_WhenOrbitFit_ThenAllAnomaliesAreZero)
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

TEST_F(OrbitFittingTest, GivenEllipticalOrbit_WhenOrbitFit_ThenGeometryIsInternallyConsistent)
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

TEST_F(OrbitFittingTest, GivenHyperbolicVelocity_WhenOrbitFit_ThenIsNotElliptic)
{
    // Escape velocity at LEO: v > sqrt(2μ/r)
    const double vEscape = std::sqrt(2.0 * kEarthMu / kLeoRadius);
    const spaceship::shared::Vec3 r {kLeoRadius, 0.0, 0.0};
    const spaceship::shared::Vec3 v {0.0, vEscape * 1.5, 0.0}; // well above escape

    const auto cache = spaceship::server::fitOrbit(r, v, kEarthMu, kEarthRadius, kEarthNetId, kTick);

    EXPECT_FALSE(cache.isElliptic);
}

TEST_F(OrbitFittingTest, GivenZeroVelocity_WhenOrbitFit_ThenIsNotElliptic)
{
    // Zero velocity → degenerate (collinear r, v; zero angular momentum)
    const spaceship::shared::Vec3 r {kLeoRadius, 0.0, 0.0};
    const spaceship::shared::Vec3 v {0.0, 0.0, 0.0};

    const auto cache = spaceship::server::fitOrbit(r, v, kEarthMu, kEarthRadius, kEarthNetId, kTick);

    EXPECT_FALSE(cache.isElliptic);
}

TEST_F(OrbitFittingTest, GivenRadialVelocity_WhenOrbitFit_ThenIsNotElliptic)
{
    // Purely radial velocity → zero angular momentum
    const spaceship::shared::Vec3 r {kLeoRadius, 0.0, 0.0};
    const spaceship::shared::Vec3 v {100.0, 0.0, 0.0}; // along r

    const auto cache = spaceship::server::fitOrbit(r, v, kEarthMu, kEarthRadius, kEarthNetId, kTick);

    EXPECT_FALSE(cache.isElliptic);
}

TEST_F(OrbitFittingTest, GivenNonEllipticState_WhenOrbitFit_ThenRelativeStateAndAltitudeStored)
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

TEST_F(OrbitFittingTest, GivenNearCircularOrbit_WhenOrbitFit_ThenPeriapsisDirectionFallsBackToPosition)
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

TEST(QualityScoreTest, GivenPureTwoBodyAcceleration_WhenQualityScoreComputed_ThenScoreNearOne)
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

TEST(QualityScoreTest, GivenLargePerturbation_WhenQualityScoreComputed_ThenScoreReduced)
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

TEST(QualityScoreTest, GivenExtremePerturbation_WhenQualityScoreComputed_ThenScoreClamped)
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

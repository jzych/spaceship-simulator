#include "server/simulation/simulation_math.hpp"

#include <gtest/gtest.h>
#include <numbers>

// ---------------------------------------------------------------------------
// GeographicTelemetryTest — body-fixed longitude/latitude (unit tests)
// ---------------------------------------------------------------------------

TEST(GeographicTelemetryTest, GivenZeroRotationPeriod_WhenBodySpinAngleComputed_ThenReturnsInitialPhase)
{
    EXPECT_DOUBLE_EQ(spaceship::server::bodySpinAngle(0.0, 1.5, 100.0), 1.5);
}

TEST(GeographicTelemetryTest, GivenOneSiderealDay_WhenBodySpinAngleComputed_ThenReturnsFullRotation)
{
    // Earth sidereal day: 86164.1 s → full rotation = 2π
    const double angle = spaceship::server::bodySpinAngle(86'164.1, 0.0, 86'164.1);
    EXPECT_NEAR(angle, 2.0 * std::numbers::pi, 1e-6);
}

TEST(GeographicTelemetryTest, GivenHalfSiderealDay_WhenBodySpinAngleComputed_ThenReturnsPiRadians)
{
    const double angle = spaceship::server::bodySpinAngle(86'164.1, 0.0, 86'164.1 / 2.0);
    EXPECT_NEAR(angle, std::numbers::pi, 1e-6);
}

TEST(GeographicTelemetryTest, GivenShipOnXAxisAndZeroSpin_WhenGeographicComputed_ThenZeroLongitudeAndLatitude)
{
    // Ship on +x axis, zero spin angle → longitude = 0, latitude = 0
    const spaceship::shared::Vec3 r {6.771e6, 0.0, 0.0};
    const auto geo = spaceship::server::toBodyFixedGeographic(r, 6.371e6, 0.0);

    EXPECT_NEAR(geo.longitudeRadians, 0.0, 1e-10);
    EXPECT_NEAR(geo.latitudeRadians, 0.0, 1e-10);
    EXPECT_NEAR(geo.altitudeMeters, 400'000.0, 1.0);
}

TEST(GeographicTelemetryTest, GivenBodyRotatedByPi_WhenGeographicComputed_ThenLongitudeShifted)
{
    // Ship still on +x axis, but body rotated by π → body-fixed position flipped
    const spaceship::shared::Vec3 r {6.771e6, 0.0, 0.0};
    const auto geo = spaceship::server::toBodyFixedGeographic(r, 6.371e6, std::numbers::pi);

    // After π rotation: x_bf = r.x * cos(π) = -r.x, z_bf = -r.x*sin(π) ≈ 0
    // longitude = atan2(0, -r.x) = π
    EXPECT_NEAR(std::abs(geo.longitudeRadians), std::numbers::pi, 1e-6);
    EXPECT_NEAR(geo.latitudeRadians, 0.0, 1e-10);
}

TEST(GeographicTelemetryTest, GivenShipOnYAxis_WhenGeographicComputed_ThenLatitudeIs90Degrees)
{
    // Ship on +y axis → latitude = π/2 (north pole)
    const spaceship::shared::Vec3 r {0.0, 6.771e6, 0.0};
    const auto geo = spaceship::server::toBodyFixedGeographic(r, 6.371e6, 0.0);

    EXPECT_NEAR(geo.latitudeRadians, std::numbers::pi / 2.0, 1e-10);
}

TEST(GeographicTelemetryTest, GivenNonRotatingBody_WhenGeographicComputedAtDifferentTimes_ThenLongitudeUnchanged)
{
    // Sun (period=0) → spin angle always 0 → longitude doesn't change with time
    const spaceship::shared::Vec3 r {1e9, 0.0, 1e9};
    const double angle1 = spaceship::server::bodySpinAngle(0.0, 0.0, 0.0);
    const double angle2 = spaceship::server::bodySpinAngle(0.0, 0.0, 1000.0);

    const auto geo1 = spaceship::server::toBodyFixedGeographic(r, 6.9634e8, angle1);
    const auto geo2 = spaceship::server::toBodyFixedGeographic(r, 6.9634e8, angle2);

    EXPECT_DOUBLE_EQ(geo1.longitudeRadians, geo2.longitudeRadians);
}

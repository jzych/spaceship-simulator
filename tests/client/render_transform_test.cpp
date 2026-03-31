#include "client/render_transform.hpp"

#include <gtest/gtest.h>
#include <cmath>

using namespace spaceship::client;
using namespace spaceship::shared;

// ---------------------------------------------------------------------------
// to_render_position — basic transform
// ---------------------------------------------------------------------------

TEST(RenderTransformTest, GivenEntityAtRenderOrigin_WhenTransformed_ThenResultIsZero)
{
    const Vec3 origin {1.0e6, 2.0e6, 3.0e6};
    const RenderVec3 result = to_render_position(origin, origin);
    EXPECT_FLOAT_EQ(result.x, 0.0f);
    EXPECT_FLOAT_EQ(result.y, 0.0f);
    EXPECT_FLOAT_EQ(result.z, 0.0f);
}

TEST(RenderTransformTest, GivenEntityOneKilometreForward_WhenTransformed_ThenXIs100000Cm)
{
    const Vec3 origin {0.0, 0.0, 0.0};
    const Vec3 pos {1000.0, 0.0, 0.0}; // 1 km along sim X (forward)
    const RenderVec3 result = to_render_position(pos, origin);
    EXPECT_FLOAT_EQ(result.x, 100'000.0f); // 1000 m * 100 cm/m
    EXPECT_FLOAT_EQ(result.y, 0.0f);
    EXPECT_FLOAT_EQ(result.z, 0.0f);
}

// ---------------------------------------------------------------------------
// to_render_position — axis remap
// ---------------------------------------------------------------------------

TEST(RenderTransformTest, GivenSimYUp_WhenTransformed_ThenUEZPositive)
{
    // Sim Y is up; UE Z is up.
    const Vec3 origin {0.0, 0.0, 0.0};
    const Vec3 pos {0.0, 1.0, 0.0}; // 1 m in sim Y
    const RenderVec3 result = to_render_position(pos, origin);
    EXPECT_FLOAT_EQ(result.x, 0.0f);
    EXPECT_FLOAT_EQ(result.y, 0.0f);
    EXPECT_FLOAT_EQ(result.z, 100.0f); // 1 m -> 100 cm in UE Z
}

TEST(RenderTransformTest, GivenSimZRight_WhenTransformed_ThenUEYNegative)
{
    // Sim Z is right; UE Y is right — but the handedness flip means Z_sim -> -Y_ue.
    const Vec3 origin {0.0, 0.0, 0.0};
    const Vec3 pos {0.0, 0.0, 1.0}; // 1 m in sim Z (right)
    const RenderVec3 result = to_render_position(pos, origin);
    EXPECT_FLOAT_EQ(result.x, 0.0f);
    EXPECT_FLOAT_EQ(result.y, -100.0f); // -1 m in UE Y
    EXPECT_FLOAT_EQ(result.z, 0.0f);
}

// ---------------------------------------------------------------------------
// to_render_position — large-world precision
// ---------------------------------------------------------------------------

TEST(RenderTransformTest, GivenTwoPositions10KmApartAt1AU_WhenTransformed_ThenFloatErrorBelowOneMm)
{
    // This is the key acceptance test for camera-relative rendering.
    // Without it, raw float at 1 AU has ~15 km precision — completely unusable.
    // With it, the subtraction happens in double and the float offset is exact.
    constexpr double kOneAU         = 149'597'870'700.0; // metres
    constexpr double kSeparation    = 10'000.0;           // 10 km in metres
    constexpr float  kExpectedDiff  = 1'000'000.0f;       // 10 km * 100 cm/m
    constexpr float  kToleranceCm   = 0.1f;               // 1 mm

    const Vec3 renderOrigin {kOneAU, 0.0, 0.0};
    const Vec3 posA = renderOrigin;
    const Vec3 posB {kOneAU + kSeparation, 0.0, 0.0};

    const RenderVec3 renderA = to_render_position(posA, renderOrigin);
    const RenderVec3 renderB = to_render_position(posB, renderOrigin);

    EXPECT_NEAR(renderB.x - renderA.x, kExpectedDiff, kToleranceCm);
}

TEST(RenderTransformTest, GivenPositionVeryFarFromOrigin_WhenTransformed_ThenNoInfOrNaN)
{
    const Vec3 origin {0.0, 0.0, 0.0};
    const Vec3 pos {1.496e13, 0.0, 0.0}; // 100 AU
    const RenderVec3 result = to_render_position(pos, origin);
    EXPECT_FALSE(std::isinf(result.x));
    EXPECT_FALSE(std::isnan(result.x));
}

// ---------------------------------------------------------------------------
// to_render_orientation
// ---------------------------------------------------------------------------

TEST(RenderTransformTest, GivenIdentityQuaternion_WhenConverted_ThenUEIdentity)
{
    const Quaternion identity {1.0, 0.0, 0.0, 0.0};
    const RenderQuat result = to_render_orientation(identity);
    EXPECT_FLOAT_EQ(result.w, 1.0f);
    EXPECT_FLOAT_EQ(result.x, 0.0f);
    EXPECT_FLOAT_EQ(result.y, 0.0f);
    EXPECT_FLOAT_EQ(result.z, 0.0f);
}

TEST(RenderTransformTest, GivenSimYAxisRotation_WhenConverted_ThenUEZAxisRotation)
{
    // 90° around sim Y (up) must become 90° around UE Z (up).
    // sim: (cos45, 0, sin45, 0) -> ue: (w, x_sim, -z_sim, y_sim) = (cos45, 0, 0, sin45)
    constexpr double kCos45 = 0.7071067811865476;
    constexpr double kSin45 = 0.7071067811865476;
    const Quaternion simQuat {kCos45, 0.0, kSin45, 0.0};
    const RenderQuat result = to_render_orientation(simQuat);
    EXPECT_NEAR(result.w, static_cast<float>(kCos45), 1e-6f);
    EXPECT_NEAR(result.x, 0.0f,                       1e-6f);
    EXPECT_NEAR(result.y, 0.0f,                       1e-6f); // -z_sim = 0
    EXPECT_NEAR(result.z, static_cast<float>(kSin45), 1e-6f); // y_sim  = sin45
}

TEST(RenderTransformTest, GivenSimXAxisRotation_WhenConverted_ThenUEXAxisRotation)
{
    // 90° around sim X (forward) must remain 90° around UE X (forward).
    // sim: (cos45, sin45, 0, 0) -> ue: (cos45, sin45, 0, 0)
    constexpr double kCos45 = 0.7071067811865476;
    constexpr double kSin45 = 0.7071067811865476;
    const Quaternion simQuat {kCos45, kSin45, 0.0, 0.0};
    const RenderQuat result = to_render_orientation(simQuat);
    EXPECT_NEAR(result.w, static_cast<float>(kCos45), 1e-6f);
    EXPECT_NEAR(result.x, static_cast<float>(kSin45), 1e-6f);
    EXPECT_NEAR(result.y, 0.0f,                       1e-6f);
    EXPECT_NEAR(result.z, 0.0f,                       1e-6f);
}

// ---------------------------------------------------------------------------
// select_render_origin
// ---------------------------------------------------------------------------

TEST(RenderTransformTest, GivenCameraPositionNonZero_WhenSelectingOrigin_ThenCameraPositionUsed)
{
    InterpolatedWorldState state;
    InterpolatedMassiveBodyState earth;
    earth.netId    = 1;
    earth.position = {1.0, 2.0, 3.0};
    state.massiveBodies.push_back(earth);

    const Vec3 cameraPos {10.0, 20.0, 30.0};
    const Vec3 result = select_render_origin(state, cameraPos);

    EXPECT_DOUBLE_EQ(result.x, 10.0);
    EXPECT_DOUBLE_EQ(result.y, 20.0);
    EXPECT_DOUBLE_EQ(result.z, 30.0);
}

TEST(RenderTransformTest, GivenZeroCameraPosition_WhenSelectingOrigin_ThenEarthPositionUsed)
{
    InterpolatedWorldState state;
    InterpolatedMassiveBodyState earth;
    earth.netId    = 1;
    earth.position = {5.0, 6.0, 7.0};
    state.massiveBodies.push_back(earth);

    const Vec3 result = select_render_origin(state, {0.0, 0.0, 0.0});

    EXPECT_DOUBLE_EQ(result.x, 5.0);
    EXPECT_DOUBLE_EQ(result.y, 6.0);
    EXPECT_DOUBLE_EQ(result.z, 7.0);
}

TEST(RenderTransformTest, GivenNoCameraAndNoEarth_WhenSelectingOrigin_ThenWorldOriginUsed)
{
    InterpolatedWorldState state; // no bodies
    const Vec3 result = select_render_origin(state, {0.0, 0.0, 0.0});

    EXPECT_DOUBLE_EQ(result.x, 0.0);
    EXPECT_DOUBLE_EQ(result.y, 0.0);
    EXPECT_DOUBLE_EQ(result.z, 0.0);
}

TEST(RenderTransformTest, GivenMultipleBodies_WhenSelectingOriginWithNoCamera_ThenEarthUsed)
{
    InterpolatedWorldState state;
    InterpolatedMassiveBodyState sun, earth, moon;
    sun.netId  = 0; sun.position  = {1.5e11, 0.0, 0.0};
    earth.netId = 1; earth.position = {0.0,   0.0, 0.0};
    moon.netId  = 2; moon.position  = {3.84e8, 0.0, 0.0};
    state.massiveBodies = {sun, earth, moon};

    const Vec3 result = select_render_origin(state, {0.0, 0.0, 0.0});

    EXPECT_DOUBLE_EQ(result.x, 0.0);
    EXPECT_DOUBLE_EQ(result.y, 0.0);
    EXPECT_DOUBLE_EQ(result.z, 0.0);
}

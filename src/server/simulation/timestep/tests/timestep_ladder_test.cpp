#include "server/simulation/timestep/timestep_controller.hpp"

#include <gtest/gtest.h>
#include <cmath>

using namespace spaceship::server;

namespace
{
constexpr double kDtMax = 1.0 / 60.0;
constexpr int    kKMax  = 6;

TimestepLadderConfig cfg()
{
    TimestepLadderConfig c {};
    c.dt_max            = kDtMax;
    c.k_max             = kKMax;
    c.tau_raise_seconds = 0.5;
    return c;
}

} // anonymous namespace

// ---------------------------------------------------------------------------
// ladderDt
// ---------------------------------------------------------------------------

TEST(TimestepLadder,
     GivenDtMaxAndK0_WhenLadderDtComputed_ThenEqualsDtMax)
{
    EXPECT_DOUBLE_EQ(TimestepController::ladderDt(0, kDtMax), kDtMax);
}

TEST(TimestepLadder,
     GivenDtMaxAndK1_WhenLadderDtComputed_ThenEqualsHalfDtMax)
{
    EXPECT_DOUBLE_EQ(TimestepController::ladderDt(1, kDtMax), kDtMax / 2.0);
}

TEST(TimestepLadder,
     GivenDtMaxAndK6_WhenLadderDtComputed_ThenEqualsDtMaxOver64)
{
    EXPECT_DOUBLE_EQ(TimestepController::ladderDt(6, kDtMax), kDtMax / 64.0);
}

// ---------------------------------------------------------------------------
// quantizeToLadder
// ---------------------------------------------------------------------------

TEST(TimestepLadder,
     GivenTargetDtExactlyDtMax_WhenQuantized_ThenChoosesK0)
{
    EXPECT_EQ(TimestepController::quantizeToLadder(kDtMax, kDtMax, kKMax), 0);
}

TEST(TimestepLadder,
     GivenTargetDtAboveDtMax_WhenQuantized_ThenClampsToK0)
{
    EXPECT_EQ(TimestepController::quantizeToLadder(kDtMax * 10.0, kDtMax, kKMax), 0);
}

TEST(TimestepLadder,
     GivenTargetDtExactlyAtLevel2_WhenQuantized_ThenChoosesK2)
{
    const double dtLevel2 = TimestepController::ladderDt(2, kDtMax);
    EXPECT_EQ(TimestepController::quantizeToLadder(dtLevel2, kDtMax, kKMax), 2);
}

TEST(TimestepLadder,
     GivenTargetDtBetweenLevel2AndLevel3_WhenQuantized_ThenChoosesK3)
{
    // dt_level2 = dtMax/4, dt_level3 = dtMax/8
    // target = 0.75 * dt_level2 = 3*dtMax/16 — strictly between levels 2 and 3
    // largest k where ladderDt(k) <= target: ladderDt(3)=dtMax/8 <= 3*dtMax/16 ✓
    //                                        ladderDt(2)=dtMax/4  <= 3*dtMax/16 ✗
    const double dtBetween = 0.75 * TimestepController::ladderDt(2, kDtMax);
    EXPECT_EQ(TimestepController::quantizeToLadder(dtBetween, kDtMax, kKMax), 3);
}

TEST(TimestepLadder,
     GivenTargetDtBelowDtMin_WhenQuantized_ThenClampsToKMax)
{
    const double tinyDt = kDtMax / 1000.0;
    EXPECT_EQ(TimestepController::quantizeToLadder(tinyDt, kDtMax, kKMax), kKMax);
}

// ---------------------------------------------------------------------------
// applyHysteresis — step-down
// ---------------------------------------------------------------------------

TEST(TimestepLadder,
     GivenStepDownRequest_WhenHysteresisApplied_ThenStepsDownImmediately)
{
    TimestepState state {};
    state.currentK       = 0;
    state.tStableSeconds = 0.0;

    const int kApplied = TimestepController::applyHysteresis(2, kDtMax, state, cfg());
    EXPECT_EQ(kApplied, 1);
    EXPECT_EQ(state.currentK, 1);
    EXPECT_DOUBLE_EQ(state.tStableSeconds, 0.0);
}

TEST(TimestepLadder,
     GivenStepDownRequestTwoLevels_WhenAppliedTwice_ThenStepsByOneEachTime)
{
    TimestepState state {};
    state.currentK = 0;

    [[maybe_unused]] auto _ = TimestepController::applyHysteresis(3, kDtMax, state, cfg());
    EXPECT_EQ(state.currentK, 1);

    _ = TimestepController::applyHysteresis(3, kDtMax, state, cfg());
    EXPECT_EQ(state.currentK, 2);
}

// ---------------------------------------------------------------------------
// applyHysteresis — step-up
// ---------------------------------------------------------------------------

TEST(TimestepLadder,
     GivenStepUpRequest_WhenDwellTimeNotMet_ThenRemainsAtCurrentLevel)
{
    TimestepState state {};
    state.currentK       = 4;
    state.tStableSeconds = 0.0;

    // tau_raise=0.5s, frameDt=1/60≈0.0167s → single call does not meet threshold
    const int kApplied = TimestepController::applyHysteresis(2, kDtMax, state, cfg());
    EXPECT_EQ(kApplied, 4);
}

TEST(TimestepLadder,
     GivenStepUpRequest_WhenDwellTimeMet_ThenStepsUpByOne)
{
    auto c = cfg();
    TimestepState state {};
    state.currentK       = 4;
    state.tStableSeconds = c.tau_raise_seconds;  // threshold already reached

    const int kApplied = TimestepController::applyHysteresis(2, kDtMax, state, c);
    EXPECT_EQ(kApplied, 3);
    EXPECT_EQ(state.currentK, 3);
}

TEST(TimestepLadder,
     GivenNoChangeInK_WhenHysteresisApplied_ThenAccumulatesStableTime)
{
    TimestepState state {};
    state.currentK       = 2;
    state.tStableSeconds = 0.0;

    [[maybe_unused]] const int k = TimestepController::applyHysteresis(2, kDtMax, state, cfg());
    EXPECT_NEAR(state.tStableSeconds, kDtMax, 1e-12);
}

// ---------------------------------------------------------------------------
// planSubsteps
// ---------------------------------------------------------------------------

TEST(TimestepLadder,
     GivenFrameDtEqualsDtMax_WhenSubstepsPlanComputed_ThenSingleSubstep)
{
    const SubstepPlan plan = TimestepController::planSubsteps(kDtMax, 0, kDtMax);
    EXPECT_EQ(plan.count, 1);
    EXPECT_DOUBLE_EQ(plan.dt, kDtMax);
    EXPECT_EQ(plan.k, 0);
}

TEST(TimestepLadder,
     GivenFrameDtWithK2_WhenSubstepsPlanComputed_ThenFourSubsteps)
{
    const SubstepPlan plan = TimestepController::planSubsteps(kDtMax, 2, kDtMax);
    EXPECT_EQ(plan.count, 4);
    EXPECT_NEAR(plan.dt * plan.count, kDtMax, 1e-15);
}

TEST(TimestepLadder,
     GivenSubstepPlanAtEachLadderLevel_WhenCounted_ThenSubstepsCoverExactFrame)
{
    for (int k = 0; k <= kKMax; ++k)
    {
        const SubstepPlan plan = TimestepController::planSubsteps(kDtMax, k, kDtMax);
        EXPECT_NEAR(plan.dt * plan.count, kDtMax, 1e-12)
            << "Coverage mismatch at k=" << k;
        EXPECT_GE(plan.count, 1);
    }
}

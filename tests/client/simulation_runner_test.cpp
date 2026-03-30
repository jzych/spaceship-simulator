#include "client/simulation_runner.hpp"

#include <gtest/gtest.h>
#include <cmath>
#include <numbers>

using namespace spaceship::client;
using namespace spaceship::server;

// ---------------------------------------------------------------------------
// SimulationRunnerTest
// ---------------------------------------------------------------------------

TEST(SimulationRunnerTest, GivenDefaultConfig_WhenConstructed_ThenBufferHasInitialSnapshot)
{
    SimulationRunner runner;

    EXPECT_FALSE(runner.snapshotBuffer().empty());
    EXPECT_EQ(runner.snapshotBuffer().size(), 1u);
    // Constructor runs snapshotIntervalTicks (3) bootstrap ticks.
    EXPECT_GT(runner.simulationElapsedSeconds(), 0.0);
}

TEST(SimulationRunnerTest, GivenDefaultConfig_WhenConstructed_ThenThreeMassiveBodiesExist)
{
    SimulationRunner runner;

    const auto& snapshot = *runner.snapshotBuffer().latest();
    EXPECT_EQ(snapshot.massiveBodies.size(), 3u);
}

TEST(SimulationRunnerTest, GivenZeroDelta_WhenAdvanced_ThenNoTicksRun)
{
    SimulationRunner runner;
    const double elapsedBefore = runner.simulationElapsedSeconds();

    const auto ticks = runner.advance(0.0);

    EXPECT_EQ(ticks, 0u);
    EXPECT_DOUBLE_EQ(runner.simulationElapsedSeconds(), elapsedBefore);
}

TEST(SimulationRunnerTest, GivenNegativeDelta_WhenAdvanced_ThenNoTicksRun)
{
    SimulationRunner runner;

    const auto ticks = runner.advance(-1.0);

    EXPECT_EQ(ticks, 0u);
}

TEST(SimulationRunnerTest, GivenSmallDelta_WhenAdvanced_ThenSimulationTimeAdvancesByScaledAmount)
{
    SimulationRunnerConfig runnerConfig;
    runnerConfig.timeScaleFactor = 1000.0; // 1 real second = 1000 sim seconds

    SimulationRunner runner({}, runnerConfig);

    const double elapsedBefore = runner.simulationElapsedSeconds();

    // Advance by 0.1 real seconds → 100 sim seconds → 6000 ticks at 60Hz
    // Capped by maxTicksPerAdvance (default 10,000), so all 6000 should run.
    const auto ticks = runner.advance(0.1);

    EXPECT_EQ(ticks, 6000u);
    EXPECT_NEAR(runner.simulationElapsedSeconds() - elapsedBefore, 100.0, 0.02);
}

TEST(SimulationRunnerTest, GivenLargeDelta_WhenAdvanced_ThenTicksCappedByMaxPerAdvance)
{
    SimulationRunnerConfig runnerConfig;
    runnerConfig.timeScaleFactor = 100'000.0;
    runnerConfig.maxTicksPerAdvance = 100;

    SimulationRunner runner({}, runnerConfig);

    // 1 real second at 100,000x = 100,000 sim seconds = 6,000,000 ticks
    // Should be capped at 100.
    const auto ticks = runner.advance(1.0);

    EXPECT_EQ(ticks, 100u);
}

TEST(SimulationRunnerTest, GivenMultipleAdvances_WhenAccumulated_ThenSnapshotBufferGrows)
{
    SimulationRunnerConfig runnerConfig;
    runnerConfig.timeScaleFactor = 100.0;

    SimulationRunner runner({}, runnerConfig);

    // Advance a few frames — each produces multiple snapshots (every 3 ticks).
    runner.advance(0.016); // ~96 ticks → ~32 snapshots
    runner.advance(0.016);
    runner.advance(0.016);

    EXPECT_GT(runner.snapshotBuffer().size(), 1u);
}

TEST(SimulationRunnerTest, GivenTimeScaleChange_WhenAdvanced_ThenNewScaleApplied)
{
    SimulationRunnerConfig runnerConfig;
    runnerConfig.timeScaleFactor = 1.0;

    SimulationRunner runner({}, runnerConfig);

    runner.advance(0.1); // 0.1 sim seconds → 6 ticks
    const double elapsed1 = runner.simulationElapsedSeconds();

    runner.setTimeScaleFactor(10.0);
    runner.advance(0.1); // now 1.0 sim seconds → 60 ticks
    const double elapsed2 = runner.simulationElapsedSeconds();

    EXPECT_NEAR(elapsed2 - elapsed1, 1.0, 0.02);
}

TEST(SimulationRunnerTest, GivenModerateTimeScale_WhenAdvanced_ThenEarthMovesVisibly)
{
    // At 1,000x, advance 0.1 real seconds = 100 sim seconds = 6,000 ticks.
    // Earth's angular velocity ≈ 1.99e-7 rad/s.
    // In 100 seconds: angle ≈ 1.99e-5 rad — tiny but nonzero position change.
    SimulationRunnerConfig runnerConfig;
    runnerConfig.timeScaleFactor = 1'000.0;

    SimulationRunner runner({}, runnerConfig);

    const auto& initialBodies = runner.snapshotBuffer().latest()->massiveBodies;
    ASSERT_GE(initialBodies.size(), 2u);
    const double initialEarthX = initialBodies[1].position.x;

    runner.advance(0.1);

    const auto& finalBodies = runner.snapshotBuffer().latest()->massiveBodies;
    ASSERT_GE(finalBodies.size(), 2u);
    const double finalEarthX = finalBodies[1].position.x;

    // Earth should have moved (X coordinate changed due to orbital motion)
    EXPECT_NE(initialEarthX, finalEarthX);
}

TEST(SimulationRunnerTest, GivenFractionalTicks_WhenAdvancedTwice_ThenRemainderCarriesOver)
{
    SimulationRunnerConfig runnerConfig;
    runnerConfig.timeScaleFactor = 1.0;

    SimulationRunner runner({}, runnerConfig);

    // Advance by less than one tick worth (1/60 = 0.0167s)
    const auto ticks1 = runner.advance(0.008);
    const auto ticks2 = runner.advance(0.008);

    // First call: 0.008s < 0.0167s → 0 ticks
    // Second call: accumulated 0.016s < 0.0167s → still 0 ticks
    // But close — one more small advance should trigger a tick
    const auto ticks3 = runner.advance(0.002);

    EXPECT_EQ(ticks1, 0u);
    EXPECT_EQ(ticks2, 0u);
    EXPECT_GE(ticks3, 1u);
}

#include "server/simulation/gravity/reference_body_selector.hpp"
#include "server/simulation/simulation_config.hpp"
#include "server/simulation/simulation_world.hpp"

#include <gtest/gtest.h>

// ---------------------------------------------------------------------------
// ReferenceBodySelectorTest — hysteresis-based body selection
// ---------------------------------------------------------------------------

class ReferenceBodySelectorTest : public ::testing::Test
{
  protected:
    // Two bodies: Sun at origin, Earth at 1 AU on +x
    static constexpr double kSunMu = 1.32712440018e20;
    static constexpr double kEarthMu = 3.986004418e14;
    static constexpr double kAU = 149'597'870'700.0;
    static constexpr spaceship::shared::NetId kSunNetId = 0U;
    static constexpr spaceship::shared::NetId kEarthNetId = 1U;

    spaceship::server::SimulationConfig config {};

    std::vector<spaceship::server::MassiveBodyState> makeSunEarth() const
    {
        return {
            spaceship::server::MassiveBodyState {
                {kSunNetId, "Sun", kSunMu, 6.9634e8}, {{0.0, 0.0, 0.0}}, {}, {}},
            spaceship::server::MassiveBodyState {
                {kEarthNetId, "Earth", kEarthMu, 6.371e6}, {{kAU, 0.0, 0.0}}, {}, {}},
        };
    }
};

TEST_F(ReferenceBodySelectorTest, GivenShipInLEO_WhenFirstUpdate_ThenEarthSelectedAsDominant)
{
    auto bodies = makeSunEarth();
    spaceship::server::ReferenceBodySelector selector;

    // Ship in LEO around Earth — Earth dominates
    const spaceship::shared::Vec3 shipPos {kAU + 6.771e6, 0.0, 0.0};
    const auto result = selector.update(shipPos, bodies, config, 1.0 / 60.0);

    EXPECT_EQ(result.bodyId, kEarthNetId);
    EXPECT_GT(result.score, 0.5);
}

TEST_F(ReferenceBodySelectorTest, GivenShipAtMidpointBetweenBodies_WhenFirstUpdate_ThenSunSelected)
{
    auto bodies = makeSunEarth();
    spaceship::server::ReferenceBodySelector selector;

    // Ship far from both, but closer to Sun in gravity terms (midway)
    const spaceship::shared::Vec3 shipPos {kAU * 0.5, 0.0, 0.0};
    const auto result = selector.update(shipPos, bodies, config, 1.0 / 60.0);

    EXPECT_EQ(result.bodyId, kSunNetId);
}

TEST_F(ReferenceBodySelectorTest, GivenShipJustOutsideEarthSOI_WhenSingleUpdate_ThenEarthRetained)
{
    auto bodies = makeSunEarth();
    spaceship::server::ReferenceBodySelector selector;
    const double dt = 1.0 / 60.0;

    // Start in LEO — locks to Earth
    const spaceship::shared::Vec3 leoPos {kAU + 6.771e6, 0.0, 0.0};
    (void)selector.update(leoPos, bodies, config, dt);

    // Move to a position where Sun marginally dominates (just past SOI boundary)
    // Earth SOI ≈ 0.929e9 m. Place ship at kAU - 1e9 (just outside Earth SOI)
    const spaceship::shared::Vec3 boundaryPos {kAU - 1.0e9, 0.0, 0.0};

    // Single update should NOT switch (hysteresis dwell not met)
    const auto result = selector.update(boundaryPos, bodies, config, dt);

    EXPECT_EQ(result.bodyId, kEarthNetId); // stays with Earth
    EXPECT_FALSE(result.changed);
}

TEST_F(ReferenceBodySelectorTest, GivenShipInDeepSpace_WhenDwellTimeExceeded_ThenSwitchesToSun)
{
    auto bodies = makeSunEarth();
    spaceship::server::ReferenceBodySelector selector;
    const double dt = 1.0 / 60.0;

    // Start in LEO — locks to Earth
    const spaceship::shared::Vec3 leoPos {kAU + 6.771e6, 0.0, 0.0};
    (void)selector.update(leoPos, bodies, config, dt);

    // Move to deep space where Sun clearly dominates
    const spaceship::shared::Vec3 deepSpace {kAU * 0.5, 0.0, 0.0};

    // Accumulate dwell time over many updates
    bool switchOccurred = false;
    spaceship::server::ReferenceBodySelection lastResult {};
    const int dwellTicks = static_cast<int>(config.referenceBodyDwellTimeSeconds / dt) + 10;
    for (int i = 0; i < dwellTicks; ++i)
    {
        lastResult = selector.update(deepSpace, bodies, config, dt);
        if (lastResult.changed)
            switchOccurred = true;
    }

    EXPECT_EQ(lastResult.bodyId, kSunNetId);
    EXPECT_TRUE(switchOccurred);
}

TEST_F(ReferenceBodySelectorTest, GivenNoBodies_WhenSelectorUpdated_ThenReturnsDefaultSelection)
{
    std::vector<spaceship::server::MassiveBodyState> emptyBodies {};
    spaceship::server::ReferenceBodySelector selector;

    const spaceship::shared::Vec3 shipPos {kAU + 6.771e6, 0.0, 0.0};
    const auto result = selector.update(shipPos, emptyBodies, config, 1.0 / 60.0);

    // Empty span → early return with default-constructed selection (unchanged state)
    EXPECT_EQ(result.bodyId, spaceship::shared::NetId {});
    EXPECT_FALSE(result.changed);
}

TEST_F(ReferenceBodySelectorTest, GivenShipAtBodyPosition_WhenSelectorUpdated_ThenBodyAtZeroDistanceIgnored)
{
    // Ship placed exactly at Earth's position → Earth distance = 0 < 1 m (kMinDist guard)
    // → Earth's gravitational contribution = 0.0 → Sun is the dominant body selected.
    auto bodies = makeSunEarth();
    spaceship::server::ReferenceBodySelector selector;

    const spaceship::shared::Vec3 shipPos {kAU, 0.0, 0.0};  // Earth is at kAU on +x
    const auto result = selector.update(shipPos, bodies, config, 1.0 / 60.0);

    // Earth contributes 0 (distance < kMinDist), Sun dominates from kAU away
    EXPECT_EQ(result.bodyId, kSunNetId);
}

TEST_F(ReferenceBodySelectorTest, GivenPartialDwellThenReturnToEarth_WhenMovedToDeepSpace_ThenFullDwellRequiredAgain)
{
    auto bodies = makeSunEarth();
    spaceship::server::ReferenceBodySelector selector;
    const double dt = 1.0 / 60.0;

    // Start in LEO — locks to Earth
    const spaceship::shared::Vec3 leoPos {kAU + 6.771e6, 0.0, 0.0};
    (void)selector.update(leoPos, bodies, config, dt);

    // Move to deep space for partial dwell (less than dwellTime)
    const spaceship::shared::Vec3 deepSpace {kAU * 0.5, 0.0, 0.0};
    const int partialTicks = static_cast<int>(config.referenceBodyDwellTimeSeconds / dt) / 2;
    for (int i = 0; i < partialTicks; ++i)
        (void)selector.update(deepSpace, bodies, config, dt);

    // Return to LEO — dwell should reset
    (void)selector.update(leoPos, bodies, config, dt);

    // Go back to deep space — need full dwell again
    spaceship::server::ReferenceBodySelection result {};
    for (int i = 0; i < partialTicks; ++i)
        result = selector.update(deepSpace, bodies, config, dt);

    EXPECT_EQ(result.bodyId, kEarthNetId); // still Earth — dwell was reset
}

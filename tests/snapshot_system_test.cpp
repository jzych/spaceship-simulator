#include "server/snapshot_system.hpp"

#include <gtest/gtest.h>

using namespace spaceship::server;

// ---------------------------------------------------------------------------
// SnapshotSystemTest — direct unit tests for snapshot summary generation
// ---------------------------------------------------------------------------

TEST(SnapshotSystemTest, GivenEmptyWorld_WhenSnapshotBuilt_ThenAllCountsAreZero)
{
    SnapshotSystem system;
    SimulationWorld world {};

    const auto summary = system.buildSnapshotSummary(world);

    EXPECT_NE(summary.find("bodies=0"), std::string::npos);
    EXPECT_NE(summary.find("ships=0"), std::string::npos);
    EXPECT_NE(summary.find("projectiles=0"), std::string::npos);
}

TEST(SnapshotSystemTest, GivenWorldWithEntities_WhenSnapshotBuilt_ThenCountsMatchContents)
{
    SnapshotSystem system;
    SimulationWorld world {};

    world.massiveBodies.push_back(MassiveBodyState {{0, "Sun", 1e20, 1e8}});
    world.massiveBodies.push_back(MassiveBodyState {{1, "Earth", 1e14, 1e6}});
    world.ships.push_back(ShipState {100});
    world.ships.push_back(ShipState {101});
    world.ships.push_back(ShipState {102});

    const auto summary = system.buildSnapshotSummary(world);

    EXPECT_NE(summary.find("bodies=2"), std::string::npos);
    EXPECT_NE(summary.find("ships=3"), std::string::npos);
    EXPECT_NE(summary.find("projectiles=0"), std::string::npos);
}

TEST(SnapshotSystemTest, GivenAnyWorld_WhenSnapshotBuilt_ThenSummaryStartsWithSnapshotPrefix)
{
    SnapshotSystem system;
    SimulationWorld world {};

    const auto summary = system.buildSnapshotSummary(world);

    EXPECT_EQ(summary.substr(0, 8), "snapshot");
}

TEST(SnapshotSystemTest, GivenWorldWithProjectiles_WhenSnapshotBuilt_ThenProjectilesCountedCorrectly)
{
    SnapshotSystem system;
    SimulationWorld world {};

    world.projectiles.push_back(ProjectileState {10'000});
    world.projectiles.push_back(ProjectileState {10'001});

    const auto summary = system.buildSnapshotSummary(world);

    EXPECT_NE(summary.find("projectiles=2"), std::string::npos);
}

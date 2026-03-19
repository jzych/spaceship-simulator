#include "server/simulation_server.hpp"

#include <gtest/gtest.h>

TEST(SimulationServerSmokeTest, BootstrapsThreeMassiveBodies)
{
    spaceship::server::SimulationServer server;

    ASSERT_EQ(server.world().massiveBodies.size(), 3U);
    EXPECT_EQ(server.world().massiveBodies[0].definition.name, "Sun");
    EXPECT_EQ(server.world().massiveBodies[1].definition.name, "Earth");
    EXPECT_EQ(server.world().massiveBodies[2].definition.name, "Moon");
}

TEST(SimulationServerSmokeTest, TickAdvancesServerClock)
{
    spaceship::server::SimulationServer server;

    server.tick();

    EXPECT_EQ(server.tickCount(), 1U);
    EXPECT_TRUE(server.lastSnapshotSummary().empty());
}

#include "client/client_snapshot_buffer.hpp"
#include "server/snapshot/snapshot_types.hpp"

#include <gtest/gtest.h>

using namespace spaceship::client;
using namespace spaceship::server;

// ---------------------------------------------------------------------------
// ClientSnapshotBufferTest — unit tests for the snapshot buffer
// ---------------------------------------------------------------------------

TEST(ClientSnapshotBufferTest, GivenNoSnapshot_WhenLatestQueried_ThenOptionalIsEmpty)
{
    ClientSnapshotBuffer buffer;

    EXPECT_FALSE(buffer.latest().has_value());
}

TEST(ClientSnapshotBufferTest, GivenPushedSnapshot_WhenLatestQueried_ThenSnapshotReturned)
{
    ClientSnapshotBuffer buffer;

    WorldSnapshot snapshot;
    snapshot.serverTick     = 5U;
    snapshot.elapsedSeconds = 0.25;
    buffer.push(std::move(snapshot));

    ASSERT_TRUE(buffer.latest().has_value());
    EXPECT_EQ(buffer.latest()->serverTick, 5U);
    EXPECT_DOUBLE_EQ(buffer.latest()->elapsedSeconds, 0.25);
}

TEST(ClientSnapshotBufferTest, GivenTwoSnapshots_WhenBothPushed_ThenLatestReturnsSecond)
{
    ClientSnapshotBuffer buffer;

    WorldSnapshot first;
    first.serverTick = 5U;
    buffer.push(std::move(first));

    WorldSnapshot second;
    second.serverTick = 10U;
    buffer.push(std::move(second));

    ASSERT_TRUE(buffer.latest().has_value());
    EXPECT_EQ(buffer.latest()->serverTick, 10U);
}

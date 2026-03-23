#include "server/collision/collision_system.hpp"

#include <gtest/gtest.h>

using namespace spaceship::server;

// ---------------------------------------------------------------------------
// CollisionSystemTest — projectile TTL-based cleanup
// ---------------------------------------------------------------------------

TEST(CollisionSystemTest, GivenPositiveTtl_WhenUpdated_ThenProjectileSurvives)
{
    CollisionSystem system;
    std::vector<ProjectileState> projectiles;
    projectiles.push_back(ProjectileState {10'000, {}, {}, {}, {}, {5.0, 100}});

    system.update(projectiles);

    ASSERT_EQ(projectiles.size(), 1U);
}

TEST(CollisionSystemTest, GivenZeroTtl_WhenUpdated_ThenProjectileRemoved)
{
    CollisionSystem system;
    std::vector<ProjectileState> projectiles;
    projectiles.push_back(ProjectileState {10'000, {}, {}, {}, {}, {0.0, 100}});

    system.update(projectiles);

    EXPECT_TRUE(projectiles.empty());
}

TEST(CollisionSystemTest, GivenNegativeTtl_WhenUpdated_ThenProjectileRemoved)
{
    CollisionSystem system;
    std::vector<ProjectileState> projectiles;
    projectiles.push_back(ProjectileState {10'000, {}, {}, {}, {}, {-1.0, 100}});

    system.update(projectiles);

    EXPECT_TRUE(projectiles.empty());
}

TEST(CollisionSystemTest, GivenMixedTtlProjectiles_WhenUpdated_ThenOnlyPositiveTtlRemain)
{
    CollisionSystem system;
    std::vector<ProjectileState> projectiles;
    projectiles.push_back(ProjectileState {10'000, {}, {}, {}, {}, {5.0, 100}});
    projectiles.push_back(ProjectileState {10'001, {}, {}, {}, {}, {-1.0, 101}});
    projectiles.push_back(ProjectileState {10'002, {}, {}, {}, {}, {3.0, 102}});
    projectiles.push_back(ProjectileState {10'003, {}, {}, {}, {}, {0.0, 103}});

    system.update(projectiles);

    ASSERT_EQ(projectiles.size(), 2U);
    EXPECT_EQ(projectiles[0].netId, 10'000U);
    EXPECT_EQ(projectiles[1].netId, 10'002U);
}

TEST(CollisionSystemTest, GivenNoProjectiles_WhenUpdated_ThenNoChange)
{
    CollisionSystem system;
    std::vector<ProjectileState> projectiles;

    EXPECT_NO_FATAL_FAILURE(system.update(projectiles));
    EXPECT_TRUE(projectiles.empty());
}

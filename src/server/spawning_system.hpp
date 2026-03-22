#pragma once

// Defines the server-side entity creation logic for ships and projectiles.

#include "server/simulation_config.hpp"
#include "server/simulation_world.hpp"

#include <optional>
#include <span>

namespace spaceship::server
{

inline constexpr shared::NetId kFirstShipNetId = 100U;
inline constexpr shared::NetId kFirstProjectileNetId = 10'000U;

struct ShipSpawnRequest
{
    shared::Transform transform {};
    shared::Velocity velocity {};
    std::optional<shared::Vec3> initialAcceleration {};
};

class SpawningSystem
{
  public:
    shared::NetId spawnShip(
        std::vector<ShipState>& ships,
        const ShipSpawnRequest& request,
        const SimulationConfig& config,
        std::span<const MassiveBodyState> massiveBodies);
    shared::NetId spawnProjectile(
        std::vector<ProjectileState>& projectiles,
        const ShipState& ship,
        const SimulationConfig& config,
        std::span<const MassiveBodyState> massiveBodies);
    void update(
        std::span<ShipState> ships,
        std::vector<ProjectileState>& projectiles,
        std::span<const MassiveBodyState> massiveBodies,
        const SimulationConfig& config);

  private:
    shared::NetId nextShipNetId_ {kFirstShipNetId};
    shared::NetId nextProjectileNetId_ {kFirstProjectileNetId};
};

} // namespace spaceship::server

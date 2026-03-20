#pragma once

// Defines the server-side entity creation logic for ships and projectiles.

#include "server/simulation_config.hpp"
#include "server/simulation_world.hpp"

#include <span>

namespace spaceship::server
{

struct ShipSpawnRequest
{
    shared::NetId netId {};
    shared::Transform transform {};
    shared::Velocity velocity {};
};

class SpawningSystem
{
  public:
    ShipState& spawnShip(
        std::vector<ShipState>& ships,
        const ShipSpawnRequest& request,
        const SimulationConfig& config) const;
    ProjectileState& spawnProjectile(
        std::vector<ProjectileState>& projectiles,
        const ShipState& ship,
        const SimulationConfig& config);
    void update(
        std::span<ShipState> ships,
        std::vector<ProjectileState>& projectiles,
        const SimulationConfig& config);

  private:
    shared::NetId nextProjectileNetId_ {10'000U};
};

} // namespace spaceship::server

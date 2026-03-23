#pragma once

// Defines the integration stage that advances dynamic entities over a fixed tick.

#include "server/simulation/simulation_config.hpp"
#include "server/simulation/simulation_world.hpp"

#include <span>

namespace spaceship::server
{

class IntegrationSystem
{
  public:
    // Phase 1 of velocity Verlet: x_{n+1} = x_n + v_n*dt + 0.5*a_n*dt²
    // Saves a_n into previousAcceleration before returning.
    void integratePositions(
        std::span<ShipState> ships,
        std::span<ProjectileState> projectiles,
        const SimulationConfig& config) const;

    // Phase 2 of velocity Verlet: v_{n+1} = v_n + 0.5*(a_n + a_{n+1})*dt
    // Reads previousAcceleration as a_n and acceleration as a_{n+1}.
    void integrateVelocities(
        std::span<ShipState> ships,
        std::span<ProjectileState> projectiles,
        const SimulationConfig& config) const;

    // Decrements TTL on all projectiles by one fixed time step.
    void decrementTtl(
        std::span<ProjectileState> projectiles,
        const SimulationConfig& config) const;
};

} // namespace spaceship::server

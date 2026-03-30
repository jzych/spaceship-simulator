#pragma once

// Selects the dominant gravitational reference body for a ship using
// gravity-fraction scoring with hysteresis to prevent rapid flapping.

#include "server/simulation/simulation_config.hpp"
#include "shared/sim_types.hpp"

#include <span>

namespace spaceship::server
{

struct MassiveBodyState;

struct ReferenceBodySelection
{
    shared::NetId bodyId {};
    double score {};
    double dwellSeconds {};
    bool changed {};
};

class ReferenceBodySelector
{
  public:
    [[nodiscard]] ReferenceBodySelection update(
        const shared::Vec3& shipPosition,
        std::span<const MassiveBodyState> massiveBodies,
        const SimulationConfig& config,
        double dt);

  private:
    shared::NetId currentBodyId_ {};
    double dwellAccumulator_ {};
    bool initialized_ {};
};

} // namespace spaceship::server

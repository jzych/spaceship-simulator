#pragma once

// Builds compact snapshot summaries from the authoritative world state.

#include "server/simulation_world.hpp"

#include <string>

namespace spaceship::server
{

class SnapshotSystem
{
  public:
    [[nodiscard]] std::string buildSnapshotSummary(const SimulationWorld& world) const;
};

} // namespace spaceship::server

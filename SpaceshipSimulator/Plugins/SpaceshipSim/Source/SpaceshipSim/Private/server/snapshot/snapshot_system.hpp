#pragma once

// Builds structured world snapshots from the authoritative world state.

#include "server/simulation/simulation_world.hpp"
#include "server/snapshot/snapshot_types.hpp"
#include "shared/sim_types.hpp"

#include <string>

namespace spaceship::server
{

class SnapshotSystem
{
  public:
    // Build a full structured snapshot from the current authoritative world state.
    // Const read only — no allocation side-effects on the world.
    [[nodiscard]] WorldSnapshot buildSnapshot(
        const SimulationWorld& world,
        shared::Tick           serverTick,
        double                 elapsedSeconds) const;

    // Produce a human-readable debug summary from an already-built snapshot.
    // Decoupled from the snapshot pipeline — call only when logging is needed.
    [[nodiscard]] static std::string debugSummary(const WorldSnapshot& snapshot);
};

} // namespace spaceship::server

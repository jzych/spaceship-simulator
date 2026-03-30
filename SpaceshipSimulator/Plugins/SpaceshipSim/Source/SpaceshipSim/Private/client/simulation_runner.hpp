#pragma once

// Runs the authoritative simulation at a configurable time scale and feeds
// snapshots into a ClientSnapshotBuffer.  Designed for single-process use
// (no networking) — the server and client live in the same executable.
//
// Typical usage (inside a frame loop):
//     runner.advance(frameDeltaSeconds);
//     auto state = runner.snapshotBuffer().interpolate(renderTime);

#include "client/client_snapshot_buffer.hpp"
#include "server/simulation/simulation_config.hpp"
#include "server/simulation/simulation_server.hpp"

namespace spaceship::client
{

struct SimulationRunnerConfig
{
    // How many real-time seconds equal one simulation second.
    // 100,000 → Earth orbits the Sun in ~5 minutes of wall-clock time.
    double timeScaleFactor {100'000.0};

    // Snapshot buffer capacity (number of snapshots retained for interpolation).
    std::size_t snapshotBufferCapacity {64};

    // Maximum sim ticks per advance() call — safety cap to prevent freezing
    // when frameDeltaSeconds is unusually large (e.g. after a breakpoint).
    std::size_t maxTicksPerAdvance {10'000};
};

class SimulationRunner
{
  public:
    explicit SimulationRunner(
        const server::SimulationConfig& simConfig = {},
        const SimulationRunnerConfig&   runnerConfig = {});

    // Advance the simulation by the wall-clock delta scaled by timeScaleFactor.
    // Internally runs as many server ticks as needed and pushes resulting
    // snapshots into the buffer.  Returns the number of ticks executed.
    std::size_t advance(double frameDeltaSeconds);

    [[nodiscard]] const ClientSnapshotBuffer& snapshotBuffer() const { return snapshotBuffer_; }
    [[nodiscard]] ClientSnapshotBuffer&       snapshotBuffer()       { return snapshotBuffer_; }

    [[nodiscard]] const server::SimulationServer& server() const { return server_; }

    [[nodiscard]] double simulationElapsedSeconds() const;

    [[nodiscard]] double timeScaleFactor() const { return runnerConfig_.timeScaleFactor; }
    void setTimeScaleFactor(double factor) { runnerConfig_.timeScaleFactor = factor; }

  private:
    server::SimulationServer server_;
    ClientSnapshotBuffer     snapshotBuffer_;
    SimulationRunnerConfig   runnerConfig_;

    // Fractional tick accumulator — carries sub-tick time between advance() calls.
    double accumulatedSimSeconds_ {0.0};
};

} // namespace spaceship::client

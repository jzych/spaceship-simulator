#include "client/simulation_runner.hpp"

#include <algorithm>
#include <cassert>

namespace spaceship::client
{

SimulationRunner::SimulationRunner(
    const server::SimulationConfig& simConfig,
    const SimulationRunnerConfig&   runnerConfig)
    : server_(simConfig)
    , snapshotBuffer_(runnerConfig.snapshotBufferCapacity)
    , runnerConfig_(runnerConfig)
{
    assert(runnerConfig_.timeScaleFactor > 0.0);

    // Run enough ticks to produce the first real snapshot (with entities).
    // The server produces a snapshot every snapshotIntervalTicks (default 3).
    for (shared::Tick i = 0; i < simConfig.snapshotIntervalTicks; ++i)
        server_.tick();

    snapshotBuffer_.push(server_.lastSnapshot());
}

std::size_t SimulationRunner::advance(double frameDeltaSeconds)
{
    if (frameDeltaSeconds <= 0.0)
        return 0;

    const double scaledDelta = frameDeltaSeconds * runnerConfig_.timeScaleFactor;
    accumulatedSimSeconds_ += scaledDelta;

    const double fixedDt = server::SimulationConfig{}.fixedDeltaSeconds;
    const auto ticksBudget = static_cast<std::size_t>(accumulatedSimSeconds_ / fixedDt);
    const auto ticksToRun  = std::min(ticksBudget, runnerConfig_.maxTicksPerAdvance);

    for (std::size_t i = 0; i < ticksToRun; ++i)
    {
        server_.tick();

        // The server produces a snapshot every snapshotIntervalTicks (default 3).
        // We detect a new snapshot by comparing elapsed times.
        const auto& snapshot = server_.lastSnapshot();
        if (snapshotBuffer_.empty() ||
            snapshot.elapsedSeconds > snapshotBuffer_.latest()->elapsedSeconds)
        {
            snapshotBuffer_.push(snapshot);
        }
    }

    // Subtract only the ticks we actually ran from the accumulator,
    // carrying the fractional remainder for the next call.
    accumulatedSimSeconds_ -= static_cast<double>(ticksToRun) * fixedDt;

    return ticksToRun;
}

double SimulationRunner::simulationElapsedSeconds() const
{
    return server_.lastSnapshot().elapsedSeconds;
}

} // namespace spaceship::client

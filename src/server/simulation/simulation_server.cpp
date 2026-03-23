#include "server/simulation/simulation_server.hpp"

#include "server/collision/collision_system.hpp"
#include "server/control/ship_control_system.hpp"
#include "server/simulation/bootstrap.hpp"
#include "server/simulation/gravity/gravity_system.hpp"
#include "server/simulation/gravity/massive_body_motion_system.hpp"
#include "server/simulation/gravity/orbit_cache_system.hpp"
#include "server/simulation/integration_system.hpp"
#include "server/simulation/timestep/timescale_heuristics.hpp"
#include "server/simulation/timestep/timestep_controller.hpp"
#include "server/snapshot/snapshot_system.hpp"
#include "server/spawning/spawning_system.hpp"

#include <algorithm>
#include <unordered_map>
#include <vector>

namespace spaceship::server
{

// ---------------------------------------------------------------------------
// Impl — all systems and mutable state; hidden from the public header.
// ---------------------------------------------------------------------------

struct SimulationServer::Impl
{
    SimulationConfig config {};
    SimulationWorld world {};
    shared::Tick tickCount {};
    double elapsedSeconds {};
    std::string lastSnapshotSummary {};

    // Ship NetId → index in world.ships for O(1) control updates.
    // Valid as long as ships are never removed mid-vector; update on spawn.
    std::unordered_map<shared::NetId, std::size_t> shipIndex {};

    MassiveBodyMotionSystem massiveBodyMotionSystem {};
    SpawningSystem          spawningSystem {};
    ShipControlSystem       shipControlSystem {};
    GravitySystem           gravitySystem {};
    IntegrationSystem       integrationSystem {};
    OrbitCacheSystem        orbitCacheSystem {};
    CollisionSystem         collisionSystem {};
    SnapshotSystem          snapshotSystem {};

    // Adaptive timestep diagnostics (most recent frame).
    std::vector<TimestepDiagnostics> lastTimestepDiagnostics {};

    std::optional<std::reference_wrapper<ShipState>> findShip(shared::NetId shipNetId)
    {
        const auto it = shipIndex.find(shipNetId);
        if (it == shipIndex.end())
            return std::nullopt;
        return std::ref(world.ships[it->second]);
    }
};

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------

SimulationServer::SimulationServer(const SimulationConfig& config)
    : impl_(std::make_unique<Impl>())
{
    impl_->config = config;
    impl_->world  = createInitialWorld();
}

SimulationServer::SimulationServer(SimulationWorld world, const SimulationConfig& config)
    : impl_(std::make_unique<Impl>())
{
    impl_->config = config;
    impl_->world  = std::move(world);
}

SimulationServer::~SimulationServer() = default;
SimulationServer::SimulationServer(SimulationServer&&) noexcept = default;
SimulationServer& SimulationServer::operator=(SimulationServer&&) noexcept = default;

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

shared::NetId SimulationServer::spawnShip(const ShipSpawnRequest& request)
{
    const auto netId = impl_->spawningSystem.spawnShip(
        impl_->world.ships, request, impl_->config, impl_->world.massiveBodies);
    impl_->shipIndex[netId] = impl_->world.ships.size() - 1;
    return netId;
}

void SimulationServer::updateShipControl(
    shared::NetId shipNetId, const shared::ShipControl& control)
{
    const auto ship = impl_->findShip(shipNetId);
    if (ship.has_value())
        ship->get().control = control;
}

void SimulationServer::tick()
{
    const double frameDt = impl_->config.fixedDeltaSeconds;

    if (!impl_->config.useAdaptiveTimestep)
    {
        // ---- Fixed-step path ----
        impl_->massiveBodyMotionSystem.update(impl_->world.massiveBodies, impl_->elapsedSeconds);
        impl_->elapsedSeconds += frameDt;

        impl_->spawningSystem.update(
            impl_->world.ships, impl_->world.projectiles, impl_->world.massiveBodies, impl_->config);

        impl_->shipControlSystem.update(impl_->world.ships, impl_->config);

        // CCD: detect and resolve collisions before integrating positions.
        // Uses pre-integration positions and velocities as the interval start state.
        impl_->world.collisionEvents.clear();
        impl_->collisionSystem.detectAndResolve(
            impl_->world.ships,
            impl_->world.projectiles,
            impl_->world.massiveBodies,
            impl_->world.collisionEvents,
            impl_->config,
            frameDt);

        // Rebuild ship index after possible despawns.
        impl_->shipIndex.clear();
        for (std::size_t i = 0; i < impl_->world.ships.size(); ++i)
            impl_->shipIndex[impl_->world.ships[i].netId] = i;

        impl_->integrationSystem.integratePositions(
            impl_->world.ships, impl_->world.projectiles, impl_->config);

        impl_->gravitySystem.update(
            impl_->world.massiveBodies, impl_->world.ships, impl_->world.projectiles);

        impl_->integrationSystem.integrateVelocities(
            impl_->world.ships, impl_->world.projectiles, impl_->config);
    }
    else
    {
        // ---- Adaptive substep path ----
        const TimestepLadderConfig& ladder = impl_->config.timestepLadder;

        impl_->spawningSystem.update(
            impl_->world.ships, impl_->world.projectiles, impl_->world.massiveBodies, impl_->config);

        // ShipControlSystem is called once per outer tick (thrust direction doesn't change
        // within a frame — only the magnitude applied per substep changes via dt).
        impl_->shipControlSystem.update(impl_->world.ships, impl_->config);

        // Determine the global minimum ladder level across all entities
        // (conservative: all entities share the tightest required dt this frame).
        // Retain per-entity dtTarget so diagnostics can report the pre-quantization heuristic value.
        struct EntityHeuristicResult { shared::NetId netId; double dtTarget; };
        std::vector<EntityHeuristicResult> entityHeuristics;
        entityHeuristics.reserve(impl_->world.ships.size() + impl_->world.projectiles.size());

        int kGlobal = 0;  // start at coarsest
        for (auto& ship : impl_->world.ships)
        {
            const double dtTarget = computeTargetTimestep(
                ship.transform.position,
                ship.velocity.linear,
                ship.acceleration,
                ship.timestepState.aPrev,
                ship.timestepState.dtPrev,
                impl_->world.massiveBodies,
                ladder);
            const int kDesired = TimestepController::quantizeToLadder(dtTarget, ladder.dt_max, ladder.k_max);
            const int kApplied = TimestepController::applyHysteresis(kDesired, frameDt, ship.timestepState, ladder);
            kGlobal = std::max(kGlobal, kApplied);
            entityHeuristics.push_back({ship.netId, dtTarget});
        }
        for (auto& proj : impl_->world.projectiles)
        {
            const double dtTarget = computeTargetTimestep(
                proj.transform.position,
                proj.velocity.linear,
                proj.acceleration,
                proj.timestepState.aPrev,
                proj.timestepState.dtPrev,
                impl_->world.massiveBodies,
                ladder);
            const int kDesired = TimestepController::quantizeToLadder(dtTarget, ladder.dt_max, ladder.k_max);
            const int kApplied = TimestepController::applyHysteresis(kDesired, frameDt, proj.timestepState, ladder);
            kGlobal = std::max(kGlobal, kApplied);
            entityHeuristics.push_back({proj.netId, dtTarget});
        }

        const SubstepPlan plan = TimestepController::planSubsteps(frameDt, kGlobal, ladder.dt_max);

        // Record diagnostics — dtRequested is the raw heuristic target (pre-quantization).
        impl_->lastTimestepDiagnostics.clear();
        for (const auto& [netId, dtTarget] : entityHeuristics)
        {
            impl_->lastTimestepDiagnostics.push_back({
                netId,
                dtTarget,
                plan.dt,
                plan.k,
                plan.count,
            });
        }

        // Execute substeps.
        impl_->world.collisionEvents.clear();
        for (int s = 0; s < plan.count; ++s)
        {
            impl_->massiveBodyMotionSystem.update(
                impl_->world.massiveBodies, impl_->elapsedSeconds);
            impl_->elapsedSeconds += plan.dt;

            // CCD per substep — catches tunneling at sub-frame granularity.
            impl_->collisionSystem.detectAndResolve(
                impl_->world.ships,
                impl_->world.projectiles,
                impl_->world.massiveBodies,
                impl_->world.collisionEvents,
                impl_->config,
                plan.dt);

            // Rebuild ship index after possible despawns.
            impl_->shipIndex.clear();
            for (std::size_t i = 0; i < impl_->world.ships.size(); ++i)
                impl_->shipIndex[impl_->world.ships[i].netId] = i;

            impl_->integrationSystem.integratePositions(
                impl_->world.ships, impl_->world.projectiles, plan.dt);

            impl_->gravitySystem.update(
                impl_->world.massiveBodies, impl_->world.ships, impl_->world.projectiles);

            impl_->integrationSystem.integrateVelocities(
                impl_->world.ships, impl_->world.projectiles, plan.dt);

            // Update jerk history on each entity after each substep.
            for (auto& ship : impl_->world.ships)
            {
                ship.timestepState.aPrev  = ship.acceleration;
                ship.timestepState.dtPrev = plan.dt;
            }
            for (auto& proj : impl_->world.projectiles)
            {
                proj.timestepState.aPrev  = proj.acceleration;
                proj.timestepState.dtPrev = plan.dt;
            }
        }
    }

    // Post-integration systems run once per outer tick regardless of substep count.
    impl_->collisionSystem.decrementTtl(impl_->world.projectiles, frameDt);
    impl_->orbitCacheSystem.update(
        impl_->world.ships, impl_->world.massiveBodies, impl_->tickCount + 1, impl_->config);
    impl_->collisionSystem.update(impl_->world.projectiles);

    ++impl_->tickCount;

    if (impl_->config.snapshotIntervalTicks > 0 &&
        impl_->tickCount % impl_->config.snapshotIntervalTicks == 0)
    {
        impl_->lastSnapshotSummary = impl_->snapshotSystem.buildSnapshotSummary(impl_->world);
    }
}

// ---------------------------------------------------------------------------
// Accessors
// ---------------------------------------------------------------------------

shared::Tick SimulationServer::tickCount() const
{
    return impl_->tickCount;
}

const SimulationWorld& SimulationServer::world() const
{
    return impl_->world;
}

const std::string& SimulationServer::lastSnapshotSummary() const
{
    return impl_->lastSnapshotSummary;
}

const std::vector<TimestepDiagnostics>& SimulationServer::timestepDiagnostics() const
{
    return impl_->lastTimestepDiagnostics;
}

} // namespace spaceship::server

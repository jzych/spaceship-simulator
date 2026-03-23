#include "server/simulation/simulation_server.hpp"

#include "server/collision/collision_system.hpp"
#include "server/control/ship_control_system.hpp"
#include "server/simulation/bootstrap.hpp"
#include "server/simulation/gravity/gravity_system.hpp"
#include "server/simulation/gravity/massive_body_motion_system.hpp"
#include "server/simulation/gravity/orbit_cache_system.hpp"
#include "server/simulation/integration_system.hpp"
#include "server/snapshot/snapshot_system.hpp"
#include "server/spawning/spawning_system.hpp"

#include <algorithm>
#include <unordered_map>

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
    impl_->massiveBodyMotionSystem.update(impl_->world.massiveBodies, impl_->elapsedSeconds);
    impl_->elapsedSeconds += impl_->config.fixedDeltaSeconds;

    impl_->spawningSystem.update(
        impl_->world.ships, impl_->world.projectiles, impl_->world.massiveBodies, impl_->config);

    // ship.acceleration carries gravity(x_n) from the end of the previous tick.
    // ShipControlSystem writes fresh thrust into ship.thrustAcceleration.
    // integratePositions computes a_n = gravity(x_n) + thrust, saves it, advances x.
    impl_->shipControlSystem.update(impl_->world.ships, impl_->config);
    impl_->integrationSystem.integratePositions(
        impl_->world.ships, impl_->world.projectiles, impl_->config);

    // One gravity call per tick: gravity(x_{n+1}) stored in ship.acceleration.
    // integrateVelocities computes a_{n+1} = gravity(x_{n+1}) + thrust (same tick).
    impl_->gravitySystem.update(
        impl_->world.massiveBodies, impl_->world.ships, impl_->world.projectiles);

    // Verlet phase 2: v_{n+1} = v_n + 0.5*(a_n + a_{n+1})*dt
    impl_->integrationSystem.integrateVelocities(
        impl_->world.ships, impl_->world.projectiles, impl_->config);

    // TTL decrement and projectile expiry are both projectile-lifetime concerns.
    // tickCount_ + 1 = the tick being committed (pre-increment).
    impl_->collisionSystem.decrementTtl(impl_->world.projectiles, impl_->config);
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

} // namespace spaceship::server

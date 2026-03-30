#include "server/simulation/gravity/orbit_cache_system.hpp"
#include "server/simulation/gravity/orbit_fitting.hpp"
#include "server/simulation/simulation_math.hpp"

#include <algorithm>

namespace spaceship::server
{

namespace
{

constexpr shared::Tick kActiveShipRefreshIntervalTicks = 1;
constexpr shared::Tick kInactiveShipRefreshIntervalTicks = 60;

const MassiveBodyState* findBody(
    std::span<const MassiveBodyState> massiveBodies,
    shared::NetId bodyId)
{
    const auto it = std::find_if(
        massiveBodies.begin(), massiveBodies.end(),
        [bodyId](const MassiveBodyState& b) { return b.definition.netId == bodyId; });
    return (it != massiveBodies.end()) ? &(*it) : nullptr;
}

bool isActiveShip(const ShipState& ship)
{
    return ship.control.throttle != 0.0;
}

} // namespace

void OrbitCacheSystem::update(
    std::span<ShipState> ships,
    std::span<const MassiveBodyState> massiveBodies,
    shared::Tick currentTick,
    const SimulationConfig& config) const
{
    if (massiveBodies.empty())
        return;

    const double dt = config.fixedDeltaSeconds;

    for (auto& ship : ships)
    {
        // Detect thrust state change → invalidate
        const bool nowActive = isActiveShip(ship);
        if (ship.orbitCache.wasThrusting != nowActive)
            ship.orbitCache.dirty = true;

        // Update reference body selection
        const auto selection = ship.referenceBodySelector.update(
            ship.transform.position, massiveBodies, config, dt);

        if (selection.changed)
            ship.orbitCache.dirty = true;

        // Check if refresh is needed
        const bool needsRefresh = ship.orbitCache.dirty ||
            (currentTick >= ship.orbitCache.validUntilTick) ||
            (ship.orbitCache.epoch == 0);

        // Find reference body
        const auto* refBody = findBody(massiveBodies, selection.bodyId);
        if (refBody == nullptr)
            continue;

        const shared::Vec3 rRel = subtract(ship.transform.position, refBody->transform.position);

        // Orbit fit (expensive): only when refresh is needed
        if (needsRefresh)
        {
            const shared::Vec3 vRel = subtract(ship.velocity.linear, refBody->velocity.linear);
            const double mu = refBody->definition.muMetersCubedPerSecondSquared;
            const double bodyRadius = refBody->definition.radiusMeters;

            ship.orbitCache = fitOrbit(rRel, vRel, mu, bodyRadius, selection.bodyId, currentTick);

            const shared::Vec3 totalAccel = add(ship.acceleration, ship.thrustAcceleration);
            ship.orbitCache.qualityScore = computeQualityScore(totalAccel, rRel, mu);

            const shared::Tick refreshInterval = nowActive
                ? kActiveShipRefreshIntervalTicks
                : kInactiveShipRefreshIntervalTicks;
            ship.orbitCache.validUntilTick = currentTick + refreshInterval;
            ship.orbitCache.dirty = false;
            ship.orbitCache.wasThrusting = nowActive;
        }

        // Geographic telemetry (cheap): always update using pre-computed spin angle
        const auto geo = toBodyFixedGeographic(
            rRel, refBody->definition.radiusMeters, refBody->spinAngleRadians);
        ship.orbitCache.altitudeMeters = geo.altitudeMeters;
        ship.orbitCache.longitudeRadians = geo.longitudeRadians;
        ship.orbitCache.latitudeRadians = geo.latitudeRadians;
    }
}

} // namespace spaceship::server

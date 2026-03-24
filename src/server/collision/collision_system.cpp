#include "server/collision/collision_system.hpp"
#include "server/collision/broad_phase.hpp"
#include "server/collision/sweep_math.hpp"
#include "server/simulation/simulation_math.hpp"

#include <algorithm>
#include <cmath>
#include <unordered_map>
#include <unordered_set>

namespace spaceship::server
{

// ---------------------------------------------------------------------------
// TTL cleanup
// ---------------------------------------------------------------------------

void CollisionSystem::decrementTtl(
    std::span<ProjectileState> projectiles,
    double dt) const noexcept
{
    for (auto& projectile : projectiles)
        projectile.params.ttlSeconds -= dt;
}

void CollisionSystem::update(std::vector<ProjectileState>& projectiles) const noexcept
{
    std::erase_if(
        projectiles,
        [](const ProjectileState& p) { return p.params.ttlSeconds <= 0.0; });
}

// ---------------------------------------------------------------------------
// CCD pipeline helpers
// ---------------------------------------------------------------------------

namespace
{

// Reduced-mass relative kinetic energy for two colliding objects.
//
// Uses the centre-of-mass frame kinetic energy formula:
//   E_rel = 0.5 * mu_r * |dv|^2
// where mu_r is the reduced mass:
//   mu_r = (mA * mB) / (mA + mB)
// and dv is the relative velocity between the two objects.
//
// E_rel determines the severity of the impact — higher energy means
// more destructive. Compared against per-pair-type thresholds in
// SimulationConfig to decide whether one or both entities are destroyed.
double computeRelativeEnergy(
    double massA, const shared::Vec3& velA,
    double massB, const shared::Vec3& velB) noexcept
{
    const double reducedMass  = (massA * massB) / (massA + massB);
    const shared::Vec3 relativeVelocity = subtract(velA, velB);
    const double relativeSpeedSquared   = dot(relativeVelocity, relativeVelocity);
    return 0.5 * reducedMass * relativeSpeedSquared;
}

// Perfectly inelastic centre-of-mass velocity (coefficient of restitution e = 0).
//
// After a perfectly inelastic collision both objects move at the same
// velocity — the momentum-weighted average:
//   v_cm = (mA * vA + mB * vB) / (mA + mB)
//
// The survivor of a partially-destructive hit is assigned v_cm so that
// momentum is conserved while all relative kinetic energy is absorbed.
shared::Vec3 computeCentreOfMassVelocity(
    double massA, const shared::Vec3& velA,
    double massB, const shared::Vec3& velB) noexcept
{
    const double totalMass = massA + massB;
    return {
        (massA * velA.x + massB * velB.x) / totalMass,
        (massA * velA.y + massB * velB.y) / totalMass,
        (massA * velA.z + massB * velB.z) / totalMass,
    };
}

// A pending collision to resolve, sorted before processing.
//
// Sorted by (toi, pairIdLow, pairIdHigh) to produce deterministic resolution
// order. pairIdLow/pairIdHigh are NOT a range of entity ids — they are the
// smaller and larger NetId of the two participants in this specific collision
// pair, used purely as a tie-breaking key when multiple collisions share the
// same TOI.
struct PendingHit
{
    double          toi       {};
    shared::NetId   pairIdLow  {};  // min(netIdA, netIdB) — deterministic sort key
    shared::NetId   pairIdHigh {};  // max(netIdA, netIdB) — deterministic sort key
    std::size_t     idxA      {};   // index into SmallSnapshot array
    std::size_t     idxB      {};   // index into SmallSnapshot array OR massive body array
    bool            bIsMassive {};   // true → idxB is a massive body index
};

// Immutable kinematic snapshot of one small entity for one CCD interval.
// No raw pointers; velocity write-back uses a NetId→Vec3 map.
struct SmallSnapshot
{
    shared::NetId      netId    {};
    shared::EntityKind kind     {};
    shared::Vec3       posStart {};
    shared::Vec3       velStart {};
    double             radius   {};
    double             massKg   {};
};

// Resolve all sorted hits in ascending (toi, pairIdLow, pairIdHigh) order.
//
// Each hit involves exactly two entities. Processing in sorted order ensures
// deterministic outcomes regardless of broad-phase iteration order. If an
// entity was already destroyed by an earlier hit in this same interval, any
// later hit involving that entity is skipped.
//
// Fills `destroyed` with all netIds that should be removed and
// `vcmWriteBack` with the survivor velocities to apply before erasure.
void resolveHits(
    std::span<const PendingHit>         hits,
    std::span<const SmallSnapshot>      snaps,
    std::span<const MassiveBodyState>   massiveBodies,
    const SimulationConfig&             config,
    std::unordered_set<shared::NetId>&  destroyed,
    std::unordered_map<shared::NetId, shared::Vec3>& vcmWriteBack,
    std::vector<CollisionEvent>&        outEvents)
{
    for (const auto& hit : hits)
    {
        // Skip if either participant was already destroyed by an earlier hit this interval.
        if (destroyed.count(hit.pairIdLow) || destroyed.count(hit.pairIdHigh))
            continue;

        if (hit.bIsMassive)
        {
            // Small-vs-massive: always despawn the small object.
            const auto& s    = snaps[hit.idxA];
            const auto& body = massiveBodies[hit.idxB];

            // For small-vs-massive, reduced mass mu_r ≈ m_small (since M_body >> m_small).
            // E_rel = 0.5 * m_small * |v_small - v_body|^2
            const shared::Vec3 relativeVelocity = subtract(s.velStart, body.velocity.linear);
            const double       relativeSpeedSq  = dot(relativeVelocity, relativeVelocity);
            const double       impactEnergy = 0.5 * s.massKg * relativeSpeedSq;

            const bool smallIsA = (s.netId < body.definition.netId);
            CollisionEvent ev;
            ev.toi        = hit.toi;
            ev.eRelJoules = impactEnergy;
            ev.outcome    = CollisionOutcome::SmallDespawned;
            if (smallIsA)
            {
                ev.netIdA = s.netId;                ev.kindA = s.kind;
                ev.netIdB = body.definition.netId;  ev.kindB = shared::EntityKind::MassiveBody;
            }
            else
            {
                ev.netIdA = body.definition.netId;  ev.kindA = shared::EntityKind::MassiveBody;
                ev.netIdB = s.netId;                ev.kindB = s.kind;
            }
            outEvents.push_back(ev);
            destroyed.insert(s.netId);
        }
        else
        {
            // Small-vs-small: canonically order so A has the lower netId.
            // The lower-netId entity is the deterministic tie-break survivor
            // when exactly one entity must be despawned.
            const auto& rawA = snaps[hit.idxA];
            const auto& rawB = snaps[hit.idxB];
            const SmallSnapshot* sA = (rawA.netId < rawB.netId) ? &rawA : &rawB;
            const SmallSnapshot* sB = (rawA.netId < rawB.netId) ? &rawB : &rawA;

            const double       massA   = sA->massKg;
            const double       massB   = sB->massKg;
            const double       impactEnergy    = computeRelativeEnergy(massA, sA->velStart, massB, sB->velStart);
            const shared::Vec3 survivorVelocity = computeCentreOfMassVelocity(massA, sA->velStart, massB, sB->velStart);

            CollisionEvent ev;
            ev.toi        = hit.toi;
            ev.eRelJoules = impactEnergy;
            ev.netIdA     = sA->netId;  ev.kindA = sA->kind;
            ev.netIdB     = sB->netId;  ev.kindB = sB->kind;

            // --- Projectile vs Projectile ---
            if (sA->kind == shared::EntityKind::Projectile &&
                sB->kind == shared::EntityKind::Projectile)
            {
                if (impactEnergy > config.projectileProjectileDestroyEnergyJoules)
                {
                    // High energy: both projectiles destroyed.
                    ev.outcome = CollisionOutcome::BothDespawned;
                    destroyed.insert(sA->netId);
                    destroyed.insert(sB->netId);
                }
                else
                {
                    // Low energy: B (higher netId) destroyed; A survives
                    // with the perfectly inelastic centre-of-mass velocity.
                    ev.outcome = CollisionOutcome::BDespawned;
                    destroyed.insert(sB->netId);
                    vcmWriteBack[sA->netId] = survivorVelocity;
                }
            }
            // --- Ship vs Projectile ---
            else if ((sA->kind == shared::EntityKind::Ship      && sB->kind == shared::EntityKind::Projectile) ||
                     (sA->kind == shared::EntityKind::Projectile && sB->kind == shared::EntityKind::Ship))
            {
                const SmallSnapshot* ship = (sA->kind == shared::EntityKind::Ship) ? sA : sB;
                const SmallSnapshot* proj = (sA->kind == shared::EntityKind::Ship) ? sB : sA;
                const bool projIsB = (sB == proj);

                // Projectile is always destroyed on ship impact.
                destroyed.insert(proj->netId);

                if (impactEnergy > config.shipProjectileDestroyShipEnergyJoules)
                {
                    // High energy: ship is also destroyed.
                    ev.outcome = CollisionOutcome::BothDespawned;
                    destroyed.insert(ship->netId);
                }
                else
                {
                    // Low energy: ship survives with centre-of-mass velocity.
                    ev.outcome = projIsB ? CollisionOutcome::BDespawned
                                         : CollisionOutcome::ADespawned;
                    vcmWriteBack[ship->netId] = survivorVelocity;
                }
            }
            // --- Ship vs Ship ---
            else
            {
                if (impactEnergy > config.shipShipDestroyBothEnergyJoules)
                {
                    // High energy: both ships destroyed.
                    ev.outcome = CollisionOutcome::BothDespawned;
                    destroyed.insert(sA->netId);
                    destroyed.insert(sB->netId);
                }
                else
                {
                    // Low energy: B (higher netId) destroyed; A survives
                    // with the perfectly inelastic centre-of-mass velocity.
                    ev.outcome = CollisionOutcome::BDespawned;
                    destroyed.insert(sB->netId);
                    vcmWriteBack[sA->netId] = survivorVelocity;
                }
            }

            outEvents.push_back(ev);
        }
    }
}

} // anonymous namespace

// ---------------------------------------------------------------------------
// detectAndResolve
// ---------------------------------------------------------------------------

void CollisionSystem::detectAndResolve(
    std::vector<ShipState>&                ships,
    std::vector<ProjectileState>&          projectiles,
    std::span<const MassiveBodyState>      massiveBodies,
    std::vector<CollisionEvent>&           outEvents,
    const SimulationConfig&                config,
    double                                 dt) const
{
    const std::size_t nSmall = ships.size() + projectiles.size();
    if (nSmall == 0)
        return;

    // ---- 1. Build SmallSnapshot list ----------------------------------------
    std::vector<SmallSnapshot> snaps;
    snaps.reserve(nSmall);
    for (const auto& ship : ships)
    {
        snaps.push_back({ship.netId, shared::EntityKind::Ship,
                         ship.transform.position, ship.velocity.linear,
                         ship.collider.radiusMeters, ship.massProperties.massKg});
    }
    for (const auto& proj : projectiles)
    {
        snaps.push_back({proj.netId, shared::EntityKind::Projectile,
                         proj.transform.position, proj.velocity.linear,
                         proj.collider.radiusMeters, proj.massProperties.massKg});
    }

    // ---- 2. Build IntervalSnapshots for broad phase (linear end-pos approx) --
    std::vector<IntervalSnapshot> bpSnaps;
    bpSnaps.reserve(nSmall);
    for (const auto& s : snaps)
    {
        IntervalSnapshot is;
        is.netId    = s.netId;
        is.kind     = s.kind;
        is.posStart = s.posStart;
        is.velStart = s.velStart;
        is.posEnd   = add(s.posStart, scale(s.velStart, dt));  // linear approx; see header note
        is.velEnd   = s.velStart;
        is.radius   = s.radius;
        is.massKg   = s.massKg;
        bpSnaps.push_back(is);
    }

    // ---- 3. Broad phase (small-vs-small) ------------------------------------
    const auto candidates = broadPhaseSmallObjects(bpSnaps);

    // ---- 4. Narrow phase: collect all hits ----------------------------------
    std::vector<PendingHit> hits;

    for (const auto& [idxA, idxB] : candidates)
    {
        const auto& sA = snaps[idxA];
        const auto& sB = snaps[idxB];
        const auto  toi = sphereSweepTOI(
            sA.posStart, sA.velStart, sB.posStart, sB.velStart,
            sA.radius, sB.radius, dt);
        if (toi.has_value())
            hits.push_back({*toi, std::min(sA.netId, sB.netId),
                            std::max(sA.netId, sB.netId), idxA, idxB, false});
    }

    for (std::size_t i = 0; i < nSmall; ++i)
    {
        const auto& s = snaps[i];
        for (std::size_t m = 0; m < massiveBodies.size(); ++m)
        {
            const auto& body = massiveBodies[m];
            const auto  toi  = sphereSweepTOI(
                s.posStart, s.velStart,
                body.transform.position, body.velocity.linear,
                s.radius, body.definition.radiusMeters, dt);
            if (toi.has_value())
                hits.push_back({*toi, std::min(s.netId, body.definition.netId),
                                std::max(s.netId, body.definition.netId), i, m, true});
        }
    }

    // ---- 5. Deterministic sort: (toi, pairIdLow, pairIdHigh) ascending ------
    // Ensures identical resolution order across runs and platforms.
    // pairIdLow/pairIdHigh are the smaller/larger NetId of the two collision
    // participants — they break ties when multiple pairs collide at the same TOI.
    std::sort(hits.begin(), hits.end(), [](const PendingHit& a, const PendingHit& b) {
        if (a.toi        != b.toi)        return a.toi        < b.toi;
        if (a.pairIdLow  != b.pairIdLow)  return a.pairIdLow  < b.pairIdLow;
        return a.pairIdHigh < b.pairIdHigh;
    });

    // ---- 6. Resolve hits ----------------------------------------------------
    std::unordered_set<shared::NetId>              destroyed;
    std::unordered_map<shared::NetId, shared::Vec3> vcmWriteBack;

    resolveHits(hits, snaps, massiveBodies, config, destroyed, vcmWriteBack, outEvents);

    // ---- 7. Apply survivor velocities, then remove destroyed entities -------
    for (auto& ship : ships)
        if (auto it = vcmWriteBack.find(ship.netId); it != vcmWriteBack.end())
            ship.velocity.linear = it->second;
    for (auto& proj : projectiles)
        if (auto it = vcmWriteBack.find(proj.netId); it != vcmWriteBack.end())
            proj.velocity.linear = it->second;

    std::erase_if(ships,       [&](const ShipState& s)      { return destroyed.count(s.netId) > 0; });
    std::erase_if(projectiles, [&](const ProjectileState& p) { return destroyed.count(p.netId) > 0; });
}

} // namespace spaceship::server

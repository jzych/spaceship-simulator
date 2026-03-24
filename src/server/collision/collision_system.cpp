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
    double dt) const
{
    for (auto& projectile : projectiles)
        projectile.params.ttlSeconds -= dt;
}

void CollisionSystem::update(std::vector<ProjectileState>& projectiles) const
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
double computeRelativeEnergy(
    double mA, const shared::Vec3& vA,
    double mB, const shared::Vec3& vB) noexcept
{
    const double mu_r = (mA * mB) / (mA + mB);
    const shared::Vec3 dv = subtract(vA, vB);
    return 0.5 * mu_r * dot(dv, dv);
}

// Perfectly inelastic centre-of-mass velocity.
shared::Vec3 computeVcm(
    double mA, const shared::Vec3& vA,
    double mB, const shared::Vec3& vB) noexcept
{
    const double total = mA + mB;
    return {
        (mA * vA.x + mB * vB.x) / total,
        (mA * vA.y + mB * vB.y) / total,
        (mA * vA.z + mB * vB.z) / total,
    };
}

// A pending collision to resolve, sorted before processing.
struct PendingHit
{
    double          toi       {};
    shared::NetId   netIdMin  {};  // sort key (smaller netId)
    shared::NetId   netIdMax  {};  // sort key (larger netId)
    std::size_t     idxA      {};  // index into SmallSnapshot array
    std::size_t     idxB      {};  // index into SmallSnapshot array OR massive body array
    bool            bIsMassive {}; // true → idxB is a massive body index
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

// Resolve all sorted hits in (toi, minNetId, maxNetId) order.
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
        if (destroyed.count(hit.netIdMin) || destroyed.count(hit.netIdMax))
            continue;

        if (hit.bIsMassive)
        {
            // Small-vs-massive: always despawn the small object.
            const auto& s    = snaps[hit.idxA];
            const auto& body = massiveBodies[hit.idxB];

            // Approximate E_rel: use small object mass as mu_r (body mass >> small mass)
            const shared::Vec3 dv  = subtract(s.velStart, body.velocity.linear);
            const double       eRel = 0.5 * s.massKg * dot(dv, dv);

            const bool smallIsA = (s.netId < body.definition.netId);
            CollisionEvent ev;
            ev.toi        = hit.toi;
            ev.eRelJoules = eRel;
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
            // Small-vs-small: A = lower netId, B = higher netId.
            const auto& rawA = snaps[hit.idxA];
            const auto& rawB = snaps[hit.idxB];
            const SmallSnapshot* sA = (rawA.netId < rawB.netId) ? &rawA : &rawB;
            const SmallSnapshot* sB = (rawA.netId < rawB.netId) ? &rawB : &rawA;

            const double       mA   = sA->massKg;
            const double       mB   = sB->massKg;
            const double       eRel = computeRelativeEnergy(mA, sA->velStart, mB, sB->velStart);
            const shared::Vec3 vcm  = computeVcm(mA, sA->velStart, mB, sB->velStart);

            CollisionEvent ev;
            ev.toi        = hit.toi;
            ev.eRelJoules = eRel;
            ev.netIdA     = sA->netId;  ev.kindA = sA->kind;
            ev.netIdB     = sB->netId;  ev.kindB = sB->kind;

            if (sA->kind == shared::EntityKind::Projectile &&
                sB->kind == shared::EntityKind::Projectile)
            {
                if (eRel > config.projectileProjectileDestroyEnergyJoules)
                {
                    ev.outcome = CollisionOutcome::BothDespawned;
                    destroyed.insert(sA->netId);
                    destroyed.insert(sB->netId);
                }
                else
                {
                    // Despawn B (higher netId); A survives with v_cm
                    ev.outcome = CollisionOutcome::BDespawned;
                    destroyed.insert(sB->netId);
                    vcmWriteBack[sA->netId] = vcm;
                }
            }
            else if ((sA->kind == shared::EntityKind::Ship      && sB->kind == shared::EntityKind::Projectile) ||
                     (sA->kind == shared::EntityKind::Projectile && sB->kind == shared::EntityKind::Ship))
            {
                const SmallSnapshot* ship = (sA->kind == shared::EntityKind::Ship) ? sA : sB;
                const SmallSnapshot* proj = (sA->kind == shared::EntityKind::Ship) ? sB : sA;
                const bool projIsB = (sB == proj);

                destroyed.insert(proj->netId);

                if (eRel > config.shipProjectileDestroyShipEnergyJoules)
                {
                    ev.outcome = CollisionOutcome::BothDespawned;
                    destroyed.insert(ship->netId);
                }
                else
                {
                    ev.outcome = projIsB ? CollisionOutcome::BDespawned
                                         : CollisionOutcome::ADespawned;
                    vcmWriteBack[ship->netId] = vcm;
                }
            }
            else
            {
                // Ship vs ship
                if (eRel > config.shipShipDestroyBothEnergyJoules)
                {
                    ev.outcome = CollisionOutcome::BothDespawned;
                    destroyed.insert(sA->netId);
                    destroyed.insert(sB->netId);
                }
                else
                {
                    // Despawn B (higher netId); A survives with v_cm
                    ev.outcome = CollisionOutcome::BDespawned;
                    destroyed.insert(sB->netId);
                    vcmWriteBack[sA->netId] = vcm;
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

    // ---- 5. Sort by (toi, minNetId, maxNetId) for deterministic resolution --
    std::sort(hits.begin(), hits.end(), [](const PendingHit& a, const PendingHit& b) {
        if (a.toi      != b.toi)      return a.toi      < b.toi;
        if (a.netIdMin != b.netIdMin) return a.netIdMin < b.netIdMin;
        return a.netIdMax < b.netIdMax;
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

#pragma once

// Shared types for the CCD collision pipeline.
// CollisionEvent carries structured outcome data for each resolved hit.
// IntervalSnapshot captures start-of-interval kinematic state for CCD.

#include "shared/sim_types.hpp"

namespace spaceship::server
{

// Which despawn outcome resulted from a collision.
enum class CollisionOutcome : std::uint8_t
{
    BothDespawned,      // both participants destroyed
    ADespawned,         // only entity A destroyed; B survives with v_cm
    BDespawned,         // only entity B destroyed; A survives with v_cm
    SmallDespawned,     // small vs massive: small object destroyed
};

// Structured record of one resolved collision, emitted per hit.
struct CollisionEvent
{
    shared::NetId         netIdA       {};
    shared::NetId         netIdB       {};
    shared::EntityKind    kindA        {};
    shared::EntityKind    kindB        {};
    double                toi          {};  // time-of-impact within interval [0, dt]
    double                eRelJoules   {};  // reduced-mass relative kinetic energy (J)
    CollisionOutcome      outcome      {};
};

// Per-entity state snapshot bracketing one integration interval.
// posStart/velStart: state at t=0 (before integration).
// posEnd/velEnd:     state at t=dt (after integration).
struct IntervalSnapshot
{
    shared::NetId      netId    {};
    shared::EntityKind kind     {};
    shared::Vec3       posStart {};
    shared::Vec3       velStart {};
    shared::Vec3       posEnd   {};
    shared::Vec3       velEnd   {};
    double             radius   {};
    double             massKg   {};
};

} // namespace spaceship::server

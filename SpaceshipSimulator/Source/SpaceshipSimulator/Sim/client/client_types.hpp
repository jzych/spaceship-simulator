#pragma once

// Output types for client-side snapshot interpolation.
// These mirror the server snapshot structs but are decoupled from server
// headers so the renderer never depends on server internals.

#include "shared/sim_types.hpp"

#include <vector>

namespace spaceship::client
{

struct InterpolatedMassiveBodyState
{
    shared::NetId netId        {};
    shared::Vec3  position     {};   // metres, SI
    shared::Vec3  velocity     {};   // m/s,    SI
    double        radiusMeters {};
};

struct InterpolatedShipState
{
    shared::NetId      netId        {};
    shared::Vec3       position     {};
    shared::Vec3       velocity     {};
    shared::Quaternion orientation  {};
    double             radiusMeters {};
    double             throttle     {};   // [0, 1]
};

struct InterpolatedProjectileState
{
    shared::NetId netId        {};
    shared::Vec3  position     {};
    shared::Vec3  velocity     {};
    double        radiusMeters {};
    shared::NetId ownerNetId   {};
};

// Fully interpolated world state ready for the renderer.
// CollisionEvents are discrete and non-interpolable — handled separately.
struct InterpolatedWorldState
{
    double renderTime {};

    std::vector<InterpolatedMassiveBodyState> massiveBodies {};
    std::vector<InterpolatedShipState>        ships         {};
    std::vector<InterpolatedProjectileState>  projectiles   {};
};

// Default interpolation delay used to compute render time from current clock:
//   renderServerTime = now + clockOffset - kDefaultInterpolationDelaySeconds
// See game_compedium.md § "Tick rates and time model".
constexpr double kDefaultInterpolationDelaySeconds = 0.1;

} // namespace spaceship::client

#pragma once

// Derived orbit telemetry for a ship, computed from canonical integrator state.
// This is NOT authoritative simulation state — it is cached for rendering and UI.

#include "shared/sim_types.hpp"

namespace spaceship::server
{

struct OrbitCache
{
    // Reference body
    shared::NetId referenceBodyId {};
    shared::Tick epoch {};
    double qualityScore {};

    // Relative state at epoch
    shared::Vec3 relativePosition {};
    shared::Vec3 relativeVelocity {};

    // Ellipse geometry (valid only when isElliptic == true)
    bool isElliptic {};
    double semiMajorAxis {};
    double semiMinorAxis {};
    double eccentricity {};
    double periapsisRadius {};
    double apoapsisRadius {};
    shared::Vec3 orbitNormal {};
    shared::Vec3 periapsisDirection {};
    shared::Vec3 sideDirection {};
    shared::Vec3 ellipseCenter {};

    // Phase at epoch
    double trueAnomaly {};
    double eccentricAnomaly {};
    double meanAnomaly {};

    // Geographic telemetry (body-fixed frame) — computed by Phase 9d
    double altitudeMeters {};
    double longitudeRadians {};
    double latitudeRadians {};

    // Cache validity
    shared::Tick validUntilTick {};
    bool dirty {};
    bool wasThrusting {};
};

} // namespace spaceship::server

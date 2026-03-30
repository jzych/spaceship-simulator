#include "server/simulation/gravity/orbit_fitting.hpp"
#include "server/simulation/simulation_math.hpp"

#include <algorithm>
#include <cmath>

namespace spaceship::server
{

namespace
{

constexpr double kAngularMomentumEpsilon = 1e-10;
constexpr double kNearCircularEccentricityThreshold = 1e-6;
constexpr double kQualityScaleEta = 10.0;

} // namespace

OrbitCache fitOrbit(
    const shared::Vec3& relativePosition,
    const shared::Vec3& relativeVelocity,
    double mu,
    double bodyRadius,
    shared::NetId referenceBodyId,
    shared::Tick currentTick)
{
    OrbitCache cache {};
    cache.referenceBodyId = referenceBodyId;
    cache.epoch = currentTick;
    cache.relativePosition = relativePosition;
    cache.relativeVelocity = relativeVelocity;

    const double distanceFromCenter = length(relativePosition);
    cache.altitudeMeters = distanceFromCenter - bodyRadius;

    // Specific angular momentum vector: h = r x v
    // Its magnitude determines whether the orbit is degenerate (radial/stationary).
    const shared::Vec3 angularMomentum = cross(relativePosition, relativeVelocity);
    const double angularMomentumMag = length(angularMomentum);

    if (angularMomentumMag < kAngularMomentumEpsilon)
        return cache;  // Degenerate: radial trajectory or zero velocity

    const shared::Vec3 angularMomentumDir = scale(angularMomentum, 1.0 / angularMomentumMag);

    // Specific orbital energy (vis-viva per unit mass):
    //   epsilon = 0.5 * |v|^2 - mu / |r|
    // Negative energy → bound (elliptical) orbit.
    // Zero or positive → parabolic/hyperbolic escape trajectory.
    const double speedSquared = lengthSquared(relativeVelocity);
    const double specificEnergy = 0.5 * speedSquared - mu / distanceFromCenter;

    if (specificEnergy >= 0.0)
        return cache;  // Unbound orbit — not elliptical

    // Eccentricity vector: points from focus toward periapsis.
    //   e_vec = (v x h) / mu  -  r_hat
    // Its magnitude is the orbital eccentricity (0 = circle, 0<e<1 = ellipse).
    const shared::Vec3 eccentricityVec = subtract(
        scale(cross(relativeVelocity, angularMomentum), 1.0 / mu),
        scale(relativePosition, 1.0 / distanceFromCenter));
    const double eccentricity = length(eccentricityVec);

    if (eccentricity >= 1.0)
        return cache;  // Not elliptical (parabolic/hyperbolic)

    // --- Elliptical orbit confirmed ---
    cache.isElliptic = true;
    cache.eccentricity = eccentricity;

    // Semi-major axis from the vis-viva relation:
    //   a = -mu / (2 * epsilon)
    // Negative energy gives positive semi-major axis.
    const double semiMajorAxis = -mu / (2.0 * specificEnergy);
    cache.semiMajorAxis = semiMajorAxis;

    // Semi-minor axis:  b = a * sqrt(1 - e^2)
    cache.semiMinorAxis = semiMajorAxis * std::sqrt(1.0 - eccentricity * eccentricity);

    // Periapsis (closest approach) and apoapsis (farthest point):
    //   r_peri = a * (1 - e)
    //   r_apo  = a * (1 + e)
    cache.periapsisRadius = semiMajorAxis * (1.0 - eccentricity);
    cache.apoapsisRadius = semiMajorAxis * (1.0 + eccentricity);

    cache.orbitNormal = angularMomentumDir;

    // In-plane basis vectors: periapsisDir (toward periapsis) and sideDir (90 deg ahead).
    shared::Vec3 periapsisDir;
    if (eccentricity > kNearCircularEccentricityThreshold)
    {
        periapsisDir = normalize(eccentricityVec);
    }
    else
    {
        // Near-circular fallback: project position onto orbital plane as reference direction.
        const shared::Vec3 projected = projectOntoPlane(relativePosition, angularMomentumDir);
        if (length(projected) < kAngularMomentumEpsilon)
            return cache;  // Degenerate: position lies along orbit normal
        periapsisDir = normalize(projected);
    }
    const shared::Vec3 sideDir = cross(angularMomentumDir, periapsisDir);

    cache.periapsisDirection = periapsisDir;
    cache.sideDirection = sideDir;

    // Ellipse geometric center offset from the focus (body position):
    //   center = -a * e * periapsisDir
    cache.ellipseCenter = scale(periapsisDir, -semiMajorAxis * eccentricity);

    // True anomaly (nu): angle from periapsis to current position, measured at focus.
    // For eccentric orbits, compute from eccentricity vector; for near-circular, from periapsisDir.
    if (eccentricity > kNearCircularEccentricityThreshold)
    {
        const double denominator = eccentricity * distanceFromCenter;
        const double cosNu = dot(eccentricityVec, relativePosition) / denominator;
        const double sinNu = dot(angularMomentumDir, cross(eccentricityVec, relativePosition)) / denominator;
        cache.trueAnomaly = std::atan2(sinNu, cosNu);
    }
    else
    {
        // Near-circular: true anomaly from position relative to periapsisDir.
        const double cosNu = dot(periapsisDir, relativePosition) / distanceFromCenter;
        const double sinNu = dot(sideDir, relativePosition) / distanceFromCenter;
        cache.trueAnomaly = std::atan2(sinNu, cosNu);
    }

    // Eccentric anomaly (E): auxiliary angle on the circumscribed circle.
    //   E = atan2(sqrt(1 - e^2) * sin(nu),  e + cos(nu))
    const double sinTrueAnomaly = std::sin(cache.trueAnomaly);
    const double cosTrueAnomaly = std::cos(cache.trueAnomaly);
    cache.eccentricAnomaly = std::atan2(
        std::sqrt(1.0 - eccentricity * eccentricity) * sinTrueAnomaly,
        eccentricity + cosTrueAnomaly);

    // Mean anomaly (M): uniformly-advancing angle proportional to elapsed time.
    //   M = E - e * sin(E)    (Kepler's equation)
    cache.meanAnomaly = cache.eccentricAnomaly - eccentricity * std::sin(cache.eccentricAnomaly);

    return cache;
}

double computeQualityScore(
    const shared::Vec3& totalAcceleration,
    const shared::Vec3& relativePosition,
    double mu)
{
    const double distanceFromCenter = length(relativePosition);
    constexpr double kMinRadius = 1.0;
    if (distanceFromCenter < kMinRadius)
        return 0.0;

    // Two-body gravitational acceleration: a_2body = -mu * r_hat / |r|^2
    // Written as: a_2body = r * (-mu / |r|^3) to reuse the position vector direction.
    const double inverseCubeDistance = -mu / (distanceFromCenter * distanceFromCenter * distanceFromCenter);
    const shared::Vec3 twoBodyAccel = scale(relativePosition, inverseCubeDistance);
    const double twoBodyAccelMag = length(twoBodyAccel);

    // Perturbation acceleration: everything beyond the dominant two-body term.
    //   a_pert = a_total - a_2body
    const shared::Vec3 perturbationAccel = subtract(totalAcceleration, twoBodyAccel);
    const double perturbationAccelMag = length(perturbationAccel);

    // Quality score: 1.0 when perturbation is negligible, approaching 0.0 when
    // perturbation dominates. Uses a Lorentzian decay:
    //   Q = 1 / (1 + k * eta)   where eta = |a_pert| / |a_2body|
    constexpr double kEpsilon = 1e-15;
    const double perturbationRatio = perturbationAccelMag / std::max(twoBodyAccelMag, kEpsilon);

    return std::clamp(1.0 / (1.0 + kQualityScaleEta * perturbationRatio), 0.0, 1.0);
}

} // namespace spaceship::server

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

    const double r = length(relativePosition);
    cache.altitudeMeters = r - bodyRadius;

    // Specific angular momentum: h = r × v
    const shared::Vec3 h = cross(relativePosition, relativeVelocity);
    const double hMag = length(h);

    // Degenerate: zero angular momentum (radial trajectory or zero velocity)
    if (hMag < kAngularMomentumEpsilon)
        return cache;

    const shared::Vec3 hHat = scale(h, 1.0 / hMag);

    // Specific orbital energy: ε = 0.5 * |v|² - μ / |r|
    const double vSq = lengthSquared(relativeVelocity);
    const double epsilon = 0.5 * vSq - mu / r;

    // Must be bound (negative energy) for an ellipse
    if (epsilon >= 0.0)
        return cache;

    // Eccentricity vector: e_vec = (v × h) / μ - r̂
    const shared::Vec3 eVec = subtract(
        scale(cross(relativeVelocity, h), 1.0 / mu),
        scale(relativePosition, 1.0 / r));
    const double e = length(eVec);

    if (e >= 1.0)
        return cache;

    // Elliptical orbit confirmed
    cache.isElliptic = true;
    cache.eccentricity = e;

    // Semi-major axis: a = -μ / (2ε)
    const double a = -mu / (2.0 * epsilon);
    cache.semiMajorAxis = a;
    cache.semiMinorAxis = a * std::sqrt(1.0 - e * e);
    cache.periapsisRadius = a * (1.0 - e);
    cache.apoapsisRadius = a * (1.0 + e);

    cache.orbitNormal = hHat;

    // In-plane basis vectors
    shared::Vec3 pHat;
    if (e > kNearCircularEccentricityThreshold)
    {
        pHat = normalize(eVec);
    }
    else
    {
        // Near-circular fallback: project position onto orbital plane
        const shared::Vec3 projected = projectOntoPlane(relativePosition, hHat);
        if (length(projected) < kAngularMomentumEpsilon)
            return cache; // degenerate: position lies along orbit normal
        pHat = normalize(projected);
    }
    const shared::Vec3 qHat = cross(hHat, pHat);

    cache.periapsisDirection = pHat;
    cache.sideDirection = qHat;

    // Ellipse center: c = -a * e * p_hat
    cache.ellipseCenter = scale(pHat, -a * e);

    // True anomaly
    if (e > kNearCircularEccentricityThreshold)
    {
        const double cosNu = dot(eVec, relativePosition) / (e * r);
        const double sinNu = dot(hHat, cross(eVec, relativePosition)) / (e * r);
        cache.trueAnomaly = std::atan2(sinNu, cosNu);
    }
    else
    {
        // Near-circular: true anomaly from position relative to p_hat
        const double cosNu = dot(pHat, relativePosition) / r;
        const double sinNu = dot(qHat, relativePosition) / r;
        cache.trueAnomaly = std::atan2(sinNu, cosNu);
    }

    // Eccentric anomaly: E = atan2(sqrt(1-e²)*sin(ν), e + cos(ν))
    const double sinNu = std::sin(cache.trueAnomaly);
    const double cosNu = std::cos(cache.trueAnomaly);
    cache.eccentricAnomaly = std::atan2(
        std::sqrt(1.0 - e * e) * sinNu,
        e + cosNu);

    // Mean anomaly: M = E - e * sin(E)
    cache.meanAnomaly = cache.eccentricAnomaly - e * std::sin(cache.eccentricAnomaly);

    return cache;
}

double computeQualityScore(
    const shared::Vec3& totalAcceleration,
    const shared::Vec3& relativePosition,
    double mu)
{
    const double r = length(relativePosition);
    constexpr double kMinRadius = 1.0;
    if (r < kMinRadius)
        return 0.0;

    // Two-body acceleration: a_2b = -μ * r / |r|³
    const shared::Vec3 a2b = scale(relativePosition, -mu / (r * r * r));
    const double a2bMag = length(a2b);

    // Perturbation: a_pert = a_total - a_2b
    const shared::Vec3 aPert = subtract(totalAcceleration, a2b);
    const double aPertMag = length(aPert);

    constexpr double kEpsilon = 1e-15;
    const double eta = aPertMag / std::max(a2bMag, kEpsilon);

    return std::clamp(1.0 / (1.0 + kQualityScaleEta * eta), 0.0, 1.0);
}

} // namespace spaceship::server

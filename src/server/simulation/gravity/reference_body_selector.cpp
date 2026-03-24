#include "server/simulation/gravity/reference_body_selector.hpp"
#include "server/simulation/simulation_math.hpp"
#include "server/simulation/simulation_world.hpp"

#include <algorithm>
#include <array>

namespace spaceship::server
{

ReferenceBodySelection ReferenceBodySelector::update(
    const shared::Vec3& shipPosition,
    std::span<const MassiveBodyState> massiveBodies,
    const SimulationConfig& config,
    double dt)
{
    if (massiveBodies.empty())
        return {};

    // Score each body by gravitational acceleration magnitude at the ship's position.
    // Uses |a| = mu / d^2 (inverse-square law without direction).
    struct BodyScore
    {
        shared::NetId netId {};
        double accelMag {};
        double score {};
    };

    constexpr std::size_t kMaxBodies = 8;
    const auto bodyCount = std::min(massiveBodies.size(), kMaxBodies);

    double totalAccelMag = 0.0;
    std::array<BodyScore, kMaxBodies> scores {};

    for (std::size_t i = 0; i < bodyCount; ++i)
    {
        const auto& body = massiveBodies[i];
        const shared::Vec3 toBody = subtract(body.transform.position, shipPosition);
        const double distance = length(toBody);
        constexpr double kMinDist = 1.0;

        // Gravitational acceleration magnitude: |a| = mu / d^2
        const double accelMag = (distance < kMinDist)
            ? 0.0
            : body.definition.muMetersCubedPerSecondSquared / (distance * distance);

        scores[i] = {body.definition.netId, accelMag, 0.0};
        totalAccelMag += accelMag;
    }

    // Normalize scores to [0, 1] — fraction of total gravitational influence.
    constexpr double kEps = 1e-30;
    for (std::size_t i = 0; i < bodyCount; ++i)
        scores[i].score = scores[i].accelMag / (totalAccelMag + kEps);

    const auto scoresBegin = scores.begin();
    const auto scoresEnd = scoresBegin + static_cast<std::ptrdiff_t>(bodyCount);

    // First call: pick the highest score
    if (!initialized_)
    {
        const auto best = std::max_element(
            scoresBegin, scoresEnd,
            [](const BodyScore& a, const BodyScore& b) { return a.score < b.score; });
        currentBodyId_ = best->netId;
        initialized_ = true;
        dwellAccumulator_ = 0.0;
        return {currentBodyId_, best->score, 0.0, true};
    }

    // Find current body's score and best challenger
    double currentScore = 0.0;
    shared::NetId bestChallengerId = currentBodyId_;
    double bestChallengerScore = 0.0;

    for (auto it = scoresBegin; it != scoresEnd; ++it)
    {
        if (it->netId == currentBodyId_)
        {
            currentScore = it->score;
        }
        else if (it->score > bestChallengerScore)
        {
            bestChallengerScore = it->score;
            bestChallengerId = it->netId;
        }
    }

    // Hysteresis: challenger must exceed current by a configurable margin
    // to prevent oscillation between bodies near the sphere of influence boundary.
    if (bestChallengerScore > currentScore * (1.0 + config.referenceBodyHysteresisDelta))
    {
        dwellAccumulator_ += dt;

        if (dwellAccumulator_ >= config.referenceBodyDwellTimeSeconds)
        {
            currentBodyId_ = bestChallengerId;
            dwellAccumulator_ = 0.0;
            return {currentBodyId_, bestChallengerScore, 0.0, true};
        }

        return {currentBodyId_, currentScore, dwellAccumulator_, false};
    }

    // Advantage lost — reset dwell
    dwellAccumulator_ = 0.0;
    return {currentBodyId_, currentScore, 0.0, false};
}

} // namespace spaceship::server

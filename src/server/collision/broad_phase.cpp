#include "server/collision/broad_phase.hpp"

#include <algorithm>
#include <cmath>

namespace spaceship::server
{

namespace
{

struct Aabb
{
    double minX, minY, minZ;
    double maxX, maxY, maxZ;
};

[[nodiscard]] Aabb makeSweptAabb(const IntervalSnapshot& s) noexcept
{
    const double r = s.radius;
    return Aabb {
        std::min(s.posStart.x, s.posEnd.x) - r,
        std::min(s.posStart.y, s.posEnd.y) - r,
        std::min(s.posStart.z, s.posEnd.z) - r,
        std::max(s.posStart.x, s.posEnd.x) + r,
        std::max(s.posStart.y, s.posEnd.y) + r,
        std::max(s.posStart.z, s.posEnd.z) + r,
    };
}

[[nodiscard]] bool aabbsOverlap(const Aabb& a, const Aabb& b) noexcept
{
    return a.maxX >= b.minX && a.minX <= b.maxX
        && a.maxY >= b.minY && a.minY <= b.maxY
        && a.maxZ >= b.minZ && a.minZ <= b.maxZ;
}

} // anonymous namespace

std::vector<CandidatePair> broadPhaseSmallObjects(
    std::span<const IntervalSnapshot> snapshots) noexcept
{
    std::vector<CandidatePair> pairs;
    const auto n = snapshots.size();

    for (std::size_t i = 0; i < n; ++i)
    {
        const Aabb aabbI = makeSweptAabb(snapshots[i]);
        for (std::size_t j = i + 1; j < n; ++j)
        {
            const Aabb aabbJ = makeSweptAabb(snapshots[j]);
            if (aabbsOverlap(aabbI, aabbJ))
            {
                pairs.push_back({i, j});
            }
        }
    }

    return pairs;
}

} // namespace spaceship::server

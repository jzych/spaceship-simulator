#include "server/collision/broad_phase.hpp"

#include <algorithm>
#include <cmath>

namespace spaceship::server
{

namespace
{

// Axis-aligned bounding box in 3D.
struct Aabb
{
    double minX, minY, minZ;
    double maxX, maxY, maxZ;
};

// Build a swept AABB that encloses the sphere at both its start and end
// positions during the interval, expanded by the sphere radius on each axis.
// This is the conservative bounding volume for linear-motion CCD.
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

// Two AABBs overlap if and only if they overlap on all three axes.
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

    // O(n^2) all-pairs test. Acceptable for the PoC entity count (<~50).
    // Each pair is emitted with idxA < idxB to avoid duplicates.
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

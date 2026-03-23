#pragma once

// Swept-AABB broad phase for CCD collision detection.
// Builds a bounding box over each entity's swept path (posStart→posEnd expanded by radius)
// and returns all index pairs whose boxes overlap.
// This is an O(n²) rebuild-every-interval implementation suitable for the PoC.

#include "server/collision/collision_types.hpp"

#include <span>
#include <vector>

namespace spaceship::server
{

// Index pair (i < j) into the snapshots array whose swept AABBs overlap.
struct CandidatePair
{
    std::size_t idxA {};
    std::size_t idxB {};
};

// Returns all overlapping swept-AABB candidate pairs for narrow-phase testing.
// Massive bodies are not passed here; they are checked directly in narrow phase.
[[nodiscard]] std::vector<CandidatePair> broadPhaseSmallObjects(
    std::span<const IntervalSnapshot> snapshots) noexcept;

} // namespace spaceship::server

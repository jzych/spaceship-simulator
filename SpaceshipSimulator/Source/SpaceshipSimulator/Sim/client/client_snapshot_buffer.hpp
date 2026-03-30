#pragma once

// Time-keyed ring buffer of authoritative world snapshots for client-side
// interpolation. Snapshots must be pushed in monotonically increasing
// elapsedSeconds order; out-of-order pushes are silently discarded.
//
// interpolate(renderTime) returns a fully interpolated world state by finding
// the two bracketing snapshots [t0, t1] where t0 ≤ renderTime ≤ t1.
// Returns std::nullopt when renderTime is outside the buffered range or the
// buffer is empty.
//
// latest() is kept for backward compatibility and returns the most recently
// pushed snapshot.

#include "client/client_types.hpp"
#include "server/snapshot/snapshot_types.hpp"

#include <cstddef>
#include <optional>
#include <vector>

namespace spaceship::client
{

class ClientSnapshotBuffer
{
  public:
    explicit ClientSnapshotBuffer(std::size_t capacity = 64);

    // Push a new snapshot. Discards it if its elapsedSeconds is ≤ the
    // newest snapshot currently in the buffer (duplicate or out-of-order).
    void push(server::WorldSnapshot snapshot);

    // Return the interpolated world state at renderTime, or std::nullopt if
    // renderTime is outside [oldest.elapsedSeconds, newest.elapsedSeconds]
    // or the buffer holds no snapshots.
    [[nodiscard]] std::optional<InterpolatedWorldState> interpolate(double renderTime) const;

    // See class-level comment for rationale.
    [[nodiscard]] std::optional<server::WorldSnapshot> latest() const;

    [[nodiscard]] std::size_t size()     const;
    [[nodiscard]] bool        empty()    const;
    [[nodiscard]] std::size_t capacity() const;

  private:
    // Circular storage: ring_[head_] is the oldest, ring_[(head_ + count_ - 1) % capacity_] newest.
    std::vector<server::WorldSnapshot>   ring_;
    std::size_t                          head_     {};
    std::size_t                          count_    {};
    std::size_t                          capacity_;

    // Translate logical index [0 = oldest, count_-1 = newest] to the physical ring_ slot.
    [[nodiscard]] const server::WorldSnapshot& at(std::size_t logicalIndex) const;
};

} // namespace spaceship::client

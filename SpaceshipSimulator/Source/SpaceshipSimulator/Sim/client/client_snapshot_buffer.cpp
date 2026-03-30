#include "client/client_snapshot_buffer.hpp"
#include "client/snapshot_interpolator.hpp"

#include <cassert>
#include <utility>

namespace spaceship::client
{

ClientSnapshotBuffer::ClientSnapshotBuffer(std::size_t capacity)
    : ring_(capacity), capacity_(capacity)
{
    assert(capacity > 0 && "ClientSnapshotBuffer capacity must be > 0");
}

void ClientSnapshotBuffer::push(server::WorldSnapshot snapshot)
{
    // Discard out-of-order and duplicate timestamps
    if (count_ > 0 && snapshot.elapsedSeconds <= at(count_ - 1).elapsedSeconds)
        return;

    if (count_ < capacity_)
    {
        ring_[(head_ + count_) % capacity_] = std::move(snapshot);
        ++count_;
    }
    else
    {
        // Buffer full: overwrite the oldest slot and advance head
        ring_[head_] = std::move(snapshot);
        head_ = (head_ + 1) % capacity_;
    }
}

std::optional<InterpolatedWorldState> ClientSnapshotBuffer::interpolate(double renderTime) const
{
    if (count_ == 0)
        return std::nullopt;

    // Reject renderTime outside the buffered range
    if (renderTime < at(0).elapsedSeconds || renderTime > at(count_ - 1).elapsedSeconds)
        return std::nullopt;

    if (count_ == 1)
    {
        // Range check already guarantees renderTime == at(0).elapsedSeconds here
        return interpolator::fromSnapshot(at(0), renderTime);
    }

    // Binary search: find the largest logical index lo where at(lo).elapsedSeconds <= renderTime
    std::size_t lo = 0;
    std::size_t hi = count_ - 1;
    while (lo + 1 < hi)
    {
        const std::size_t mid = lo + (hi - lo) / 2;
        if (at(mid).elapsedSeconds <= renderTime)
            lo = mid;
        else
            hi = mid;  // mid > lo, so search space always shrinks
    }

    // Loop exits with at(lo).elapsedSeconds <= renderTime < at(hi).elapsedSeconds
    return interpolator::interpolateWorldState(at(lo), at(hi), renderTime);
}

std::optional<server::WorldSnapshot> ClientSnapshotBuffer::latest() const
{
    if (count_ == 0)
        return std::nullopt;
    return at(count_ - 1);
}

std::size_t ClientSnapshotBuffer::size() const
{
    return count_;
}

bool ClientSnapshotBuffer::empty() const
{
    return count_ == 0;
}

std::size_t ClientSnapshotBuffer::capacity() const
{
    return capacity_;
}

const server::WorldSnapshot& ClientSnapshotBuffer::at(std::size_t logicalIndex) const
{
    assert(logicalIndex < count_);
    return ring_[(head_ + logicalIndex) % capacity_];
}

} // namespace spaceship::client

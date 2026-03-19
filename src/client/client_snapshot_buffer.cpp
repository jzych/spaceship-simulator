#include "client/client_snapshot_buffer.hpp"

#include <utility>

namespace spaceship::client
{

void ClientSnapshotBuffer::push(SnapshotFrame frame)
{
    latest_ = std::move(frame);
}

std::optional<SnapshotFrame> ClientSnapshotBuffer::latest() const
{
    return latest_;
}

} // namespace spaceship::client

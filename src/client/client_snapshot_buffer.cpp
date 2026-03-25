#include "client/client_snapshot_buffer.hpp"

#include <utility>

namespace spaceship::client
{

void ClientSnapshotBuffer::push(spaceship::server::WorldSnapshot snapshot)
{
    latest_ = std::move(snapshot);
}

const std::optional<spaceship::server::WorldSnapshot>& ClientSnapshotBuffer::latest() const
{
    return latest_;
}

} // namespace spaceship::client

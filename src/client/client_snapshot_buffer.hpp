#pragma once

// Stores the latest authoritative snapshot for client-side interpolation.

#include "server/snapshot/snapshot_types.hpp"

#include <optional>

namespace spaceship::client
{

class ClientSnapshotBuffer
{
  public:
    void push(spaceship::server::WorldSnapshot snapshot);
    [[nodiscard]] const std::optional<spaceship::server::WorldSnapshot>& latest() const;

  private:
    std::optional<spaceship::server::WorldSnapshot> latest_ {};
};

} // namespace spaceship::client

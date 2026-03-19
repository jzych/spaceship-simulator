#pragma once

// Stores the latest authoritative snapshot summary for future client-side interpolation.

#include "shared/sim_types.hpp"

#include <optional>
#include <string>

namespace spaceship::client
{

struct SnapshotFrame
{
    shared::Tick serverTick {};
    std::string summary {};
};

class ClientSnapshotBuffer
{
  public:
    void push(SnapshotFrame frame);
    [[nodiscard]] std::optional<SnapshotFrame> latest() const;

  private:
    std::optional<SnapshotFrame> latest_ {};
};

} // namespace spaceship::client

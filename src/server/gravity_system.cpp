#include "server/gravity_system.hpp"

namespace spaceship::server
{

void GravitySystem::update(std::vector<PendingEvent>& events) const
{
    events.push_back({"gravity-step"});
}

} // namespace spaceship::server

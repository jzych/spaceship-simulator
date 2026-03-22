#pragma once

// Declares creation of the initial authoritative simulation world.

#include "server/simulation_world.hpp"

namespace spaceship::server
{

SimulationWorld createInitialWorld();

// Single Earth at origin — used for isolated gravity and orbital tests.
SimulationWorld createEarthOnlyAtOriginWorld();

} // namespace spaceship::server

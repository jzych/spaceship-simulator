#pragma once

// Shared test constants and fixtures used across multiple test files.

#include "server/simulation/bootstrap.hpp"
#include "server/simulation/simulation_math.hpp"
#include "server/simulation/simulation_server.hpp"
#include "server/simulation/simulation_world.hpp"

#include <gtest/gtest.h>
#include <cmath>
#include <numbers>

namespace spaceship::test
{

// --- Shared constants ---

constexpr double kDefaultShipAccelerationMetersPerSecondSquared = 20'000.0 / 1'000.0;
constexpr double kExpectedVelocityDeltaPerTick =
    kDefaultShipAccelerationMetersPerSecondSquared * shared::constants::kFixedDeltaSeconds;
constexpr double kQuarterTurnZAxisHalfAngleComponent = std::numbers::sqrt2_v<double> / 2.0;
constexpr double kDefaultProjectileMuzzleSpeedMetersPerSecond = 1'000.0;

constexpr double kEarthMu = 3.986004418e14;
constexpr double kEarthRadius = 6.371e6;
constexpr double kLeoAltitude = 400'000.0;
constexpr double kLeoRadius = kEarthRadius + kLeoAltitude;

// --- Shared fixtures ---

// Single Earth at origin for precise gravity and orbital tests.
class EarthOnlyCenteredTest : public ::testing::Test
{
  protected:
    server::SimulationServer server {server::createEarthOnlyAtOriginWorld()};
};

} // namespace spaceship::test

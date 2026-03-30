# Spaceship Simulator

[![CI](https://github.com/jzych/spaceship-simulator/actions/workflows/ci.yml/badge.svg?branch=main)](https://github.com/jzych/spaceship-simulator/actions/workflows/ci.yml)
[![Quality Gate Status](https://sonarcloud.io/api/project_badges/measure?project=jzych_spaceship-simulator&metric=alert_status)](https://sonarcloud.io/summary/new_code?id=jzych_spaceship-simulator)

Real-time spaceship physics simulation with an Unreal Engine 5 visualizer.
Covers gravity, orbit mechanics, collision, and adaptive timestep.

## Repository layout

```
SpaceshipSimulator/          UE5 project (open SpaceshipSimulator.uproject)
  Source/SpaceshipSimulator/
    Sim/                     Simulation engine — server/, client/, shared/
    Actors/                  UE5 planet actors
    Subsystems/              SimulationSubsystem (tick loop, coordinate mapping)
tests/                       GoogleTest suites (compiled by CMake only)
CMakeLists.txt               Standalone build for tests
```

## Running tests

```sh
cmake -S . -B build -DBUILD_TESTING=ON
cmake --build build
ctest --test-dir build --output-on-failure
```

## UE5 development

Open `SpaceshipSimulator/SpaceshipSimulator.uproject` in UE5.7+.
UBT compiles the simulation sources directly — no separate CMake step required.

After editing simulation code, use **Live Coding** (Ctrl+Alt+F11) or rebuild
from VS Code (`SpaceshipSimulatorEditor Win64 Development Build` task).

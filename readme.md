# Spaceship Simulator

[![CI](https://github.com/jzych/spaceship-simulator/actions/workflows/ci.yml/badge.svg?branch=main)](https://github.com/jzych/spaceship-simulator/actions/workflows/ci.yml)
[![codecov](https://codecov.io/gh/jzych/spaceship-simulator/branch/main/graph/badge.svg)](https://app.codecov.io/gh/jzych/spaceship-simulator?branch=main)
[![Quality Gate](https://sonarcloud.io/api/project_badges/quality_gate?project=jzych_spaceship-simulator)](https://sonarcloud.io/dashboard?id=jzych_spaceship-simulator)

A lightweight C++20 simulation of a planetary system. The project uses CMake for build management and Google Test for testing. Continuous integration is configured with GitHub Actions, Codecov for coverage reporting, SonarCloud for static analysis, and OpenAI Codex to review pull requests automatically.

## Running locally

1. Configure the project with CMake and build using Ninja:

   ```sh
   cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Debug
   cmake --build build
   ```

2. Run the simulator:

   ```sh
   ./build/spaceship-simulator
   ```

3. Run the unit tests:

   ```sh
   ctest --test-dir build -VV
   ```

For more information on agent workflows and contributing guidelines, see [AGENTS.md](AGENTS.md).

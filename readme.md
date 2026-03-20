# Spaceship Simulator

[![CI](https://github.com/jzych/spaceship-simulator/actions/workflows/ci.yml/badge.svg?branch=main)](https://github.com/jzych/spaceship-simulator/actions/workflows/ci.yml)
[![codecov](https://codecov.io/gh/jzych/spaceship-simulator/branch/main/graph/badge.svg)](https://app.codecov.io/gh/jzych/spaceship-simulator?branch=main)
[![Quality Gate Status](https://sonarcloud.io/api/project_badges/measure?project=jzych_spaceship-simulator&metric=alert_status)](https://sonarcloud.io/summary/new_code?id=jzych_spaceship-simulator)

## Running locally

1. Build the project:

   ```sh
   cmake --build build
   ```

2. Run the project:

   ```sh
   ./build/spaceship-simulator
   ```

3. Run the tests:

   ```sh
   ctest --test-dir build -VV
   ```

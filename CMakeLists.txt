cmake_minimum_required(VERSION 3.20)
project(spaceship-simulator LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(CTest)

set(SIM_ROOT SpaceshipSimulator/Plugins/SpaceshipSim/Source/SpaceshipSim/Private)

add_library(spaceship-shared INTERFACE)
target_include_directories(spaceship-shared INTERFACE ${SIM_ROOT})

add_library(spaceship-server
    ${SIM_ROOT}/server/collision/broad_phase.cpp
    ${SIM_ROOT}/server/collision/broad_phase.hpp
    ${SIM_ROOT}/server/collision/collision_system.cpp
    ${SIM_ROOT}/server/collision/collision_system.hpp
    ${SIM_ROOT}/server/collision/collision_types.hpp
    ${SIM_ROOT}/server/collision/sweep_math.cpp
    ${SIM_ROOT}/server/collision/sweep_math.hpp
    ${SIM_ROOT}/server/control/ship_control_system.cpp
    ${SIM_ROOT}/server/control/ship_control_system.hpp
    ${SIM_ROOT}/server/simulation/bootstrap.cpp
    ${SIM_ROOT}/server/simulation/bootstrap.hpp
    ${SIM_ROOT}/server/simulation/gravity/gravity_system.cpp
    ${SIM_ROOT}/server/simulation/gravity/gravity_system.hpp
    ${SIM_ROOT}/server/simulation/gravity/massive_body_motion_system.cpp
    ${SIM_ROOT}/server/simulation/gravity/massive_body_motion_system.hpp
    ${SIM_ROOT}/server/simulation/gravity/orbit_cache.hpp
    ${SIM_ROOT}/server/simulation/gravity/orbit_cache_system.cpp
    ${SIM_ROOT}/server/simulation/gravity/orbit_cache_system.hpp
    ${SIM_ROOT}/server/simulation/gravity/orbit_fitting.cpp
    ${SIM_ROOT}/server/simulation/gravity/orbit_fitting.hpp
    ${SIM_ROOT}/server/simulation/gravity/reference_body_selector.cpp
    ${SIM_ROOT}/server/simulation/gravity/reference_body_selector.hpp
    ${SIM_ROOT}/server/simulation/integration_system.cpp
    ${SIM_ROOT}/server/simulation/integration_system.hpp
    ${SIM_ROOT}/server/simulation/simulation_config.hpp
    ${SIM_ROOT}/server/simulation/simulation_math.hpp
    ${SIM_ROOT}/server/simulation/simulation_server.cpp
    ${SIM_ROOT}/server/simulation/simulation_server.hpp
    ${SIM_ROOT}/server/simulation/simulation_world.hpp
    ${SIM_ROOT}/server/simulation/timestep/timescale_heuristics.cpp
    ${SIM_ROOT}/server/simulation/timestep/timescale_heuristics.hpp
    ${SIM_ROOT}/server/simulation/timestep/timestep_controller.cpp
    ${SIM_ROOT}/server/simulation/timestep/timestep_controller.hpp
    ${SIM_ROOT}/server/simulation/timestep/timestep_types.hpp
    ${SIM_ROOT}/server/snapshot/snapshot_system.cpp
    ${SIM_ROOT}/server/snapshot/snapshot_system.hpp
    ${SIM_ROOT}/server/snapshot/snapshot_types.hpp
    ${SIM_ROOT}/server/spawning/spawning_system.cpp
    ${SIM_ROOT}/server/spawning/spawning_system.hpp
)
target_include_directories(spaceship-server PUBLIC ${SIM_ROOT})
target_link_libraries(spaceship-server PUBLIC spaceship-shared)

add_library(spaceship-client-stub
    ${SIM_ROOT}/client/client_snapshot_buffer.cpp
    ${SIM_ROOT}/client/client_snapshot_buffer.hpp
    ${SIM_ROOT}/client/client_types.hpp
    ${SIM_ROOT}/client/simulation_runner.cpp
    ${SIM_ROOT}/client/simulation_runner.hpp
    ${SIM_ROOT}/client/snapshot_interpolator.cpp
    ${SIM_ROOT}/client/snapshot_interpolator.hpp
)
target_include_directories(spaceship-client-stub PUBLIC ${SIM_ROOT})
target_link_libraries(spaceship-client-stub
    PUBLIC  spaceship-shared
    PRIVATE spaceship-server
)

add_executable(spaceship-simulator main.cpp)
target_link_libraries(spaceship-simulator PRIVATE spaceship-server spaceship-client-stub)

if(BUILD_TESTING)
    # Expose test-only API (e.g. SimulationServer::injectProjectile) in the server library.
    target_compile_definitions(spaceship-server PRIVATE BUILD_TESTING)
    target_compile_definitions(spaceship-client-stub PRIVATE BUILD_TESTING)

    include(FetchContent)

    FetchContent_Declare(
        googletest
        URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip
        DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    )
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(googletest)

    add_executable(spaceship-simulator-tests
        tests/client/client_snapshot_buffer_test.cpp
        tests/client/simulation_runner_test.cpp
        tests/client/snapshot_interpolator_test.cpp
        tests/server/collision/broad_phase_test.cpp
        tests/server/collision/collision_pipeline_test.cpp
        tests/server/collision/collision_system_test.cpp
        tests/server/collision/sweep_math_test.cpp
        tests/server/control/ship_control_system_test.cpp
        tests/server/simulation/gravity/geographic_telemetry_test.cpp
        tests/server/simulation/gravity/gravity_system_test.cpp
        tests/server/simulation/gravity/gravity_test.cpp
        tests/server/simulation/gravity/massive_body_motion_test.cpp
        tests/server/simulation/gravity/orbit_cache_test.cpp
        tests/server/simulation/gravity/orbit_fitting_test.cpp
        tests/server/simulation/gravity/reference_body_selector_test.cpp
        tests/server/simulation/integration_system_test.cpp
        tests/server/simulation/integration_test.cpp
        tests/server/simulation/ship_behavior_test.cpp
        tests/server/simulation/simulation_math_test.cpp
        tests/server/simulation/simulation_server_test.cpp
        tests/server/simulation/timestep/timescale_heuristics_test.cpp
        tests/server/simulation/timestep/timestep_ladder_test.cpp
        tests/server/simulation/timestep/timestep_controller_test.cpp
        tests/server/simulation/timestep/energy_regression_test.cpp
        tests/server/snapshot/snapshot_system_test.cpp
        tests/server/spawning/spawning_system_test.cpp
    )
    target_include_directories(spaceship-simulator-tests
        PRIVATE
            ${SIM_ROOT}
            tests
    )
    target_compile_definitions(spaceship-simulator-tests PRIVATE BUILD_TESTING)
    target_link_libraries(spaceship-simulator-tests
        PRIVATE
            GTest::gtest_main
            spaceship-server
            spaceship-client-stub
    )

    include(GoogleTest)
    gtest_discover_tests(spaceship-simulator-tests)
endif()

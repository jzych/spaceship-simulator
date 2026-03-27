cmake_minimum_required(VERSION 3.20)
project(spaceship-simulator LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(CTest)

add_library(spaceship-shared INTERFACE)
target_include_directories(spaceship-shared INTERFACE src)

add_library(spaceship-server
    src/server/collision/broad_phase.cpp
    src/server/collision/broad_phase.hpp
    src/server/collision/collision_system.cpp
    src/server/collision/collision_system.hpp
    src/server/collision/collision_types.hpp
    src/server/collision/sweep_math.cpp
    src/server/collision/sweep_math.hpp
    src/server/control/ship_control_system.cpp
    src/server/control/ship_control_system.hpp
    src/server/simulation/bootstrap.cpp
    src/server/simulation/bootstrap.hpp
    src/server/simulation/gravity/gravity_system.cpp
    src/server/simulation/gravity/gravity_system.hpp
    src/server/simulation/gravity/massive_body_motion_system.cpp
    src/server/simulation/gravity/massive_body_motion_system.hpp
    src/server/simulation/gravity/orbit_cache.hpp
    src/server/simulation/gravity/orbit_cache_system.cpp
    src/server/simulation/gravity/orbit_cache_system.hpp
    src/server/simulation/gravity/orbit_fitting.cpp
    src/server/simulation/gravity/orbit_fitting.hpp
    src/server/simulation/gravity/reference_body_selector.cpp
    src/server/simulation/gravity/reference_body_selector.hpp
    src/server/simulation/integration_system.cpp
    src/server/simulation/integration_system.hpp
    src/server/simulation/simulation_config.hpp
    src/server/simulation/simulation_math.hpp
    src/server/simulation/simulation_server.cpp
    src/server/simulation/simulation_server.hpp
    src/server/simulation/simulation_world.hpp
    src/server/simulation/timestep/timescale_heuristics.cpp
    src/server/simulation/timestep/timescale_heuristics.hpp
    src/server/simulation/timestep/timestep_controller.cpp
    src/server/simulation/timestep/timestep_controller.hpp
    src/server/simulation/timestep/timestep_types.hpp
    src/server/snapshot/snapshot_system.cpp
    src/server/snapshot/snapshot_system.hpp
    src/server/snapshot/snapshot_types.hpp
    src/server/spawning/spawning_system.cpp
    src/server/spawning/spawning_system.hpp
)
target_include_directories(spaceship-server PUBLIC src)
target_link_libraries(spaceship-server PUBLIC spaceship-shared)

add_library(spaceship-client-stub
    src/client/client_snapshot_buffer.cpp
    src/client/client_snapshot_buffer.hpp
    src/client/client_types.hpp
    src/client/snapshot_interpolator.cpp
    src/client/snapshot_interpolator.hpp
)
target_include_directories(spaceship-client-stub PUBLIC src)
target_link_libraries(spaceship-client-stub
    PUBLIC  spaceship-shared
    PRIVATE spaceship-server
)

add_executable(spaceship-simulator src/main.cpp)
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
        src/client/tests/client_snapshot_buffer_test.cpp
        src/client/tests/snapshot_interpolator_test.cpp
        src/server/collision/tests/broad_phase_test.cpp
        src/server/collision/tests/collision_pipeline_test.cpp
        src/server/collision/tests/collision_system_test.cpp
        src/server/collision/tests/sweep_math_test.cpp
        src/server/control/tests/ship_control_system_test.cpp
        src/server/simulation/gravity/tests/geographic_telemetry_test.cpp
        src/server/simulation/gravity/tests/gravity_system_test.cpp
        src/server/simulation/gravity/tests/gravity_test.cpp
        src/server/simulation/gravity/tests/massive_body_motion_test.cpp
        src/server/simulation/gravity/tests/orbit_cache_test.cpp
        src/server/simulation/gravity/tests/orbit_fitting_test.cpp
        src/server/simulation/gravity/tests/reference_body_selector_test.cpp
        src/server/simulation/tests/integration_system_test.cpp
        src/server/simulation/tests/integration_test.cpp
        src/server/simulation/tests/ship_behavior_test.cpp
        src/server/simulation/tests/simulation_math_test.cpp
        src/server/simulation/tests/simulation_server_test.cpp
        src/server/simulation/timestep/tests/timescale_heuristics_test.cpp
        src/server/simulation/timestep/tests/timestep_ladder_test.cpp
        src/server/simulation/timestep/tests/timestep_controller_test.cpp
        src/server/simulation/timestep/tests/energy_regression_test.cpp
        src/server/snapshot/tests/snapshot_system_test.cpp
        src/server/spawning/tests/spawning_system_test.cpp
    )
    target_include_directories(spaceship-simulator-tests
        PRIVATE
            src
            src/server/tests
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

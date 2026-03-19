cmake_minimum_required(VERSION 3.20)
project(spaceship-simulator LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(CTest)

add_library(spaceship-shared INTERFACE)
target_include_directories(spaceship-shared INTERFACE src)

add_library(spaceship-server
    src/server/bootstrap.cpp
    src/server/bootstrap.hpp
    src/server/collision_system.cpp
    src/server/collision_system.hpp
    src/server/gravity_system.cpp
    src/server/gravity_system.hpp
    src/server/integration_system.cpp
    src/server/integration_system.hpp
    src/server/simulation_config.hpp
    src/server/simulation_server.cpp
    src/server/simulation_server.hpp
    src/server/simulation_world.hpp
    src/server/snapshot_system.cpp
    src/server/snapshot_system.hpp
    src/server/spawning_system.cpp
    src/server/spawning_system.hpp
)
target_include_directories(spaceship-server PUBLIC src)
target_link_libraries(spaceship-server PUBLIC spaceship-shared)

add_library(spaceship-client-stub
    src/client/client_snapshot_buffer.cpp
    src/client/client_snapshot_buffer.hpp
)
target_include_directories(spaceship-client-stub PUBLIC src)
target_link_libraries(spaceship-client-stub PUBLIC spaceship-shared)

add_executable(spaceship-simulator src/main.cpp)
target_link_libraries(spaceship-simulator PRIVATE spaceship-server spaceship-client-stub)

if(BUILD_TESTING)
    include(FetchContent)

    FetchContent_Declare(
        googletest
        URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip
        DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    )
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(googletest)

    add_executable(spaceship-simulator-tests
        tests/simulation_server_smoke_test.cpp
    )
    target_link_libraries(spaceship-simulator-tests
        PRIVATE
            GTest::gtest_main
            spaceship-server
    )

    include(GoogleTest)
    gtest_discover_tests(spaceship-simulator-tests)
endif()

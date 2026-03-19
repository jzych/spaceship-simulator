#include "server/simulation_server.hpp"

#include <iostream>

int main()
{
    spaceship::server::SimulationServer server;

    std::cout << "Spaceship Simulator\n";
    std::cout << "Massive bodies: " << server.world().massiveBodies.size() << "\n";

    server.tick();

    std::cout << "Server tick: " << server.tickCount() << "\n";
    return 0;
}

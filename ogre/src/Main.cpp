#include "controller/Controller.hpp"
#include <iostream>

std::vector<Position> getTrajectory() {
    std::vector<Position> trajectory;
    Position position;
    while (std::cin >> position) {
        trajectory.push_back(position);
    }
    return trajectory;
}

int main(int argc, char* argv[]) {
    Controller controller;

    if (argc > 1)
        controller.moveAlongTrajectory(getTrajectory());

    return 0;
}

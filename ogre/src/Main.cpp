#include "controller/Controller.hpp"
#include "model/Model.hpp"
#include "view/View.hpp"
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
    View view;
    Model model(view);
    Controller controller(model, view);

    if (argc > 1)
        controller.moveAlongTrajectory(getTrajectory());

    return 0;
}

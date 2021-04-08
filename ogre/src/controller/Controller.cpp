#include "Controller.hpp"

Controller::Controller() : view(), model(view) {
    view.init(this);
}

Controller::~Controller() {
    view.close();
}

bool Controller::keyPressed(const OgreBites::KeyboardEvent& evt) {
    if (evt.keysym.sym == OgreBites::SDLK_ESCAPE) {
        view.end();
    } else if (evt.keysym.sym == OgreBites::SDLK_SPACE) {
        view.save("output/scene.png");
    } else if (evt.keysym.sym == 'w') {
        model.move(Direction::FORWARD);
    } else if (evt.keysym.sym == 's') {
        model.move(Direction::BACKWARD);
    } else if (evt.keysym.sym == 'a') {
        model.move(Direction::LEFT);
    } else if (evt.keysym.sym == 'd') {
        model.move(Direction::RIGHT);
    } else if (evt.keysym.sym == 'q') {
        model.move(Direction::DOWN);
    } else if (evt.keysym.sym == 'e') {
        model.move(Direction::UP);
    }
    return true;
}

void Controller::moveAlongTrajectory(std::vector<Position> trajectory) {
    for (int i = 0; i < trajectory.size(); ++i) {
        model.move(trajectory[i]);
        view.save("output/scene" + std::to_string(i) + ".png");
    }
}
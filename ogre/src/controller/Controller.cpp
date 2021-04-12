#include "Controller.hpp"

Controller::Controller(Config&& config)
    : view(), model(view), config(std::move(config)) {
    view.init(this);
    model.move(config.position);
}

Controller::~Controller() {
    view.close();
}

bool Controller::keyPressed(OgreBites::KeyboardEvent const& evt) {
    if (evt.keysym.sym == OgreBites::SDLK_ESCAPE) {
        view.end();
    } else if (evt.keysym.sym == OgreBites::SDLK_SPACE) {
        view.save(config.outputDir / "scene.png");
    } else if (evt.keysym.sym == OgreBites::SDLK_RETURN) {
        moveAlongTrajectory();
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

bool Controller::mouseMoved(OgreBites::MouseMotionEvent const& evt) {
    model.move(evt);
    return true;
}

void Controller::moveAlongTrajectory() {
    for (int i = 0; i < config.trajectory.size(); ++i) {
        model.move(config.trajectory[i]);
        view.update();
        view.save(config.outputDir / ("scene" + std::to_string(i) + ".png"));
    }
}
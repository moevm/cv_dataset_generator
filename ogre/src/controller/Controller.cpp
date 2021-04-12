#include "Controller.hpp"

Controller::Controller(Config config)
    : view(), model(view), config(std::move(config)) {
    view.init(this);
    model.move(config.position);
}

Controller::~Controller() {
    view.close();
}

bool Controller::keyPressed(OgreBites::KeyboardEvent const& evt) {
    switch (evt.keysym.sym) {
    case OgreBites::SDLK_ESCAPE:
        view.end();
        break;
    case OgreBites::SDLK_SPACE:
        view.save(config.outputDir / ("scene_" + model.getPosition().toString() + ".png"));
        break;
    case OgreBites::SDLK_RETURN:
        moveAlongTrajectory();
        break;
    case OgreBites::SDLK_RIGHT:
        config.trajectory.next();
        model.move(config.trajectory.get());
        view.update();
        break;
    case OgreBites::SDLK_LEFT:
        config.trajectory.prev();
        model.move(config.trajectory.get());
        view.update();
        break;
    case 'w':
        model.move(Direction::FORWARD);
        break;
    case 's':
        model.move(Direction::BACKWARD);
        break;
    case 'a':
        model.move(Direction::LEFT);
        break;
    case 'd':
        model.move(Direction::RIGHT);
        break;
    case 'q':
        model.move(Direction::DOWN);
        break;
    case 'e':
        model.move(Direction::UP);
        break;
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
        view.save(config.outputDir / ("scene_" + std::to_string(i) + '_' +
                                      config.trajectory[i].toString() + ".png"));
    }
}
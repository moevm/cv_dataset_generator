#include "Controller.hpp"

Controller::Controller(Config& config)
    : view(config), model(view, config), config(config) {
    view.init(this);
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
        view.save(config.outputDir /
                  ("scene_" + model.getCamera().getPosition().toString() + ".png"));
        break;
    case OgreBites::SDLK_RETURN:
        moveAlongTrajectory();
        break;
    case OgreBites::SDLK_RIGHT:
        config.trajectory.next();
        moveCamera(model.getCamera(), config.trajectory.get());
        view.update();
        break;
    case OgreBites::SDLK_LEFT:
        config.trajectory.prev();
        moveCamera(model.getCamera(), config.trajectory.get());
        view.update();
        break;
    case 'w':
        moveCamera(model.getCamera(), Direction::FORWARD);
        break;
    case 's':
        moveCamera(model.getCamera(), Direction::BACKWARD);
        break;
    case 'a':
        moveCamera(model.getCamera(), Direction::LEFT);
        break;
    case 'd':
        moveCamera(model.getCamera(), Direction::RIGHT);
        break;
    case 'q':
        moveCamera(model.getCamera(), Direction::DOWN);
        break;
    case 'e':
        moveCamera(model.getCamera(), Direction::UP);
        break;
    }
    return true;
}

bool Controller::mouseMoved(OgreBites::MouseMotionEvent const& evt) {
    moveCamera(model.getCamera(), evt);
    return true;
}

void Controller::moveAlongTrajectory() {
    for (int i = 0; i < config.trajectory.size(); ++i) {
        moveCamera(model.getCamera(), config.trajectory[i]);
        view.update();
        view.save(config.outputDir / ("scene_" + std::to_string(i) + '_' +
                                      config.trajectory[i].toString() + ".png"));
    }
}
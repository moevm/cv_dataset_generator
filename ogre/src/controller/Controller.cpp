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
        model.getCamera().move(config.trajectory.get());
        view.update();
        break;
    case OgreBites::SDLK_LEFT:
        config.trajectory.prev();
        model.getCamera().move(config.trajectory.get());
        view.update();
        break;
    case 'w':
        model.getCamera().move(Direction::FORWARD);
        break;
    case 's':
        model.getCamera().move(Direction::BACKWARD);
        break;
    case 'a':
        model.getCamera().move(Direction::LEFT);
        break;
    case 'd':
        model.getCamera().move(Direction::RIGHT);
        break;
    case 'q':
        model.getCamera().move(Direction::DOWN);
        break;
    case 'e':
        model.getCamera().move(Direction::UP);
        break;
    }
    return true;
}

bool Controller::mouseMoved(OgreBites::MouseMotionEvent const& evt) {
    model.getCamera().move(evt);
    return true;
}

void Controller::moveAlongTrajectory() {
    for (int i = 0; i < config.trajectory.size(); ++i) {
        model.getCamera().move(config.trajectory[i]);
        view.update();
        view.save(config.outputDir / ("scene_" + std::to_string(i) + '_' +
                                      config.trajectory[i].toString() + ".png"));
    }
}
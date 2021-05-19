#pragma once
#include "../model/Model.hpp"
#include "../model/Position.hpp"
#include "../view/View.hpp"
#include "Config.hpp"
#include "OgreInput.h"
#include <vector>

class Controller : public OgreBites::InputListener {
public:
    Controller(Config&);
    ~Controller() override;
    bool keyPressed(OgreBites::KeyboardEvent const&) override;
    bool mouseMoved(OgreBites::MouseMotionEvent const&) override;

private:
    void moveAlongTrajectory();

    template <class Movement>
    void moveCamera(Camera& camera, Movement&& movement) {
        camera.move(std::forward<Movement>(movement));
        view.statusUpdate(camera.getPosition());
    }

    View view;
    Model model;
    Config& config;
};
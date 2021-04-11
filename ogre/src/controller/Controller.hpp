#pragma once
#include "../model/Model.hpp"
#include "../model/Position.hpp"
#include "../view/View.hpp"
#include "Config.hpp"
#include "OgreInput.h"
#include <vector>

class Controller : public OgreBites::InputListener {
public:
    Controller(Config&& = {});
    ~Controller() override;
    bool keyPressed(const OgreBites::KeyboardEvent&) override;
    // bool mouseMoved(const MouseMotionEvent&) override;

private:
    void moveAlongTrajectory();

    View view;
    Model model;
    Config config;
};
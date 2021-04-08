#pragma once
#include "../model/Model.hpp"
#include "../model/Position.hpp"
#include "../view/View.hpp"
#include "OgreInput.h"
#include <vector>

class Controller : public OgreBites::InputListener {
public:
    Controller();
    ~Controller() override;
    bool keyPressed(const OgreBites::KeyboardEvent&) override;
    // bool mouseMoved(const MouseMotionEvent&) override;
    void moveAlongTrajectory(std::vector<Position>);

private:
    View view;
    Model model;
};
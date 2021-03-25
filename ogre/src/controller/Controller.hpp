#pragma once
#include "../model/Model.hpp"
#include "../view/View.hpp"
#include "OgreInput.h"

class Controller : public OgreBites::InputListener {
public:
    Controller(Model&, View&);
    ~Controller() override;
    bool keyPressed(const OgreBites::KeyboardEvent&) override;

private:
    Model& model;
    View& view;
};
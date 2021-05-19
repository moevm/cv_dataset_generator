#pragma once
#include "../../view/View.hpp"
#include "../Position.hpp"
#include "Ogre.h"

class Entity {
public:
    virtual Position getPosition() const;

protected:
    Entity(View&);

    Ogre::SceneNode* sceneNode;
};
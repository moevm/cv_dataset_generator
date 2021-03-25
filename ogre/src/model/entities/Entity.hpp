#pragma once
#include "../../view/View.hpp"
#include "Ogre.h"

class Entity {
protected:
    Entity(View&);

    Ogre::SceneNode* sceneNode;
};
#pragma once
#include "Entity.hpp"

class Cube : Entity {
public:
    Cube(View&);

private:
    Ogre::Entity* cube;
};

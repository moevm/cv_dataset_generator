#pragma once
#include "Entity.hpp"

class Cube : public Entity {
public:
    Cube(View&);

private:
    Ogre::Entity* cube;
};

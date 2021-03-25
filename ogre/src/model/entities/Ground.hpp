#pragma once
#include "Entity.hpp"

class Ground : Entity {
public:
    Ground(View&);

private:
    Ogre::Entity* ground;

    Ogre::MeshPtr create();
};

#pragma once
#include "Entity.hpp"

class Ground : public Entity {
public:
    Ground(View&);

private:
    Ogre::Entity* ground;

    Ogre::MeshPtr create();
};

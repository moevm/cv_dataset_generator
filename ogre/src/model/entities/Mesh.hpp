#pragma once
#include "Entity.hpp"

class Mesh : public Entity {
public:
    Mesh(View&);

private:
    Ogre::Entity* mesh;

    Ogre::MeshPtr create();
};
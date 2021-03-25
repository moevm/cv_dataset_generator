#pragma once
#include "Entity.hpp"

class Sphere : Entity {
public:
    Sphere(View&);

private:
    Ogre::Entity* sphere;

    Ogre::MeshPtr create(Ogre::ManualObject*);
};
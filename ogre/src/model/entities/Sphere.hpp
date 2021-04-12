#pragma once
#include "Entity.hpp"

class Sphere : public Entity {
public:
    Sphere(View&);

private:
    Ogre::Entity* sphere;

    Ogre::MeshPtr create(Ogre::ManualObject*);
};
#pragma once
#include "../Direction.hpp"
#include "Entity.hpp"

class Camera : Entity {
public:
    Camera(View&);
    Ogre::Camera* getCamera();
    void move(Direction);

private:
    Ogre::Camera* camera;
};
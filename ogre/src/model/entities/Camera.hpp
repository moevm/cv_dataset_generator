#pragma once
#include "../Position.hpp"
#include "Entity.hpp"

class Camera : Entity {
public:
    Camera(View&);
    Ogre::Camera* getCamera();
    void move(Direction);
    void move(Position const&);

private:
    Ogre::Camera* camera;
};
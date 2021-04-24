#pragma once
#include "../Position.hpp"
#include "Entity.hpp"
#include "OgreCameraMan.h"
#include "OgreInput.h"

class Camera : public Entity {
public:
    Camera(View&);
    Ogre::Camera* getCamera();
    void callibrate(CameraInfo const&);
    void move(Direction);
    void move(Position const&);
    void move(OgreBites::MouseMotionEvent const&);

private:
    Ogre::Camera* camera;
    OgreBites::CameraMan* cameraMan;
};
#pragma once
#include "../Position.hpp"
#include "Entity.hpp"
#include "OgreCameraMan.h"
#include "OgreInput.h"
#include <sensor_msgs/CameraInfo.h>

class Camera : public Entity {
public:
    Camera(View&);
    Ogre::Camera* getCamera();
    void callibrate(sensor_msgs::CameraInfo const&);
    void move(Direction);
    void move(Position const&);
    void move(OgreBites::MouseMotionEvent const&);

private:
    Ogre::Camera* camera;
    OgreBites::CameraMan* cameraMan;
};
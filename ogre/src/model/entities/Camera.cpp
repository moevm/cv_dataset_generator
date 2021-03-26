#include "Camera.hpp"

Camera::Camera(View& view)
    : Entity(view), camera(view.createCamera("MainCamera")) {
    camera->setNearClipDistance(0.1);
    camera->setFarClipDistance(0);
    camera->setAutoAspectRatio(true);

    sceneNode->setPosition(0, 20, 30);
    sceneNode->lookAt(Ogre::Vector3(0, 5, 0), Ogre::Node::TS_PARENT);
    sceneNode->attachObject(camera);
}

Ogre::Camera* Camera::getCamera() {
    return camera;
}

void Camera::move(Direction direction) {
    const Ogre::Real delta = 0.3;
    switch (direction) {
    case Direction::FORWARD:
        sceneNode->translate(0, 0, -delta);
        break;
    case Direction::BACKWARD:
        sceneNode->translate(0, 0, delta);
        break;
    case Direction::LEFT:
        sceneNode->translate(-delta, 0, 0);
        break;
    case Direction::RIGHT:
        sceneNode->translate(delta, 0, 0);
        break;
    case Direction::UP:
        sceneNode->translate(0, -delta, 0);
        break;
    case Direction::DOWN:
        sceneNode->translate(0, delta, 0);
        break;
    }
}

void Camera::move(Position const& position) {
    sceneNode->setPosition(position.x, position.y, position.z);
    sceneNode->setOrientation(Ogre::Quaternion());
    sceneNode->pitch(position.pitch);
    sceneNode->yaw(position.yaw);
    sceneNode->roll(position.roll);
}
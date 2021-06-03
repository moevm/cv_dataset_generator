#include "Light.hpp"

Light::Light(View& view) : Entity(view), light(view.createLight()) {
    light->setType(Ogre::Light::LT_SPOTLIGHT);
    light->setDiffuseColour(1, 0.952, 0.937);
    light->setSpecularColour(1, 0.984, 0.980);

    sceneNode->setPosition(10, 60, 50);
    sceneNode->setDirection(-10, -60, -50);
    sceneNode->attachObject(light);
}
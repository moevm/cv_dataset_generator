#include "Light.hpp"

Light::Light(View& view) : Entity(view), light(view.createLight()) {
    light->setType(Ogre::Light::LT_SPOTLIGHT);
    light->setDiffuseColour(0.7, 0.5, 0.6);
    light->setSpecularColour(1.0, 0, 1.0);

    sceneNode->setPosition(10, 60, 50);
    sceneNode->setDirection(-10, -60, -50);
    sceneNode->attachObject(light);
}
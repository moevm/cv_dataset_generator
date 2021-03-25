#include "Cube.hpp"

Cube::Cube(View& view)
    : Entity(view), cube(view.createEntity(Ogre::SceneManager::PrefabType::PT_CUBE)) {
    cube->setCastShadows(true);
    cube->setMaterialName("Red");

    sceneNode->setPosition(-2, 6, 0);
    Ogre::Real scale = 0.07;
    sceneNode->setScale(scale, scale, scale);
    sceneNode->yaw(Ogre::Degree(30));
    sceneNode->roll(Ogre::Degree(60));
    sceneNode->attachObject(cube);
}
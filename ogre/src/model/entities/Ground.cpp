#include "Ground.hpp"

Ground::Ground(View& view) : Entity(view), ground(view.createEntity(create())) {
    ground->setCastShadows(false);
    ground->setMaterialName("Yellow");

    sceneNode->setPosition(0, 0, -5);
    sceneNode->attachObject(ground);
}

Ogre::MeshPtr Ground::create() {
    Ogre::Plane plane(Ogre::Vector3::UNIT_Y, 0);
    Ogre::Real size = 40;
    return Ogre::MeshManager::getSingleton().createPlane(
        "ground", Ogre::RGN_DEFAULT, plane, size, size, 20, 20, true, 1, 5, 5,
        Ogre::Vector3::UNIT_Z);
}
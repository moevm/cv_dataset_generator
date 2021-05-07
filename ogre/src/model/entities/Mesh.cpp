#include "Mesh.hpp"

Mesh::Mesh(View& view) : Entity(view), mesh(view.createEntity(create())) {
    mesh->setCastShadows(true);
    mesh->setMaterialName("cottage_texture");

    sceneNode->setPosition(0, 14, 10);
    // Ogre::Real scale = 0.05;
    // sceneNode->setScale(scale, scale, scale);
    sceneNode->attachObject(mesh);
}

Ogre::MeshPtr Mesh::create() {
    return Ogre::MeshManager::getSingleton().load(
        "Cube.002.mesh", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
}
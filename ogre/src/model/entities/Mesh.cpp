#include "Mesh.hpp"

Mesh::Mesh(View& view, Ogre::String const& meshFile, Ogre::String const& materialName)
    : Entity(view), mesh(view.createEntity(create(meshFile))) {
    mesh->setCastShadows(true);
    mesh->setMaterialName(materialName);

    sceneNode->setPosition(0, 0, 0);
    sceneNode->attachObject(mesh);
}

Ogre::MeshPtr Mesh::create(Ogre::String const& meshFile) {
    return Ogre::MeshManager::getSingleton().load(
        meshFile, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
}
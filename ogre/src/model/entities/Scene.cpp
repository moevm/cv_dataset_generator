#include "Scene.hpp"

Scene::Scene(View& view, Ogre::String const& sceneFile) : Entity(view) {
    sceneNode->loadChildren(sceneFile);
    sceneNode->setPosition(0, 0, 0);
}

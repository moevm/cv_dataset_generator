#include "Entity.hpp"

Entity::Entity(View& view) : sceneNode(view.createSceneNode()) {
}

Position Entity::getPosition() const {
    auto position = sceneNode->getPosition();
    return {position.x, position.y, position.z, 0.0, 0.0, 0.0};
}
#pragma once
#include "Entity.hpp"

class Scene : public Entity {
public:
    Scene(View&, Ogre::String const&);
};
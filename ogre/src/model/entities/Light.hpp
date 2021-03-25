#pragma once
#include "Entity.hpp"

class Light : Entity {
public:
    Light(View&);

private:
    Ogre::Light* light;
};
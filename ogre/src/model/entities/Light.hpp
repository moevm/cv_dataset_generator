#pragma once
#include "Entity.hpp"

class Light : public Entity {
public:
    Light(View&);

private:
    Ogre::Light* light;
};
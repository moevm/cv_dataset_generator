#pragma once
#include "Entity.hpp"

class Mesh : public Entity {
public:
    Mesh(View&, Ogre::String const&, Ogre::String const&);

private:
    Ogre::Entity* mesh;

    Ogre::MeshPtr create(Ogre::String const&);
};
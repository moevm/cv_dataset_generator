#pragma once
#include "../controller/Config.hpp"
#include "Ogre.h"
#include "OgreApplicationContext.h"

class View : OgreBites::ApplicationContext {
public:
    View(Config&);

    void init(OgreBites::InputListener*);
    void resize(unsigned int, unsigned int);
    void close();
    void end();
    void save(Ogre::String const&);
    void update();

    template <class... Args>
    Ogre::SceneNode* createSceneNode(Args&&... args) {
        return sceneManager->getRootSceneNode()->createChildSceneNode(
            std::forward<Args>(args)...);
    }

    template <class... Args>
    Ogre::Entity* createEntity(Args&&... args) {
        return sceneManager->createEntity(std::forward<Args>(args)...);
    }

    template <class... Args>
    Ogre::Light* createLight(Args&&... args) {
        return sceneManager->createLight(std::forward<Args>(args)...);
    }

    template <class... Args>
    Ogre::Camera* createCamera(Args&&... args) {
        return sceneManager->createCamera(std::forward<Args>(args)...);
    }

    template <class... Args>
    Ogre::ManualObject* createManualObject(Args&&... args) {
        return sceneManager->createManualObject(std::forward<Args>(args)...);
    }

    template <class... Args>
    Ogre::Viewport* addViewport(Args&&... args) {
        return getRenderWindow()->addViewport(std::forward<Args>(args)...);
    }

private:
    Ogre::SceneManager* sceneManager;
    Config& config;

    void setup() override;
    void setupOverlay();
    void distort(Ogre::String const&);
};
#pragma once
#include "Ogre.h"
#include "OgreApplicationContext.h"

class View : OgreBites::ApplicationContext {
public:
    View();

    void init(OgreBites::InputListener*);
    void close();
    void end();
    void save(Ogre::String const&);

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

    void setup() override;
};
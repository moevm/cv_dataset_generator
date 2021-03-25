#include "View.hpp"

View::View() : OgreBites::ApplicationContext("CV Dataset Generator") {
    initApp();
}

void View::init(OgreBites::InputListener* inputListener) {
    addInputListener(inputListener);
    getRoot()->startRendering();
}

void View::close() {
    closeApp();
}

void View::end() {
    getRoot()->queueEndRendering();
}

void View::save(Ogre::String const& filename) {
    getRenderWindow()->writeContentsToFile(filename);
}

void View::setup() {
    OgreBites::ApplicationContext::setup();

    sceneManager = getRoot()->createSceneManager();
    sceneManager->setAmbientLight(Ogre::ColourValue(0.5, 0.5, 0.5));
    sceneManager->setShadowTechnique(Ogre::ShadowTechnique::SHADOWTYPE_STENCIL_ADDITIVE);
    Ogre::RTShader::ShaderGenerator* shadergen =
        Ogre::RTShader::ShaderGenerator::getSingletonPtr();
    shadergen->addSceneManager(sceneManager);
}
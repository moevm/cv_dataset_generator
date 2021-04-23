#include "View.hpp"
// #include <opencv4/opencv2/opencv.hpp>

View::View() : OgreBites::ApplicationContext("CV Dataset Generator") {
    initApp();
}

void View::init(OgreBites::InputListener* inputListener) {
    addInputListener(inputListener);
    getRoot()->startRendering();
}

void View::resize(unsigned int width, unsigned int height) {
    getRenderWindow()->resize(width, height);
}

void View::close() {
    closeApp();
}

void View::end() {
    getRoot()->queueEndRendering();
}

void View::save(Ogre::String const& filename) {
    getRenderWindow()->writeContentsToFile(filename);
    // distort(filename);
}

void View::update() {
    getRoot()->renderOneFrame();
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

void View::distort(Ogre::String const& filename) {
    // TODO
    // cv::Mat image = cv::imread(filename);
    // image.convertTo(image, CV_32FC2);
    // image.resize(2);
    // cv::Mat output;
    // cv::fisheye::distortPoints(image, output, {}, {});
    // cv::imwrite(filename, output);
}
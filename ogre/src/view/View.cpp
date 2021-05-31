#include "View.hpp"
#include "OgreOverlay.h"
#include "OgreOverlayManager.h"
#include "OgreOverlaySystem.h"
#include "OgreTextAreaOverlayElement.h"
#include <opencv4/opencv2/opencv.hpp>

View::View(Config& config)
    : OgreBites::ApplicationContext("CV Dataset Generator"), config(config) {
    initApp();
    setupOverlay();
}

void View::init(OgreBites::InputListener* inputListener) {
    if (config.cameraInfo)
        resize(config.cameraInfo->width, config.cameraInfo->height);
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
    Ogre::OverlayElement* overlay =
        Ogre::OverlayManager::getSingleton().getOverlayElement("StatusPanel");
    overlay->hide();
    update();
    getRenderWindow()->writeContentsToFile(filename);
    overlay->show();
    update();
    distort(filename);
}

void View::update() {
    getRenderWindow()->update();
}

void View::statusUpdate(Position const& position) {
    Ogre::TextAreaOverlayElement* textArea = static_cast<Ogre::TextAreaOverlayElement*>(
        Ogre::OverlayManager::getSingleton().getOverlayElement(
            "StatusTextArea"));
    textArea->setCaption(position.displayString());
}

void View::setup() {
    OgreBites::ApplicationContext::setup();

    sceneManager = getRoot()->createSceneManager();
    sceneManager->setAmbientLight(Ogre::ColourValue(0.192, 0.152, 0.149));
    sceneManager->setShadowTechnique(Ogre::ShadowTechnique::SHADOWTYPE_STENCIL_ADDITIVE);
    Ogre::RTShader::ShaderGenerator* shadergen =
        Ogre::RTShader::ShaderGenerator::getSingletonPtr();
    shadergen->addSceneManager(sceneManager);
}

void View::setupOverlay() {
    Ogre::OverlayManager* overlayManager = Ogre::OverlayManager::getSingletonPtr();
    Ogre::Overlay* overlay = overlayManager->getByName("StatusOverlay");
    overlay->show();
    sceneManager->addRenderQueueListener(getOverlaySystem());
}

void View::distort(Ogre::String const& filename) {
    if (!config.cameraInfo)
        return;
    cv::Mat image = cv::imread(filename);
    cv::Mat cameraMatrix = cv::Mat(3, 3, CV_64F, config.cameraInfo->K.data());
    cv::Mat distCoeffs = cv::Mat(1, 5, CV_64F, config.cameraInfo->D.data());

    cv::Size imageSize = {image.cols, image.rows};
    cv::Mat mapX = cv::Mat(imageSize, CV_32FC1);
    cv::Mat mapY = cv::Mat(imageSize, CV_32FC1);
    std::vector<cv::Point2f> points, distPoints;
    for (int y = 0; y < imageSize.height; ++y)
        for (int x = 0; x < imageSize.width; ++x)
            distPoints.emplace_back(x, y);
    cv::undistortPoints(distPoints, points, cameraMatrix, distCoeffs,
                        cv::noArray(), cameraMatrix);

    for (int y = 0; y < imageSize.height; ++y)
        for (int x = 0; x < imageSize.width; ++x) {
            auto const& point = points[y * imageSize.width + x];
            mapX.at<float>(y, x) = point.x;
            mapY.at<float>(y, x) = point.y;
        }

    cv::Mat output;
    cv::remap(image, output, mapX, mapY, cv::INTER_CUBIC);
    cv::imwrite(filename, output);
}
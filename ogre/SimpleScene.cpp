#include "Ogre.h"
#include "OgreApplicationContext.h"

class SimpleScene : public OgreBites::ApplicationContext, public OgreBites::InputListener {
public:
    SimpleScene();
    void setup() override;
    bool keyPressed(const OgreBites::KeyboardEvent& evt) override;
};

SimpleScene::SimpleScene()
    : OgreBites::ApplicationContext("Simple virtual scene") {
}

Ogre::ManualObject* createCubeMesh(Ogre::String name, Ogre::String matName) {
    Ogre::ManualObject* cube = new Ogre::ManualObject(name);

    cube->begin(matName);

    // Front
    cube->position(-1, 1, 1);
    cube->normal(0, 0, 1);
    cube->position(-1, -1, 1);
    cube->normal(0, 0, 1);
    cube->position(1, -1, 1);
    cube->normal(0, 0, 1);
    cube->position(1, 1, 1);
    cube->normal(0, 0, 1);

    // Back
    cube->position(-1, 1, -1);
    cube->normal(0, 0, -1);
    cube->position(-1, -1, -1);
    cube->normal(0, 0, -1);
    cube->position(1, -1, -1);
    cube->normal(0, 0, -1);
    cube->position(1, 1, -1);
    cube->normal(0, 0, -1);

    // Top
    cube->position(-1, 1, 1);
    cube->normal(0, 1, 0);
    cube->position(-1, 1, -1);
    cube->normal(0, 1, 0);
    cube->position(1, 1, -1);
    cube->normal(0, 1, 0);
    cube->position(1, 1, 1);
    cube->normal(0, 1, 0);

    // Bottom
    cube->position(-1, -1, 1);
    cube->normal(0, -1, 0);
    cube->position(-1, -1, -1);
    cube->normal(0, -1, 0);
    cube->position(1, -1, -1);
    cube->normal(0, -1, 0);
    cube->position(1, -1, 1);
    cube->normal(0, -1, 0);

    // Right
    cube->position(1, -1, 1);
    cube->normal(1, 0, 0);
    cube->position(1, -1, -1);
    cube->normal(1, 0, 0);
    cube->position(1, 1, -1);
    cube->normal(1, 0, 0);
    cube->position(1, 1, 1);
    cube->normal(1, 0, 0);

    // Left
    cube->position(-1, -1, 1);
    cube->normal(-1, 0, 0);
    cube->position(-1, -1, -1);
    cube->normal(-1, 0, 0);
    cube->position(-1, 1, -1);
    cube->normal(-1, 0, 0);
    cube->position(-1, 1, 1);
    cube->normal(-1, 0, 0);

    for (Ogre::uint32 i = 0; i < 6 * 4; i += 4) {
        cube->quad(i, i + 1, i + 2, i + 3);
    }

    cube->end();

    return cube;
}

void SimpleScene::setup() {
    // base setup
    OgreBites::ApplicationContext::setup();

    // register for input events
    addInputListener(this);

    // get a pointer to the already created root
    Ogre::Root* root = getRoot();
    Ogre::SceneManager* scnMgr = root->createSceneManager();

    scnMgr->setAmbientLight(Ogre::ColourValue(0.5, 0.5, 0.5));
    scnMgr->setShadowTechnique(Ogre::ShadowTechnique::SHADOWTYPE_STENCIL_ADDITIVE);

    // register our scene with the RTSS
    Ogre::RTShader::ShaderGenerator* shadergen =
        Ogre::RTShader::ShaderGenerator::getSingletonPtr();
    shadergen->addSceneManager(scnMgr);

    // add spotlight
    Ogre::Light* light = scnMgr->createLight("SpotLight");
    light->setType(Ogre::Light::LT_SPOTLIGHT);
    light->setDiffuseColour(0.7, 0.5, 0.6);
    light->setSpecularColour(1.0, 0, 1.0);
    Ogre::SceneNode* lightNode = scnMgr->getRootSceneNode()->createChildSceneNode();
    lightNode->setPosition(10, 20, 20);
    lightNode->setDirection(-10, -20, -20);
    lightNode->attachObject(light);

    // add the camera
    Ogre::Camera* cam = scnMgr->createCamera("MainCamera");
    cam->setNearClipDistance(5);
    cam->setFarClipDistance(0);
    cam->setAutoAspectRatio(true);

    Ogre::SceneNode* camNode = scnMgr->getRootSceneNode()->createChildSceneNode();
    camNode->setPosition(0, 20, 30);
    camNode->lookAt(Ogre::Vector3(0, 5, 0), Ogre::Node::TS_PARENT);
    camNode->attachObject(cam);

    // render camera into the main window
    Ogre::Viewport* viewPort = getRenderWindow()->addViewport(cam);
    viewPort->setBackgroundColour(Ogre::ColourValue(0.4, 0.5, 0.6));

    // add a cube
    Ogre::Entity* cube =
        scnMgr->createEntity(createCubeMesh("Cube", "Red")->convertToMesh("CubeMesh"));
    cube->setCastShadows(true);

    Ogre::SceneNode* cubeNode = scnMgr->getRootSceneNode()->createChildSceneNode();
    cubeNode->setPosition(0, 5, 0);
    cubeNode->setScale(3, 3, 3);
    cubeNode->yaw(Ogre::Degree(30));
    cubeNode->roll(Ogre::Degree(60));
    cubeNode->attachObject(cube);

    // add ground
    Ogre::Plane plane(Ogre::Vector3::UNIT_Y, 0);
    Ogre::MeshManager::getSingleton().createPlane("ground", Ogre::RGN_DEFAULT,
                                                  plane, 30, 30, 20, 20, true, 1,
                                                  5, 5, Ogre::Vector3::UNIT_Z);
    Ogre::Entity* groundEntity = scnMgr->createEntity("ground");
    groundEntity->setCastShadows(false);
    groundEntity->setMaterialName("Yellow");

    Ogre::SceneNode* groundNode = scnMgr->getRootSceneNode()->createChildSceneNode();
    groundNode->setPosition(0, 0, -5);
    groundNode->attachObject(groundEntity);
}

bool SimpleScene::keyPressed(const OgreBites::KeyboardEvent& evt) {
    if (evt.keysym.sym == OgreBites::SDLK_ESCAPE) {
        getRoot()->queueEndRendering();
    }
    return true;
}

int main(int argc, char* argv[]) {
    SimpleScene app;
    app.initApp();
    app.getRoot()->startRendering();
    app.closeApp();
    return 0;
}

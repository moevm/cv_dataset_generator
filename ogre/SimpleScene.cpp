#include "Ogre.h"
#include "OgreApplicationContext.h"

class SimpleScene : public OgreBites::ApplicationContext, public OgreBites::InputListener {
    Ogre::SceneManager* scnMgr;
    Ogre::SceneNode* camNode;

    void spotlight() {
        Ogre::Light* light = scnMgr->createLight("SpotLight");
        light->setType(Ogre::Light::LT_SPOTLIGHT);
        light->setDiffuseColour(0.7, 0.5, 0.6);
        light->setSpecularColour(1.0, 0, 1.0);

        Ogre::SceneNode* lightNode = scnMgr->getRootSceneNode()->createChildSceneNode();
        lightNode->setPosition(10, 30, 20);
        lightNode->setDirection(-10, -30, -20);
        lightNode->attachObject(light);
    }

    void camera() {
        Ogre::Camera* cam = scnMgr->createCamera("MainCamera");
        cam->setNearClipDistance(5);
        cam->setFarClipDistance(0);
        cam->setAutoAspectRatio(true);

        camNode = scnMgr->getRootSceneNode()->createChildSceneNode();
        camNode->setPosition(0, 20, 30);
        camNode->lookAt(Ogre::Vector3(0, 5, 0), Ogre::Node::TS_PARENT);
        camNode->attachObject(cam);

        // render camera into the main window
        Ogre::Viewport* viewPort = getRenderWindow()->addViewport(cam);
        viewPort->setBackgroundColour(Ogre::ColourValue(0.4, 0.5, 0.6));
    }

    void cube() {
        Ogre::Entity* cube =
            scnMgr->createEntity("Cube", Ogre::SceneManager::PrefabType::PT_CUBE);
        cube->setCastShadows(true);
        cube->setMaterialName("Red");

        Ogre::SceneNode* cubeNode = scnMgr->getRootSceneNode()->createChildSceneNode();
        cubeNode->setPosition(-2, 6, 0);
        Ogre::Real scale = 0.07;
        cubeNode->setScale(scale, scale, scale);
        cubeNode->yaw(Ogre::Degree(30));
        cubeNode->roll(Ogre::Degree(60));
        cubeNode->attachObject(cube);
    }

    void sphere() {
        Ogre::Entity* sphere =
            scnMgr->createEntity("Sphere", Ogre::SceneManager::PrefabType::PT_SPHERE);
        sphere->setCastShadows(true);
        sphere->setMaterialName("Grey");

        Ogre::SceneNode* sphereNode = scnMgr->getRootSceneNode()->createChildSceneNode();
        sphereNode->setPosition(7, 14, 10);
        Ogre::Real scale = 0.05;
        sphereNode->setScale(scale, scale, scale);
        sphereNode->attachObject(sphere);
    }

    void ground() {
        Ogre::Plane plane(Ogre::Vector3::UNIT_Y, 0);
        Ogre::Real size = 40;
        Ogre::MeshManager::getSingleton().createPlane(
            "ground", Ogre::RGN_DEFAULT, plane, size, size, 20, 20, true, 1, 5,
            5, Ogre::Vector3::UNIT_Z);
        Ogre::Entity* groundEntity = scnMgr->createEntity("ground");
        groundEntity->setCastShadows(false);
        groundEntity->setMaterialName("Yellow");

        Ogre::SceneNode* groundNode = scnMgr->getRootSceneNode()->createChildSceneNode();
        groundNode->setPosition(0, 0, -5);
        groundNode->attachObject(groundEntity);
    }

public:
    SimpleScene() : OgreBites::ApplicationContext("Simple virtual scene") {
    }

    void setup() override {
        // base setup
        OgreBites::ApplicationContext::setup();

        // register for input events
        addInputListener(this);

        // get a pointer to the already created root
        Ogre::Root* root = getRoot();
        scnMgr = root->createSceneManager();

        scnMgr->setAmbientLight(Ogre::ColourValue(0.5, 0.5, 0.5));
        scnMgr->setShadowTechnique(Ogre::ShadowTechnique::SHADOWTYPE_STENCIL_ADDITIVE);

        // register our scene with the RTSS
        Ogre::RTShader::ShaderGenerator* shadergen =
            Ogre::RTShader::ShaderGenerator::getSingletonPtr();
        shadergen->addSceneManager(scnMgr);

        spotlight();
        camera();
        cube();
        sphere();
        ground();
    }

    bool keyPressed(const OgreBites::KeyboardEvent& evt) override {
        if (evt.keysym.sym == OgreBites::SDLK_ESCAPE) {
            getRoot()->queueEndRendering();
        } else if (evt.keysym.sym == 'w') {
            camNode->translate(0, 0, -.3);
        } else if (evt.keysym.sym == 's') {
            camNode->translate(0, 0, .3);
        } else if (evt.keysym.sym == 'a') {
            camNode->translate(-.3, 0, 0);
        } else if (evt.keysym.sym == 'd') {
            camNode->translate(.3, 0, 0);
        } else if (evt.keysym.sym == 'q') {
            camNode->translate(0, -.3, 0);
        } else if (evt.keysym.sym == 'e') {
            camNode->translate(0, .3, 0);
        }
        return true;
    }
};

int main(int argc, char* argv[]) {
    SimpleScene app;
    app.initApp();
    app.getRoot()->startRendering();
    app.closeApp();
    return 0;
}

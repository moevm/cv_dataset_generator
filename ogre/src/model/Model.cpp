#include "Model.hpp"

Model::Model(View& view, Config& config)
    : view(view),
      light(std::make_unique<Light>(view)),
      camera(std::make_unique<Camera>(view)),
      scene(std::make_unique<Scene>(view, config.modelFile)) {
    if (config.cameraInfo)
        getCamera().callibrate(*config.cameraInfo);
    getCamera().move(config.position);
    view.statusUpdate(getCamera().getPosition());
    view.addViewport(camera->getCamera())->setBackgroundColour(Ogre::ColourValue(0.4, 0.5, 0.6));
}

Camera& Model::getCamera() {
    return *camera;
}

Camera const& Model::getCamera() const {
    return *camera;
}
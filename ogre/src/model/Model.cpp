#include "Model.hpp"

Model::Model(View& view, Config const& config)
    : view(view),
      light(std::make_unique<Light>(view)),
      camera(std::make_unique<Camera>(view)),
      cube(std::make_unique<Cube>(view)),
      sphere(std::make_unique<Sphere>(view)),
      ground(std::make_unique<Ground>(view)) {
    if (config.cameraInfo)
        getCamera().callibrate(*config.cameraInfo);
    view.addViewport(camera->getCamera())->setBackgroundColour(Ogre::ColourValue(0.4, 0.5, 0.6));
}

Camera& Model::getCamera() {
    return *camera;
}

Camera const& Model::getCamera() const {
    return *camera;
}
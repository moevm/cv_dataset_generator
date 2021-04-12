#include "Model.hpp"

Model::Model(View& view)
    : view(view),
      light(std::make_unique<Light>(view)),
      camera(std::make_unique<Camera>(view)),
      cube(std::make_unique<Cube>(view)),
      sphere(std::make_unique<Sphere>(view)),
      ground(std::make_unique<Ground>(view)) {
    view.addViewport(camera->getCamera())->setBackgroundColour(Ogre::ColourValue(0.4, 0.5, 0.6));
}

void Model::move(Direction direction) {
    camera->move(direction);
}

void Model::move(Position const& position) {
    camera->move(position);
}

void Model::move(OgreBites::MouseMotionEvent const& evt) {
    camera->move(evt);
}